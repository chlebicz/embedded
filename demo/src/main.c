#include <stdlib.h>

#include "LPC17xx.h"        /* Dodano dla usunięcia błędów MISRA 17.3 związanych z m m.in. NVIC i LPC_SC */
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_dac.h"    /* Dodano dla usunięcia błędów MISRA 17.3 (funkcje DAC_Init) */

#include "joystick.h"
#include "oled.h"
#include "light.h"
#include "acc.h"

#include "temp.h"
#include "music.h"
#include "pca9532.h" /* Dodano dla pca9532_setLeds */

#define LUX_DARK_THRESHOLD   100U
#define LUX_LIGHT_THRESHOLD  150U

/* Forward declarations to satisfy MISRA-C:2012 Rule 8.4 */
void SysTick_Handler(void);
void TIMER1_IRQHandler(void);
void TIMER2_IRQHandler(void);
void check_failed(uint8_t *file, uint32_t line);

/* Forward declarations for static functions used before definition */
static void update_oled_message(void);
static void set_pwm_value(int channel, int value);
void Timer0_Wait(uint32_t time); /* Prototyp usunie ostrzezenia o braku deklaracji */

enum Theme
{
  DARK,
  LIGHT
};
static enum Theme curr_theme = DARK;

static int curr_value = 0;                      // Moc silnika

static uint16_t volume = 0U;

static void init_adc(void)
{
  PINSEL_CFG_Type PinCfg;

  /*
   * Init ADC pin connect
   * AD0.0 on P0.23
   */
  PinCfg.Funcnum = 1U;
  PinCfg.OpenDrain = 0U;
  PinCfg.Pinmode = 0U;
  PinCfg.Pinnum = 23U;
  PinCfg.Portnum = 0U;
  PINSEL_ConfigPin(&PinCfg);

  /* Configuration for ADC :
   * Frequency at 0.2Mhz
   * ADC channel 0, no Interrupt
   */
  ADC_Init(LPC_ADC, 200000U);
  ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, DISABLE);
  ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

static volatile uint8_t ticks = 0U;

static void timer_reset(void)
{
  ticks = 0U;

  TIM_TIMERCFG_Type TIM_ConfigStruct;
  TIM_MATCHCFG_Type TIM_MatchConfigStruct;

  // Konfiguracja timera na odliczanie w milisekundach (1000 us = 1 ms)
  TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
  TIM_ConfigStruct.PrescaleValue = 1000U;

  // Konfiguracja rejestru dopasowania (Match 0) na 1000 ms
  TIM_MatchConfigStruct.MatchChannel = 0U;
  TIM_MatchConfigStruct.IntOnMatch = ENABLE; // Wywołaj przerwanie, gdy doliczy do 1000
  TIM_MatchConfigStruct.ResetOnMatch = ENABLE; // Zresetuj licznik po osiągnięciu wartości
  TIM_MatchConfigStruct.StopOnMatch = DISABLE; // Zatrzymaj timer po 1 sekundzie (uruchomimy go znowu ręcznie)
  TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
  TIM_MatchConfigStruct.MatchValue = 1000U;

  // Inicjalizacja
  TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TIM_ConfigStruct);
  TIM_ConfigMatch(LPC_TIM1, &TIM_MatchConfigStruct);

  // Włączenie przerwań dla Timera 1 w kontrolerze NVIC
  NVIC_SetPriority(TIMER1_IRQn, 10U);
  NVIC_EnableIRQ(TIMER1_IRQn);
}

static void timer_stop(void)
{
  NVIC_DisableIRQ(TIMER1_IRQn);
  ticks = 0U;
}

static void init_delay_timer(void)
{
  TIM_TIMERCFG_Type TIM_ConfigStruct;

  // Set prescaler to tick exactly every 1 microsecond
  TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
  TIM_ConfigStruct.PrescaleValue = 1;
  
  // Initialize Timer 3
  TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &TIM_ConfigStruct);
}

static void delay_us(uint32_t us)
{
  LPC_TIM3->TCR = 0x02; // Reset Timer Counter (TC = 0)
  LPC_TIM3->TCR = 0x01; // Enable and Start Timer

  // Wait until the Timer Counter reaches the requested microseconds
  while (LPC_TIM3->TC < us) 
  {
    // Blocking wait
  }

  LPC_TIM3->TCR = 0x00; // Stop the Timer to save power
}

static void increase_amplifier_volume(int levels)
{
  // Ustaw pin kierunku głośności (UP/DN) na WYSOKI (1 = Zgłaśnianie)
  GPIO_SetValue(0U, ((uint32_t)1U<<28U));

  for (int i = 0; i < levels; i++)
  {
    // Impuls zegara (Wysoki -> Niski)
    GPIO_SetValue(0U, ((uint32_t)1U<<27U));    // CLK High
    delay_us(10U);                 // Krótka przerwa
    GPIO_ClearValue(0U, ((uint32_t)1U<<27U));  // CLK Low
    delay_us(10U);                 // Krótka przerwa
  }
}

static void music_init(void)
{
  static const uint32_t SAMPLE_RATE = 8000U; // hz
  // 1. OBUDŹ WZMACNIACZ LM4811
  // Ustawienie pinów sterujących wzmacniaczem jako wyjścia
  GPIO_SetDir(0U, ((uint32_t)1U<<27U), 1U);
  GPIO_SetDir(0U, ((uint32_t)1U<<28U), 1U);
  GPIO_SetDir(2U, ((uint32_t)1U<<13U), 1U);
  GPIO_SetDir(0U, ((uint32_t)1U<<26U), 1U);

  // Stan niski na pinie 2.13 wyłącza tryb "shutdown" wzmacniacza
  GPIO_ClearValue(0U, ((uint32_t)1U<<27U)); // LM4811-clk
  GPIO_ClearValue(0U, ((uint32_t)1U<<28U)); // LM4811-up/dn
  GPIO_ClearValue(2U, ((uint32_t)1U<<13U)); // LM4811-shutdn

  increase_amplifier_volume(16); // max

  // 2. SKONFIGURUJ PIN DAC
  PINSEL_CFG_Type PinCfgDAC;
  PinCfgDAC.Funcnum = 2U;   // Func 2 to AOUT (DAC)
  PinCfgDAC.OpenDrain = 0U;
  PinCfgDAC.Pinmode = 0U;
  PinCfgDAC.Portnum = 0U;
  PinCfgDAC.Pinnum = 26U;
  PINSEL_ConfigPin(&PinCfgDAC);

  // Inicjalizacja peryferium DAC
  DAC_Init(LPC_DAC);

  // 3. SKONFIGURUJ TIMER 2
  TIM_TIMERCFG_Type TIM_ConfigStruct;
  TIM_MATCHCFG_Type TIM_MatchConfigStruct;

  LPC_SC->PCONP |= ((uint32_t)1U << 22U); // Zasilanie Timera 2

  TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
  TIM_ConfigStruct.PrescaleValue = 1U;

  TIM_MatchConfigStruct.MatchChannel = 0U;
  TIM_MatchConfigStruct.IntOnMatch = ENABLE;
  TIM_MatchConfigStruct.ResetOnMatch = ENABLE;
  TIM_MatchConfigStruct.StopOnMatch = DISABLE;
  TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

  // Czas trwania jednej próbki w mikrosekundach
  TIM_MatchConfigStruct.MatchValue = 1000000U / SAMPLE_RATE;

  TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TIM_ConfigStruct);
  TIM_ConfigMatch(LPC_TIM2, &TIM_MatchConfigStruct);

  NVIC_SetPriority(TIMER2_IRQn, 10U);
  NVIC_EnableIRQ(TIMER2_IRQn);

  TIM_Cmd(LPC_TIM2, ENABLE);
}

static volatile uint32_t msTicks = 0U; // Zegar systemowy dla termometru

void SysTick_Handler(void)
{
  msTicks++;
}

static uint32_t getTicks(void)
{
  return msTicks;
}

// 5. The Handler
void TIMER1_IRQHandler(void)
{
  // Check if the interrupt came from Match 0
  if (TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT) == SET)
  {
    ticks++;

    // Clear the interrupt flag so it doesn't loop infinitely
    TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
  }
}

#define MUSIC_SIZE (sizeof(music) / sizeof(music[0]))
#define AUDIO_START_OFFSET 44U

void TIMER2_IRQHandler(void)
{  
  if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT) == SET)
  {
	  static volatile uint32_t current_sample_index = AUDIO_START_OFFSET;
    // Wyczyść flagę przerwania
    TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);

    // Pobierz 8-bitową próbkę (0..255)
    uint32_t sample = music[current_sample_index];

    // Skalowanie głośności:
    // volume: 0..4096
    // sample DAC: 10-bit (0..1023)
    uint32_t dac_value = ((sample << 2U) * volume) >> 12U;

    // Wyślij do DAC
    DAC_UpdateValue(LPC_DAC, dac_value);

    // Następna próbka
    current_sample_index++;

    // Zapętlenie audio
    if (current_sample_index >= MUSIC_SIZE)
    {
      current_sample_index = AUDIO_START_OFFSET;
    }
  }
}

static void rotate_motor(uint8_t joyState)
{
  if ((joyState & JOYSTICK_CENTER) != 0U)
  {
    curr_value = 0;
    TIM_Cmd(LPC_TIM1, ENABLE);
  }

  if ((curr_value < 500) && (curr_value > 0))
  {
    curr_value = 500;
  }

  if ((curr_value > -500) && (curr_value < 0))
  {
    curr_value = -500;
  }

  if ((joyState & JOYSTICK_RIGHT) != 0U)
  {
    curr_value += 3;
  }

  if ((joyState & JOYSTICK_LEFT) != 0U)
  {
    curr_value -= 3;
  }

  if (curr_value > 1000)
  {
    curr_value = 1000;
  }
  else if (curr_value < -1000)
  {
    curr_value = -1000;
  } 
  else 
  {
    /* Zgodnie z MISRA puste else powinno zawierać komentarz */
  }

  if (curr_value > 0)
  {
    set_pwm_value(1, curr_value); // Pin P2.0 dostaje sygnał PWM (pulsujące napięcie)
    set_pwm_value(2, 0);          // Pin P2.3 jest zwarty do masy (GND)
  }
  else if (curr_value < 0)
  {
    set_pwm_value(1, 0);           // Pin P2.0 jest zwarty do masy
    set_pwm_value(2, -curr_value); // Pin P2.3 dostaje sygnał PWM
  }
  else
  {
    set_pwm_value(1, 0);
    set_pwm_value(2, 0);
  }
}

static void update_oled_theme_based_on_light(void)
{
  uint32_t lux = light_read();

  if (lux > LUX_LIGHT_THRESHOLD)
  {
    curr_theme = LIGHT;
  }
  else if (lux < LUX_DARK_THRESHOLD)
  {
    curr_theme = DARK;
  } 
  else 
  {
    /* Puste else (MISRA) */
  }
}

#define LINE_COUNT 5U
#define LINE_LENGTH 12U

static uint8_t oled_buffer[LINE_COUNT][LINE_LENGTH + 1U] = {
  "            ",
  "            ",
  "            ",
  "            ",
  "            "
};

static void oled_buffer_put(uint8_t line, const uint8_t* data)
{
  /* Usunięto early return (MISRA 15.5) i wyeliminowano modyfikację argumentu 'data' (MISRA 17.8) */
  if ((line < LINE_COUNT) && (data != NULL))
  {
    const uint8_t* ptr = data;
    for (uint32_t i = 0U; i < LINE_LENGTH; i++)
    {
      if (*ptr != '\0')
      {
        oled_buffer[line][i] = *ptr;
        ptr++;
      }
      else
      {
        oled_buffer[line][i] = ' ';
      }
    }
  }
}

#define CHAR_WIDTH 6U
#define CHAR_HEIGHT 8U

static void update_oled_with_buffer(void)
{
  static enum Theme prev_theme = DARK;
  static uint8_t first_draw = TRUE;

  static uint8_t prev_oled_buffer[LINE_COUNT][LINE_LENGTH + 1U] = {
    "            ",
    "            ",
    "            ",
    "            ",
    "            "
  };

  /* Poprawka MISRA 12.3: Rozdzielono wielokrotne deklaracje zmiennych dla uniknięcia wirtualnego operatora przecinka */
  oled_color_t oled_fg;
  oled_color_t oled_bg;

  if (curr_theme == LIGHT)
  {
    oled_bg = OLED_COLOR_WHITE;
    oled_fg = OLED_COLOR_BLACK;
  }
  else
  {
    oled_bg = OLED_COLOR_BLACK;
    oled_fg = OLED_COLOR_WHITE;
  }

  if ((curr_theme != prev_theme) || (first_draw != 0U))
  {
    oled_clearScreen(oled_bg);
  }

  for (uint32_t line = 0U; line < LINE_COUNT; ++line)
  {
    for (uint32_t i = 0U; i < LINE_LENGTH; ++i)
    {
      if ((oled_buffer[line][i] != prev_oled_buffer[line][i]) || (curr_theme != prev_theme) || (first_draw != 0U))
      {
        oled_putChar(
          (uint16_t)(i * (CHAR_WIDTH + 2U)),
          (uint16_t)(line * (CHAR_HEIGHT + 2U)),
          oled_buffer[line][i],
          oled_fg,
          oled_bg
        );
        prev_oled_buffer[line][i] = oled_buffer[line][i];
      }
    }
  }

  prev_theme = curr_theme;
  first_draw = FALSE;
}

static void update_oled_message(void)
{
  const char* state = "";          // Stoi/Wciaganie/Opuszczanie
  const char* power = "";          // wartosc mocy

  if (curr_value == 0)
  {
    state = "Stoi";
  }
  else if (curr_value > 0)
  {
    state = "Wciaganie";
  }
  else
  {
    state = "Opuszczanie";
  }

  /* Poprawka MISRA 10.3 & 10.8: Złożone operacje przed rzutowaniem przeniesione do tymczasowych zmiennych uint32_t */
  uint32_t u_curr = (uint32_t)abs(curr_value);
  
  uint32_t calc_tens = ((u_curr / 10U) % 10U) + 48U;
  uint8_t tens = (uint8_t)calc_tens;
  
  uint32_t calc_hundreds = ((u_curr / 100U) % 10U) + 48U;
  uint8_t hundreds = (uint8_t)calc_hundreds;
  
  uint32_t calc_thousands = ((u_curr / 1000U) % 10U) + 48U;
  uint8_t thousands = (uint8_t)calc_thousands;

  if ((curr_value == 1000) || (curr_value == -1000))
  {
    thousands = (uint8_t)53U; /* ASCII '5' jako uint8_t */
  }

  char secondLine[] = { '1', '0', '0', '%', ' ', 'm', 'o', 'c', 'y', '\0' };
  secondLine[0] = (char)thousands;
  secondLine[1] = (char)hundreds;
  secondLine[2] = (char)tens;
  power = secondLine;

  if (state[0] != '\0')
  {
    oled_buffer_put(0U, (const uint8_t*)state);
  }

  if (power[0] != '\0')
  {
    oled_buffer_put(1U, (const uint8_t*)power);
  }
}

static void init_pwm(void)
{
  LPC_PWM1->MR0 = 1000U; // okres pwm
  LPC_PWM1->LER |= (1U << 0U); // zatwierdzenie MR0
  // rejestry MR są 32-bitowe

  // bit 0 - wlaczenie glownego licznika i prescalera
  // bit 3 - pwm enable
  LPC_PWM1->TCR |= (1U << 0U) | (1U << 3U);

  PINSEL_CFG_Type PinCfg;
  PinCfg.Portnum = 2U;
  PinCfg.Pinmode = 0U;
  PinCfg.Funcnum = 1U;
  PinCfg.OpenDrain = 0U;

  // PIO1_9
  PinCfg.Pinnum = 0U;
  PINSEL_ConfigPin(&PinCfg);
  LPC_PWM1->MR1 = 500U; // 50%
  LPC_PWM1->LER |= (1U << 1U); // zatwierdzenie rejestru MR1
  LPC_PWM1->PCR |= (((uint32_t)1U << (9U + 0U))); // aktywacja wyjscia sygnalu dla kanalu 2

  // PIO2_3
  PinCfg.Pinnum = 3U;
  PINSEL_ConfigPin(&PinCfg);
  LPC_PWM1->MR4 = 500U; // 50%
  LPC_PWM1->LER |= (1U << 4U); // zatwierdzenie MR4
  LPC_PWM1->PCR |= (((uint32_t)1U << (9U + 3U)));
}

static void set_pwm_value(int channel, int value)
{
  if (channel == 1)
  {
    LPC_PWM1->MR1 = (uint32_t)value;
    LPC_PWM1->LER |= (1U << 1U);
  }
  else if (channel == 2)
  {
    LPC_PWM1->MR4 = (uint32_t)value;
    LPC_PWM1->LER |= (1U << 4U);
  }
  else
  {
    /* Ignoruj inny kanał */
  }
}

static void init_ssp(void)
{
  SSP_CFG_Type SSP_ConfigStruct;
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = 2U;
  PinCfg.OpenDrain = 0U;
  PinCfg.Pinmode = 0U;
  PinCfg.Portnum = 0U;
  PinCfg.Pinnum = 7U;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum = 8U;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum = 9U;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Funcnum = 0U;
  PinCfg.Portnum = 2U;
  PinCfg.Pinnum = 2U;
  PINSEL_ConfigPin(&PinCfg);

  SSP_ConfigStructInit(&SSP_ConfigStruct);

  // Initialize SSP peripheral with parameter given in structure above
  SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

  // Enable SSP peripheral
  SSP_Cmd(LPC_SSP1, ENABLE);
}

static void init_i2c(void)
{
  PINSEL_CFG_Type PinCfg;

  /* Initialize I2C2 pin connect */
  PinCfg.Funcnum = 2U;
  PinCfg.Pinnum = 10U; //GPIO_26-SDA P0.10 - do przesyłania danych
  PinCfg.Portnum = 0U;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum = 11U; //GPIO_27-SCL P0.11 - do synchronizacji
  PINSEL_ConfigPin(&PinCfg);

  // Initialize I2C peripheral
  // 100kHZ - taktowanie zegara SCL
  I2C_Init(LPC_I2C2, 100000U);

  /* Enable I2C1 operation */
  I2C_Cmd(LPC_I2C2, ENABLE);
}

static void update_volume(void)
{
  ADC_StartCmd(LPC_ADC, ADC_START_NOW);
  // Wait conversion complete
  while (ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE) == RESET)
  {
    /* MISRA 15.6 - pętla while musi zawierać nawiasy {} */
  }
  volume = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0); // 0 to 4096
}

static void update_leds(int8_t x, int8_t y)
{
  uint16_t led_mask = 0x0000;

  if ((x > 32) || (x < -32))
  {
    led_mask |= 0x00FF;
  }
  else if ((x > 25) || (x < -25))
  {
    led_mask |= 0x003F;
  }
  else if ((x > 17) || (x < -17))
  {
    led_mask |= 0x000F;
  }
  else if ((x > 7) || (x < -7))
  {
    led_mask |= 0x0003;
  }

  if ((y > 32) || (y < -32))
  {
    led_mask |= 0xFF00;
  }
  else if ((y > 25) || (y < -25))
  {
    led_mask |= 0xFC00;
  }
  else if ((y > 17) || (y < -17))
  {
    led_mask |= 0xF000;
  }
  else if ((y > 7) || (y < -7))
  {
    led_mask |= 0xC000;
  }

  static uint8_t prev_turned = FALSE;
  if ((x > 7) || (x < -7) || (y > 7) || (y < -7))
  {
    pca9532_setLeds(led_mask, 0xFFFF);
    prev_turned = TRUE;
  }
  else if (prev_turned)
  {
    pca9532_setLeds(led_mask, 0xFFFF);
    prev_turned = FALSE;
  }
}

int main(void)
{
  //uint8_t state = 0U;
  int8_t xoff = 0;
  int8_t yoff = 0;
  int8_t zoff = 0;
  int8_t x = 0; // (lewo – prawo)
  int8_t y = 0; // (przód – tył)
  int8_t z = 0; // (góra – dół)

  init_i2c();
  init_ssp();
  init_adc();
  init_pwm();
  acc_init();
  light_enable();

  init_delay_timer();

  joystick_init();
  oled_init();
  temp_init(&getTicks);
  if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
  {
    while (1)
    {
      /* Przechwycenie błędu jeśli zegar systemowy zawiedzie */
    }
  }
  acc_read(&x, &y, &z);

  xoff = -x;
  yoff = -y;
  zoff = -z;
  music_init();

  int cnt = 0;
  //int is_tilt_warn = 0;
  int is_time_warn = 0;
  int is_temp_warn = 0;
  
  while (1)
  {
    update_oled_theme_based_on_light();
    uint8_t state = joystick_read();
    acc_read(&x, &y, &z);
    x = x + xoff;
    y = y + yoff;
    z = z + zoff;

    update_leds(x, y);
    
    const uint8_t tilt_alert[] = "PRZECHYL!";
    const uint8_t time_alert[] = "CZAS!";
    const uint8_t temp_alert[] = "TEMP!";
    const uint8_t reset[] = "";

    int is_tilt_warn;
    if ((x > 17) || (x < -17) || (y > 17) || (y < -17))
    {
      is_tilt_warn = 1;
    }
    else
    {
      is_tilt_warn = 0;
    }

    if (state != 0U)
    {
      rotate_motor(state);

      static int prev_value = 69;

      if ((abs(curr_value - prev_value) > 100) && (curr_value != 0))
      {
        timer_reset();
      }

      if (curr_value == 0)
      {
        timer_stop();
      }

      if (prev_value != curr_value)
      {
        update_oled_message();
        prev_value = curr_value;
      }
    }

    /* Poprawka MISRA 10.8: Zmiana literału '0' na 48U i unikanie bezpośredniego rzutowania złożonego wyrażenia */
    uint32_t calc_units = (ticks % 10U) + 48U;
    uint8_t units = (uint8_t)calc_units;
    
    uint32_t calc_tens_ticks = ((ticks / 10U) % 10U) + 48U;
    uint8_t tens_ticks = (uint8_t)calc_tens_ticks;
    
    uint8_t value[] = { tens_ticks, units, '\0' };

    // Line 2 (Y=30) mapped for timer
    oled_buffer_put(2U, value);

    if (ticks >= 50U)
    {
      curr_value = 0;
      set_pwm_value(1, 0);
      set_pwm_value(2, 0);
      timer_stop();
    }
    else if (ticks >= 30U)
    {
      is_time_warn = 1;
    }
    else if (ticks == 0U) 
    {
      is_time_warn = 0;
    } 
    else 
    {
      /* Puste else */
    }

    if ((cnt % 100) == 0)
    {
      int32_t t_val = temp_read();
      
      int32_t t_int_calc = t_val / 10;
      int32_t t_dec_calc = t_val % 10;
      uint8_t t_int = (uint8_t)t_int_calc;
      uint8_t t_dec = (uint8_t)t_dec_calc;
      
      char temp_str[] = { 'T', 'e', 'm', 'p', ':', ' ', '0', '0', '.', '0', 'C', '\0' };

      if (t_int >= 30U)
      {
        is_temp_warn = 1;
      }
      else
      {
        is_temp_warn = 0;
      }

      /* Poprawka MISRA 10.8: Użyto zmiennych pomocniczych typu uin32_t dla bezpieczeństwa operacji */
      uint32_t t_int_u32 = (uint32_t)t_int;
      uint32_t char1_val = ((t_int_u32 / 10U) % 10U) + 48U;
      temp_str[6] = (char)char1_val;

      uint32_t char2_val = (t_int_u32 % 10U) + 48U;
      temp_str[7] = (char)char2_val;

      uint32_t t_dec_u32 = (uint32_t)t_dec;
      uint32_t char3_val = t_dec_u32 + 48U;
      temp_str[9] = (char)char3_val;
      
      if (temp_str[6] == '0')
      {
        temp_str[6] = ' ';
      }

      // Line 4 (Y=50) mapped for temperature
      oled_buffer_put(4U, (const uint8_t*)temp_str);
      cnt = 0;
    }

    /* Zmiana MISRA 14.4 - uzywamy jawnych warunków dla is_*_warn */
    if (is_tilt_warn != 0) 
    {
      oled_buffer_put(3U, tilt_alert);
    } 
    else if (is_time_warn != 0) 
    {
      oled_buffer_put(3U, time_alert);
    } 
    else if (is_temp_warn != 0)
    {
      oled_buffer_put(3U, temp_alert);
    } 
    else 
    {
      oled_buffer_put(3U, reset);
    }

    // Flush to screen only when lines actually changed
    update_oled_with_buffer();

    update_volume();

    cnt++;
    Timer0_Wait(1U);
  }
}

void check_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;

  while (1)
  {
    /* Aktywne czekanie / zatrzymanie po awarii asercji */
  }
}