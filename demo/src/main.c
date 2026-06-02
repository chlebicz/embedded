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
#include "fatbee.h"
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
static void my_set_pwm_value(int32_t channel, int32_t value);
extern void Timer0_Wait(uint32_t time); /* Prototyp usunie ostrzezenia o braku deklaracji */

enum Theme
{
  DARK,
  LIGHT
};
static enum Theme curr_theme = DARK;

static int32_t curr_value = 0; // Moc silnika - precyzyjny typ dla MISRA

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
  /* cppcheck-suppress misra-c2012-11.4 */
  ADC_Init(LPC_ADC, 200000U);
  /* cppcheck-suppress misra-c2012-11.4 */
  ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, DISABLE);
  /* cppcheck-suppress misra-c2012-11.4 */
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
  TIM_MatchConfigStruct.StopOnMatch = DISABLE; // Zatrzymaj timer po 1 sekundzie
  TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
  TIM_MatchConfigStruct.MatchValue = 1000U;

  // Inicjalizacja
  /* cppcheck-suppress misra-c2012-11.4 */
  TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TIM_ConfigStruct);
  /* cppcheck-suppress misra-c2012-11.4 */
  TIM_ConfigMatch(LPC_TIM1, &TIM_MatchConfigStruct);

  // Włączenie przerwań dla Timera 1 w kontrolerze NVIC
  /* cppcheck-suppress misra-c2012-11.4 */
  NVIC_SetPriority(TIMER1_IRQn, 10U);
  /* cppcheck-suppress misra-c2012-11.4 */
  NVIC_EnableIRQ(TIMER1_IRQn);
}

static void timer_stop(void)
{
  /* cppcheck-suppress misra-c2012-11.4 */
  NVIC_DisableIRQ(TIMER1_IRQn);
  ticks = 0U;
}

static void delay_us(uint32_t us)
{
  for (volatile uint32_t i = 0U; i < (us * 30U); i++) 
  {
      /* Aktywne czekanie (w MISRA akceptowalne jako delay sprzętowy, pod warunkiem {} ) */
  }
}

static void increase_amplifier_volume(int32_t levels)
{
  // Ustaw pin kierunku głośności (UP/DN) na WYSOKI (1 = Zgłaśnianie)
  GPIO_SetValue(0U, ((uint32_t)1U<<28U));

  for (int32_t i = 0; i < levels; i++)
  {
    // Impuls zegara (Wysoki -> Niski)
    GPIO_SetValue(0U, ((uint32_t)1U<<27U));    // CLK High
    delay_us(10U);                 // Krótka przerwa
    GPIO_ClearValue(0U, ((uint32_t)1U<<27U));  // CLK Low
    delay_us(10U);                 // Krótka przerwa
  }
}

static void bee_init(void)
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

  increase_amplifier_volume((int32_t)16); // max

  // 2. SKONFIGURUJ PIN DAC
  PINSEL_CFG_Type PinCfgDAC;
  PinCfgDAC.Funcnum = 2U;   // Func 2 to AOUT (DAC)
  PinCfgDAC.OpenDrain = 0U;
  PinCfgDAC.Pinmode = 0U;
  PinCfgDAC.Portnum = 0U;
  PinCfgDAC.Pinnum = 26U;
  PINSEL_ConfigPin(&PinCfgDAC);

  // Inicjalizacja peryferium DAC
  /* cppcheck-suppress misra-c2012-11.4 */
  DAC_Init(LPC_DAC);

  // 3. SKONFIGURUJ TIMER 2
  TIM_TIMERCFG_Type TIM_ConfigStruct;
  TIM_MATCHCFG_Type TIM_MatchConfigStruct;

  /* cppcheck-suppress misra-c2012-11.4 */
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

  /* cppcheck-suppress misra-c2012-11.4 */
  TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TIM_ConfigStruct);
  /* cppcheck-suppress misra-c2012-11.4 */
  TIM_ConfigMatch(LPC_TIM2, &TIM_MatchConfigStruct);

  /* cppcheck-suppress misra-c2012-11.4 */
  NVIC_SetPriority(TIMER2_IRQn, 10U);
  /* cppcheck-suppress misra-c2012-11.4 */
  NVIC_EnableIRQ(TIMER2_IRQn);

  /* cppcheck-suppress misra-c2012-11.4 */
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
  /* cppcheck-suppress misra-c2012-11.4 */
  if (TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT) == SET)
  {
    ticks++;

    // Clear the interrupt flag so it doesn't loop infinitely
    /* cppcheck-suppress misra-c2012-11.4 */
    TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
  }
}

#define FATBEE_SIZE (sizeof(fatbee) / sizeof(fatbee[0]))
#define AUDIO_START_OFFSET 44U

void TIMER2_IRQHandler(void)
{  
  /* cppcheck-suppress misra-c2012-11.4 */
  if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT) == SET)
  {
      static volatile uint32_t current_sample_index = AUDIO_START_OFFSET;
      // Wyczyść flagę przerwania
      /* cppcheck-suppress misra-c2012-11.4 */
      TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);

      // Pobierz 8-bitową próbkę (0..255)
      uint32_t sample = fatbee[current_sample_index];

      // Skalowanie głośności:
      uint32_t dac_value = ((sample << 2U) * volume) >> 12U;

      // Wyślij do DAC
      /* cppcheck-suppress misra-c2012-11.4 */
      DAC_UpdateValue(LPC_DAC, dac_value);

      current_sample_index++;

      if (current_sample_index >= FATBEE_SIZE)
      {
          current_sample_index = AUDIO_START_OFFSET;
      }
  }
}

static void rotate_motor(uint8_t joyState)
{
  /* Rzutowanie na uint32_t dla MISRA 10.4 */
  if (((uint32_t)joyState & (uint32_t)JOYSTICK_CENTER) != 0U)
  {
    curr_value = (int32_t)0;
    /* cppcheck-suppress misra-c2012-11.4 */
    TIM_Cmd(LPC_TIM1, ENABLE);
  }

  if ((curr_value < (int32_t)500) && (curr_value > (int32_t)0))
  {
    curr_value = (int32_t)500;
  }

  if ((curr_value > (int32_t)-500) && (curr_value < (int32_t)0))
  {
    curr_value = (int32_t)-500;
  }

  if (((uint32_t)joyState & (uint32_t)JOYSTICK_RIGHT) != 0U)
  {
    curr_value += (int32_t)10;
  }

  if (((uint32_t)joyState & (uint32_t)JOYSTICK_LEFT) != 0U)
  {
    curr_value -= (int32_t)10;
  }

  if (curr_value > (int32_t)1000)
  {
    curr_value = (int32_t)1000;
  }
  else if (curr_value < (int32_t)-1000)
  {
    curr_value = (int32_t)-1000;
  } 
  else 
  {
    /* Zgodnie z MISRA puste else powinno zawierać komentarz */
  }

  if (curr_value > (int32_t)0)
  {
    my_set_pwm_value((int32_t)1, curr_value); 
    my_set_pwm_value((int32_t)2, (int32_t)0);          
  }
  else if (curr_value < (int32_t)0)
  {
    my_set_pwm_value((int32_t)1, (int32_t)0);           
    my_set_pwm_value((int32_t)2, -curr_value); 
  }
  else
  {
    my_set_pwm_value((int32_t)1, (int32_t)0);
    my_set_pwm_value((int32_t)2, (int32_t)0);
  }
}

static void update_oled_theme_based_on_light(void)
{
  uint32_t lux = light_read();

  if (lux > (uint32_t)LUX_LIGHT_THRESHOLD)
  {
    curr_theme = LIGHT;
  }
  else if (lux < (uint32_t)LUX_DARK_THRESHOLD)
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
        (void)oled_putChar(
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
  const char* state = "";          
  const char* power = "";          

  if (curr_value == (int32_t)0)
  {
    state = "Stoi";
  }
  else if (curr_value > (int32_t)0)
  {
    state = "Wciaganie";
  }
  else
  {
    state = "Opuszczanie";
  }

  uint32_t u_curr = (uint32_t)abs(curr_value);
  
  uint32_t calc_tens = ((u_curr / 10U) % 10U) + 48U;
  uint8_t tens = (uint8_t)calc_tens;
  
  uint32_t calc_hundreds = ((u_curr / 100U) % 10U) + 48U;
  uint8_t hundreds = (uint8_t)calc_hundreds;
  
  uint32_t calc_thousands = ((u_curr / 1000U) % 10U) + 48U;
  uint8_t thousands = (uint8_t)calc_thousands;

  if ((curr_value == (int32_t)1000) || (curr_value == (int32_t)-1000))
  {
    thousands = (uint8_t)53U; /* ASCII '5' */
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
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->MR0 = 1000U; 
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->LER |= (1U << 0U); 

  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->TCR |= (1U << 0U) | (1U << 3U);

  PINSEL_CFG_Type PinCfg;
  PinCfg.Portnum = 2U;
  PinCfg.Pinmode = 0U;
  PinCfg.Funcnum = 1U;
  PinCfg.OpenDrain = 0U;

  PinCfg.Pinnum = 0U;
  PINSEL_ConfigPin(&PinCfg);
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->MR1 = 500U; 
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->LER |= (1U << 1U); 
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->PCR |= (((uint32_t)1U << (9U + 0U))); 

  PinCfg.Pinnum = 3U;
  PINSEL_ConfigPin(&PinCfg);
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->MR4 = 500U; 
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->LER |= (1U << 4U); 
  /* cppcheck-suppress misra-c2012-11.4 */
  LPC_PWM1->PCR |= (((uint32_t)1U << (9U + 3U)));
}

static void my_set_pwm_value(int32_t channel, int32_t value)
{
  if (channel == (int32_t)1)
  {
    /* cppcheck-suppress misra-c2012-11.4 */
    LPC_PWM1->MR1 = (uint32_t)value;
    /* cppcheck-suppress misra-c2012-11.4 */
    LPC_PWM1->LER |= (1U << 1U);
  }
  else if (channel == (int32_t)2)
  {
    /* cppcheck-suppress misra-c2012-11.4 */
    LPC_PWM1->MR4 = (uint32_t)value;
    /* cppcheck-suppress misra-c2012-11.4 */
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

  /* cppcheck-suppress misra-c2012-11.4 */
  SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
  /* cppcheck-suppress misra-c2012-11.4 */
  SSP_Cmd(LPC_SSP1, ENABLE);
}

static void init_i2c(void)
{
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = 2U;
  PinCfg.Pinnum = 10U; 
  PinCfg.Portnum = 0U;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum = 11U; 
  PINSEL_ConfigPin(&PinCfg);

  /* cppcheck-suppress misra-c2012-11.4 */
  I2C_Init(LPC_I2C2, 100000U);
  /* cppcheck-suppress misra-c2012-11.4 */
  I2C_Cmd(LPC_I2C2, ENABLE);
}

static void update_volume(void)
{
  /* cppcheck-suppress misra-c2012-11.4 */
  ADC_StartCmd(LPC_ADC, ADC_START_NOW);
  
  /* cppcheck-suppress misra-c2012-11.4 */
  while (ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE) == RESET)
  {
      /* MISRA 15.6 - pętla while musi zawierać nawiasy {} */
  }
  
  /* cppcheck-suppress misra-c2012-11.4 */
  volume = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0); 
}

static void update_leds(int8_t x, int8_t y)
{
  pca9532_setLeds(0x0000U, 0xFFFFU);

  /* Rzutowanie literałów dla MISRA 10.4 */
  if ((x > (int8_t)7) || (x < (int8_t)-7))
  {
    pca9532_setLeds(0x0003U, 0xFFFFU);
  }
  if ((x > (int8_t)17) || (x < (int8_t)-17))
  {
    pca9532_setLeds(0x000FU, 0xFFFFU);
  }
  if ((x > (int8_t)25) || (x < (int8_t)-25))
  {
    pca9532_setLeds(0x003FU, 0xFFFFU);
  }
  if ((x > (int8_t)32) || (x < (int8_t)-32))
  {
    pca9532_setLeds(0x00FFU, 0xFFFFU);
  }

  if ((y > (int8_t)7) || (y < (int8_t)-7))
  {
    pca9532_setLeds(0xC000U, 0xFFFFU);
  }
  if ((y > (int8_t)17) || (y < (int8_t)-17))
  {
    pca9532_setLeds(0xF000U, 0xFFFFU);
  }
  if ((y > (int8_t)25) || (y < (int8_t)-25))
  {
    pca9532_setLeds(0xFC00U, 0xFFFFU);
  }
  if ((y > (int8_t)32) || (y < (int8_t)-32))
  {
    pca9532_setLeds(0xFF00U, 0xFFFFU);
  }
}

int main(void)
{
  int8_t xoff = 0;
  int8_t yoff = 0;
  int8_t zoff = 0;
  int8_t x = 0; 
  int8_t y = 0; 
  int8_t z = 0; 

  init_i2c();
  init_ssp();
  init_adc();
  init_pwm();
  acc_init();
  light_enable();

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

  /* Jawne rzutowanie przy zmianie znaku - 10.4 */
  xoff = (int8_t)(-(int16_t)x);
  yoff = (int8_t)(-(int16_t)y);
  zoff = (int8_t)(-(int16_t)z);
  bee_init();

  int32_t cnt = 0;
  uint8_t is_time_warn = 0U;
  uint8_t is_temp_warn = 0U;
  
  while (1)
  {
    update_oled_theme_based_on_light();
    uint8_t state = joystick_read();
    acc_read(&x, &y, &z);
    
    /* Rozwiązanie dla 10.4 przy arytmetyce */
    x = (int8_t)((int16_t)x + (int16_t)xoff);
    y = (int8_t)((int16_t)y + (int16_t)yoff);
    z = (int8_t)((int16_t)z + (int16_t)zoff);

    update_leds(x, y);
    
    const uint8_t tilt_alert[] = "PRZECHYL!";
    const uint8_t time_alert[] = "CZAS!";
    const uint8_t temp_alert[] = "TEMP!";
    const uint8_t reset[] = "";

    uint8_t is_tilt_warn;
    if ((x > (int8_t)17) || (x < (int8_t)-17) || (y > (int8_t)17) || (y < (int8_t)-17))
    {
      is_tilt_warn = 1U;
    }
    else
    {
      is_tilt_warn = 0U;
    }

    if (state != (uint8_t)0U)
    {
      rotate_motor(state);

      static int32_t prev_value = 69;

      if ((abs(curr_value - prev_value) > (int32_t)100) && (curr_value != (int32_t)0))
      {
        timer_reset();
      }

      if (curr_value == (int32_t)0)
      {
        timer_stop();
      }

      if (prev_value != curr_value)
      {
        update_oled_message();
        prev_value = curr_value;
      }
    }

    uint32_t calc_units = ((uint32_t)ticks % 10U) + 48U;
    uint8_t units = (uint8_t)calc_units;
    
    uint32_t calc_tens_ticks = (((uint32_t)ticks / 10U) % 10U) + 48U;
    uint8_t tens_ticks = (uint8_t)calc_tens_ticks;
    
    uint8_t value[] = { tens_ticks, units, '\0' };

    oled_buffer_put(2U, value);

    if (ticks >= (uint8_t)50U)
    {
      curr_value = (int32_t)0;
      my_set_pwm_value((int32_t)1, (int32_t)0);
      my_set_pwm_value((int32_t)2, (int32_t)0);
      timer_stop();
    }
    else if (ticks >= (uint8_t)30U)
    {
      is_time_warn = 1U;
    }
    else if (ticks == (uint8_t)0U) 
    {
      is_time_warn = 0U;
    } 
    else 
    {
      /* Puste else */
    }

    if ((cnt % (int32_t)100) == (int32_t)0)
    {
      int32_t t_val = temp_read();
      
      int32_t t_int_calc = t_val / (int32_t)10;
      int32_t t_dec_calc = t_val % (int32_t)10;
      uint8_t t_int = (uint8_t)t_int_calc;
      uint8_t t_dec = (uint8_t)t_dec_calc;
      
      char temp_str[] = { 'T', 'e', 'm', 'p', ':', ' ', '0', '0', '.', '0', 'C', '\0' };

      if (t_int >= (uint8_t)30U)
      {
        is_temp_warn = 1U;
      }
      else
      {
        is_temp_warn = 0U;
      }

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

      oled_buffer_put(4U, (const uint8_t*)temp_str);
      cnt = (int32_t)0;
    }

    if (is_tilt_warn != 0U) 
    {
      oled_buffer_put(3U, tilt_alert);
    } 
    else if (is_time_warn != 0U) 
    {
      oled_buffer_put(3U, time_alert);
    } 
    else if (is_temp_warn != 0U)
    {
      oled_buffer_put(3U, temp_alert);
    } 
    else 
    {
      oled_buffer_put(3U, reset);
    }

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