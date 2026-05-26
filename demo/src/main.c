#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"

#include "joystick.h"
#include "oled.h"
#include "light.h"
#include "acc.h"
#include "led7seg.h"

#include "temp.h"
#include "fatbee.h"

#define LUX_DARK_THRESHOLD   100
#define LUX_LIGHT_THRESHOLD  150

enum Theme
{
  DARK,
  LIGHT
};
static enum Theme curr_theme = DARK;
static enum Theme prev_theme = DARK;

static int curr_value = 0;                      // Moc silnika

static void init_adc(void)
{
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init ADC pin connect
	 * AD0.0 on P0.23
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 23;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC :
	 * 	Frequency at 0.2Mhz
	 *  ADC channel 0, no Interrupt
	 */
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,ADC_CHANNEL_0,DISABLE);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_0,ENABLE);

}

volatile uint8_t ticks = 0;

void timer_reset(void)
{
  ticks = 0;

  TIM_TIMERCFG_Type TIM_ConfigStruct;
  TIM_MATCHCFG_Type TIM_MatchConfigStruct;

  // Konfiguracja timera na odliczanie w milisekundach (1000 us = 1 ms)
  TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
  TIM_ConfigStruct.PrescaleValue = 1000;

  // Konfiguracja rejestru dopasowania (Match 0) na 1000 ms
  TIM_MatchConfigStruct.MatchChannel = 0;
  TIM_MatchConfigStruct.IntOnMatch = ENABLE; // Wywołaj przerwanie, gdy doliczy do 1000
  TIM_MatchConfigStruct.ResetOnMatch = ENABLE; // Zresetuj licznik po osiągnięciu wartości
  TIM_MatchConfigStruct.StopOnMatch = DISABLE; // Zatrzymaj timer po 1 sekundzie (uruchomimy go znowu ręcznie)
  TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
  TIM_MatchConfigStruct.MatchValue = 1000;

  // Inicjalizacja
  TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TIM_ConfigStruct);
  TIM_ConfigMatch(LPC_TIM1, &TIM_MatchConfigStruct);

  // Włączenie przerwań dla Timera 1 w kontrolerze NVIC
  NVIC_SetPriority(TIMER1_IRQn, 10);
  NVIC_EnableIRQ(TIMER1_IRQn);
}

void timer_stop(void)
{
  NVIC_DisableIRQ(TIMER1_IRQn);
  ticks = 0;
}

const int SAMPLE_RATE = 8000; // hz

static void delay_us(uint32_t us)
{
  for (volatile uint32_t i = 0; i < (us * 30); i++) {}
}

static void decrease_amplifier_volume(int levels)
{
  // Ustaw pin kierunku głośności (UP/DN) na NISKI (0 = Ściszanie)
  GPIO_ClearValue(0, 1<<28);

  for (int i = 0; i < levels; i++)
  {
    // Impuls zegara (Wysoki -> Niski)
    GPIO_SetValue(0, 1<<27);   // CLK High
    delay_us(10);              // Krótka przerwa
    GPIO_ClearValue(0, 1<<27); // CLK Low
    delay_us(10);              // Krótka przerwa
  }
}

static void increase_amplifier_volume(int levels)
{
  // Ustaw pin kierunku głośności (UP/DN) na WYSOKI (1 = Zgłaśnianie)
  GPIO_SetValue(0, 1<<28);

  for (int i = 0; i < levels; i++)
  {
    // Impuls zegara (Wysoki -> Niski)
    GPIO_SetValue(0, 1<<27);   // CLK High
    delay_us(10);              // Krótka przerwa
    GPIO_ClearValue(0, 1<<27); // CLK Low
    delay_us(10);              // Krótka przerwa
  }
}

static uint8_t curr_volume = 16;
static uint8_t prev_volume = 16;

static void update_volume(void)
{
  ADC_StartCmd(LPC_ADC, ADC_START_NOW);
  // Wait for the conversion to complete
  while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)));
  uint16_t adc_value = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0); // 0 to 4096

  curr_volume = adc_value >> 8; // / 256
  if (curr_volume > prev_volume)
  {
    increase_amplifier_volume(curr_volume - prev_volume);
    prev_volume = curr_volume;
  }
  else if (curr_volume < prev_volume)
  {
    decrease_amplifier_volume(prev_volume - curr_volume);
    prev_volume = curr_volume;
  }
}

static void bee_init(void)
{
  // 1. OBUDŹ WZMACNIACZ LM4811
  // Ustawienie pinów sterujących wzmacniaczem jako wyjścia
  GPIO_SetDir(0, 1<<27, 1);
  GPIO_SetDir(0, 1<<28, 1);
  GPIO_SetDir(2, 1<<13, 1);
  GPIO_SetDir(0, 1<<26, 1);

  // Stan niski na pinie 2.13 wyłącza tryb "shutdown" wzmacniacza
  GPIO_ClearValue(0, 1<<27); // LM4811-clk
  GPIO_ClearValue(0, 1<<28); // LM4811-up/dn
  GPIO_ClearValue(2, 1<<13); // LM4811-shutdn

  increase_amplifier_volume(16); // max

  // 2. SKONFIGURUJ PIN DAC
  PINSEL_CFG_Type PinCfgDAC;
  PinCfgDAC.Funcnum = 2;   // Func 2 to AOUT (DAC)
  PinCfgDAC.OpenDrain = 0;
  PinCfgDAC.Pinmode = 0;
  PinCfgDAC.Portnum = 0;
  PinCfgDAC.Pinnum = 26;
  PINSEL_ConfigPin(&PinCfgDAC);

  // Inicjalizacja peryferium DAC
  DAC_Init(LPC_DAC);

  // 3. SKONFIGURUJ TIMER 2
  TIM_TIMERCFG_Type TIM_ConfigStruct;
  TIM_MATCHCFG_Type TIM_MatchConfigStruct;

  LPC_SC->PCONP |= (1 << 22); // Zasilanie Timera 2

  TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
  TIM_ConfigStruct.PrescaleValue = 1;

  TIM_MatchConfigStruct.MatchChannel = 0;
  TIM_MatchConfigStruct.IntOnMatch = ENABLE;
  TIM_MatchConfigStruct.ResetOnMatch = ENABLE;
  TIM_MatchConfigStruct.StopOnMatch = DISABLE;
  TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

  // Czas trwania jednej próbki w mikrosekundach
  TIM_MatchConfigStruct.MatchValue = 1000000 / SAMPLE_RATE;

  TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TIM_ConfigStruct);
  TIM_ConfigMatch(LPC_TIM2, &TIM_MatchConfigStruct);

  NVIC_SetPriority(TIMER2_IRQn, 10);
  NVIC_EnableIRQ(TIMER2_IRQn);

  TIM_Cmd(LPC_TIM2, ENABLE);
}

volatile uint32_t msTicks = 0; // Zegar systemowy dla termometru

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

#define FATBEE_SIZE (sizeof(fatbee) / sizeof(fatbee[0]))

#define AUDIO_START_OFFSET 44

volatile uint32_t current_sample_index = AUDIO_START_OFFSET;

void TIMER2_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR0_INT) == SET)
    {
        // Wyczyść flagę przerwania
        TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);

        // Pobierz 8-bitową próbkę (0..255)
        uint32_t sample = fatbee[current_sample_index];

        // Skalowanie głośności:
        // volume: 0..4096
        // sample DAC: 10-bit (0..1023)
        uint32_t dac_value = sample << 2U;

        // Wyślij do DAC
        DAC_UpdateValue(LPC_DAC, dac_value);

        // Następna próbka
        current_sample_index++;

        // Zapętlenie audio
        if (current_sample_index >= FATBEE_SIZE)
        {
            current_sample_index = AUDIO_START_OFFSET;
        }
    }
}

static void rotate_motor(uint8_t joyState)
{
  if ((joyState & JOYSTICK_CENTER) != 0)
  {
    curr_value = 0;
    TIM_Cmd(LPC_TIM1, ENABLE);
  }

  if (curr_value < 500 && curr_value > 0)
  {
    curr_value = 500;
  }

  if (curr_value > -500 && curr_value < 0)
  {
    curr_value = -500;
  }

  if ((joyState & JOYSTICK_RIGHT) != 0)
  {
    curr_value+=10;
  }

  if ((joyState & JOYSTICK_LEFT) != 0)
  {
    curr_value-=10;
  }

  if (curr_value > 1000)
  {
    curr_value = 1000;
  }
  else if (curr_value < -1000)
  {
    curr_value = -1000;
  }

  if (curr_value > 0)
  {
    my_set_pwm_value(1, curr_value); // Pin P2.0 dostaje sygnał PWM (pulsujące napięcie)
    my_set_pwm_value(2, 0);          // Pin P2.3 jest zwarty do masy (GND)
  }
  else if (curr_value < 0)
  {
    my_set_pwm_value(1, 0);           // Pin P2.0 jest zwarty do masy
    my_set_pwm_value(2, -curr_value); // Pin P2.3 dostaje sygnał PWM
  }
  else
  {
    my_set_pwm_value(1, 0);
    my_set_pwm_value(2, 0);
  }
}

void update_oled_theme_based_on_light(void)
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
}

static int abs_val(int old_val)
{
  if (old_val < 0)
  {
    return -old_val;
  }
  return old_val;
}

#define LINE_COUNT 5
#define LINE_LENGTH 12

uint8_t first_draw = TRUE;

uint8_t oled_buffer[LINE_COUNT][LINE_LENGTH + 1] = {
  "            ",
  "            ",
  "            ",
  "            ",
  "            "
};

uint8_t prev_oled_buffer[LINE_COUNT][LINE_LENGTH + 1] = {
  "            ",
  "            ",
  "            ",
  "            ",
  "            "
};

void oled_buffer_put(uint8_t line, const uint8_t* data)
{
  if (line >= LINE_COUNT || data == NULL)
  {
    return;
  }

  for (int i = 0; i < LINE_LENGTH; i++)
  {
    if (*data != '\0')
    {
      oled_buffer[line][i] = *data;
      data++;
    }
    else
    {
      oled_buffer[line][i] = ' ';
    }
  }
}

#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8

void update_oled_with_buffer(void)
{
  oled_color_t oled_fg, oled_bg;
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

  if (curr_theme != prev_theme || first_draw)
  {
    oled_clearScreen(oled_bg);
  }

  for (int line = 0; line < LINE_COUNT; ++line)
  {
    for (int i = 0; i < LINE_LENGTH; ++i)
    {
      if (oled_buffer[line][i] != prev_oled_buffer[line][i] || curr_theme != prev_theme || first_draw)
      {
        oled_putChar(
          i * (CHAR_WIDTH + 2),
          line * (CHAR_HEIGHT + 2),
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

void update_oled_message()
{
  uint8_t* state = (uint8_t*)"";           // Stoi/Wciaganie/Opuszczanie
  uint8_t* power = (uint8_t*)"";           // wartosc mocy

  if (curr_value == 0)
  {
    state = (uint8_t*)"Stoi";
  }
  else if (curr_value > 0)
  {
    state = (uint8_t*)"Wciaganie";
  }
  else if (curr_value < 0)
  {
    state = (uint8_t*)"Opuszczanie";
  }

  uint8_t tens = ((abs_val(curr_value) / 10) % 10) + '0';
  uint8_t hundreds = ((abs_val(curr_value) / 100) % 10) + '0';
  uint8_t thousands = ((abs_val(curr_value) / 1000) % 10) + '0';

  if (curr_value == 1000 || curr_value == -1000)
  {
    thousands = '5';
  }

  uint8_t secondLine[] = "100% mocy";
  secondLine[0] = thousands;
  secondLine[1] = hundreds;
  secondLine[2] = tens;
  power = secondLine;

  if (state[0] != '\0')
  {
    oled_buffer_put(0, state);
  }

  if (power[0] != '\0')
  {
    oled_buffer_put(1, power);
  }
}

static void init_pwm(void)
{
  LPC_PWM1->MR0 = 1000; // okres pwm
  LPC_PWM1->LER |= (1 << 0); // zatwierdzenie MR0
  // rejestry MR są 32-bitowe

  // bit 0 - wlaczenie glownego licznika i prescalera
  // bit 3 - pwm enable
  LPC_PWM1->TCR |= (1 << 0) | (1 << 3);

  PINSEL_CFG_Type PinCfg;
  PinCfg.Portnum = 2;
  PinCfg.Pinmode = 0;
  PinCfg.Funcnum = 1;
  PinCfg.OpenDrain = 0;

  // PIO1_9
  PinCfg.Pinnum = 0;
  PINSEL_ConfigPin(&PinCfg);
  LPC_PWM1->MR1 = 500; // 50%
  LPC_PWM1->LER |= (1 << 1); // zatwierdzenie rejestru MR1
  LPC_PWM1->PCR |= (1 << (9 + 0)); // aktywacja wyjscia sygnalu dla kanalu 2

  // PIO2_3
  PinCfg.Pinnum = 3;
  PINSEL_ConfigPin(&PinCfg);
  LPC_PWM1->MR4 = 500; // 50%
  LPC_PWM1->LER |= (1 << 4); // zatwierdzenie MR4
  LPC_PWM1->PCR |= (1 << (9 + 3));
}

void my_set_pwm_value(int channel, int value)
{
  if (channel == 1)
  {
    LPC_PWM1->MR1 = value;
    LPC_PWM1->LER |= (1 << 1);
  }
  else if (channel == 2)
  {
    LPC_PWM1->MR4 = value;
    LPC_PWM1->LER |= (1 << 4);
  }
}

static void init_ssp(void)
{
  SSP_CFG_Type SSP_ConfigStruct;
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = 2;
  PinCfg.OpenDrain = 0;
  PinCfg.Pinmode = 0;
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 7;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum = 8;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum = 9;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Funcnum = 0;
  PinCfg.Portnum = 2;
  PinCfg.Pinnum = 2;
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
  PinCfg.Funcnum = 2;
  PinCfg.Pinnum = 10; //GPIO_26-SDA P0.10 - do przesyłania danych
  PinCfg.Portnum = 0;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Pinnum = 11; //GPIO_27-SCL P0.11 - do synchronizacji
  PINSEL_ConfigPin(&PinCfg);

  // Initialize I2C peripheral
  // 100kHZ - taktowanie zegara SCL
  I2C_Init(LPC_I2C2, 100000);

  /* Enable I2C1 operation */
  I2C_Cmd(LPC_I2C2, ENABLE);
}

int main(void)
{
  uint8_t state = 0;
  int8_t xoff = 0;
  int8_t yoff = 0;
  int8_t zoff = 0;
  int8_t x = 0; //(lewo – prawo)
  int8_t y = 0; //(przód – tył)
  int8_t z = 0; //(góra – dół)

  init_i2c();
  init_ssp();
  init_adc();
  init_pwm();
  acc_init();
  light_enable();

  joystick_init();
  oled_init();
  temp_init(&getTicks);
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    while (1);  // Przechwycenie błędu jeśli zegar systemowy zawiedzie
  }
  acc_read(&x, &y, &z);

  xoff = 0 - x;
  yoff = 0 - y;
  zoff = 0 - z;
  bee_init();

  int cnt = 0;
  while (1)
  {
    update_oled_theme_based_on_light();
    state = joystick_read();
    acc_read(&x, &y, &z);
    x = x + xoff;
    y = y + yoff;
    z = z + zoff;

    uint16_t ledOn = 0xffff;
    pca9532_setLeds(0x0000, 0xffff);

    if (x > 7 || x < -7)
      pca9532_setLeds(0x003, 0xffff);
    if (x > 17 || x < -17)
      pca9532_setLeds(0x000F, 0xffff);
    if (x > 25 || x < -25)
      pca9532_setLeds(0x003f, 0xffff);
    if (x > 32 || x < -32)
      pca9532_setLeds(0x00ff, 0xffff);

    if (y > 7 || y < -7)
      pca9532_setLeds(0xC000, 0xffff);
    if (y > 17 || y < -17)
      pca9532_setLeds(0xF000, 0xffff);
    if (y > 25 || y < -25)
      pca9532_setLeds(0xFC00, 0xffff);
    if (y > 32 || y < -32)
      pca9532_setLeds(0xFF00, 0xffff);

    static int is_achtung = 0;

    uint8_t alert[] = "ACHTUNG";
    uint8_t reset[] = "";

    // Line 3 (Y=40) mapped for warnings
    if (x > 17 || x < -17 || y > 17 || y < -17)
    {
      if (!is_achtung)
      {
        oled_buffer_put(3, alert);
        is_achtung = 1;
      }
    }
    else
    {
      if (is_achtung)
      {
        is_achtung = 0;
        oled_buffer_put(3, reset);
      }
    }

    if (state != 0)
    {
      rotate_motor(state);

      static int prev_value = 69;

      if (abs_val(curr_value - prev_value) > 100 && curr_value != 0)
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

    uint8_t units = (ticks % 10) + '0';
    uint8_t tens = ((ticks / 10) % 10) + '0';
    uint8_t value[] = { tens, units, '\0' };

    // Line 2 (Y=30) mapped for timer
    oled_buffer_put(2, value);

    if (ticks >= 50)
    {
      curr_value = 0;
      my_set_pwm_value(1, 0);
      my_set_pwm_value(2, 0);
      timer_stop();
    }
    else if (ticks >= 30)
    {
      oled_buffer_put(3, alert); // Uses Line 3
    }
    else if (ticks == 0 && !is_achtung)
    {
      oled_buffer_put(3, reset);
    }

    if (cnt % 100 == 0)
    {
      int32_t t_val = temp_read();
      uint8_t t_int = t_val / 10;
      uint8_t t_dec = t_val % 10;
      uint8_t temp_str[] = "Temp: 00.0 C";

      temp_str[6] = ((t_int / 10) % 10) + '0';
      temp_str[7] = (t_int % 10) + '0';
      temp_str[9] = t_dec + '0';
      if (temp_str[6] == '0')
      {
        temp_str[6] = ' ';
      }

      // Line 4 (Y=50) mapped for temperature
      oled_buffer_put(4, temp_str);
      cnt = 0;
    }

    // Flush to screen only when lines actually changed
    update_oled_with_buffer();

    update_volume();

    cnt++;
    Timer0_Wait(1);
  }
}

void check_failed(uint8_t *file, uint32_t line)
{
  while(1);
}
