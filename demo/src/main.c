#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"

#include "joystick.h"
#include "oled.h"
#include "light.h"

#define LUX_DARK_THRESHOLD   100
#define LUX_LIGHT_THRESHOLD  150

static oled_color_t oled_bg = OLED_COLOR_BLACK; // Aktualny kolor tła
static oled_color_t oled_fg = OLED_COLOR_WHITE; // Aktualny kolor tekstu
static uint8_t current_theme = 0;               // 0 - ciemny, 1 - jasny

static int curr_value = 0;                      // Moc silnika

static void rotate_motor(uint8_t joyState)
{
  if ((joyState & JOYSTICK_CENTER) != 0)
  {
    curr_value = 0;
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
  uint8_t changed = 0;

  if (lux > LUX_LIGHT_THRESHOLD && current_theme == 0)
  {
    oled_bg = OLED_COLOR_WHITE;
    oled_fg = OLED_COLOR_BLACK;
    current_theme = 1;
    changed = 1;
  }
  else if (lux < LUX_DARK_THRESHOLD && current_theme == 1)
  {
    oled_bg = OLED_COLOR_BLACK;
    oled_fg = OLED_COLOR_WHITE;
    current_theme = 0;
    changed = 1;
  }

  if (changed)
  {
    update_oled_message();
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

  if (curr_value == 1000)
  {
    thousands = '5';
  }

  uint8_t secondLine[] = "100% mocy";
  secondLine[0] = thousands;
  secondLine[1] = hundreds;
  secondLine[2] = tens;
  power = secondLine;

  oled_clearScreen(oled_bg);

  // 96x64 pixel rozdzielczość oled

  if (state[0] != '\0')
  {
    oled_putString(6, 10, state, oled_fg, oled_bg);
  }

  if (power[0] != '\0')
  {
    oled_putString(6, 20, power, oled_fg, oled_bg);
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

  /*
   * Initialize SPI(Serial Peripheral Interface) pin connect
   * P0.7 - SCK (Serial Clock) //sygnał zegarowy generowany przez mastera, synchronizuje przesył danych
   * P0.8 - MISO (Master Out, Slave In)
   * P0.9 - MOSI (Master In, Slave Out)
   * P2.2 - SSEL - used as GPIO (General-Purpose Input/Output) aktywuje wybrane urządzenie slave do komunikacji
   *
   * Jak działa komunikacja SPI?
   *
   * Master wybiera urządzenie slave, ustawiając jego linię CS w state niski.
   * Generuje sygnał zegarowy na linii SCK.
   * Na każdą zmianę stateu zegara master wysyła bit na MOSI, a slave może równocześnie przesłać bit na MISO.
   * Po zakończeniu transmisji master ustawia CS(u nas P2.2) w state wysoki, dezaktywując slave’a.
   */

  //Funcnum = 2 to druga alternatywna funkcja pinu
  PinCfg.Funcnum = 2;
  PinCfg.OpenDrain = 0;
  //jak nie ma sygnału to jest state wysoki
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

  init_i2c();
  init_ssp();
  init_pwm();
  light_enable();

  joystick_init();
  oled_init();

  oled_clearScreen(OLED_COLOR_BLACK);
  while (1)
  {
    update_oled_theme_based_on_light();
    state = joystick_read();
    if (state != 0)
    {
      rotate_motor(state);

      static int prev_value = 69;
      if (prev_value != curr_value)
      {
        update_oled_message();
        prev_value = curr_value;
      }
    }

    Timer0_Wait(1);
  }
}

void check_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while(1);
}