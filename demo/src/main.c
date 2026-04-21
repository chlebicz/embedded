#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"

#include "joystick.h"
#include "oled.h"
#include "light.h"

static int curr_value = 0;

static void rotate_motor(uint8_t joyState)
{
	if ((joyState & JOYSTICK_CENTER) != 0) {
		curr_value = 0;
	}

    if ((joyState & JOYSTICK_RIGHT) != 0) {
    	curr_value++;
    }

    if ((joyState & JOYSTICK_LEFT) != 0) {
    	curr_value--;
    }

    if (curr_value > 1000) {
    	curr_value = 1000;
    } else if (curr_value < -1000) {
    	curr_value = -1000;
    }

    if (curr_value > 0) {
    	my_set_pwm_value(1, curr_value);
    	my_set_pwm_value(2, 0);
    } else if (curr_value < 0) {
    	my_set_pwm_value(1, 0);
    	my_set_pwm_value(2, -curr_value);
    } else {
    	my_set_pwm_value(1, 0);
    	my_set_pwm_value(2, 0);
    }
}

/// 1. Definicje i współdzielony stan (na górze pliku)
#define LUX_DARK_THRESHOLD   100
#define LUX_LIGHT_THRESHOLD  150

static oled_color_t oled_bg = OLED_COLOR_BLACK; // Aktualny kolor tła
static oled_color_t oled_fg = OLED_COLOR_WHITE; // Aktualny kolor tekstu
static uint8_t* oled_message = (uint8_t*)"";    // Aktualna treść z joysticka
static uint8_t current_theme = 0;               // 0 - ciemny, 1 - jasny

static void refresh_oled_display(void) {
    oled_clearScreen(oled_bg);

    if (oled_message[0] != '\0') {
        oled_putString(0, 0, oled_message, oled_fg, oled_bg);
    }
}

void update_oled_theme_based_on_light(void) {
    uint32_t lux = light_read();
    uint8_t changed = 0;

    if (lux > LUX_LIGHT_THRESHOLD && current_theme == 0) {
        oled_bg = OLED_COLOR_WHITE;
        oled_fg = OLED_COLOR_BLACK;
        current_theme = 1;
        changed = 1;
    }
    else if (lux < LUX_DARK_THRESHOLD && current_theme == 1) {
        oled_bg = OLED_COLOR_BLACK;
        oled_fg = OLED_COLOR_WHITE;
        current_theme = 0;
        changed = 1;
    }

    if (changed) {
        refresh_oled_display();
    }
}

static void update_oled_message(uint8_t joyState) {
    uint8_t* last_msg = oled_message;

    if ((joyState & JOYSTICK_CENTER) != 0) {
        oled_message = (uint8_t*)"";
    }

//    uint8_t units = (curr_value % 10) + '0';
//    uint8_t tens = ((curr_value / 10) % 10) + '0';
//    uint8_t hundreds = ((curr_value / 100) % 10) + '0';
//    uint8_t thousands = ((curr_value / 1000) % 10) + '0';

    if ((joyState & JOYSTICK_RIGHT) != 0) {
        oled_message = (uint8_t*)"Prawo";
    } else if ((joyState & JOYSTICK_LEFT) != 0) {
    	oled_message = (uint8_t*)"Lewo";
    }

    if (oled_message != last_msg) {
        refresh_oled_display();
    }
}

static void init_pwm(void) {
	LPC_PWM1->MR0 = 1000; // okres pwm
	LPC_PWM1->LER |= (1 << 0); // zatwierdzenie MR0

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

	// PIO1_10
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);
	LPC_PWM1->MR4 = 500; // 50%
	LPC_PWM1->LER |= (1 << 4); // zatwierdzenie MR4
	LPC_PWM1->PCR |= (1 << (9 + 3));
}

void my_set_pwm_value(int channel, int value) {
	if (channel == 1) {
		LPC_PWM1->MR1 = value;
		LPC_PWM1->LER |= (1 << 1);
	} else if (channel == 2) {
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
     * Master wybiera urządzenie slave, ustawiając jego linię CS w stan niski.
     * Generuje sygnał zegarowy na linii SCK.
     * Na każdą zmianę stanu zegara master wysyła bit na MOSI, a slave może równocześnie przesłać bit na MISO.
     * Po zakończeniu transmisji master ustawia CS(u nas P2.2) w stan wysoki, dezaktywując slave’a.
     */

    //Funcnum = 2 to druga alternatywna funkcja pinu
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
    //jak nie ma sygnału to jest stan wysoki
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
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}


int main (void) {
    uint8_t state = 0;


    init_i2c();
    init_ssp();
    init_pwm();
    light_enable();

    joystick_init();
    oled_init();

    oled_clearScreen(OLED_COLOR_BLACK);
    while (1) {

        /* ####### Motor and oled ###### */
        /* # */

        update_oled_theme_based_on_light();
		state = joystick_read();
		if (state != 0) {

			rotate_motor(state);
			update_oled_message(state);
		}

        /* # */
        /* ############################################# */

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