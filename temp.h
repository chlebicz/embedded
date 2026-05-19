volatile uint32_t msTicks = 0; // Zegar systemowy dla termometru

void SysTick_Handler(void) {
    msTicks++;
}

static uint32_t getTicks(void) {
    return msTicks;
}

init_i2c();
  init_ssp();
  init_pwm();
  acc_init();
  light_enable();

  joystick_init();
  oled_init();
  
  // --- KONFIGURACJA DLA TERMOMETRU ---
  temp_init(&getTicks); // Przekazujemy wskaźnik do getTicks

  if (SysTick_Config(SystemCoreClock / 1000)) {
      while (1);  // Przechwycenie błędu jeśli zegar systemowy zawiedzie
  }
  // ------------------------------------

  acc_read(&x, &y, &z);
  xoff = 0-x;
  yoff = 0-y;
  zoff = 0-z;

  oled_clearScreen(OLED_COLOR_BLACK);
  while (1) 
  {
      // ... Twoja pętla główna

// --- ODCZYT TERMOMETRU ---
    int32_t t_val = temp_read();
    uint8_t t_int = t_val / 10;
    uint8_t t_dec = t_val % 10;
    
    uint8_t temp_str[] = "Temp: 00.0 C";
    temp_str[6] = ((t_int / 10) % 10) + '0'; 
    temp_str[7] = (t_int % 10) + '0';        
    temp_str[9] = t_dec + '0';               
    
    if (temp_str[6] == '0') {
        temp_str[6] = ' '; // ładniejsze formatowanie
    }
    
    // Wyświetlamy na pozycji Y=50, żeby nie zasłonić alertów i ticksów
    oled_putString(6, 50, temp_str, oled_fg, oled_bg);
    // -------------------------

    Timer0_Wait(1);
  } // koniec pętli while(1)