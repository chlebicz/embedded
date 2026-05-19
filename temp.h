int32_t t_val = temp_read();
uint8_t t_int = t_val / 10;
uint8_t t_dec = t_val % 10;
uint8_t temp_str[] = "Temp: 00.0 C";
temp_str[6] = ((t_int / 10) % 10) + '0'; 
temp_str[7] = (t_int % 10) + '0';        
temp_str[9] = t_dec + '0';               
if (temp_str[6] == '0') temp_str[6] = ' ';
oled_putString(6, 50, temp_str, oled_fg, oled_bg);