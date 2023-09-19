
#include <stdint.h>
#include <stdbool.h>

void eeprom_close (void);
void set_eeprom_name (char *name);
uint8_t eeprom_get_char (uint32_t addr );
void eeprom_put_char (uint32_t addr, uint8_t new_value );