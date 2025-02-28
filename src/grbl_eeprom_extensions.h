#include "grbl/nvs.h"

bool memcpy_to_eeprom(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum);
bool memcpy_from_eeprom(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum);
