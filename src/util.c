#include "util.h"

/*
 * Hex to ASCII
 * 
 * Converts the given byte
 * to it's hexadecimal character
 * equivalent
 */
uint16_t hex2ascii (uint8_t x, bool caps) {
  uint16_t ascii;

  ascii =  (x >> 4) + ((x & 0xF0) > 0x90 ? ( caps ? 0x37 : 0x57 ) : 0x30) << 8;
  ascii |= (x & 0x0F) + ((x & 0x0F) > 0x09 ? ( caps ? 0x37 : 0x57 ) : 0x30);
  return ascii;
}
