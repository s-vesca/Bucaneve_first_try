#ifndef LED_HANDLER_H
#define LED_HANDLER_H
#include <stdint.h>

uint8_t led_handler_init(uint32_t _freq_khz);
void blink_led();

#endif //LED_HANDLER_H