/*
 * Name: led.h
 * Author: Martin Stankus
 *
 */

#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>

#define LED_ID8						8u
#define LED_ID7						7u
#define LED_ID6						6u
#define LED_ID5						5u
#define LED_ID4						4u
#define LED_ID3						3u
#define LED_ID2						2u
#define LED_ID1						1u

#define LED_ID_RED					4u
#define LED_ID_GREEN				2u
#define LED_ID_BLUE					1u

#define LED_RGB_PWM_PERIOD			2400u

void led_init(void);
void led_rgb_init(void);

void led_high_bgraph(uint8_t len);
void led_low_bgraph(uint8_t len);
void led_bgraph(uint8_t len);

void led_high_bindisp(uint8_t val);
void led_low_bindisp(uint8_t val);
void led_bindisp(uint8_t val);

void led_on(uint8_t id);
void led_off(uint8_t id);
void led_toggle(uint8_t id);

void led_rgb_set(uint8_t id, uint32_t duty);

#endif /* _LED_H_ */
