// Host stand-in (test/sim): servo PWM goes to the simulated line as a width.
#pragma once
#include <cstdint>
typedef unsigned int uint;
#define GPIO_FUNC_PWM 4
#define GPIO_FUNC_SIO 5
namespace sim { void pwmLevel(uint8_t pin, uint16_t us); void gpioFunc(uint8_t pin, int fn); }
inline uint pwm_gpio_to_slice_num(uint gpio) { return (gpio >> 1) & 7; }
inline void pwm_set_clkdiv(uint, float) {}
inline void pwm_set_wrap(uint, uint16_t) {}
inline void pwm_set_enabled(uint, bool) {}
inline void pwm_set_gpio_level(uint gpio, uint16_t level) { sim::pwmLevel((uint8_t)gpio, level); }
inline void gpio_set_function(uint gpio, int fn) { sim::gpioFunc((uint8_t)gpio, fn); }
