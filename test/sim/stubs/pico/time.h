// Host stand-in (test/sim): a repeating timer runs from the simulation clock, like an IRQ.
#pragma once
#include <cstdint>
struct repeating_timer;
typedef bool (*repeating_timer_callback_t)(repeating_timer*);
struct repeating_timer { repeating_timer_callback_t cb = nullptr; void* user_data = nullptr; uint32_t ms = 0; };
typedef repeating_timer repeating_timer_t;
namespace sim { bool addTimer(repeating_timer* t); void cancelTimer(repeating_timer* t); }
inline bool add_repeating_timer_ms(int32_t ms, repeating_timer_callback_t cb, void* user, repeating_timer* t) {
	t->cb = cb; t->user_data = user; t->ms = (uint32_t)(ms < 0 ? -ms : ms); return sim::addTimer(t);
}
inline bool cancel_repeating_timer(repeating_timer* t) { sim::cancelTimer(t); return true; }
