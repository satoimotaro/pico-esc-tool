// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Host stand-in for the Arduino-Pico core, for the escs simulation (test/sim). Time is the
// simulation's clock: every delay advances it, and so does every millis()/micros() call (by 1 us),
// so a busy-wait on millis() also moves on. GPIO goes to the simulated signal lines.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <string>

typedef unsigned int uint;
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define F(x) x

namespace sim {
uint64_t nowUs();
void     advance(uint32_t us);
void     gpioMode(uint8_t pin, int mode);
void     gpioWrite(uint8_t pin, int level);
int      gpioRead(uint8_t pin);
}
inline unsigned long millis() { sim::advance(1); return (unsigned long)(sim::nowUs() / 1000); }
inline unsigned long micros() { sim::advance(1); return (unsigned long)sim::nowUs(); }
inline void delay(unsigned long ms) { sim::advance((uint32_t)ms * 1000); }
inline void delayMicroseconds(unsigned int us) { sim::advance(us); }
inline void pinMode(uint8_t p, int m) { sim::gpioMode(p, m); }
inline void digitalWrite(uint8_t p, int v) { sim::gpioWrite(p, v); }
inline int  digitalRead(uint8_t p) { return sim::gpioRead(p); }

class Print {
public:
	virtual ~Print() {}
	virtual size_t write(uint8_t b) = 0;
	virtual size_t write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
	size_t write(const char* s) { return write((const uint8_t*)s, strlen(s)); }
	size_t print(const char* s) { return write(s); }
	size_t println(const char* s = "") { size_t n = write(s); return n + write("\n"); }
	size_t printf(const char* f, ...) __attribute__((format(printf, 2, 3))) {
		char b[512]; va_list a; va_start(a, f); int n = vsnprintf(b, sizeof b, f, a); va_end(a);
		return write((const uint8_t*)b, n < 0 ? 0 : (size_t)(n < (int)sizeof b ? n : (int)sizeof b - 1));
	}
	virtual void flush() {}
};
class Stream : public Print {
public:
	virtual int available() { return 0; }
	virtual int read() { return -1; }
};
// What the firmware prints (console::out()): kept for the test to look at.
class CaptureStream : public Stream {
public:
	std::string out;
	size_t write(uint8_t b) override { out.push_back((char)b); return 1; }
	using Print::write;
};
extern CaptureStream Serial;
