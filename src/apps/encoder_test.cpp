// encoder_test — standalone AS5600 magnetic-encoder read over USB serial (isolated bring-up app).
//
// Wiring (this bench): AS5600  SDA=GP16  SCL=GP17  (RP2040 I2C0)  DIR=GND  VCC=3V3  GND=GND.
// The AS5600 is a 12-bit absolute magnetic angle sensor at I2C addr 0x36.
//
// Build/flash:  ~/.pio-venv/bin/pio run -e encoder_test -t upload
// Then read the port at 115200; it streams one line ~5 Hz. Ctrl-C to stop reading (motor untouched).
//
// This app does NOT drive any ESC/motor — it only reads I2C. Reflash esc_tool to resume the tool.

#include <Arduino.h>
#include <Wire.h>

static const uint8_t AS5600_ADDR   = 0x36;
static const uint8_t REG_STATUS    = 0x0B;   // bit5 MD (magnet detected), bit4 ML (too weak), bit3 MH (too strong)
static const uint8_t REG_RAW_ANGLE = 0x0C;   // 0x0C/0x0D unscaled 0..4095
static const uint8_t REG_ANGLE     = 0x0E;   // 0x0E/0x0F scaled/filtered 0..4095
static const uint8_t REG_AGC       = 0x1A;   // automatic gain (magnet distance indicator)
static const uint8_t REG_MAGNITUDE = 0x1B;   // 0x1B/0x1C CORDIC magnitude

// Read n bytes starting at reg. Returns true on success.
static bool as5600_read(uint8_t reg, uint8_t* buf, uint8_t n) {
	Wire.beginTransmission(AS5600_ADDR);
	Wire.write(reg);
	if (Wire.endTransmission(false) != 0) return false;   // repeated-start
	uint8_t got = Wire.requestFrom((int)AS5600_ADDR, (int)n);
	if (got != n) return false;
	for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
	return true;
}
static int as5600_u12(uint8_t reg) {   // read a 12-bit big-endian value; -1 on I2C error
	uint8_t b[2];
	if (!as5600_read(reg, b, 2)) return -1;
	return ((b[0] & 0x0F) << 8) | b[1];
}

static bool present = false;

void setup() {
	Serial.begin(115200);
	Wire.setSDA(16);
	Wire.setSCL(17);
	Wire.begin();
	Wire.setClock(400000);
	delay(50);
	// probe: does 0x36 ACK?
	Wire.beginTransmission(AS5600_ADDR);
	present = (Wire.endTransmission() == 0);
}

void loop() {
	if (!present) {
		// re-probe in case it was plugged after boot
		Wire.beginTransmission(AS5600_ADDR);
		present = (Wire.endTransmission() == 0);
		if (!present) { Serial.println("enc| AS5600 NOT FOUND at 0x36 — check SDA=GP16 SCL=GP17, 3V3, GND, pull-ups"); delay(500); return; }
		Serial.println("enc| AS5600 detected at 0x36");
	}
	uint8_t st = 0; as5600_read(REG_STATUS, &st, 1);
	uint8_t agc = 0; as5600_read(REG_AGC, &agc, 1);
	int raw = as5600_u12(REG_RAW_ANGLE);
	int ang = as5600_u12(REG_ANGLE);
	int mag = as5600_u12(REG_MAGNITUDE);
	int md = (st >> 5) & 1, ml = (st >> 4) & 1, mh = (st >> 3) & 1;
	float deg = (raw >= 0) ? raw * 360.0f / 4096.0f : -1.0f;
	// enc| raw=<0..4095> ang=<0..4095> deg=<0..360> md=<0/1> ml=<0/1> mh=<0/1> agc=<0..255> mag=<..>
	Serial.printf("enc| raw=%4d ang=%4d deg=%6.1f md=%d ml=%d mh=%d agc=%3d mag=%d\n",
	              raw, ang, deg, md, ml, mh, agc, mag);
	delay(200);
}

void setup1() {}
void loop1() {}
