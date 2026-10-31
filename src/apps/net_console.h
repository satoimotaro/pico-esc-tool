// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// net_console — the CLI over the home Wi-Fi as well as USB. The Pico joins the LAN as a station
// (credentials in wifi_secrets.h, gitignored) and listens on ESC_TCP_PORT; one TCP client at a time,
// a new one replaces the old. Input is taken from USB and TCP alike; output goes to both. So the
// host tools work unchanged over the network:
//     python am32tool.py --port socket://<ip>:2323 read 0
// The join never blocks the loop (beginNoBlock), and a dropped link is retried every RETRY_MS.
// Unlike the SETUP-mode access point, the radio stays on in DRIVE: that is the point of it.
#pragma once
#include <Arduino.h>
#include <WiFi.h>

class NetConsole : public Stream {
public:
	explicit NetConsole(uint16_t port) : server_(port), port_(port) {}

	// usb = the stream the CLI had before (USB CDC); it keeps working alongside TCP.
	void begin(Stream& usb, const char* ssid, const char* pass, const char* hostname) {
		usb_ = &usb; ssid_ = ssid; pass_ = pass;
		WiFi.setHostname(hostname);
		WiFi.mode(WIFI_STA);
		join();
	}

	void poll() {
		bool up = WiFi.status() == WL_CONNECTED;
		if (up && !listening_) { server_.begin(); server_.setNoDelay(true); listening_ = true; }
		if (!up) {
			if (listening_) { client_.stop(); server_.stop(); listening_ = false; }
			if (millis() - joinMs_ > RETRY_MS) join();
			return;
		}
		if (server_.hasClient()) {
			if (client_) client_.stop();
			client_ = server_.accept();
			client_.setNoDelay(true);
		}
	}

	bool        connected() { return WiFi.status() == WL_CONNECTED; }
	bool        hasClient() { return client_.connected(); }
	IPAddress   ip()        { return WiFi.localIP(); }
	uint16_t    port() const { return port_; }
	const char* ssid() const { return ssid_; }

	// Stream: USB first, then TCP.
	int available() override { return usb_->available() + (client_ ? client_.available() : 0); }
	int read() override {
		if (usb_->available()) return usb_->read();
		return client_ ? client_.read() : -1;
	}
	int peek() override {
		if (usb_->available()) return usb_->peek();
		return client_ ? client_.peek() : -1;
	}
	size_t write(uint8_t b) override { return write(&b, 1); }
	size_t write(const uint8_t* buf, size_t n) override {
		usb_->write(buf, n);
		if (client_.connected()) client_.write(buf, n);
		return n;
	}
	void flush() override { usb_->flush(); if (client_.connected()) client_.flush(); }

private:
	static const uint32_t RETRY_MS = 15000;
	void join() { joinMs_ = millis(); WiFi.beginNoBlock(ssid_, pass_); }

	Stream*     usb_ = &Serial;
	WiFiServer  server_;
	WiFiClient  client_;
	uint16_t    port_;
	const char* ssid_ = "";
	const char* pass_ = "";
	bool        listening_ = false;
	uint32_t    joinMs_ = 0;
};
