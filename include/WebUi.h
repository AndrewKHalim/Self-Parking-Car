#pragma once

#include <Arduino.h>

// Defined in src/dashboard_html.cpp. The server only needs the pointer, which
// stays in flash; no copy of the page is made at runtime.
extern const char DASHBOARD_HTML[] PROGMEM;
