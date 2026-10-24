#pragma once

// The one place the network settings live. Leaving the placeholders is fine, because after a 15 s join the web task brings up its own access point. Either way the page is reachable, on a lab network with no route off the car.

constexpr char WIFI_SSID[] = "your-network";
constexpr char WIFI_PASSWORD[] = "your-password";

constexpr char AP_SSID[] = "autoparking";
constexpr char AP_PASSWORD[] = "autoparking";

constexpr char HOSTNAME[] = "autoparking";
