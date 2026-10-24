#pragma once

// the one place the network settings live. put the ssid and the password of
// the network the car should join here.
//
// leaving the placeholders in place is not an error. the web task gives the
// join fifteen seconds and then brings up its own access point instead, so the
// page is reachable either way and a car with no network still reports.
//
// the fallback access point is named autoparking and the password is the one
// below. it is a short password on a lab network with no route off the car,
// which is the only reason one that short belongs in the source.

constexpr char WIFI_SSID[] = "your-network";
constexpr char WIFI_PASSWORD[] = "your-password";

constexpr char AP_SSID[] = "autoparking";
constexpr char AP_PASSWORD[] = "autoparking";

constexpr char HOSTNAME[] = "autoparking";
