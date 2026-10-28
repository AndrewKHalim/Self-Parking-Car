#pragma once

// The one place the network settings live. Access point only: there is no
// station join and no fallback timer, so the page is reachable on any bench.
// The credentials are public on purpose; this AP carries no uplink.

constexpr bool WIFI_ENABLED = true;

constexpr char AP_SSID[] = "autoparking";
constexpr char AP_PASS[] = "autoparking";
