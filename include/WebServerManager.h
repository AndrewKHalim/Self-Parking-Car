#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include <stdlib.h>

#include "TelemetryLog.h"
#include "WebUi.h"
#include "wifi_config.h"

// The ring lives in main.cpp. The manager only reads it, so the control loop
// never waits on a socket.
extern TelemetryLog telemetry;

// main.cpp keeps calling these; the server owns its own FreeRTOS task now.
void webBegin();
void webPoll();

namespace web_detail {

// The page asks for at most 600 samples per poll; more would just be ignored.
constexpr size_t MAX_SAMPLES = 600;
// 16 samples is 640 B of stack. The httpd task gets 4 KB, so the whole batch
// (600 = 24 KB) can only ever be copied in pieces.
constexpr size_t COPY_BATCH = 16;
constexpr size_t SEND_CHUNK = 768;

static httpd_handle_t server = nullptr;
static bool wifiUp = false;

static void ipText(IPAddress ip, char* out, size_t cap) {
    snprintf(out, cap, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

// Same clamping as TelemetryLog::copySince, so a batch is labelled with the
// sequence numbers it actually holds after a wrap or a reset. total is the
// sample count, so total + 1 is the first seq that does not exist yet.
static uint32_t batchStart(uint32_t since, uint32_t first, uint32_t total) {
    uint32_t from = (since == 0) ? first : since + 1;
    if (from < first) from = first;
    if (from > total + 1) from = total + 1;
    return from;
}

// Streaming keeps the response off the heap. Every wrapper below is deliberately
// thin: the unit conversion and field names stay in TelemetryLog.h, so only
// this file changes if the transport does.
static esp_err_t sendSamplePage(httpd_req_t* req, uint32_t since) {
    LogSample batch[COPY_BATCH];
    uint32_t first = 0;
    uint32_t total = 0;
    size_t got = telemetry.copySince(since, batch, COPY_BATCH, first, total);

    httpd_resp_set_type(req, "application/json");

    char head[96];
    int headLen = snprintf(head, sizeof(head),
                           "{\"first\":%lu,\"total\":%lu,\"samples\":[",
                           (unsigned long)first, (unsigned long)total);
    esp_err_t err = httpd_resp_send_chunk(req, head, headLen);

    char chunk[SEND_CHUNK];
    size_t used = 0;
    size_t sent = 0;
    bool firstSample = true;
    uint32_t sinceArg = since;

    while (err == ESP_OK && got > 0 && sent < MAX_SAMPLES) {
        const uint32_t seq = batchStart(sinceArg, first, total);
        size_t take = got;
        if (take > MAX_SAMPLES - sent) take = MAX_SAMPLES - sent;

        for (size_t i = 0; i < take; i++) {
            char one[320];
            int oneLen = (int)TelemetryLog::sampleToJson(one, sizeof(one),
                                                         seq + i, batch[i]);
            if (oneLen <= 0 || (size_t)oneLen >= sizeof(one)) continue;

            const size_t need = (size_t)oneLen + (firstSample ? 0 : 1);
            if (used + need > sizeof(chunk)) {
                err = httpd_resp_send_chunk(req, chunk, used);
                used = 0;
                if (err != ESP_OK) break;
            }
            if (!firstSample) chunk[used++] = ',';
            memcpy(chunk + used, one, (size_t)oneLen);
            used += (size_t)oneLen;
            firstSample = false;
        }
        if (err != ESP_OK) break;

        sent += take;
        sinceArg = seq + (uint32_t)take - 1;
        if (sent < MAX_SAMPLES) {
            got = telemetry.copySince(sinceArg, batch, COPY_BATCH, first, total);
        }
    }
    if (used > 0) err = httpd_resp_send_chunk(req, chunk, used);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, "]}", 2);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);
    return err;
}

static esp_err_t sendCsv(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"parking-log.csv\"");

    char chunk[SEND_CHUNK];
    size_t used = TelemetryLog::writeCsvHeader(chunk, sizeof(chunk));
    if (used >= sizeof(chunk)) used = sizeof(chunk) - 1;

    LogSample batch[COPY_BATCH];
    uint32_t first = 0;
    uint32_t total = 0;
    uint32_t sinceArg = 0;
    esp_err_t err = ESP_OK;

    for (;;) {
        const size_t got = telemetry.copySince(sinceArg, batch, COPY_BATCH,
                                               first, total);
        if (got == 0) break;
        const uint32_t seq = batchStart(sinceArg, first, total);

        for (size_t i = 0; i < got; i++) {
            char row[256];
            int rowLen = (int)TelemetryLog::writeCsvRow(row, sizeof(row),
                                                        seq + i, batch[i]);
            if (rowLen <= 0 || (size_t)rowLen >= sizeof(row)) continue;

            if (used + (size_t)rowLen > sizeof(chunk)) {
                err = httpd_resp_send_chunk(req, chunk, used);
                used = 0;
                if (err != ESP_OK) return err;
            }
            memcpy(chunk + used, row, (size_t)rowLen);
            used += (size_t)rowLen;
        }
        sinceArg = seq + (uint32_t)got - 1;
    }
    if (used > 0) err = httpd_resp_send_chunk(req, chunk, used);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);
    return err;
}

static esp_err_t handleRoot(httpd_req_t* req) {
    // DASHBOARD_HTML is flash-resident, so this streams it without a heap copy.
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handleInfo(httpd_req_t* req) {
    char ip[16] = "";
    if (wifiUp) ipText(WiFi.softAPIP(), ip, sizeof(ip));

    uint8_t state = 0;
    const uint32_t total = telemetry.total();
    LogSample last;
    if (total >= 1 && telemetry.get(total, last)) state = last.state;

    char body[224];
    int len = snprintf(body, sizeof(body),
                       "{\"caps\":%lu,\"first\":%lu,\"total\":%lu,"
                       "\"uptime_ms\":%lu,\"ip\":\"%s\",\"state\":\"%s\","
                       "\"wifi\":\"%s\",\"build\":\"%s %s\"}",
                       (unsigned long)TelemetryLog::CAPACITY,
                       (unsigned long)telemetry.firstSeq(),
                       (unsigned long)total,
                       (unsigned long)millis(), ip,
                       TelemetryLog::stateName(state),
                       wifiUp ? "ap" : "off", __DATE__, __TIME__);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, len);
}

static esp_err_t handleSince(httpd_req_t* req) {
    uint32_t since = 0;
    char query[24];
    const size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen > 1 && qlen <= sizeof(query) &&
        httpd_req_get_url_query_str(req, query, qlen) == ESP_OK) {
        char value[12];
        if (httpd_query_key_value(query, "seq", value, sizeof(value)) == ESP_OK) {
            since = (uint32_t)strtoul(value, nullptr, 10);
        }
    }
    return sendSamplePage(req, since);
}

static esp_err_t handleCsv(httpd_req_t* req) { return sendCsv(req); }

static esp_err_t handleReset(httpd_req_t* req) {
    telemetry.clear();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"cleared\":true}", HTTPD_RESP_USE_STRLEN);
}

static bool addRoute(const char* uri, esp_err_t (*handler)(httpd_req_t*)) {
    httpd_uri_t route = {};
    route.uri = uri;
    route.method = HTTP_GET;
    route.handler = handler;
    return httpd_register_uri_handler(server, &route) == ESP_OK;
}

static bool startServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // Priority 1 on core 0: the httpd task never preempts the loopTask on
    // core 1, so a slow client can only stall the page, not the car.
    config.task_priority = 1;
    config.core_id = 0;
    // Handler locals run ~1.7 KB, on top of httpd's own frames.
    config.stack_size = 6144;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) != ESP_OK) return false;

    bool ok = addRoute("/", handleRoot) &&
              addRoute("/api/info", handleInfo) &&
              addRoute("/api/since", handleSince) &&
              addRoute("/log.csv", handleCsv) &&
              addRoute("/api/reset", handleReset);
    if (!ok) {
        httpd_stop(server);
        server = nullptr;
    }
    return ok;
}

}  // namespace web_detail

inline void webBegin() {
    using namespace web_detail;

    if (!WIFI_ENABLED) {
        Serial.println("web ui disabled in wifi_config.h");
        return;
    }

    // Access point only: no station join, so the page is always reachable.
    WiFi.mode(WIFI_AP);
    wifiUp = WiFi.softAP(AP_SSID, AP_PASS);
    if (!wifiUp) {
        Serial.println("softAP failed; dashboard unavailable");
        return;
    }
    if (!startServer()) {
        Serial.println("httpd start failed; dashboard unavailable");
        return;
    }

    IPAddress ip = WiFi.softAPIP();
    Serial.printf("dashboard: ssid %s pass %s http://%u.%u.%u.%u/\n",
                  AP_SSID, AP_PASS, ip[0], ip[1], ip[2], ip[3]);
}

inline void webPoll() {
    // Empty on purpose: the httpd task services the server, not loop().
}
