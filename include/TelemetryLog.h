#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// One fixed-size record per control tick. Packed because the ring is large and
// every field is already a narrow integer; the serializers do the unit work.
// Distances are millimetres, angles are tenths of a degree.
struct __attribute__((packed)) LogSample {
    uint32_t t;         // millis() at the sample
    int16_t rawL;
    int16_t rawR;
    int16_t rawB;
    int16_t fL;
    int16_t fR;
    int16_t fB;
    int16_t latL;
    int16_t latR;
    int16_t err;
    int16_t pidP;
    int16_t pidI;
    int16_t pidD;
    int16_t pidOut;
    int8_t sv;
    int16_t wl;
    int16_t wr;
    uint8_t dir;        // 0 stop, 1 reverse, 2 forward
    uint8_t pwm;
    uint8_t state;      // ParkingState value, 0..5
    uint8_t flags;      // bit0 leftValid, bit1 rightValid, bit2 backValid,
                        // bit3 proximityAlert, bit4 bothSides, bit5 pidActive
    uint8_t pf;         // pull-forward attempt counter
};

static_assert(sizeof(LogSample) == 40, "LogSample must stay a compact record");

// A fixed ring, so the logger never allocates and never blocks the control
// loop. Capacity is a power of two; the mask is cheaper than a modulo.
// The httpd task reads on core 0 while the control loop records on core 1, so
// every access goes through one spinlock. No lock is ever held over a socket
// write: copySince snapshots a whole batch and returns.
class TelemetryLog {
public:
    static constexpr uint32_t CAPACITY = 2048;
    static constexpr uint32_t MASK = CAPACITY - 1;

    void record(const LogSample& sample) {
        portENTER_CRITICAL(&mux);
        buffer[nextSeq & MASK] = sample;
        nextSeq++;
        if (count < CAPACITY) count++;
        portEXIT_CRITICAL(&mux);
    }

    uint32_t total() const {
        portENTER_CRITICAL(&mux);
        uint32_t v = nextSeq - 1;  // valid seqs are 1..total
        portEXIT_CRITICAL(&mux);
        return v;
    }

    uint32_t available() const {
        portENTER_CRITICAL(&mux);
        uint32_t v = count;
        portEXIT_CRITICAL(&mux);
        return v;
    }

    uint32_t firstSeq() const {
        portENTER_CRITICAL(&mux);
        uint32_t v = nextSeq - count;
        portEXIT_CRITICAL(&mux);
        return v;
    }

    bool get(uint32_t seq, LogSample& out) const {
        portENTER_CRITICAL(&mux);
        const bool ok = seq >= nextSeq - count && seq < nextSeq;
        if (ok) out = buffer[seq & MASK];
        portEXIT_CRITICAL(&mux);
        return ok;
    }

    // Copies up to max samples newer than "since" in one snapshot and also
    // returns the ring bounds from that same snapshot. since == 0 means "from
    // the oldest sample", because seq numbers start at 1.
    size_t copySince(uint32_t since, LogSample* out, size_t max,
                     uint32_t& first, uint32_t& total) const {
        portENTER_CRITICAL(&mux);
        total = nextSeq - 1;       // sample count, valid seqs 1..total
        first = total + 1 - count; // oldest seq still held
        uint32_t from = (since == 0) ? first : since + 1;
        if (from < first) from = first;
        if (from > total + 1) from = total + 1;
        size_t n = 0;
        while (from <= total && n < max) {
            out[n++] = buffer[from & MASK];
            from++;
        }
        portEXIT_CRITICAL(&mux);
        return n;
    }

    void clear() {
        portENTER_CRITICAL(&mux);
        nextSeq = 1;  // 0 stays the "nothing yet" sentinel
        count = 0;
        portEXIT_CRITICAL(&mux);
    }

    static const char* stateName(uint8_t state) {
        switch (state) {
            case 0: return "BOOT";
            case 1: return "IDLE_READY";
            case 2: return "REVERSE_PARK";
            case 3: return "PULL_FORWARD";
            case 4: return "PARKED_DONE";
            case 5: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }

    static const char* directionName(uint8_t dir) {
        switch (dir) {
            case 1: return "REV";
            case 2: return "FWD";
            default: return "STOP";
        }
    }

    // Thin serializers. They know nothing about HTTP, so a transport change
    // touches only the wrapper in WebServerManager.h.
    static size_t sampleToJson(char* dst, size_t cap, uint32_t seq,
                               const LogSample& s) {
        return snprintf(
            dst, cap,
            "{\"seq\":%lu,\"t\":%lu,\"st\":\"%s\","
            "\"rawL\":%d,\"rawR\":%d,\"rawB\":%d,"
            "\"fL\":%d,\"fR\":%d,\"fB\":%d,"
            "\"latL\":%d,\"latR\":%d,\"err\":%d,"
            "\"p\":%d,\"i\":%d,\"d\":%d,\"out\":%d,"
            "\"sv\":%d,\"wl\":%d,\"wr\":%d,"
            "\"dir\":\"%s\",\"pwm\":%u,\"valid\":%u,"
            "\"prox\":%u,\"both\":%u,\"pf\":%u}",
            (unsigned long)seq, (unsigned long)s.t, stateName(s.state),
            (int)s.rawL, (int)s.rawR, (int)s.rawB,
            (int)s.fL, (int)s.fR, (int)s.fB,
            (int)s.latL, (int)s.latR, (int)s.err,
            (int)s.pidP, (int)s.pidI, (int)s.pidD, (int)s.pidOut,
            (int)s.sv, (int)s.wl, (int)s.wr,
            directionName(s.dir), (unsigned)s.pwm,
            (unsigned)(s.flags & 0x07), (unsigned)((s.flags >> 3) & 0x01),
            (unsigned)((s.flags >> 4) & 0x01), (unsigned)s.pf);
    }

    static size_t writeCsvHeader(char* dst, size_t cap) {
        return snprintf(
            dst, cap,
            "seq,t_ms,state,raw_left_mm,raw_right_mm,raw_back_mm,"
            "filt_left_mm,filt_right_mm,filt_back_mm,"
            "lat_left_mm,lat_right_mm,error_mm,"
            "pid_p_d10,pid_i_d10,pid_d_d10,pid_out_d10,"
            "servo_deg,wheel_left_d10,wheel_right_d10,"
            "motor_dir,motor_pwm,valid_mask,proximity,pull_forward\n");
    }

    static size_t writeCsvRow(char* dst, size_t cap, uint32_t seq,
                              const LogSample& s) {
        return snprintf(
            dst, cap,
            "%lu,%lu,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
            "%s,%u,%u,%u,%u\n",
            (unsigned long)seq, (unsigned long)s.t, stateName(s.state),
            (int)s.rawL, (int)s.rawR, (int)s.rawB,
            (int)s.fL, (int)s.fR, (int)s.fB,
            (int)s.latL, (int)s.latR, (int)s.err,
            (int)s.pidP, (int)s.pidI, (int)s.pidD, (int)s.pidOut,
            (int)s.sv, (int)s.wl, (int)s.wr,
            directionName(s.dir), (unsigned)s.pwm,
            (unsigned)(s.flags & 0x07), (unsigned)((s.flags >> 3) & 0x01),
            (unsigned)s.pf);
    }

private:
    LogSample buffer[CAPACITY];
    uint32_t nextSeq = 1;  // 0 is never a sample, so "since=0" is unambiguous
    uint32_t count = 0;
    // Mutable: the const readers take the same lock as record().
    mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};
