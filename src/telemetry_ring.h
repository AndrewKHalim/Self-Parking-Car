#pragma once

// One fixed record per control tick behind a spinlock, so the web side can read a run without ever making the control loop wait on a socket. 1024 records is 34 s at 30 Hz. The record layout is this tree's own, and the CSV row is serialized here so the route only has to slice it.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "supervisor.h"

// The host test has no FreeRTOS and runs one thread, so the lock compiles away. The firmware always takes it, since the control and web tasks run on different cores.
#ifndef TELEMETRY_RING_HOST_TEST
#include <Arduino.h>
#endif

// Distances are millimetres and angles milliradians, which is what the wire already carries. The sequence number starts at 1 so 0 can mean nothing recorded yet.
struct Sample {
  uint32_t t_ms = 0;
  int16_t x_mm = 0;
  int16_t psi_mrad = 0;
  int16_t rear_mm = 0;
  int16_t gL_mm = 0;
  int16_t gR_mm = 0;
  int16_t clearance_mm = 0;
  int16_t psi_cap_mrad = 0;
  int16_t servo_deg = 90;
  uint16_t strokes = 0;
  uint8_t phase = 0;
  uint8_t stroke = 0;
  uint8_t flags = 0;
  uint32_t seq = 0;
};

static_assert(sizeof(Sample) <= 48, "Sample must stay a compact record");

// The flag byte. A single sensor can be out of range while the pose still uses the last good pair, so each reading has its own bit.
constexpr uint8_t SAMPLE_VALID = 1 << 0;
constexpr uint8_t SAMPLE_CALIBRATED = 1 << 1;
constexpr uint8_t SAMPLE_REAR_FAULT = 1 << 2;
constexpr uint8_t SAMPLE_TRAVEL_FAULT = 1 << 3;
constexpr uint8_t SAMPLE_OUT_OF_SPEC = 1 << 4;
constexpr uint8_t SAMPLE_LEFT_VALID = 1 << 5;
constexpr uint8_t SAMPLE_RIGHT_VALID = 1 << 6;
constexpr uint8_t SAMPLE_REAR_VALID = 1 << 7;

// Every row is the same length, so the response filler can find row n from the request offset alone. A value that outgrew its field would shift every later row, so the widths cover the whole integer range behind them.
constexpr size_t CSV_ROW_LEN = 96;
constexpr char CSV_HEADER[] =
    "t_ms,x_mm,psi_mrad,rear_mm,gL_mm,gR_mm,clearance_mm,psi_cap_mrad,"
    "servo_deg,phase,stroke,strokes,flags,seq\n";
constexpr size_t CSV_HEADER_LEN = sizeof(CSV_HEADER) - 1;

// Millimetres as a narrow integer. A sensor reports 8.19 m with no target and 65.535 m on a timeout, so the value saturates and the flag bits say whether it was a reading at all.
inline int16_t sampleMm(float m) {
  const long v = lroundf(m * 1000.0f);
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

inline uint8_t sampleFlags(const Telemetry& t, float gL, float gR, float gRear) {
  uint8_t f = 0;
  if (t.valid) f |= SAMPLE_VALID;
  if (t.calibrated) f |= SAMPLE_CALIBRATED;
  if (t.rearFault) f |= SAMPLE_REAR_FAULT;
  if (t.travelFault) f |= SAMPLE_TRAVEL_FAULT;
  if (t.outOfSpec) f |= SAMPLE_OUT_OF_SPEC;
  if (geom::validRange(gL)) f |= SAMPLE_LEFT_VALID;
  if (geom::validRange(gR)) f |= SAMPLE_RIGHT_VALID;
  if (geom::validRange(gRear)) f |= SAMPLE_REAR_VALID;
  return f;
}

class TelemetryRing {
 public:
  static constexpr uint32_t CAPACITY = 1024;
  static constexpr uint32_t MASK = CAPACITY - 1;

  void record(const Telemetry& t) {
    Sample s;
    s.t_ms = (uint32_t)lroundf(t.t * 1000.0f);
    s.x_mm = sampleMm(t.x);
    s.psi_mrad = sampleMm(t.psi);
    s.rear_mm = sampleMm(t.rear);
    s.gL_mm = sampleMm(t.gL);
    s.gR_mm = sampleMm(t.gR);
    s.clearance_mm = sampleMm(t.clearance);
    s.psi_cap_mrad = sampleMm(t.psiCap);
    s.servo_deg = (int16_t)lroundf(t.servo);
    s.strokes = (uint16_t)t.strokes;
    s.phase = (uint8_t)t.phase;
    s.stroke = (uint8_t)t.stroke;
    s.flags = sampleFlags(t, t.gL, t.gR, t.rear);

    lock();
    s.seq = nextSeq;
    buffer[nextSeq & MASK] = s;
    nextSeq++;
    if (count < CAPACITY) count++;
    unlock();
  }

  // The newest sequence number, 0 before the first record.
  uint32_t total() const {
    lock();
    const uint32_t v = nextSeq - 1;
    unlock();
    return v;
  }

  // One record by sequence, false once the ring has wrapped past it.
  bool get(uint32_t seq, Sample& out) const {
    lock();
    const bool ok = seq >= nextSeq - count && seq < nextSeq;
    if (ok) out = buffer[seq & MASK];
    unlock();
    return ok;
  }

  // Copies up to max records newer than since and returns the ring bounds from that same snapshot. since 0 starts at the oldest record held.
  size_t copySince(uint32_t since, Sample* out, size_t max,
                   uint32_t& first, uint32_t& total) const {
    lock();
    total = nextSeq - 1;
    first = total + 1 - count;
    uint32_t from = (since == 0) ? first : since + 1;
    if (from < first) from = first;
    if (from > total + 1) from = total + 1;
    size_t n = 0;
    while (from <= total && n < max) {
      out[n++] = buffer[from & MASK];
      from++;
    }
    unlock();
    return n;
  }

  // The row the filler serves at a given sequence. Widths are fixed and zero padded, so seq and time keep their digits and signed values keep their sign.
  static void csvRow(uint32_t seq, const Sample& s, char* dst, size_t cap) {
    snprintf(dst, cap,
             "%010lu,%+06d,%+06d,%+06d,%+06d,%+06d,%+06d,%+06d,%+06d,"
             "%03u,%03u,%05u,%03u,%010lu\n",
             (unsigned long)s.t_ms, (int)s.x_mm, (int)s.psi_mrad,
             (int)s.rear_mm, (int)s.gL_mm, (int)s.gR_mm,
             (int)s.clearance_mm, (int)s.psi_cap_mrad, (int)s.servo_deg,
             (unsigned)s.phase, (unsigned)s.stroke, (unsigned)s.strokes,
             (unsigned)s.flags, (unsigned long)seq);
  }

 private:
  void lock() const {
#ifndef TELEMETRY_RING_HOST_TEST
    portENTER_CRITICAL(&mux);
#endif
  }

  void unlock() const {
#ifndef TELEMETRY_RING_HOST_TEST
    portEXIT_CRITICAL(&mux);
#endif
  }

  Sample buffer[CAPACITY];
  uint32_t nextSeq = 1;
  uint32_t count = 0;
  // Mutable so the const readers take the same lock as record.
#ifndef TELEMETRY_RING_HOST_TEST
  mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif
};
