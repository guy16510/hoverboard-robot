/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <cstdint>

namespace gs::balance {

struct Vector3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct ImuSample {
  Vector3 accel_g;
  Vector3 gyro_dps;
  float temperature_c = 0.0f;
  uint64_t timestamp_us = 0u;
  bool accelerometer_plausible = false;
};

struct PitchEstimate {
  float raw_pitch_deg = 0.0f;
  float filtered_pitch_deg = 0.0f;
  float pitch_rate_dps = 0.0f;
  float dt_seconds = 0.0f;
  bool valid = false;
};

struct MotorCommand {
  float left = 0.0f;
  float right = 0.0f;
  bool enabled = false;
  uint64_t created_us = 0u;
};

class IImu {
public:
  virtual ~IImu() = default;
  virtual bool begin() = 0;
  virtual bool sample(ImuSample &sample) = 0;
};

class IAttitudeEstimator {
public:
  virtual ~IAttitudeEstimator() = default;
  virtual void reset(const ImuSample &sample) = 0;
  virtual bool update(const ImuSample &sample) = 0;
  virtual const PitchEstimate &estimate() const = 0;
};

struct BalanceInput;
struct BalanceOutput;

class IBalanceController {
public:
  virtual ~IBalanceController() = default;
  virtual BalanceOutput update(const BalanceInput &input) = 0;
  virtual void reset(float applied_left, float applied_right) = 0;
  virtual void clear() = 0;
};

class IMotorCommandSink {
public:
  virtual ~IMotorCommandSink() = default;
  virtual void write(const MotorCommand &command) = 0;
};

struct ControlRequest;

class ICommandSource {
public:
  virtual ~ICommandSource() = default;
  virtual bool latest(ControlRequest &request) const = 0;
};

class ITelemetrySink {
public:
  virtual ~ITelemetrySink() = default;
  virtual void publish(const PitchEstimate &estimate,
                       const BalanceOutput &output) = 0;
};

class IClock {
public:
  virtual ~IClock() = default;
  virtual uint64_t nowMicros() const = 0;
};

} // namespace gs::balance
