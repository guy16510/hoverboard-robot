/* SPDX-License-Identifier: GPL-3.0-only */
#include "test_harness.h"

#include "balance_configuration.h"
#include "balance_user_config.h"
#include "balance_controller.h"
#include "balance_state_machine.h"
#include "command_arbiter.h"
#include "complementary_pitch_estimator.h"
#include "control_runtime.h"
#include "loop_metrics.h"
#include "motor_transport_metrics.h"
#include "mpu6050.h"
#include "serial_command_source.h"
#include "serial_messages.h"
#include "serial_protocol.h"
#include "swd_transport_budget.h"
#include "web_command_source.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>

using namespace gs::balance;

unsigned gs_esp32_tests_failed = 0;
unsigned gs_esp32_tests_run = 0;

namespace {

class FakeClock final : public IClock {
public:
  uint64_t nowMicros() const override { return now_us_; }
  void set(uint64_t now_us) { now_us_ = now_us; }

private:
  uint64_t now_us_ = 0;
};

class FakeMpuBus final : public IMpu6050Bus {
public:
  bool begin(uint32_t frequency_hz) override {
    frequency_hz_ = frequency_hz;
    return begin_ok_;
  }

  bool read(uint8_t address, uint8_t reg, uint8_t *bytes,
            size_t length) override {
    ++reads_;
    if (!read_ok_ || address != responding_address_) {
      return false;
    }
    if (reg == Mpu6050Registers::kWhoAmI && length == 1u) {
      bytes[0] = who_am_i_;
      return true;
    }
    if (reg == Mpu6050Registers::kAccelXoutH && length == burst_.size()) {
      for (size_t index = 0; index < burst_.size(); ++index) {
        bytes[index] = burst_[index];
      }
      return true;
    }
    return false;
  }

  bool write(uint8_t address, uint8_t reg, uint8_t value) override {
    if (!write_ok_ || address != responding_address_) {
      return false;
    }
    writes_[write_count_++] = {reg, value};
    return true;
  }

  void setBurst(const std::array<uint8_t, 14> &burst) { burst_ = burst; }

  struct Write {
    uint8_t reg;
    uint8_t value;
  };

  bool begin_ok_ = true;
  bool read_ok_ = true;
  bool write_ok_ = true;
  uint8_t responding_address_ = 0x68;
  uint8_t who_am_i_ = 0x68;
  uint32_t frequency_hz_ = 0;
  uint32_t reads_ = 0;
  std::array<Write, 8> writes_{};
  size_t write_count_ = 0;

private:
  std::array<uint8_t, 14> burst_{};
};

class RecordingMotorSink final : public IMotorCommandSink {
public:
  void write(const MotorCommand &command) override {
    last_ = command;
    ++writes_;
  }

  MotorCommand last_{};
  unsigned writes_ = 0;
};

std::array<uint8_t, 14> makeBurst(int16_t ax, int16_t ay, int16_t az,
                                  int16_t temperature, int16_t gx, int16_t gy,
                                  int16_t gz) {
  std::array<uint8_t, 14> bytes{};
  const int16_t values[] = {ax, ay, az, temperature, gx, gy, gz};
  for (size_t index = 0; index < 7u; ++index) {
    bytes[index * 2u] =
        static_cast<uint8_t>(static_cast<uint16_t>(values[index]) >> 8u);
    bytes[index * 2u + 1u] =
        static_cast<uint8_t>(static_cast<uint16_t>(values[index]));
  }
  return bytes;
}

void testMpuAddressDetectionAndRegisterConfiguration() {
  FakeClock clock;
  FakeMpuBus bus;
  bus.responding_address_ = 0x69;
  bus.who_am_i_ = 0x69;
  Mpu6050Imu imu(bus, clock, Mpu6050Config{});

  GS_ESP32_EXPECT_TRUE(imu.begin());
  GS_ESP32_EXPECT_EQ(0x69, imu.address());
  GS_ESP32_EXPECT_EQ(400000, bus.frequency_hz_);
  GS_ESP32_EXPECT_EQ(6, bus.write_count_);
  GS_ESP32_EXPECT_EQ(Mpu6050Registers::kPowerManagement1, bus.writes_[0].reg);
  GS_ESP32_EXPECT_EQ(0x01, bus.writes_[0].value);
  GS_ESP32_EXPECT_EQ(Mpu6050Registers::kSampleRateDivider, bus.writes_[2].reg);
  GS_ESP32_EXPECT_EQ(4, bus.writes_[2].value);
}

void testMpuRejectsInvalidIdentity() {
  FakeClock clock;
  FakeMpuBus bus;
  bus.who_am_i_ = 0x42;
  Mpu6050Imu imu(bus, clock, Mpu6050Config{});

  GS_ESP32_EXPECT_FALSE(imu.begin());
  GS_ESP32_EXPECT_EQ(0, imu.address());
}

void testMpuAcceptsMpu6500CompatibleIdentity() {
  FakeClock clock;
  FakeMpuBus bus;
  bus.responding_address_ = 0x68;
  bus.who_am_i_ = 0x70;
  Mpu6050Imu imu(bus, clock, Mpu6050Config{});

  GS_ESP32_EXPECT_TRUE(imu.begin());
  GS_ESP32_EXPECT_EQ(0x68, imu.address());
  GS_ESP32_EXPECT_EQ(6, bus.write_count_);
}

void testMpuRejectsUnsafeOrAmbiguousConfiguration() {
  FakeClock clock;
  FakeMpuBus bus;
  Mpu6050Config duplicate_axis;
  duplicate_axis.axis_map.index = {0u, 0u, 2u};
  Mpu6050Imu duplicate(bus, clock, duplicate_axis);
  GS_ESP32_EXPECT_FALSE(duplicate.begin());
  GS_ESP32_EXPECT_EQ(0, bus.reads_);

  Mpu6050Config slow;
  slow.sample_rate_hz = 100u;
  Mpu6050Imu undersampled(bus, clock, slow);
  GS_ESP32_EXPECT_FALSE(undersampled.begin());

  Mpu6050Config invalid_range;
  invalid_range.gyroscope_range_dps = 300u;
  Mpu6050Imu unsupported(bus, clock, invalid_range);
  GS_ESP32_EXPECT_FALSE(unsupported.begin());
}

void testSensorByteDecodingAndAxisMapping() {
  const auto burst = makeBurst(16384, -8192, 4096, 0, 131, -262, 65);
  Mpu6050Config config;
  config.axis_map = AxisMap{{1, 0, 2}, {-1, 1, -1}};
  const ImuSample sample = Mpu6050Decoder::decode(burst.data(), 1234u, config);

  GS_ESP32_EXPECT_NEAR(0.5, sample.accel_g.x, 0.0001);
  GS_ESP32_EXPECT_NEAR(1.0, sample.accel_g.y, 0.0001);
  GS_ESP32_EXPECT_NEAR(-0.25, sample.accel_g.z, 0.0001);
  GS_ESP32_EXPECT_NEAR(2.0, sample.gyro_dps.x, 0.0001);
  GS_ESP32_EXPECT_NEAR(1.0, sample.gyro_dps.y, 0.0001);
  GS_ESP32_EXPECT_NEAR(-0.496, sample.gyro_dps.z, 0.01);
  GS_ESP32_EXPECT_EQ(1234, sample.timestamp_us);
}

void testGyroCalibrationCompletesOnlyWhileStationary() {
  GyroBiasCalibrator calibrator(4u, 3.0f);
  ImuSample moving{};
  moving.accel_g = {0.0f, 0.0f, 1.0f};
  moving.gyro_dps = {4.0f, 0.0f, 0.0f};
  GS_ESP32_EXPECT_FALSE(calibrator.add(moving));
  GS_ESP32_EXPECT_EQ(0, calibrator.acceptedSamples());

  ImuSample stationary{};
  stationary.accel_g = {0.0f, 0.0f, 1.0f};
  stationary.gyro_dps = {1.0f, -2.0f, 0.5f};
  for (unsigned sample = 0; sample < 4u; ++sample) {
    calibrator.add(stationary);
  }

  GS_ESP32_EXPECT_TRUE(calibrator.complete());
  GS_ESP32_EXPECT_NEAR(1.0, calibrator.bias().x, 0.0001);
  GS_ESP32_EXPECT_NEAR(-2.0, calibrator.bias().y, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.5, calibrator.bias().z, 0.0001);
}

void testDefaultCalibrationAllowsBoundedZeroRateOffset() {
  const Mpu6050Config config;
  GyroBiasCalibrator calibrator(2u, config.stationary_gyro_limit_dps);
  ImuSample stationary{};
  stationary.accel_g = {0.0f, 0.0f, 1.0f};
  stationary.gyro_dps = {3.25f, 1.25f, -0.25f};

  GS_ESP32_EXPECT_FALSE(calibrator.add(stationary));
  GS_ESP32_EXPECT_TRUE(calibrator.add(stationary));
  GS_ESP32_EXPECT_NEAR(3.25, calibrator.bias().x, 0.0001);
}

void testMpuCountersAndTimeout() {
  FakeClock clock;
  FakeMpuBus bus;
  bus.setBurst(makeBurst(0, 0, 16384, 0, 0, 0, 0));
  Mpu6050Config config;
  config.calibration_samples = 1u;
  config.timeout_us = 10000u;
  Mpu6050Imu imu(bus, clock, config);
  GS_ESP32_EXPECT_TRUE(imu.begin());

  ImuSample sample{};
  clock.set(5000u);
  GS_ESP32_EXPECT_TRUE(imu.sample(sample));
  GS_ESP32_EXPECT_FALSE(imu.timedOut(4999u));
  GS_ESP32_EXPECT_FALSE(imu.timedOut(14999u));
  GS_ESP32_EXPECT_TRUE(imu.timedOut(15001u));

  bus.read_ok_ = false;
  GS_ESP32_EXPECT_FALSE(imu.sample(sample));
  GS_ESP32_EXPECT_EQ(1, imu.diagnostics().i2c_errors);
  GS_ESP32_EXPECT_EQ(1, imu.diagnostics().missed_samples);
}

void testMpuDiagnosticsExposeCalibrationProgressAndBias() {
  FakeClock clock;
  FakeMpuBus bus;
  bus.setBurst(makeBurst(0, 0, 16384, 0, 131, -262, 65));
  Mpu6050Config config;
  config.calibration_samples = 2u;
  Mpu6050Imu imu(bus, clock, config);
  GS_ESP32_EXPECT_TRUE(imu.begin());

  ImuSample sample{};
  clock.set(1000u);
  GS_ESP32_EXPECT_TRUE(imu.sample(sample));
  GS_ESP32_EXPECT_EQ(1, imu.diagnostics().calibration_samples);
  GS_ESP32_EXPECT_FALSE(imu.diagnostics().calibration_complete);
  GS_ESP32_EXPECT_NEAR(1.0, imu.diagnostics().raw_gyro_dps.x, 0.0001);
  GS_ESP32_EXPECT_NEAR(-2.0, imu.diagnostics().raw_gyro_dps.y, 0.0001);
  GS_ESP32_EXPECT_NEAR(1.0, imu.diagnostics().gyro_bias_dps.x, 0.0001);
  GS_ESP32_EXPECT_NEAR(-2.0, imu.diagnostics().gyro_bias_dps.y, 0.0001);

  clock.set(6000u);
  GS_ESP32_EXPECT_TRUE(imu.sample(sample));
  GS_ESP32_EXPECT_EQ(2, imu.diagnostics().calibration_samples);
  GS_ESP32_EXPECT_TRUE(imu.diagnostics().calibration_complete);
  GS_ESP32_EXPECT_NEAR(0.0, sample.gyro_dps.x, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, sample.gyro_dps.y, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, sample.gyro_dps.z, 0.0001);
}

void testComplementaryFilterConvergesAndRejectsInvalidDeltaTime() {
  ComplementaryPitchEstimator estimator(
      ComplementaryPitchConfig{0.98f, 2.0f, 0.001f, 0.02f});
  ImuSample upright{};
  upright.accel_g = {0.0f, 0.0f, 1.0f};
  upright.timestamp_us = 1000u;
  estimator.reset(upright);
  GS_ESP32_EXPECT_NEAR(2.0, estimator.estimate().filtered_pitch_deg, 0.001);

  ImuSample tilted = upright;
  tilted.accel_g = {0.5f, 0.0f, 0.8660254f};
  for (uint64_t time_us = 6000u; time_us <= 2006000u; time_us += 5000u) {
    tilted.timestamp_us = time_us;
    estimator.update(tilted);
  }
  GS_ESP32_EXPECT_NEAR(32.0, estimator.estimate().filtered_pitch_deg, 0.1);

  tilted.timestamp_us += 1000000u;
  GS_ESP32_EXPECT_FALSE(estimator.update(tilted));
  GS_ESP32_EXPECT_NEAR(32.0, estimator.estimate().filtered_pitch_deg, 0.1);
}

void testControllerSaturationAntiWindupAndYawMix() {
  CascadedBalanceConfig config = CascadedBalanceConfig::conservative();
  config.inner = {30.0f, 10.0f, 0.5f, 5.0f, 100.0f};
  config.outer = {2.0f, 1.0f, 0.0f, 1.0f, 3.0f};
  config.output_limit = 100.0f;
  config.yaw_limit = 20.0f;
  config.slew_per_second = 100000.0f;
  CascadedBalanceController controller(config);

  BalanceInput input{};
  input.pitch_deg = -10.0f;
  input.pitch_rate_dps = 0.0f;
  input.desired_velocity = 2.0f;
  input.desired_yaw_rate = 20.0f;
  input.dt_seconds = 0.005f;
  const BalanceOutput saturated = controller.update(input);

  GS_ESP32_EXPECT_NEAR(100.0, saturated.left, 0.001);
  GS_ESP32_EXPECT_NEAR(60.0, saturated.right, 0.001);
  GS_ESP32_EXPECT_TRUE(saturated.saturated);
  GS_ESP32_EXPECT_TRUE(std::fabs(saturated.inner_integral) <= 5.0f);
  GS_ESP32_EXPECT_TRUE(std::fabs(saturated.outer_integral) <= 1.0f);

  controller.reset(40.0f, -20.0f);
  BalanceInput neutral{};
  neutral.dt_seconds = 0.005f;
  const BalanceOutput bumpless = controller.update(neutral);
  GS_ESP32_EXPECT_NEAR(40.0, bumpless.left, 0.001);
  GS_ESP32_EXPECT_NEAR(-20.0, bumpless.right, 0.001);
}

void testControllerStopsIntegratingWhenSaturationWouldIncrease() {
  CascadedBalanceConfig config = CascadedBalanceConfig::conservative();
  config.inner = {1.0f, 10.0f, 0.0f, 100.0f, 10.0f};
  config.outer = {};
  config.output_limit = 10.0f;
  config.slew_per_second = 100000.0f;
  CascadedBalanceController controller(config);

  BalanceInput saturated{};
  saturated.pitch_deg = -100.0f;
  saturated.dt_seconds = 0.01f;
  for (unsigned step = 0u; step < 100u; ++step) {
    const BalanceOutput output = controller.update(saturated);
    GS_ESP32_EXPECT_NEAR(0.0, output.inner_integral, 0.0001);
  }

  BalanceInput reversed{};
  reversed.pitch_deg = 1.0f;
  reversed.dt_seconds = 0.01f;
  const BalanceOutput recovery = controller.update(reversed);
  GS_ESP32_EXPECT_TRUE(recovery.left < 0.0f);
  GS_ESP32_EXPECT_TRUE(recovery.right < 0.0f);
}

void testControllerRejectsNonFiniteInputAndRuntimeDisablesOutput() {
  RecordingMotorSink sink;
  CascadedBalanceController controller(CascadedBalanceConfig::conservative());
  ControlRuntime runtime(controller, sink, false);
  BalanceInput invalid{};
  invalid.pitch_deg = NAN;
  invalid.dt_seconds = 0.005f;

  const BalanceOutput output = runtime.step(invalid, true);
  GS_ESP32_EXPECT_FALSE(output.valid);
  GS_ESP32_EXPECT_FALSE(sink.last_.enabled);
  GS_ESP32_EXPECT_NEAR(0.0, sink.last_.left, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, sink.last_.right, 0.0001);
}

void testControllerSignsCorrectGeneratedTrace() {
  CascadedBalanceConfig config = CascadedBalanceConfig::conservative();
  config.slew_per_second = 100000.0f;
  CascadedBalanceController controller(config);
  BalanceInput forward_fall{};
  forward_fall.pitch_deg = 5.0f;
  forward_fall.dt_seconds = 0.005f;
  const BalanceOutput correction = controller.update(forward_fall);

  GS_ESP32_EXPECT_TRUE(correction.left < 0.0f);
  GS_ESP32_EXPECT_TRUE(correction.right < 0.0f);
}

void testUprightOffsetAndImuSignDefineOneBalanceFrame() {
  CascadedBalanceConfig config = CascadedBalanceConfig::conservative();
  config.imu_sign = -1.0f;
  config.upright_offset_deg = 7.0f;
  config.slew_per_second = 100000.0f;

  GS_ESP32_EXPECT_NEAR(0.0, balanceFramePitchDeg(-7.0f, config), 0.0001);
  GS_ESP32_EXPECT_NEAR(13.0, balanceFramePitchDeg(-20.0f, config), 0.0001);

  CascadedBalanceController controller(config);
  BalanceInput upright{};
  upright.pitch_deg = -7.0f;
  upright.dt_seconds = 0.005f;
  const BalanceOutput output = controller.update(upright);
  GS_ESP32_EXPECT_NEAR(0.0, output.left, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, output.right, 0.0001);
}

void testGeneratedPendulumTraceReducesSmallPitchError() {
  CascadedBalanceConfig config = CascadedBalanceConfig::conservative();
  config.outer = {};
  config.slew_per_second = 100000.0f;
  CascadedBalanceController controller(config);
  float pitch = 5.0f;
  float pitch_rate = 0.0f;
  float maximum_pitch = pitch;
  for (unsigned step = 0u; step < 1000u; ++step) {
    BalanceInput input{};
    input.pitch_deg = pitch;
    input.pitch_rate_dps = pitch_rate;
    input.dt_seconds = 0.005f;
    const BalanceOutput output = controller.update(input);
    const float common = (output.left + output.right) * 0.5f;
    const float pitch_acceleration = 4.0f * pitch + common;
    pitch_rate += pitch_acceleration * input.dt_seconds;
    pitch += pitch_rate * input.dt_seconds;
    maximum_pitch = std::max(maximum_pitch, std::fabs(pitch));
  }
  GS_ESP32_EXPECT_TRUE(std::fabs(pitch) < 5.0f);
  GS_ESP32_EXPECT_TRUE(maximum_pitch < 6.0f);
}

SafetySnapshot healthySnapshot() {
  SafetySnapshot healthy{};
  healthy.imu_healthy = true;
  healthy.calibrated = true;
  healthy.approximately_upright = true;
  healthy.motor_feedback_healthy = true;
  healthy.motor_transport_healthy = true;
  healthy.zero_output_acknowledged = true;
  healthy.loop_healthy = true;
  return healthy;
}

bool canArmWith(const SafetySnapshot &candidate) {
  BalanceStateMachine machine({12.0f, 100000u});
  const SafetySnapshot healthy = healthySnapshot();
  machine.update(healthy, 0.0f, 0u);
  machine.update(healthy, 0.0f, 1u);
  return machine.arm(candidate);
}

bool faultsArmedBalanceWith(const SafetySnapshot &candidate) {
  BalanceStateMachine machine({12.0f, 100000u});
  const SafetySnapshot healthy = healthySnapshot();
  machine.update(healthy, 0.0f, 0u);
  machine.update(healthy, 0.0f, 1u);
  if (!machine.arm(healthy)) {
    return false;
  }
  machine.update(candidate, 0.0f, 2u);
  return machine.state() == BalanceState::kFault && !machine.outputEnabled();
}

void testEveryArmingGateAndActiveFaultDisablesOutput() {
  SafetySnapshot candidate = healthySnapshot();
  candidate.imu_healthy = false;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));
  GS_ESP32_EXPECT_TRUE(faultsArmedBalanceWith(candidate));

  candidate = healthySnapshot();
  candidate.calibrated = false;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));

  candidate = healthySnapshot();
  candidate.approximately_upright = false;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));

  candidate = healthySnapshot();
  candidate.motor_feedback_healthy = false;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));
  GS_ESP32_EXPECT_TRUE(faultsArmedBalanceWith(candidate));

  candidate = healthySnapshot();
  candidate.motor_transport_healthy = false;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));
  GS_ESP32_EXPECT_TRUE(faultsArmedBalanceWith(candidate));

  candidate = healthySnapshot();
  candidate.zero_output_acknowledged = false;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));

  candidate = healthySnapshot();
  candidate.loop_healthy = false;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));
  GS_ESP32_EXPECT_TRUE(faultsArmedBalanceWith(candidate));

  candidate = healthySnapshot();
  candidate.controller_fault = true;
  GS_ESP32_EXPECT_FALSE(canArmWith(candidate));
  GS_ESP32_EXPECT_TRUE(faultsArmedBalanceWith(candidate));
}

void testStateMachineArmDriveFallAndExplicitRearm() {
  BalanceStateMachine machine({12.0f, 100000u});
  SafetySnapshot healthy = healthySnapshot();

  GS_ESP32_EXPECT_EQ(BalanceState::kBoot, machine.state());
  machine.update(healthy, 0.0f, 0u);
  GS_ESP32_EXPECT_EQ(BalanceState::kImuCalibrating, machine.state());
  machine.update(healthy, 0.0f, 1u);
  GS_ESP32_EXPECT_EQ(BalanceState::kDisarmed, machine.state());
  GS_ESP32_EXPECT_TRUE(machine.arm(healthy));
  GS_ESP32_EXPECT_EQ(BalanceState::kArmedBalance, machine.state());
  machine.setDriving(true);
  GS_ESP32_EXPECT_EQ(BalanceState::kDriving, machine.state());

  machine.update(healthy, 13.0f, 1000u);
  machine.update(healthy, 13.0f, 101001u);
  GS_ESP32_EXPECT_EQ(BalanceState::kFallen, machine.state());
  machine.update(healthy, 0.0f, 200000u);
  GS_ESP32_EXPECT_EQ(BalanceState::kFallen, machine.state());
  GS_ESP32_EXPECT_TRUE(machine.arm(healthy));
  GS_ESP32_EXPECT_EQ(BalanceState::kArmedBalance, machine.state());
}

void testStateMachineFaultDisablesAndRequiresClear() {
  BalanceStateMachine machine({12.0f, 100000u});
  SafetySnapshot healthy = healthySnapshot();
  machine.update(healthy, 0.0f, 0u);
  machine.update(healthy, 0.0f, 1u);
  GS_ESP32_EXPECT_TRUE(machine.arm(healthy));

  SafetySnapshot faulted = healthy;
  faulted.imu_healthy = false;
  machine.update(faulted, 0.0f, 2u);
  GS_ESP32_EXPECT_EQ(BalanceState::kFault, machine.state());
  GS_ESP32_EXPECT_FALSE(machine.arm(healthy));
  GS_ESP32_EXPECT_TRUE(machine.clearFault(healthy));
  GS_ESP32_EXPECT_EQ(BalanceState::kDisarmed, machine.state());
}

void testStateMachineFaultsWhenImuIsUnavailableDuringCalibration() {
  BalanceStateMachine machine({12.0f, 100000u});
  SafetySnapshot unavailable{};
  unavailable.loop_healthy = true;
  machine.update(unavailable, 0.0f, 0u);
  machine.update(unavailable, 0.0f, 5000u);
  GS_ESP32_EXPECT_EQ(BalanceState::kFault, machine.state());
  GS_ESP32_EXPECT_FALSE(machine.outputEnabled());
}

void testStateMachineWaitsSafelyForImuCalibration() {
  BalanceStateMachine machine({12.0f, 100000u});
  SafetySnapshot calibrating{};
  calibrating.imu_healthy = true;
  calibrating.loop_healthy = true;

  machine.update(calibrating, 0.0f, 0u);
  machine.update(calibrating, 0.0f, 5000u);
  GS_ESP32_EXPECT_EQ(BalanceState::kImuCalibrating, machine.state());
  GS_ESP32_EXPECT_FALSE(machine.outputEnabled());
}

void testDisarmedDiagnosticAllowsMissingMotorsButArmedBalanceDoesNot() {
  BalanceStateMachine machine({12.0f, 100000u});
  SafetySnapshot healthy = healthySnapshot();
  machine.update(healthy, 0.0f, 0u);
  machine.update(healthy, 0.0f, 5000u);
  GS_ESP32_EXPECT_EQ(BalanceState::kDisarmed, machine.state());

  SafetySnapshot motors_absent = healthy;
  motors_absent.motor_feedback_healthy = false;
  motors_absent.motor_transport_healthy = false;
  machine.update(motors_absent, 0.0f, 10000u);
  GS_ESP32_EXPECT_EQ(BalanceState::kDisarmed, machine.state());

  GS_ESP32_EXPECT_TRUE(machine.arm(healthy));
  machine.update(motors_absent, 0.0f, 15000u);
  GS_ESP32_EXPECT_EQ(BalanceState::kFault, machine.state());
}

void testFaultCanClearToDiagnosticWithoutMotorsButCannotArm() {
  BalanceStateMachine machine({12.0f, 100000u});
  SafetySnapshot healthy = healthySnapshot();
  machine.update(healthy, 0.0f, 0u);
  machine.update(healthy, 0.0f, 5000u);
  GS_ESP32_EXPECT_TRUE(machine.arm(healthy));

  SafetySnapshot imu_fault = healthy;
  imu_fault.imu_healthy = false;
  machine.update(imu_fault, 0.0f, 10000u);
  GS_ESP32_EXPECT_EQ(BalanceState::kFault, machine.state());

  SafetySnapshot diagnostic = healthy;
  diagnostic.motor_feedback_healthy = false;
  diagnostic.motor_transport_healthy = false;
  diagnostic.zero_output_acknowledged = false;
  GS_ESP32_EXPECT_TRUE(machine.clearFault(diagnostic));
  GS_ESP32_EXPECT_EQ(BalanceState::kDisarmed, machine.state());
  GS_ESP32_EXPECT_FALSE(machine.arm(diagnostic));
}

void testSerialLeaseWinsOverWebAndDisconnectZerosMovement() {
  CommandArbiter arbiter;
  ControlRequest web{};
  web.source = CommandSource::kWeb;
  web.linear_velocity = 1.0f;
  web.yaw_rate = 2.0f;
  web.lease_expires_us = 1000u;
  arbiter.submit(web);

  ControlRequest serial = web;
  serial.source = CommandSource::kSerial;
  serial.linear_velocity = 3.0f;
  arbiter.submit(serial);
  GS_ESP32_EXPECT_EQ(CommandSource::kSerial, arbiter.resolve(500u).source);

  arbiter.disconnect(CommandSource::kSerial);
  const ControlRequest stopped = arbiter.resolve(500u);
  GS_ESP32_EXPECT_EQ(CommandSource::kNone, stopped.source);
  GS_ESP32_EXPECT_NEAR(0.0, stopped.linear_velocity, 0.001);
  GS_ESP32_EXPECT_NEAR(0.0, stopped.yaw_rate, 0.001);
  GS_ESP32_EXPECT_FALSE(stopped.disarm);

  web.sequence = 2u;
  arbiter.submit(web);
  const ControlRequest explicit_handoff = arbiter.resolve(500u);
  GS_ESP32_EXPECT_EQ(CommandSource::kWeb, explicit_handoff.source);
}

void testEmergencyStopAndLocalDisarmOverrideLeases() {
  CommandArbiter arbiter;
  ControlRequest serial{};
  serial.source = CommandSource::kSerial;
  serial.linear_velocity = 4.0f;
  serial.lease_expires_us = 1000u;
  arbiter.submit(serial);
  arbiter.setLocalDisarm(true);
  GS_ESP32_EXPECT_TRUE(arbiter.resolve(500u).disarm);

  arbiter.setEmergencyStop(true);
  const ControlRequest emergency = arbiter.resolve(500u);
  GS_ESP32_EXPECT_TRUE(emergency.emergency_stop);
  GS_ESP32_EXPECT_NEAR(0.0, emergency.linear_velocity, 0.001);
}

void testHigherPrioritySourceAndAnyEmergencyStopPreemptOwner() {
  CommandArbiter arbiter;
  ControlRequest web{};
  web.source = CommandSource::kWeb;
  web.sequence = 1u;
  web.linear_velocity = 1.0f;
  web.lease_expires_us = 1000u;
  arbiter.submit(web);
  GS_ESP32_EXPECT_EQ(CommandSource::kWeb, arbiter.resolve(100u).source);

  ControlRequest serial = web;
  serial.source = CommandSource::kSerial;
  serial.sequence = 10u;
  arbiter.submit(serial);
  GS_ESP32_EXPECT_EQ(CommandSource::kSerial, arbiter.resolve(200u).source);

  web.sequence = 2u;
  web.emergency_stop = true;
  web.disarm = true;
  arbiter.submit(web);
  const ControlRequest emergency = arbiter.resolve(300u);
  GS_ESP32_EXPECT_EQ(CommandSource::kWeb, emergency.source);
  GS_ESP32_EXPECT_TRUE(emergency.emergency_stop);
}

void testDryRunCalculatesButNeverSendsNonzeroOutput() {
  RecordingMotorSink sink;
  CascadedBalanceController controller(CascadedBalanceConfig::conservative());
  ControlRuntime runtime(controller, sink, true);
  BalanceInput input{};
  input.pitch_deg = 5.0f;
  input.dt_seconds = 0.005f;

  (void)runtime.step(input, true);
  const BalanceOutput calculated = runtime.step(input, true);
  GS_ESP32_EXPECT_TRUE(std::fabs(calculated.left) > 0.0f);
  GS_ESP32_EXPECT_EQ(2, sink.writes_);
  GS_ESP32_EXPECT_NEAR(0.0, sink.last_.left, 0.001);
  GS_ESP32_EXPECT_NEAR(0.0, sink.last_.right, 0.001);
  GS_ESP32_EXPECT_FALSE(sink.last_.enabled);
}

void testDirectTransportCommandIsBoundedAndHonorsDryRun() {
  RecordingMotorSink sink;
  CascadedBalanceController controller(CascadedBalanceConfig::conservative());
  ControlRuntime active(controller, sink, false);

  const BalanceOutput direct = active.stepDirect(20.0f, -30.0f, true);
  GS_ESP32_EXPECT_NEAR(20.0, direct.left, 0.001);
  GS_ESP32_EXPECT_NEAR(-30.0, direct.right, 0.001);
  GS_ESP32_EXPECT_TRUE(sink.last_.enabled);
  GS_ESP32_EXPECT_NEAR(20.0, sink.last_.left, 0.001);
  GS_ESP32_EXPECT_NEAR(-30.0, sink.last_.right, 0.001);

  const BalanceOutput bounded = active.stepDirect(500.0f, -500.0f, true);
  GS_ESP32_EXPECT_NEAR(kMaximumTransportTestCommand, bounded.left, 0.001);
  GS_ESP32_EXPECT_NEAR(-kMaximumTransportTestCommand, bounded.right, 0.001);

  ControlRuntime dry_run(controller, sink, true);
  const BalanceOutput calculated = dry_run.stepDirect(10.0f, -10.0f, true);
  GS_ESP32_EXPECT_NEAR(10.0, calculated.left, 0.001);
  GS_ESP32_EXPECT_NEAR(-10.0, calculated.right, 0.001);
  GS_ESP32_EXPECT_FALSE(sink.last_.enabled);
  GS_ESP32_EXPECT_NEAR(0.0, sink.last_.left, 0.001);
  GS_ESP32_EXPECT_NEAR(0.0, sink.last_.right, 0.001);
}

void testRuntimeResetsControllerAcrossDisabledTransition() {
  RecordingMotorSink sink;
  CascadedBalanceController controller(CascadedBalanceConfig::conservative());
  ControlRuntime runtime(controller, sink, false);
  BalanceInput tilted{};
  tilted.pitch_deg = -5.0f;
  tilted.dt_seconds = 0.005f;
  for (unsigned step = 0u; step < 100u; ++step) {
    const BalanceOutput diagnostic = runtime.step(tilted, false);
    GS_ESP32_EXPECT_TRUE(diagnostic.valid);
    GS_ESP32_EXPECT_TRUE(diagnostic.left > 0.0f);
    GS_ESP32_EXPECT_TRUE(diagnostic.right > 0.0f);
    GS_ESP32_EXPECT_FALSE(sink.last_.enabled);
  }

  const BalanceOutput first_armed = runtime.step(tilted, true);
  GS_ESP32_EXPECT_NEAR(0.0, first_armed.left, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, first_armed.right, 0.0001);
  GS_ESP32_EXPECT_TRUE(sink.last_.enabled);

  const BalanceOutput correcting = runtime.step(tilted, true);
  GS_ESP32_EXPECT_TRUE(correcting.left > 0.0f);
  GS_ESP32_EXPECT_TRUE(correcting.right > 0.0f);
}

void testLoopMetricsReportRateJitterDeadlineAndAges() {
  LoopMetrics metrics(5000u);
  metrics.record({1000u, 4900u, 200u, 100u, 200u});
  metrics.record({6100u, 5100u, 250u, 150u, 250u});
  metrics.record({11300u, 5200u, 300u, 200u, 300u});
  const LoopStatistics stats = metrics.statistics();

  GS_ESP32_EXPECT_NEAR(194.17, stats.average_frequency_hz, 0.1);
  GS_ESP32_EXPECT_EQ(4900, stats.minimum_period_us);
  GS_ESP32_EXPECT_EQ(5200, stats.maximum_period_us);
  GS_ESP32_EXPECT_EQ(200, stats.worst_jitter_us);
  GS_ESP32_EXPECT_EQ(2, stats.missed_deadlines);
  GS_ESP32_EXPECT_EQ(300, stats.maximum_execution_us);
  GS_ESP32_EXPECT_EQ(200, stats.sensor_sample_age_us);
  GS_ESP32_EXPECT_EQ(300, stats.motor_command_age_us);
  GS_ESP32_EXPECT_EQ(250, LoopMetrics::ageUs(1250u, 1000u));
  GS_ESP32_EXPECT_EQ(0, LoopMetrics::ageUs(1000u, 1250u));
  GS_ESP32_EXPECT_EQ(UINT32_MAX, LoopMetrics::ageUs(UINT64_MAX, 0u));
}

void testSerialFrameRoundTripAndLittleEndianEncoding() {
  SerialFrame frame{};
  frame.type = SerialMessageType::kSetVelocityAndYaw;
  frame.flags = 0x03u;
  frame.sequence = 0x1234u;
  frame.payload_length = 4u;
  frame.payload[0] = 0x78u;
  frame.payload[1] = 0x56u;
  frame.payload[2] = 0x34u;
  frame.payload[3] = 0x12u;
  uint8_t encoded[kSerialMaximumFrameSize] = {};

  const size_t length = SerialProtocol::encode(frame, encoded, sizeof(encoded));
  GS_ESP32_EXPECT_EQ(15, length);
  GS_ESP32_EXPECT_EQ(0xa5, encoded[0]);
  GS_ESP32_EXPECT_EQ(0x5a, encoded[1]);
  GS_ESP32_EXPECT_EQ(0x34, encoded[5]);
  GS_ESP32_EXPECT_EQ(0x12, encoded[6]);
  GS_ESP32_EXPECT_EQ(0x04, encoded[7]);
  GS_ESP32_EXPECT_EQ(0x00, encoded[8]);

  SerialParser parser(50000u);
  SerialFrame decoded{};
  SerialParseResult result = SerialParseResult::kIncomplete;
  for (size_t index = 0; index < length; ++index) {
    result = parser.feed(encoded[index], index * 100u, decoded);
  }
  GS_ESP32_EXPECT_EQ(SerialParseResult::kFrame, result);
  GS_ESP32_EXPECT_EQ(SerialMessageType::kSetVelocityAndYaw, decoded.type);
  GS_ESP32_EXPECT_EQ(0x1234, decoded.sequence);
  GS_ESP32_EXPECT_EQ(4, decoded.payload_length);
  GS_ESP32_EXPECT_EQ(0x78, decoded.payload[0]);
}

void testSerialParserRejectsCrcAndMalformedLength() {
  SerialFrame frame{};
  frame.type = SerialMessageType::kHeartbeat;
  frame.sequence = 7u;
  uint8_t encoded[kSerialMaximumFrameSize] = {};
  const size_t length = SerialProtocol::encode(frame, encoded, sizeof(encoded));
  encoded[length - 1u] ^= 0x01u;

  SerialParser parser(50000u);
  SerialFrame decoded{};
  SerialParseResult result = SerialParseResult::kIncomplete;
  for (size_t index = 0; index < length; ++index) {
    result = parser.feed(encoded[index], index * 100u, decoded);
  }
  GS_ESP32_EXPECT_EQ(SerialParseResult::kInvalidCrc, result);

  const uint8_t malformed[] = {
      0xa5u,
      0x5au,
      kSerialProtocolVersion,
      static_cast<uint8_t>(SerialMessageType::kStatus),
      0u,
      0u,
      0u,
      static_cast<uint8_t>(kSerialMaximumPayloadSize + 1u),
      0u,
  };
  result = SerialParseResult::kIncomplete;
  for (size_t index = 0; index < sizeof(malformed); ++index) {
    result = parser.feed(malformed[index], 10000u + index, decoded);
  }
  GS_ESP32_EXPECT_EQ(SerialParseResult::kMalformed, result);
}

void testSerialParserTimesOutPartialFrame() {
  SerialParser parser(1000u);
  SerialFrame decoded{};
  GS_ESP32_EXPECT_EQ(SerialParseResult::kIncomplete,
                     parser.feed(0xa5u, 100u, decoded));
  GS_ESP32_EXPECT_EQ(SerialParseResult::kIncomplete,
                     parser.feed(0x5au, 200u, decoded));
  GS_ESP32_EXPECT_EQ(SerialParseResult::kTimeout,
                     parser.feed(kSerialProtocolVersion, 1201u, decoded));
}

void testSerialParserRoundTripsEveryPayloadWidthAndRejectsSingleBitDamage() {
  uint32_t random = 0x6d2b79f5u;
  for (uint16_t payload_length = 0u;
       payload_length <= kSerialMaximumPayloadSize; ++payload_length) {
    SerialFrame frame{};
    frame.type = SerialMessageType::kStatus;
    frame.flags = static_cast<uint8_t>(payload_length);
    frame.sequence = static_cast<uint16_t>(payload_length * 1307u);
    frame.payload_length = payload_length;
    for (uint16_t index = 0u; index < payload_length; ++index) {
      random ^= random << 13u;
      random ^= random >> 17u;
      random ^= random << 5u;
      frame.payload[index] = static_cast<uint8_t>(random);
    }

    uint8_t encoded[kSerialMaximumFrameSize] = {};
    const size_t encoded_length =
        SerialProtocol::encode(frame, encoded, sizeof(encoded));
    SerialParser parser(50000u);
    SerialFrame decoded{};
    SerialParseResult result = SerialParseResult::kIncomplete;
    result = parser.feed(0x11u, 1u, decoded);
    result = parser.feed(0xa5u, 2u, decoded);
    result = parser.feed(0x22u, 3u, decoded);
    for (size_t index = 0u; index < encoded_length; ++index) {
      result = parser.feed(encoded[index], 4u + index, decoded);
    }

    GS_ESP32_EXPECT_EQ(SerialParseResult::kFrame, result);
    GS_ESP32_EXPECT_EQ(frame.sequence, decoded.sequence);
    GS_ESP32_EXPECT_EQ(payload_length, decoded.payload_length);
    for (uint16_t index = 0u; index < payload_length; ++index) {
      GS_ESP32_EXPECT_EQ(frame.payload[index], decoded.payload[index]);
    }
  }

  SerialFrame maximum{};
  maximum.type = SerialMessageType::kStatus;
  maximum.sequence = 0x5a5au;
  maximum.payload_length = kSerialMaximumPayloadSize;
  maximum.payload.fill(0x33u);
  uint8_t encoded[kSerialMaximumFrameSize] = {};
  const size_t encoded_length =
      SerialProtocol::encode(maximum, encoded, sizeof(encoded));
  for (size_t damaged_index = 0u; damaged_index < encoded_length;
       ++damaged_index) {
    encoded[damaged_index] ^= 0x01u;
    SerialParser parser(50000u);
    SerialFrame decoded{};
    SerialParseResult result = SerialParseResult::kIncomplete;
    for (size_t index = 0u; index < encoded_length; ++index) {
      result = parser.feed(encoded[index], index, decoded);
    }
    GS_ESP32_EXPECT_FALSE(result == SerialParseResult::kFrame);
    encoded[damaged_index] ^= 0x01u;
  }
}

void testSerialSequenceRejectsStaleAndAcceptsRollover() {
  SerialSequence sequence;
  GS_ESP32_EXPECT_TRUE(sequence.accept(0xfffeu));
  GS_ESP32_EXPECT_TRUE(sequence.accept(0xffffu));
  GS_ESP32_EXPECT_TRUE(sequence.accept(0x0000u));
  GS_ESP32_EXPECT_FALSE(sequence.accept(0xffffu));
  GS_ESP32_EXPECT_FALSE(sequence.accept(0x0000u));
  GS_ESP32_EXPECT_TRUE(sequence.accept(0x0001u));
}

void testMovementCommandLifetimeAndPayloadCodec() {
  MovementCommand command{};
  command.linear_velocity_milli = -250;
  command.yaw_rate_milli = 125;
  command.lease_id = 0x12345678u;
  command.lifetime_ms = 500u;
  uint8_t payload[kSerialMaximumPayloadSize] = {};

  const size_t length =
      MovementCommandCodec::encode(command, payload, sizeof(payload));
  GS_ESP32_EXPECT_EQ(10, length);
  GS_ESP32_EXPECT_EQ(0x06, payload[0]);
  GS_ESP32_EXPECT_EQ(0xff, payload[1]);
  GS_ESP32_EXPECT_EQ(0x78, payload[4]);
  GS_ESP32_EXPECT_EQ(0x56, payload[5]);

  MovementCommand decoded{};
  GS_ESP32_EXPECT_TRUE(MovementCommandCodec::decode(payload, length, decoded));
  GS_ESP32_EXPECT_EQ(-250, decoded.linear_velocity_milli);
  GS_ESP32_EXPECT_EQ(125, decoded.yaw_rate_milli);
  GS_ESP32_EXPECT_EQ(0x12345678, decoded.lease_id);
  GS_ESP32_EXPECT_EQ(500, decoded.lifetime_ms);
  GS_ESP32_EXPECT_FALSE(decoded.expired(1000000u, 1499999u));
  GS_ESP32_EXPECT_TRUE(decoded.expired(1000000u, 1500001u));
  GS_ESP32_EXPECT_FALSE(
      MovementCommandCodec::decode(payload, length - 1u, decoded));
}

void testBalanceConfigurationCodecValidatesAndUpdatesOneKey() {
  CascadedBalanceConfig config = CascadedBalanceConfig::conservative();
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length = BalanceConfigurationCodec::encodeValue(
      BalanceConfigKey::kInnerProportional, 12.345f, payload, sizeof(payload));
  GS_ESP32_EXPECT_EQ(5, length);
  GS_ESP32_EXPECT_EQ(static_cast<uint8_t>(BalanceConfigKey::kInnerProportional),
                     payload[0]);
  GS_ESP32_EXPECT_EQ(0x39, payload[1]);
  GS_ESP32_EXPECT_EQ(0x30, payload[2]);

  GS_ESP32_EXPECT_TRUE(
      BalanceConfigurationCodec::applyUpdate(payload, length, config));
  GS_ESP32_EXPECT_NEAR(12.345, config.inner.proportional, 0.0001);

  const size_t unsafe_length = BalanceConfigurationCodec::encodeValue(
      BalanceConfigKey::kOutputLimit, 251.0f, payload, sizeof(payload));
  GS_ESP32_EXPECT_EQ(5, unsafe_length);
  GS_ESP32_EXPECT_FALSE(
      BalanceConfigurationCodec::applyUpdate(payload, unsafe_length, config));
  GS_ESP32_EXPECT_NEAR(250.0, config.output_limit, 0.0001);

  const size_t offset_length = BalanceConfigurationCodec::encodeValue(
      BalanceConfigKey::kUprightOffset,
      user_config::kUprightMountingOffsetDeg, payload, sizeof(payload));
  GS_ESP32_EXPECT_TRUE(
      BalanceConfigurationCodec::applyUpdate(payload, offset_length, config));
  GS_ESP32_EXPECT_NEAR(20.0, config.upright_offset_deg, 0.0001);
}

void testBinaryTelemetryPayloadsAndResponseQueueAreBounded() {
  ProtocolCapabilities capabilities{};
  capabilities.dry_run = true;
  capabilities.web_enabled = false;
  capabilities.control_rate_hz = 200u;
  capabilities.motor_rate_hz = 10u;
  uint8_t payload[kSerialMaximumPayloadSize] = {};
  const size_t length = SerialMessageCodec::encodeCapabilities(
      capabilities, payload, sizeof(payload));
  GS_ESP32_EXPECT_EQ(12, length);
  GS_ESP32_EXPECT_EQ(kSerialProtocolVersion, payload[0]);
  GS_ESP32_EXPECT_EQ(1, payload[1]);
  GS_ESP32_EXPECT_EQ(0xc8, payload[4]);
  GS_ESP32_EXPECT_EQ(0x00, payload[5]);

  SerialFrameQueue queue;
  SerialFrame frame{};
  for (uint16_t sequence = 0u; sequence < SerialFrameQueue::kCapacity;
       ++sequence) {
    frame.sequence = sequence;
    GS_ESP32_EXPECT_TRUE(queue.push(frame));
  }
  GS_ESP32_EXPECT_FALSE(queue.push(frame));
  GS_ESP32_EXPECT_EQ(1, queue.droppedFrames());
  for (uint16_t sequence = 0u; sequence < SerialFrameQueue::kCapacity;
       ++sequence) {
    SerialFrame popped{};
    GS_ESP32_EXPECT_TRUE(queue.pop(popped));
    GS_ESP32_EXPECT_EQ(sequence, popped.sequence);
  }
  GS_ESP32_EXPECT_EQ(0, queue.size());
}

void testEveryBinaryTelemetrySchemaHasDeterministicWidthAndOffsets() {
  uint8_t payload[kSerialMaximumPayloadSize] = {};

  ProtocolStatus status{};
  status.state = 3u;
  status.faults = 0x12345678u;
  GS_ESP32_EXPECT_EQ(
      16, SerialMessageCodec::encodeStatus(status, payload, sizeof(payload)));
  GS_ESP32_EXPECT_EQ(3, payload[0]);
  GS_ESP32_EXPECT_EQ(0x78, payload[4]);
  GS_ESP32_EXPECT_EQ(0x12, payload[7]);

  ProtocolImuTelemetry imu{};
  imu.address = 0x69u;
  imu.acceleration_milli_g[0] = -1000;
  imu.sample_age_us = 0x11223344u;
  imu.calibration_samples = 400u;
  imu.gyro_bias_centi_dps[0] = -125;
  GS_ESP32_EXPECT_EQ(
      44, SerialMessageCodec::encodeImu(imu, payload, sizeof(payload)));
  GS_ESP32_EXPECT_EQ(0x69, payload[0]);
  GS_ESP32_EXPECT_EQ(0x18, payload[4]);
  GS_ESP32_EXPECT_EQ(0xfc, payload[5]);
  GS_ESP32_EXPECT_EQ(0x44, payload[32]);
  GS_ESP32_EXPECT_EQ(0x11, payload[35]);
  GS_ESP32_EXPECT_EQ(0x90, payload[36]);
  GS_ESP32_EXPECT_EQ(0x01, payload[37]);
  GS_ESP32_EXPECT_EQ(0x83, payload[38]);
  GS_ESP32_EXPECT_EQ(0xff, payload[39]);

  ProtocolMotorTelemetry motor{};
  motor.applied_right = -250;
  motor.maximum_apply_latency_us = 0x12345678u;
  motor.transmit_rate_centi_hz = 1000u;
  GS_ESP32_EXPECT_EQ(
      46, SerialMessageCodec::encodeMotor(motor, payload, sizeof(payload)));
  GS_ESP32_EXPECT_EQ(0x06, payload[6]);
  GS_ESP32_EXPECT_EQ(0xff, payload[7]);
  GS_ESP32_EXPECT_EQ(0x78, payload[40]);
  GS_ESP32_EXPECT_EQ(0x12, payload[43]);
  GS_ESP32_EXPECT_EQ(0xe8, payload[44]);
  GS_ESP32_EXPECT_EQ(0x03, payload[45]);

  ProtocolOdometry odometry{};
  odometry.left = -2;
  odometry.timestamp_us = 0x0102030405060708u;
  GS_ESP32_EXPECT_EQ(20, SerialMessageCodec::encodeOdometry(odometry, payload,
                                                            sizeof(payload)));
  GS_ESP32_EXPECT_EQ(0xfe, payload[0]);
  GS_ESP32_EXPECT_EQ(0xff, payload[3]);
  GS_ESP32_EXPECT_EQ(0x08, payload[12]);
  GS_ESP32_EXPECT_EQ(0x01, payload[19]);

  ProtocolFaults faults{};
  faults.slave = 0xaabbccddu;
  GS_ESP32_EXPECT_EQ(
      16, SerialMessageCodec::encodeFaults(faults, payload, sizeof(payload)));
  GS_ESP32_EXPECT_EQ(0xdd, payload[8]);
  GS_ESP32_EXPECT_EQ(0xaa, payload[11]);
}

void testProtocolDeclaresRequiredMessageTypes() {
  GS_ESP32_EXPECT_TRUE(static_cast<uint8_t>(SerialMessageType::kHello) !=
                       static_cast<uint8_t>(SerialMessageType::kCapabilities));
  GS_ESP32_EXPECT_TRUE(static_cast<uint8_t>(SerialMessageType::kArm) !=
                       static_cast<uint8_t>(SerialMessageType::kDisarm));
  GS_ESP32_EXPECT_TRUE(
      static_cast<uint8_t>(SerialMessageType::kEmergencyStop) !=
      static_cast<uint8_t>(SerialMessageType::kClearFault));
  GS_ESP32_EXPECT_TRUE(
      static_cast<uint8_t>(SerialMessageType::kConfigurationRead) !=
      static_cast<uint8_t>(SerialMessageType::kConfigurationUpdate));
  GS_ESP32_EXPECT_TRUE(
      static_cast<uint8_t>(SerialMessageType::kAcknowledgment) !=
      static_cast<uint8_t>(SerialMessageType::kErrorResponse));
}

void feedEncodedFrame(SerialCommandSource &source, const SerialFrame &frame,
                      uint64_t start_us) {
  uint8_t encoded[kSerialMaximumFrameSize] = {};
  const size_t length = SerialProtocol::encode(frame, encoded, sizeof(encoded));
  for (size_t index = 0; index < length; ++index) {
    source.feed(encoded[index], start_us + index);
  }
}

void testSerialCommandSourceProducesLeaseBoundRequest() {
  MovementCommand movement{};
  movement.linear_velocity_milli = 750;
  movement.yaw_rate_milli = -250;
  movement.lease_id = 42u;
  movement.lifetime_ms = 500u;
  SerialFrame frame{};
  frame.type = SerialMessageType::kSetVelocityAndYaw;
  frame.sequence = 100u;
  frame.payload_length = static_cast<uint16_t>(MovementCommandCodec::encode(
      movement, frame.payload.data(), frame.payload.size()));
  SerialCommandSource source(50000u);

  feedEncodedFrame(source, frame, 1000000u);
  ControlRequest request{};
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_EQ(CommandSource::kSerial, request.source);
  GS_ESP32_EXPECT_NEAR(0.75, request.linear_velocity, 0.0001);
  GS_ESP32_EXPECT_NEAR(-0.25, request.yaw_rate, 0.0001);
  GS_ESP32_EXPECT_EQ(100, request.sequence);
  GS_ESP32_EXPECT_EQ(1500020, request.lease_expires_us);
  GS_ESP32_EXPECT_EQ(1, source.acceptedFrames());
}

void testSerialCommandSourceAcceptsOnlyBoundedDirectMotorCommands() {
  DirectMotorCommand direct{};
  direct.left = 20;
  direct.right = -30;
  direct.lease_id = 77u;
  direct.lifetime_ms = 400u;
  SerialFrame frame{};
  frame.type = SerialMessageType::kSetDirectMotor;
  frame.sequence = 101u;
  frame.payload_length = static_cast<uint16_t>(DirectMotorCommandCodec::encode(
      direct, frame.payload.data(), frame.payload.size()));
  SerialCommandSource source(50000u);

  feedEncodedFrame(source, frame, 1000000u);
  ControlRequest request{};
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_TRUE(request.direct_motor);
  GS_ESP32_EXPECT_NEAR(20.0, request.direct_left, 0.001);
  GS_ESP32_EXPECT_NEAR(-30.0, request.direct_right, 0.001);
  GS_ESP32_EXPECT_EQ(77, request.lease_id);

  direct.left = static_cast<int16_t>(kMaximumTransportTestCommand + 1);
  frame.sequence = 102u;
  frame.payload_length = static_cast<uint16_t>(DirectMotorCommandCodec::encode(
      direct, frame.payload.data(), frame.payload.size()));
  feedEncodedFrame(source, frame, 2000000u);
  GS_ESP32_EXPECT_EQ(1, source.rejectedFrames());
  GS_ESP32_EXPECT_EQ(
      static_cast<uint8_t>(SerialErrorCode::kInvalidConfiguration),
      source.lastErrorCode());
}

void testSerialCommandSourceRejectsStaleSequenceAndDisconnectsSafely() {
  SerialCommandSource source(50000u);
  SerialFrame stop{};
  stop.type = SerialMessageType::kStop;
  stop.sequence = 7u;
  feedEncodedFrame(source, stop, 100u);
  feedEncodedFrame(source, stop, 200u);
  GS_ESP32_EXPECT_EQ(1, source.acceptedFrames());
  GS_ESP32_EXPECT_EQ(1, source.rejectedFrames());

  source.disconnect();
  ControlRequest request{};
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_NEAR(0.0, request.linear_velocity, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, request.yaw_rate, 0.0001);
  GS_ESP32_EXPECT_FALSE(request.disarm);
}

void testSerialCommandSourceClassifiesCorruptFrame() {
  SerialCommandSource source(50000u);
  SerialFrame stop{};
  stop.type = SerialMessageType::kStop;
  stop.sequence = 7u;
  uint8_t encoded[kSerialMaximumFrameSize] = {};
  const size_t length = SerialProtocol::encode(stop, encoded, sizeof(encoded));
  encoded[length - 1u] ^= 1u;
  SerialParseResult result = SerialParseResult::kIncomplete;
  for (size_t index = 0u; index < length; ++index) {
    result = source.feed(encoded[index], 100u + index);
  }
  GS_ESP32_EXPECT_EQ(SerialParseResult::kInvalidCrc, result);
  GS_ESP32_EXPECT_EQ(static_cast<uint8_t>(SerialErrorCode::kInvalidCrc),
                     source.lastErrorCode());
  GS_ESP32_EXPECT_EQ(1, source.rejectedFrames());
}

void testHelloStartsANewSequenceSession() {
  SerialCommandSource source(50000u);
  MovementCommand movement{};
  movement.linear_velocity_milli = 500;
  movement.lease_id = 99u;
  movement.lifetime_ms = 500u;
  SerialFrame move{};
  move.type = SerialMessageType::kSetVelocityAndYaw;
  move.sequence = 100u;
  move.payload_length = static_cast<uint16_t>(MovementCommandCodec::encode(
      movement, move.payload.data(), move.payload.size()));
  feedEncodedFrame(source, move, 100u);

  SerialFrame hello{};
  hello.type = SerialMessageType::kHello;
  hello.sequence = 0u;
  feedEncodedFrame(source, hello, 200u);
  SerialFrame status{};
  status.type = SerialMessageType::kStatus;
  status.sequence = 1u;
  feedEncodedFrame(source, status, 300u);

  GS_ESP32_EXPECT_EQ(3, source.acceptedFrames());
  GS_ESP32_EXPECT_EQ(0, source.rejectedFrames());
  GS_ESP32_EXPECT_EQ(SerialMessageType::kStatus, source.lastFrameType());
  ControlRequest request{};
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_NEAR(0.0, request.linear_velocity, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, request.yaw_rate, 0.0001);
}

void testSerialCommandSourceMapsSafetyCommands() {
  SerialCommandSource source(50000u);
  SerialFrame emergency{};
  emergency.type = SerialMessageType::kEmergencyStop;
  emergency.sequence = 1u;
  feedEncodedFrame(source, emergency, 100u);
  ControlRequest request{};
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_TRUE(request.emergency_stop);
  GS_ESP32_EXPECT_TRUE(request.disarm);
}

void testSerialCommandSourceAcceptsStatusWithoutMovement() {
  SerialCommandSource source(50000u);
  SerialFrame status{};
  status.type = SerialMessageType::kStatus;
  status.sequence = 12u;
  feedEncodedFrame(source, status, 100u);
  ControlRequest request{};
  GS_ESP32_EXPECT_FALSE(source.latest(request));
  GS_ESP32_EXPECT_EQ(1, source.acceptedFrames());
  GS_ESP32_EXPECT_EQ(12, source.lastFrameSequence());
  GS_ESP32_EXPECT_EQ(SerialMessageType::kStatus, source.lastFrameType());
}

void testSerialCommandSourceHandlesConfigurationShapesAndLeaseOwnership() {
  SerialCommandSource source(50000u);
  SerialFrame read{};
  read.type = SerialMessageType::kConfigurationRead;
  read.sequence = 1u;
  read.payload_length = 1u;
  read.payload[0] = static_cast<uint8_t>(BalanceConfigKey::kInnerProportional);
  feedEncodedFrame(source, read, 100u);
  GS_ESP32_EXPECT_EQ(1, source.acceptedFrames());

  SerialFrame malformed_update{};
  malformed_update.type = SerialMessageType::kConfigurationUpdate;
  malformed_update.sequence = 2u;
  malformed_update.payload_length = 1u;
  feedEncodedFrame(source, malformed_update, 200u);
  GS_ESP32_EXPECT_EQ(1, source.rejectedFrames());
  GS_ESP32_EXPECT_EQ(static_cast<uint8_t>(SerialErrorCode::kMalformed),
                     source.lastErrorCode());

  MovementCommand movement{};
  movement.linear_velocity_milli = 500;
  movement.lease_id = 10u;
  movement.lifetime_ms = 500u;
  SerialFrame command{};
  command.type = SerialMessageType::kSetVelocityAndYaw;
  command.sequence = 3u;
  command.payload_length = static_cast<uint16_t>(MovementCommandCodec::encode(
      movement, command.payload.data(), command.payload.size()));
  feedEncodedFrame(source, command, 1000u);

  movement.lease_id = 11u;
  command.sequence = 4u;
  command.payload_length = static_cast<uint16_t>(MovementCommandCodec::encode(
      movement, command.payload.data(), command.payload.size()));
  feedEncodedFrame(source, command, 2000u);
  GS_ESP32_EXPECT_EQ(static_cast<uint8_t>(SerialErrorCode::kLeaseConflict),
                     source.lastErrorCode());
  GS_ESP32_EXPECT_EQ(2, source.rejectedFrames());
}

void testSeparateVelocityAndYawCommandsPreserveOtherAxis() {
  SerialCommandSource source(50000u);
  MovementCommand movement{};
  movement.linear_velocity_milli = 500;
  movement.yaw_rate_milli = 250;
  movement.lease_id = 77u;
  movement.lifetime_ms = 500u;
  SerialFrame frame{};
  frame.type = SerialMessageType::kSetVelocityAndYaw;
  frame.sequence = 1u;
  frame.payload_length = static_cast<uint16_t>(MovementCommandCodec::encode(
      movement, frame.payload.data(), frame.payload.size()));
  feedEncodedFrame(source, frame, 1000u);

  movement.linear_velocity_milli = -300;
  movement.yaw_rate_milli = 999;
  frame.type = SerialMessageType::kSetLinearVelocity;
  frame.sequence = 2u;
  frame.payload_length = static_cast<uint16_t>(MovementCommandCodec::encode(
      movement, frame.payload.data(), frame.payload.size()));
  feedEncodedFrame(source, frame, 2000u);
  ControlRequest request{};
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_NEAR(-0.3, request.linear_velocity, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.25, request.yaw_rate, 0.0001);

  movement.linear_velocity_milli = 999;
  movement.yaw_rate_milli = -125;
  frame.type = SerialMessageType::kSetYawRate;
  frame.sequence = 3u;
  frame.payload_length = static_cast<uint16_t>(MovementCommandCodec::encode(
      movement, frame.payload.data(), frame.payload.size()));
  feedEncodedFrame(source, frame, 3000u);
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_NEAR(-0.3, request.linear_velocity, 0.0001);
  GS_ESP32_EXPECT_NEAR(-0.125, request.yaw_rate, 0.0001);
}

void testWebDisconnectExpiresMovementWithoutDisarmingBalance() {
  WebCommandSource source;
  source.movement(0.5f, -0.25f, 1000u);
  ControlRequest request{};
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_EQ(CommandSource::kWeb, request.source);
  GS_ESP32_EXPECT_NEAR(0.5, request.linear_velocity, 0.0001);
  source.disconnect();
  GS_ESP32_EXPECT_TRUE(source.latest(request));
  GS_ESP32_EXPECT_NEAR(0.0, request.linear_velocity, 0.0001);
  GS_ESP32_EXPECT_NEAR(0.0, request.yaw_rate, 0.0001);
  GS_ESP32_EXPECT_FALSE(request.disarm);
}

void testPulseTransportBudgetBoundsSustainableRate() {
  constexpr uint32_t worst_case_us =
      SwdTransportBudget::frameDurationUs(80u, 9u, 3u);
  constexpr uint32_t best_case_us =
      SwdTransportBudget::frameDurationUs(80u, 9u, 0u);
  GS_ESP32_EXPECT_EQ(15200, worst_case_us);
  GS_ESP32_EXPECT_EQ(6560, best_case_us);
  GS_ESP32_EXPECT_NEAR(65.789, SwdTransportBudget::maximumRateHz(worst_case_us),
                       0.001);
  GS_ESP32_EXPECT_TRUE(10.0f <
                       SwdTransportBudget::maximumRateHz(worst_case_us));
}

void testTransportMetricsModelSustainedCommandsAndLatency() {
  MotorTransportMetrics metrics;
  constexpr uint32_t kFrames = 10000u;
  for (uint32_t index = 0u; index < kFrames; ++index) {
    const uint64_t sent_us = static_cast<uint64_t>(index) * 20000u;
    const uint16_t sequence = static_cast<uint16_t>(index + 1u);
    metrics.recordTransmission(sent_us);
    metrics.beginCommand(sequence, sent_us);
    metrics.observe(sequence, 0u, 0u, sent_us + 1000u);
    metrics.observe(sequence, sequence, sequence, sent_us + 3000u);
  }
  const MotorTransportStatistics statistics = metrics.statistics();
  GS_ESP32_EXPECT_EQ(kFrames, statistics.transmitted_frames);
  GS_ESP32_EXPECT_EQ(kFrames, statistics.acknowledged_commands);
  GS_ESP32_EXPECT_EQ(kFrames, statistics.applied_commands);
  GS_ESP32_EXPECT_NEAR(50.005, statistics.transmit_rate_hz, 0.01);
  GS_ESP32_EXPECT_EQ(1000, statistics.maximum_ack_latency_us);
  GS_ESP32_EXPECT_EQ(3000, statistics.maximum_apply_latency_us);
  GS_ESP32_EXPECT_EQ(0, statistics.timeouts);
}

} // namespace

int main() {
  testMpuAddressDetectionAndRegisterConfiguration();
  testMpuRejectsInvalidIdentity();
  testMpuAcceptsMpu6500CompatibleIdentity();
  testMpuRejectsUnsafeOrAmbiguousConfiguration();
  testSensorByteDecodingAndAxisMapping();
  testGyroCalibrationCompletesOnlyWhileStationary();
  testDefaultCalibrationAllowsBoundedZeroRateOffset();
  testMpuCountersAndTimeout();
  testMpuDiagnosticsExposeCalibrationProgressAndBias();
  testComplementaryFilterConvergesAndRejectsInvalidDeltaTime();
  testControllerSaturationAntiWindupAndYawMix();
  testControllerStopsIntegratingWhenSaturationWouldIncrease();
  testControllerRejectsNonFiniteInputAndRuntimeDisablesOutput();
  testControllerSignsCorrectGeneratedTrace();
  testUprightOffsetAndImuSignDefineOneBalanceFrame();
  testGeneratedPendulumTraceReducesSmallPitchError();
  testEveryArmingGateAndActiveFaultDisablesOutput();
  testStateMachineArmDriveFallAndExplicitRearm();
  testStateMachineFaultDisablesAndRequiresClear();
  testStateMachineFaultsWhenImuIsUnavailableDuringCalibration();
  testStateMachineWaitsSafelyForImuCalibration();
  testDisarmedDiagnosticAllowsMissingMotorsButArmedBalanceDoesNot();
  testFaultCanClearToDiagnosticWithoutMotorsButCannotArm();
  testSerialLeaseWinsOverWebAndDisconnectZerosMovement();
  testEmergencyStopAndLocalDisarmOverrideLeases();
  testHigherPrioritySourceAndAnyEmergencyStopPreemptOwner();
  testDryRunCalculatesButNeverSendsNonzeroOutput();
  testDirectTransportCommandIsBoundedAndHonorsDryRun();
  testRuntimeResetsControllerAcrossDisabledTransition();
  testLoopMetricsReportRateJitterDeadlineAndAges();
  testSerialFrameRoundTripAndLittleEndianEncoding();
  testSerialParserRejectsCrcAndMalformedLength();
  testSerialParserTimesOutPartialFrame();
  testSerialParserRoundTripsEveryPayloadWidthAndRejectsSingleBitDamage();
  testSerialSequenceRejectsStaleAndAcceptsRollover();
  testMovementCommandLifetimeAndPayloadCodec();
  testBalanceConfigurationCodecValidatesAndUpdatesOneKey();
  testBinaryTelemetryPayloadsAndResponseQueueAreBounded();
  testEveryBinaryTelemetrySchemaHasDeterministicWidthAndOffsets();
  testProtocolDeclaresRequiredMessageTypes();
  testSerialCommandSourceProducesLeaseBoundRequest();
  testSerialCommandSourceAcceptsOnlyBoundedDirectMotorCommands();
  testSerialCommandSourceRejectsStaleSequenceAndDisconnectsSafely();
  testSerialCommandSourceClassifiesCorruptFrame();
  testHelloStartsANewSequenceSession();
  testSerialCommandSourceMapsSafetyCommands();
  testSerialCommandSourceAcceptsStatusWithoutMovement();
  testSerialCommandSourceHandlesConfigurationShapesAndLeaseOwnership();
  testSeparateVelocityAndYawCommandsPreserveOtherAxis();
  testWebDisconnectExpiresMovementWithoutDisarmingBalance();
  testPulseTransportBudgetBoundsSustainableRate();
  testTransportMetricsModelSustainedCommandsAndLatency();

  if (gs_esp32_tests_failed != 0u) {
    std::fprintf(stderr, "%u/%u ESP32 assertions failed\n",
                 gs_esp32_tests_failed, gs_esp32_tests_run);
    return 1;
  }
  std::printf("ESP32 balance tests: %u assertions passed\n",
              gs_esp32_tests_run);
  return 0;
}
