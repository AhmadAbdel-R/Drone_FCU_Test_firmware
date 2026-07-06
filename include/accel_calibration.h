#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

// =============================================================================
// SixFaceAccelCalibrator — six-position accelerometer zero + gain calibration.
//
// CLEAN-ROOM, INAV-inspired behavior. The craft is placed on each of its six
// faces (±X, ±Y, ±Z up); on each face the calibrator waits for stillness,
// averages N raw samples, and files the mean into the matching face bin —
// the face is AUTO-DETECTED from the dominant gravity axis, so the operator
// can present faces in any order.
//
// THE MATH
//   On face +A the sensor reads  raw_A = zero_A + 1g/gain_A  (its own axis),
//   on face −A it reads          raw_A = zero_A − 1g/gain_A.
//   Averaging the two isolates the bias, differencing isolates the scale:
//     zero_A = (mean(+A) + mean(−A)) / 2
//     gain_A = 2g / (mean(+A) − mean(−A))
//   Corrected output = (raw − zero) · gain, which maps ±1g exactly onto the
//   two calibration poses per axis. (This is the diagonal-only model — no
//   cross-axis terms; adequate for MEMS accelerometers at multirotor rates.)
//
// REJECTION
//   * A face capture aborts on motion (gyro magnitude or accel-norm window).
//   * finish() refuses unless all six bins are filled, and range-checks the
//     result (|zero| ≤ 0.5 g, gain within [0.8, 1.2]) so a mislabeled or
//     shaken capture can never persist.
//
// THREADING: owned by the flight task (tick() at loop rate). The web side
// requests captures / reads status through the owner's mutex in main.cpp.
// =============================================================================

namespace gnc {

class SixFaceAccelCalibrator {
 public:
  // Face bins by dominant axis: 0:+X 1:-X 2:+Y 3:-Y 4:+Z 5:-Z
  static constexpr uint8_t kFaceCount = 6;

  enum class State : uint8_t {
    IDLE = 0,        // no session
    WAIT_STILL,      // capture requested; waiting for stillness on a face
    SAMPLING,        // averaging raw samples
    FACE_DONE,       // last capture stored; waiting for next request/finish
    FACE_FAILED,     // last capture failed (moved / timeout / ambiguous)
  };

  struct Config {
    uint16_t samplesPerFace = 200;   // ~0.4 s at 500 Hz
    float gyroStillDps = 3.0f;       // stillness: gyro magnitude below this
    float normMinG = 0.85f;          // ...and accel norm within this window
    float normMaxG = 1.15f;
    float axisDominanceG = 0.75f;    // face detect: |axis| must exceed this
    uint32_t stillHoldMs = 400;      // continuous stillness before sampling
    uint32_t captureTimeoutMs = 15000;
  };

  struct Result {
    float zero[3] = {0, 0, 0};
    float gain[3] = {1, 1, 1};
    bool valid = false;
  };

  void configure(const Config& c) { cfg_ = c; }

  void beginSession() {
    for (auto& f : faceDone_) f = false;
    state_ = State::IDLE;
    lastError_[0] = '\0';
  }

  // Operator asks to capture the CURRENT pose. Face is auto-detected during
  // stillness. Returns false if a capture is already running.
  bool requestCapture(uint32_t nowMs) {
    if (state_ == State::WAIT_STILL || state_ == State::SAMPLING) return false;
    state_ = State::WAIT_STILL;
    captureStartMs_ = nowMs;
    stillSinceMs_ = 0;
    sampleCount_ = 0;
    activeFace_ = 0xFF;
    return true;
  }

  void cancel() {
    if (state_ == State::WAIT_STILL || state_ == State::SAMPLING) state_ = State::IDLE;
  }

  // Feed one RAW (uncorrected) body-frame accel sample + the gyro magnitude.
  // Call at loop rate; cheap no-op unless a capture is pending.
  void tick(float rawAxG, float rawAyG, float rawAzG, float gyroMagDps, uint32_t nowMs) {
    if (state_ != State::WAIT_STILL && state_ != State::SAMPLING) return;

    if ((nowMs - captureStartMs_) > cfg_.captureTimeoutMs) {
      fail("timeout waiting for a still, face-flat pose");
      return;
    }
    const float norm = sqrtf(rawAxG * rawAxG + rawAyG * rawAyG + rawAzG * rawAzG);
    const bool still = gyroMagDps < cfg_.gyroStillDps && norm >= cfg_.normMinG &&
                       norm <= cfg_.normMaxG;

    if (state_ == State::WAIT_STILL) {
      if (!still) {
        stillSinceMs_ = 0;
        return;
      }
      if (stillSinceMs_ == 0) stillSinceMs_ = nowMs;
      if ((nowMs - stillSinceMs_) < cfg_.stillHoldMs) return;
      // Still long enough — detect the face from the dominant axis.
      const uint8_t face = detectFace(rawAxG, rawAyG, rawAzG);
      if (face == 0xFF) {
        fail("no dominant axis — place the craft flat on one face");
        return;
      }
      activeFace_ = face;
      sum_[0] = sum_[1] = sum_[2] = 0.0f;
      sampleCount_ = 0;
      state_ = State::SAMPLING;
      return;
    }

    // SAMPLING
    if (!still) {
      fail("moved during capture");
      return;
    }
    // The pose must stay on the SAME face for the whole capture.
    if (detectFace(rawAxG, rawAyG, rawAzG) != activeFace_) {
      fail("face changed during capture");
      return;
    }
    sum_[0] += rawAxG;
    sum_[1] += rawAyG;
    sum_[2] += rawAzG;
    if (++sampleCount_ >= cfg_.samplesPerFace) {
      const float inv = 1.0f / static_cast<float>(sampleCount_);
      faceMean_[activeFace_][0] = sum_[0] * inv;
      faceMean_[activeFace_][1] = sum_[1] * inv;
      faceMean_[activeFace_][2] = sum_[2] * inv;
      faceDone_[activeFace_] = true;
      state_ = State::FACE_DONE;
    }
  }

  // All six faces captured -> solve zero/gain per axis + range-check.
  bool finish(Result& out) {
    for (bool f : faceDone_) {
      if (!f) {
        fail("not all six faces captured");
        return false;
      }
    }
    Result r;
    for (int a = 0; a < 3; ++a) {
      const float pos = faceMean_[a * 2][a];      // +axis face, that axis
      const float neg = faceMean_[a * 2 + 1][a];  // -axis face
      const float span = pos - neg;               // should be ~2 g
      if (span < 1.0f) {
        fail("axis span too small — captures inconsistent");
        return false;
      }
      r.zero[a] = (pos + neg) * 0.5f;
      r.gain[a] = 2.0f / span;
      if (fabsf(r.zero[a]) > 0.5f || r.gain[a] < 0.8f || r.gain[a] > 1.2f) {
        fail("result out of range — recapture (vibration / wrong pose?)");
        return false;
      }
    }
    r.valid = true;
    out = r;
    return true;
  }

  State state() const { return state_; }
  uint8_t activeFace() const { return activeFace_; }
  uint8_t facesDoneMask() const {
    uint8_t m = 0;
    for (int i = 0; i < kFaceCount; ++i) {
      if (faceDone_[i]) m |= static_cast<uint8_t>(1 << i);
    }
    return m;
  }
  uint16_t sampleCount() const { return sampleCount_; }
  uint16_t samplesPerFace() const { return cfg_.samplesPerFace; }
  const char* lastError() const { return lastError_; }

 private:
  // 0:+X 1:-X 2:+Y 3:-Y 4:+Z 5:-Z, or 0xFF when no axis dominates.
  uint8_t detectFace(float ax, float ay, float az) const {
    const float v[3] = {ax, ay, az};
    int best = 0;
    for (int i = 1; i < 3; ++i) {
      if (fabsf(v[i]) > fabsf(v[best])) best = i;
    }
    if (fabsf(v[best]) < cfg_.axisDominanceG) return 0xFF;
    return static_cast<uint8_t>(best * 2 + (v[best] > 0.0f ? 0 : 1));
  }

  void fail(const char* why) {
    strncpy(lastError_, why, sizeof(lastError_) - 1);
    lastError_[sizeof(lastError_) - 1] = '\0';
    state_ = State::FACE_FAILED;
  }

  Config cfg_;
  State state_ = State::IDLE;
  uint8_t activeFace_ = 0xFF;
  bool faceDone_[kFaceCount] = {false};
  float faceMean_[kFaceCount][3] = {{0}};
  float sum_[3] = {0, 0, 0};
  uint16_t sampleCount_ = 0;
  uint32_t captureStartMs_ = 0;
  uint32_t stillSinceMs_ = 0;
  char lastError_[64] = {};
};

}  // namespace gnc
