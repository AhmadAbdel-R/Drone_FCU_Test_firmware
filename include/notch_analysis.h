#pragma once

// =============================================================================
// NotchAnalyzer — bounded gyro capture + FFT for the dashboard's live notch /
// vibration analysis. NOT part of the control loop:
//   * The flight task only APPENDS samples (tick(), a few float stores) while a
//     capture is active — no FFT, no allocation, no blocking.
//   * The actual FFT runs in analyze(), called from loop() (core 1, prio 1,
//     far below the 500 Hz flight task) exactly once after the buffer fills.
//   * The dashboard reads cached magnitude spectra + a peak-based notch
//     recommendation.
//
// Two stages are captured per axis so the dashboard can show how much vibration
// the filters remove: stage 0 = RAW gyro (pre-notch), stage 1 = FINAL gyro
// (post dynamic-notch, i.e. what the rate PID actually sees).
//
// FFT: in-place iterative radix-2 Cooley-Tukey, Hann window, DC removed. kN is a
// power of two; at the ~500 Hz loop rate, kN=256 gives ~1.95 Hz bins to a 250 Hz
// Nyquist — ample for motor-vibration notch work. (Short names like N collide
// with library macros, hence the k-prefixed constants.)
// =============================================================================

#include <Arduino.h>
#include <math.h>
#include <atomic>

class NotchAnalyzer {
 public:
  static constexpr uint16_t kN = 256;        // FFT length (power of two)
  static constexpr uint16_t kBins = kN / 2;  // usable bins (DC..Nyquist-1)
  static constexpr uint8_t kSig = 6;         // raw x/y/z (0..2), final x/y/z (3..5)

  void start() {
    count_ = 0;
    ready_.store(false, std::memory_order_relaxed);
    full_.store(false, std::memory_order_relaxed);
    firstUs_ = lastUs_ = 0;
    active_.store(true, std::memory_order_release);
  }
  void stop() { active_.store(false, std::memory_order_relaxed); }
  void clear() {
    active_.store(false, std::memory_order_relaxed);
    full_.store(false, std::memory_order_relaxed);
    ready_.store(false, std::memory_order_relaxed);
    count_ = 0;
  }

  bool active() const { return active_.load(std::memory_order_relaxed); }
  bool ready() const { return ready_.load(std::memory_order_acquire); }
  uint16_t progress() const { return count_; }
  float effectiveHz() const { return effHz_; }
  float hzPerBin() const { return (effHz_ > 0.0f) ? (effHz_ / static_cast<float>(kN)) : 0.0f; }

  // Flight task: append one sample set. Cheap (6 float stores). raw[] is the
  // pre-notch gyro, fin[] the post-notch gyro. nowUs timestamps the sample so
  // the effective sample rate is measured rather than assumed.
  void tick(const float raw[3], const float fin[3], uint32_t nowUs) {
    if (!active_.load(std::memory_order_relaxed)) return;
    const uint16_t i = count_;
    if (i >= kN) return;
    if (i == 0) firstUs_ = nowUs;
    lastUs_ = nowUs;
    buf_[0][i] = raw[0]; buf_[1][i] = raw[1]; buf_[2][i] = raw[2];
    buf_[3][i] = fin[0]; buf_[4][i] = fin[1]; buf_[5][i] = fin[2];
    count_ = i + 1;
    if (count_ >= kN) {
      full_.store(true, std::memory_order_release);
      active_.store(false, std::memory_order_relaxed);
    }
  }

  bool needsAnalyze() const {
    return full_.load(std::memory_order_acquire) && !ready_.load(std::memory_order_acquire);
  }

  // loop() context (low priority). Computes all six spectra + the recommendation.
  void analyze() {
    if (!full_.load(std::memory_order_acquire)) return;
    const float dtTot = (lastUs_ > firstUs_) ? static_cast<float>(lastUs_ - firstUs_) * 1e-6f : 0.0f;
    effHz_ = (dtTot > 0.0f) ? (static_cast<float>(kN - 1) / dtTot) : 0.0f;
    for (uint8_t s = 0; s < kSig; ++s) computeOne(s);

    // Peak detection on the RAW axes (0..2): the dominant motor-vibration line.
    const float hpb = hzPerBin();
    const uint16_t minBin = (hpb > 0.0f) ? static_cast<uint16_t>(8.0f / hpb + 1.0f) : 4;  // ignore <8 Hz
    float bestMag = 0.0f;
    for (uint8_t ax = 0; ax < 3; ++ax) {
      uint16_t pk = minBin; float pkMag = 0.0f;
      double sum = 0.0; uint16_t nn = 0;
      for (uint16_t k = minBin; k < kBins; ++k) {
        sum += mag_[ax][k]; ++nn;
        if (mag_[ax][k] > pkMag) { pkMag = mag_[ax][k]; pk = k; }
      }
      const float floorMag = (nn > 0) ? static_cast<float>(sum / nn) : 0.0f;
      peakHz_[ax] = pk * hpb;
      peakMag_[ax] = pkMag;
      noiseFloor_[ax] = floorMag;
      // half-power width for the Q estimate
      const float half = pkMag * 0.5f;
      uint16_t lo = pk, hi = pk;
      while (lo > minBin && mag_[ax][lo] > half) --lo;
      while (hi < kBins - 1 && mag_[ax][hi] > half) ++hi;
      const float bwHz = (hi - lo) * hpb;
      qEst_[ax] = (bwHz > 0.1f) ? (peakHz_[ax] / bwHz) : 6.0f;
      const float snr = (floorMag > 1e-6f) ? (pkMag / floorMag) : 0.0f;
      const float conf = snr <= 1.0f ? 0.0f : fminf(1.0f, (snr - 1.0f) / 9.0f);  // SNR 1..10 -> 0..1
      if (pkMag > bestMag) { bestMag = pkMag; recCenterHz_ = peakHz_[ax]; recConfidence_ = conf; recQ_ = qEst_[ax]; }
    }
    if (recQ_ < 1.5f) recQ_ = 1.5f;
    if (recQ_ > 10.0f) recQ_ = 10.0f;
    ready_.store(true, std::memory_order_release);
  }

  // Copy up to `bins` magnitudes of one spectrum into out[]. stage 0=raw,1=final.
  // Returns bin count written; sets hpb to Hz-per-bin.
  uint16_t getSpectrum(uint8_t axis, uint8_t stage, float* out, uint16_t bins, float& hpb) const {
    hpb = hzPerBin();
    if (axis > 2 || stage > 1 || out == nullptr) return 0;
    const uint8_t s = stage * 3 + axis;
    const uint16_t nOut = (bins < kBins) ? bins : kBins;
    for (uint16_t k = 0; k < nOut; ++k) out[k] = mag_[s][k];
    return nOut;
  }

  float recCenterHz() const { return recCenterHz_; }
  float recQ() const { return recQ_; }
  float recConfidence() const { return recConfidence_; }
  float peakHz(uint8_t ax) const { return ax < 3 ? peakHz_[ax] : 0.0f; }
  float peakMag(uint8_t ax) const { return ax < 3 ? peakMag_[ax] : 0.0f; }
  float noiseFloor(uint8_t ax) const { return ax < 3 ? noiseFloor_[ax] : 0.0f; }

 private:
  void computeOne(uint8_t s) {
    // DC removal (subtract mean), Hann window, into re/im scratch.
    double mean = 0.0;
    for (uint16_t i = 0; i < kN; ++i) mean += buf_[s][i];
    mean /= kN;
    for (uint16_t i = 0; i < kN; ++i) {
      const float w = 0.5f * (1.0f - cosf(2.0f * static_cast<float>(M_PI) * i / (kN - 1)));
      re_[i] = (buf_[s][i] - static_cast<float>(mean)) * w;
      im_[i] = 0.0f;
    }
    fft(re_, im_);
    // Magnitude, single-sided, normalized by kN (Hann coherent gain ~0.5 -> x2).
    const float scale = 2.0f / (0.5f * kN);
    for (uint16_t k = 0; k < kBins; ++k) {
      mag_[s][k] = sqrtf(re_[k] * re_[k] + im_[k] * im_[k]) * scale;
    }
  }

  // In-place iterative radix-2 FFT (decimation-in-time).
  static void fft(float* re, float* im) {
    for (uint16_t i = 1, j = 0; i < kN; ++i) {  // bit-reversal permutation
      uint16_t bit = kN >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) { float tr = re[i]; re[i] = re[j]; re[j] = tr;
                   float ti = im[i]; im[i] = im[j]; im[j] = ti; }
    }
    for (uint16_t len = 2; len <= kN; len <<= 1) {
      const float ang = -2.0f * static_cast<float>(M_PI) / len;
      const float wpr = cosf(ang), wpi = sinf(ang);
      for (uint16_t i = 0; i < kN; i += len) {
        float wr = 1.0f, wi = 0.0f;
        for (uint16_t k = 0; k < (len >> 1); ++k) {
          const uint16_t a = i + k, b = i + k + (len >> 1);
          const float xr = re[b] * wr - im[b] * wi;
          const float xi = re[b] * wi + im[b] * wr;
          re[b] = re[a] - xr; im[b] = im[a] - xi;
          re[a] += xr; im[a] += xi;
          const float tmp = wr;
          wr = wr * wpr - wi * wpi;
          wi = tmp * wpi + wi * wpr;
        }
      }
    }
  }

  float buf_[kSig][kN];          // capture (raw xyz, final xyz)
  float mag_[kSig][kBins];       // cached magnitude spectra
  float re_[kN], im_[kN];        // FFT scratch (reused per signal)
  volatile uint16_t count_ = 0;
  uint32_t firstUs_ = 0, lastUs_ = 0;
  float effHz_ = 0.0f;
  float recCenterHz_ = 0.0f, recQ_ = 6.0f, recConfidence_ = 0.0f;
  float peakHz_[3] = {0, 0, 0}, peakMag_[3] = {0, 0, 0}, noiseFloor_[3] = {0, 0, 0}, qEst_[3] = {6, 6, 6};
  std::atomic<bool> active_{false};
  std::atomic<bool> full_{false};
  std::atomic<bool> ready_{false};
};
