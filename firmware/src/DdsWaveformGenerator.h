#ifndef DDS_WAVEFORM_GENERATOR_H
#define DDS_WAVEFORM_GENERATOR_H

#include <Arduino.h>
#include <DdsOscillator.h>

class DdsWaveformGenerator {
private:
  DdsOscillator osc;
  float _minVal = 0.0f;
  float _maxVal = 0.0f;
  float _rangeVal = 0.0f;

public:
  DdsWaveformGenerator() = default;

  // Initialize frequency, sample rate, and target output value boundaries
  void begin(float freqHz, float sampleRate, float minVal, float maxVal) {
    _minVal = minVal;
    _maxVal = maxVal;
    _rangeVal = maxVal - minVal;
    osc.begin(freqHz, sampleRate, minVal, maxVal);
  }

  // Dynamically set frequency without resetting phase
  void setFrequency(float freqHz, float sampleRate) {
    osc.setFrequency(freqHz, sampleRate);
  }

  // Dynamically change range without resetting phase
  void setRange(float minVal, float maxVal) {
    _minVal = minVal;
    _maxVal = maxVal;
    _rangeVal = maxVal - minVal;
    osc.setRange(minVal, maxVal);
  }

  // Unified waveform lookup and calculation API (Sine, Square, Triangle)
  // All three waveforms share the same phase accumulator to ensure perfect phase alignment on switches.
  float next(uint8_t waveType) {
    switch (waveType) {
      case 0: // Sine Wave: DDS table lookup
        return osc.next();
      case 1: { // Square Wave: normalize phase and compare (starts high at p=0 for phase alignment)
        float p = osc.advance();
        return (p < 0.5f) ? _maxVal : _minVal;
      }
      case 2: { // Triangle Wave: shift phase by 90 degrees to align zero-crossing with sine
        float p = osc.advance();
        float pTri = p + 0.25f;
        if (pTri >= 1.0f) pTri -= 1.0f;
        float triNorm = 1.0f - fabsf(pTri * 2.0f - 1.0f);
        return _minVal + triNorm * _rangeVal;
      }
      default:
        return osc.next();
    }
  }
};

#endif // DDS_WAVEFORM_GENERATOR_H
