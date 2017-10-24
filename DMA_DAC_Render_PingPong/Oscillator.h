/* Oscillator.h
    Simplified floating poing sine wave table oscillator.
*/

#ifndef OSCILLATOR_H
#define OSCILLATOR_H

#include <math.h>
#include <stdint.h>

const uint16_t TAB_LEN = 2048;
const float M_2PI = 6.2831853071795862319959f;

class Oscillator {

  public:

    // Initialize with specified sample rate and fundamental frequency
    Oscillator(float sample_rate, float f0) : fs(sample_rate), idx(0) {
      table_init();
      setF0(f0);
    }

    // Set fundamental frequency in Hz, within the range (0, fs/2)
    void setF0(float f0_Hz) {
      idx_inc = TAB_LEN * f0_Hz / fs;     // Map (0, fs/2) to (0, TAB_LEN)
    }

    // Render one sample
    float render() {
      idx += idx_inc;
      if (idx >= TAB_LEN)
        idx = idx - TAB_LEN;
      return wavetable[(int)idx];
    }

  private:

    // Initialize one period of the wave table
    void table_init() {
      for (int i = 0; i < TAB_LEN; ++i)
        wavetable[i] = sin(M_2PI * i / TAB_LEN);
    }

    float wavetable[TAB_LEN];   // Wave table
    float idx;                  // Current wave table index
    float idx_inc;              // Wave table index increment
    float f0;                   // Fundamental freq. value
    float fs;                   // Audio sample rate
};

#endif
