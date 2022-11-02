#pragma once

#include <Wire.h>
#include "wled.h"
#include <driver/i2s.h>
#include <driver/adc.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include <driver/adc_deprecated.h>
#include <driver/adc_types_deprecated.h>
#endif

//#include <driver/i2s_std.h>
//#include <driver/i2s_pdm.h>
//#include <driver/gpio.h>


/* ToDo: remove. ES7243 is controlled via compiler defines
   Until this configuration is moved to the webinterface
*/

// data type requested from the I2S driver - currently we always use 32bit
//#define I2S_USE_16BIT_SAMPLES   // (experimental) define this to request 16bit - more efficient but possibly less compatible
#ifdef I2S_USE_16BIT_SAMPLES
#define I2S_SAMPLE_RESOLUTION I2S_BITS_PER_SAMPLE_16BIT
#define I2S_datatype int16_t
#undef  I2S_SAMPLE_DOWNSCALE_TO_16BIT
#else
#define I2S_SAMPLE_RESOLUTION I2S_BITS_PER_SAMPLE_32BIT
#define I2S_datatype int32_t
#define I2S_SAMPLE_DOWNSCALE_TO_16BIT
#endif


/* Interface class
   AudioSource serves as base class for all microphone types
   This enables accessing all microphones with one single interface
   which simplifies the caller code
*/
class AudioSource {
  public:
    /* All public methods are virtual, so they can be overridden
       Everything but the destructor is also removed, to make sure each mic
       Implementation provides its version of this function
    */
    virtual ~AudioSource() {};

    /* Initialize
       This function needs to take care of anything that needs to be done
       before samples can be obtained from the microphone.
    */
    virtual void initialize(int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE) = 0;

    /* Deinitialize
       Release all resources and deactivate any functionality that is used
       by this microphone
    */
    virtual void deinitialize() = 0;

    /* getSamples
       Read num_samples from the microphone, and store them in the provided
       buffer
    */
    virtual void getSamples(float *buffer, uint16_t num_samples) = 0;

    /* Get an up-to-date sample without DC offset */
    virtual int getSampleWithoutDCOffset() { return _sampleNoDCOffset; };

  protected:
    // Private constructor, to make sure it is not callable except from derived classes
    AudioSource(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
      _sampleRate(sampleRate),
      _blockSize(blockSize),
      _sampleNoDCOffset(0),
      _dcOffset(0.0f),
      _shift(lshift),
      _mask(mask),
      _initialized(false)
    {};

    int _sampleRate;                // Microphone sampling rate
    int _blockSize;                 // I2S block size
    volatile int _sampleNoDCOffset; // Up-to-date sample without DCOffset
    float _dcOffset;                // Rolling average DC offset
    int16_t _shift;                 // Shift obtained samples to the right (positive) or left(negative) by this amount
    uint32_t _mask;                 // Bitmask for sample data after shifting. Bitmask 0X0FFF means that we need to convert 12bit ADC samples from unsigned to signed
    bool _initialized;              // Gets set to true if initialization is successful
};

/* Basic I2S microphone source
   All functions are marked virtual, so derived classes can replace them
*/
class I2SSource : public AudioSource {
  public:
    I2SSource(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
      AudioSource(sampleRate, blockSize, lshift, mask) {
      _config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = _sampleRate,
        .bits_per_sample = I2S_SAMPLE_RESOLUTION,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#else
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = _blockSize
      };
    }

    virtual void initialize(int8_t i2swsPin = I2S_PIN_NO_CHANGE, int8_t i2ssdPin = I2S_PIN_NO_CHANGE, int8_t i2sckPin = I2S_PIN_NO_CHANGE, int8_t mclkPin = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE) {
      if (i2swsPin != I2S_PIN_NO_CHANGE && i2ssdPin != I2S_PIN_NO_CHANGE) {
        if (!pinManager.allocatePin(i2swsPin, true, PinOwner::UM_Audioreactive) ||
            !pinManager.allocatePin(i2ssdPin, false, PinOwner::UM_Audioreactive)) { // #206
          return;
        }
      }

      // i2ssckPin needs special treatment, since it might be unused on PDM mics
      if (i2sckPin != I2S_PIN_NO_CHANGE) {
        if (!pinManager.allocatePin(i2sckPin, true, PinOwner::UM_Audioreactive)) return;
      } else {
        // This is an I2S PDM microphone, these microphones only use a clock and
        // data line, to make it simpler to debug, use the WS pin as CLK and SD
        // pin as DATA
        _config.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM); // Change mode to pdm if clock pin not provided
      }

      // Reserve the master clock pin if provided
      _mclkPin = mclkPin;
      if (mclkPin != I2S_PIN_NO_CHANGE) {
        if(!pinManager.allocatePin(mclkPin, true, PinOwner::UM_Audioreactive)) return;
        _routeMclk(mclkPin);
      }

      _pinConfig = {
        .bck_io_num = i2sckPin,
        .ws_io_num = i2swsPin,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = i2ssdPin
      };

      esp_err_t err = i2s_driver_install(I2S_NUM_0, &_config, 0, nullptr);
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to install i2s driver: %d\n", err);
        return;
      }

      err = i2s_set_pin(I2S_NUM_0, &_pinConfig);
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to set i2s pin config: %d\n", err);
        return;
      }

      _initialized = true;
    }

    virtual void deinitialize() {
      _initialized = false;
      esp_err_t err = i2s_driver_uninstall(I2S_NUM_0);
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to uninstall i2s driver: %d\n", err);
        return;
      }
      if (_pinConfig.ws_io_num   != I2S_PIN_NO_CHANGE) pinManager.deallocatePin(_pinConfig.ws_io_num,   PinOwner::UM_Audioreactive);
      if (_pinConfig.data_in_num != I2S_PIN_NO_CHANGE) pinManager.deallocatePin(_pinConfig.data_in_num, PinOwner::UM_Audioreactive);
      if (_pinConfig.bck_io_num  != I2S_PIN_NO_CHANGE) pinManager.deallocatePin(_pinConfig.bck_io_num,  PinOwner::UM_Audioreactive);
      // Release the master clock pin
      if (_mclkPin != I2S_PIN_NO_CHANGE) pinManager.deallocatePin(_mclkPin, PinOwner::UM_Audioreactive);
    }

    virtual void getSamples(float *buffer, uint16_t num_samples) {
      if (_initialized) {
        esp_err_t err;
        size_t bytes_read = 0;        /* Counter variable to check if we actually got enough data */
        I2S_datatype newSamples[num_samples]; /* Intermediary sample storage */

        // Reset dc offset
        _dcOffset = 0.0f;

        err = i2s_read(I2S_NUM_0, (void *)newSamples, sizeof(newSamples), &bytes_read, portMAX_DELAY);
        if (err != ESP_OK) {
          DEBUGSR_PRINTF("Failed to get samples: %d\n", err);
          return;
        }

        // For correct operation, we need to read exactly sizeof(samples) bytes from i2s
        if (bytes_read != sizeof(newSamples)) {
          DEBUGSR_PRINTF("Failed to get enough samples: wanted: %d read: %d\n", sizeof(newSamples), bytes_read);
          return;
        }

        // Store samples in sample buffer and update DC offset
        for (int i = 0; i < num_samples; i++) {
          // pre-shift samples down to 16bit
#ifdef I2S_SAMPLE_DOWNSCALE_TO_16BIT
          if (_shift != 0)
              newSamples[i] >>= 16;
#endif
          float currSample = 0.0f;
          if(_shift > 0)
            currSample = (float) (newSamples[i] >> _shift);
          else {
            if(_shift < 0)
              currSample = (float) (newSamples[i] << (- _shift)); // need to "pump up" 12bit ADC to full 16bit as delivered by other digital mics
            else
#ifdef I2S_SAMPLE_DOWNSCALE_TO_16BIT
              currSample = (float) newSamples[i] / 65536.0f;      // _shift == 0 -> use the chance to keep lower 16bits
#else
              currSample = (float) newSamples[i];
#endif
          }
          buffer[i] = currSample;
          _dcOffset = ((_dcOffset * 31) + currSample) / 32;
        }

        // Update no-DC sample
        _sampleNoDCOffset = buffer[num_samples - 1] - _dcOffset;
      }
    }

  protected:
    void _routeMclk(int8_t mclkPin) {
      /* Enable the mclk routing depending on the selected mclk pin
          Only I2S_NUM_0 is supported
      */
      if (mclkPin == GPIO_NUM_0) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
        WRITE_PERI_REG(PIN_CTRL,0xFFF0);
      } else if (mclkPin == GPIO_NUM_1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
        WRITE_PERI_REG(PIN_CTRL, 0xF0F0);
      } else {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
        WRITE_PERI_REG(PIN_CTRL, 0xFF00);
      }
    }

    i2s_config_t _config;
    i2s_pin_config_t _pinConfig;
    int8_t _mclkPin;
};

/* ES7243 Microphone
   This is an I2S microphone that requires ininitialization over
   I2C before I2S data can be received
*/
class ES7243 : public I2SSource {
  private:
    // I2C initialization functions for ES7243
    void _es7243I2cBegin() {
      Wire.begin(pin_ES7243_SDA, pin_ES7243_SCL, 100000U);
    }

    void _es7243I2cWrite(uint8_t reg, uint8_t val) {
#ifndef ES7243_ADDR
      Wire.beginTransmission(0x13);
#else
      Wire.beginTransmission(ES7243_ADDR);
#endif
      Wire.write((uint8_t)reg);
      Wire.write((uint8_t)val);
      Wire.endTransmission();
    }

    void _es7243InitAdc() {
      _es7243I2cBegin();
      _es7243I2cWrite(0x00, 0x01);
      _es7243I2cWrite(0x06, 0x00);
      _es7243I2cWrite(0x05, 0x1B);
      _es7243I2cWrite(0x01, 0x00); // 0x00 for 24 bit to match INMP441 - not sure if this needs adjustment to get 16bit samples from I2S
      _es7243I2cWrite(0x08, 0x43);
      _es7243I2cWrite(0x05, 0x13);
    }

public:
    ES7243(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
      I2SSource(sampleRate, blockSize, lshift, mask) {
      _config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
    };

    void initialize(int8_t sdaPin, int8_t sclPin, int8_t i2swsPin, int8_t i2ssdPin, int8_t i2sckPin, int8_t mclkPin) {
      // Reserve SDA and SCL pins of the I2C interface
      if (!pinManager.allocatePin(sdaPin, true, PinOwner::HW_I2C) ||
          !pinManager.allocatePin(sclPin, true, PinOwner::HW_I2C)) {
        return;
      }

      pin_ES7243_SDA = sdaPin;
      pin_ES7243_SCL = sclPin;

      // First route mclk, then configure ADC over I2C, then configure I2S
      _es7243InitAdc();
      I2SSource::initialize(i2swsPin, i2ssdPin, i2sckPin, mclkPin);
    }

    void deinitialize() {
      // Release SDA and SCL pins of the I2C interface
      pinManager.deallocatePin(pin_ES7243_SDA, PinOwner::HW_I2C);
      pinManager.deallocatePin(pin_ES7243_SCL, PinOwner::HW_I2C);
      I2SSource::deinitialize();
    }

  private:
    int8_t pin_ES7243_SDA;
    int8_t pin_ES7243_SCL;
};

/* ADC over I2S Microphone
   This microphone is an ADC pin sampled via the I2S interval
   This allows to use the I2S API to obtain ADC samples with high sample rates
   without the need of manual timing of the samples
*/
class I2SAdcSource : public I2SSource {
  public:
    I2SAdcSource(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
      I2SSource(sampleRate, blockSize, lshift, mask) {
      _config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = _sampleRate,
        .bits_per_sample = I2S_SAMPLE_RESOLUTION,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
#else
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 8,
        .dma_buf_len = _blockSize
      };
    }

    void initialize(int8_t audioPin, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE) {
      if(!pinManager.allocatePin(audioPin, false, PinOwner::UM_Audioreactive)) {
        return;
      }
      _audioPin = audioPin;

      // Determine Analog channel. Only Channels on ADC1 are supported
      int8_t channel = digitalPinToAnalogChannel(_audioPin);
      if (channel > 9) {
        DEBUGSR_PRINTF("Incompatible GPIO used for audio in: %d\n", _audioPin);
        return;
      } else {
        adc_gpio_init(ADC_UNIT_1, adc_channel_t(channel));
      }

      // Install Driver
      esp_err_t err = i2s_driver_install(I2S_NUM_0, &_config, 0, nullptr);
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to install i2s driver: %d\n", err);
        return;
      }

      // Enable I2S mode of ADC
      err = i2s_set_adc_mode(ADC_UNIT_1, adc1_channel_t(channel));
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to set i2s adc mode: %d\n", err);
        return;
      }

#if defined(ARDUINO_ARCH_ESP32)
      // according to docs from espressif, the ADC needs to be started explicitly
      // fingers crossed
      err = i2s_adc_enable(I2S_NUM_0);
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to enable i2s adc: %d\n", err);
        //return;
      }
#endif

      _initialized = true;
    }

    void getSamples(float *buffer, uint16_t num_samples) {
      /* Enable ADC. This has to be enabled and disabled directly before and
       * after sampling, otherwise Wifi dies
       */
      if (_initialized) {
#if !defined(ARDUINO_ARCH_ESP32)
        // old code - works for me without enable/disable, at least on ESP32.
        esp_err_t err = i2s_adc_enable(I2S_NUM_0);
        //esp_err_t err = i2s_start(I2S_NUM_0);
        if (err != ESP_OK) {
          DEBUGSR_PRINTF("Failed to enable i2s adc: %d\n", err);
          return;
        }
#endif
        I2SSource::getSamples(buffer, num_samples);

#if !defined(ARDUINO_ARCH_ESP32)
        // old code - works for me without enable/disable, at least on ESP32.
        err = i2s_adc_disable(I2S_NUM_0);
        //err = i2s_stop(I2S_NUM_0);
        if (err != ESP_OK) {
          DEBUGSR_PRINTF("Failed to disable i2s adc: %d\n", err);
          return;
        }
#endif
      }
    }

    void deinitialize() {
      pinManager.deallocatePin(_audioPin, PinOwner::UM_Audioreactive);
      _initialized = false;
      esp_err_t err;
#if defined(ARDUINO_ARCH_ESP32)
      // according to docs from espressif, the ADC needs to be stopped explicitly
      // fingers crossed
      err = i2s_adc_disable(I2S_NUM_0);
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to disable i2s adc: %d\n", err);
        //return;
      }
#endif
      err = i2s_driver_uninstall(I2S_NUM_0);
      if (err != ESP_OK) {
        DEBUGSR_PRINTF("Failed to uninstall i2s driver: %d\n", err);
        return;
      }
    }

  private:
    int8_t _audioPin;
};

/* SPH0645 Microphone
   This is an I2S microphone with some timing quirks that need
   special consideration.
*/
class SPH0654 : public I2SSource {
  public:
    SPH0654(int sampleRate, int blockSize, int16_t lshift, uint32_t mask) :
      I2SSource(sampleRate, blockSize, lshift, mask)
    {}

    void initialize(uint8_t i2swsPin, uint8_t i2ssdPin, uint8_t i2sckPin, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE, int8_t = I2S_PIN_NO_CHANGE) {
      I2SSource::initialize(i2swsPin, i2ssdPin, i2sckPin);
      REG_SET_BIT(I2S_TIMING_REG(I2S_NUM_0), BIT(9));
      REG_SET_BIT(I2S_CONF_REG(I2S_NUM_0), I2S_RX_MSB_SHIFT);
    }
};
