#include <Arduino.h>
#include <driver/i2s_std.h>
#include <FastLED.h>
#include "arduinoFFT.h"

#define LED_PIN        32
#define NUM_LEDS       60
#define BRIGHTNESS     120
#define LED_TYPE       WS2812B
#define COLOR_ORDER    GRB

CRGB leds[NUM_LEDS];

#define SAMPLES 256
#define SAMPLING_FREQUENCY 10000
#define BANDS 3

#define I2S_MIC_SERIAL_CLOCK      GPIO_NUM_26
#define I2S_MIC_LEFT_RIGHT_CLOCK  GPIO_NUM_22
#define I2S_MIC_SERIAL_DATA       GPIO_NUM_21

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

i2s_chan_handle_t rx_handle;

void setup() {
  Serial.begin(115200);
  delay(1000);

  //LED init
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  //I2S init
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLING_FREQUENCY),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = I2S_MIC_SERIAL_CLOCK,
          .ws = I2S_MIC_LEFT_RIGHT_CLOCK,
          .dout = I2S_GPIO_UNUSED,
          .din = I2S_MIC_SERIAL_DATA
      }
  };

  i2s_channel_init_std_mode(rx_handle, &std_cfg);
  i2s_channel_enable(rx_handle);
}

void loop() {
  //сбор сэмплов
  int32_t samples_read = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int32_t sample = 0;
    size_t bytes_read = 0;
    i2s_channel_read(rx_handle, &sample, sizeof(sample), &bytes_read, portMAX_DELAY);
    vReal[i] = (double)sample / 100000.0 * 2.0;
    vImag[i] = 0;
  }

  //FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  double bands_ampl[BANDS] = {0};
  int bands_weight[BANDS] = {0};

  for (int i = 0; i < (SAMPLES / 2); i++) {
    double freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;

    if (freq > 0 && freq <= 300) {           // низкие частоты
      bands_ampl[0] += vReal[i];
      bands_weight[0]++;
    } else if (freq > 300 && freq <= 2000) { // средние частоты
      bands_ampl[1] += vReal[i];
      bands_weight[1]++;
    } else if (freq > 2000 && freq <= 5000) { // высокие частоты
      bands_ampl[2] += vReal[i];
      bands_weight[2]++;
    }
  }

  for (int i = 0; i < BANDS; i++) {
    if (bands_weight[i] > 0) {
      bands_ampl[i] /= bands_weight[i];
    }
  }

  //визуализация
  FastLED.clear();

  int lowLevel  = constrain((int)(bands_ampl[0] / 400.0), 0, 20);
  int midLevel  = constrain((int)(bands_ampl[1] / 400.0), 0, 20);
  int highLevel = constrain((int)(bands_ampl[2] / 400.0), 0, 20);

  for (int i = 0; i < lowLevel; i++) {
    leds[i] = CRGB::Red;
  }
  for (int i = 20; i < 20 + midLevel && i < 40; i++) {
    leds[i] = CRGB::Green;
  }
  for (int i = 40; i < 40 + highLevel && i < 60; i++) {
    leds[i] = CRGB::Blue;
  }

  FastLED.show();

  //для дебага
  //Serial.printf("Low: %d  Mid: %d  High: %d\n", lowLevel, midLevel, highLevel);


  delay(10);
}
