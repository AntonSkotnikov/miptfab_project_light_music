#include <Arduino.h>
#include <driver/i2s_std.h>
#include <FastLED.h>
#include "arduinoFFT.h"
#include <time.h>

#define LED_PIN 32
#define NUM_LEDS 162
#define BRIGHTNESS 70
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

#define SAMPLES 256
#define SAMPLING_FREQUENCY 10000
#define BANDS 10
#define COLUMNS 14
#define LEDS_IN_COLUMN 10
int leds_logic[NUM_LEDS][3] = { 0 };

#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_22
#define I2S_MIC_SERIAL_DATA GPIO_NUM_21

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

i2s_chan_handle_t rx_handle;
const int pingPin = 14;

void setup() {
  Serial.begin(115200);
  delay(1000);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

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
      .din = I2S_MIC_SERIAL_DATA }
  };
  i2s_channel_init_std_mode(rx_handle, &std_cfg);
  i2s_channel_enable(rx_handle);
}


clock_t start = clock();
int mode = 0;
long prev_dist = 5000;
long dist;
int gate = 0;
int sense = 70;
long sense_summ = 70;
int sense_cnt = 1;
int dist_lim = 50;
int level = 0;
void loop() {
  if (mode == 0) {
    dist = get_dist();
    if (gate == 0 && dist > dist_lim && prev_dist <= dist_lim) {
      start = clock();
      //Serial.println("clock was apdated in the beginning of mode 0");
    } 
    else if (gate == 0 && dist > dist_lim && prev_dist > dist_lim && (clock() - start) > 1000) {
      gate = 1;
    }

    Serial.print(dist);
    Serial.print("cm");
    Serial.println();
    Serial.println(clock() - start);
    Serial.println(mode);
    Serial.println(sense);

    if ((clock() - start) > 10000) {
      start = clock() - 1100;
    }

    if (gate == 1) {
      int32_t samples_read = 0;
      for (int i = 0; i < SAMPLES; i++) {
        int32_t sample = 0;
        size_t bytes_read = 0;
        i2s_channel_read(rx_handle, &sample, sizeof(sample), &bytes_read, portMAX_DELAY);
        vReal[i] = (double)sample / 100000.0 * 2.0;
        vImag[i] = 0;
      }

      FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

      double bands_ampl[BANDS] = { 0 };
      int bands_weight[BANDS] = { 0 };
      int strings[BANDS] = { 0 };

      for (int i = 0; i < (SAMPLES / 2); i++) {
        double freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
        if (freq > 0 && freq <= 100) {
          bands_ampl[0] += vReal[i];
          bands_weight[0]++;
        } else if (freq > 100 && freq <= 200) {
          bands_ampl[1] += vReal[i];
          bands_weight[1]++;
        } else if (freq > 200 && freq <= 300) {
          bands_ampl[2] += vReal[i];
          bands_weight[2]++;
        } else if (freq > 300 && freq <= 725) {
          bands_ampl[3] += vReal[i];
          bands_weight[3]++;
        } else if (freq > 725 && freq <= 1150) {
          bands_ampl[4] += vReal[i];
          bands_weight[4]++;
        } else if (freq > 1150 && freq <= 1575) {
          bands_ampl[5] += vReal[i];
          bands_weight[5]++;
        } else if (freq > 1575 && freq <= 2000) {
          bands_ampl[6] += vReal[i];
          bands_weight[6]++;
        } else if (freq > 2000 && freq <= 3000) {
          bands_ampl[7] += vReal[i];
          bands_weight[7]++;
        } else if (freq > 3000 && freq <= 4000) {
          bands_ampl[8] += vReal[i];
          bands_weight[8]++;
        } else if (freq > 4000 && freq <= 5000) {
          bands_ampl[9] += vReal[i];
          bands_weight[9]++;
        }
      }
      FastLED.clear();
      for (int i = 0; i < BANDS; i++) {
        if (bands_weight[i] > 0) {
          bands_ampl[i] /= bands_weight[i];
          if (i == 3 || i == 7) {
            strings[i] = constrain((int)(bands_ampl[i] / 600.0 * sense / 100), 0, 7);
          } else if (i % 2 == 0) {
            strings[i] = constrain((int)(bands_ampl[i] / 600.0 * sense / 100), 0, 8);
          } else if (i == 1 || i == 5 || i == 9) {
            strings[i] = constrain((int)(bands_ampl[i] / 600.0 * sense / 100), 0, 9);
          }
        }
      }

      int red = 0;
      int green = 0;
      int blue = 0;
      for (int band = 0; band < BANDS; band++) {
        red = 0;
        green = 150;
        blue = 50;
        if (band % 2 == 0) {
          red = 60;
          green = 25;
          blue = 50;
          if (band == 0) {
            for (int i = 0; i < strings[band]; i++) {  //i: 0-7
              leds[7 - i] = CRGB(red, green, blue);
              leds[i + 8] = CRGB(red, green, blue);
            }
          }
          else if (band == 2) {
            for (int i = 0; i < strings[band]; i++) {
              leds[41 - i] = CRGB(red, green, blue);
              leds[i + 42] = CRGB(red, green, blue);
            }
          } 
          else if (band == 4) {
            for (int i = 0; i < strings[band]; i++) {
              leds[71 - i] = CRGB(red, green, blue);
              leds[i + 72] = CRGB(red, green, blue);
            }
          } 
          else if (band == 6) {
            for (int i = 0; i < strings[band]; i++) {
              leds[105 - i] = CRGB(red, green, blue);
              leds[i + 106] = CRGB(red, green, blue);
            }
          } 
          else if (band == 8) {
            for (int i = 0; i < strings[band]; i++) {
              leds[135 - i] = CRGB(red, green, blue);
              leds[i + 136] = CRGB(red, green, blue);
            }
          }
        } 
        else if (band == 1) {
          for (int i = 0; i < strings[band]; i++) {
            leds[24 - i] = CRGB(red, green, blue);
            leds[i + 25] = CRGB(red, green, blue);
          }
        } 
        else if (band == 3) {
          for (int i = 0; i < strings[band]; i++) {
            leds[56 - i] = CRGB(red, green, blue);
            leds[i + 57] = CRGB(red, green, blue);
          }
        } 
        else if (band == 5) {
          for (int i = 0; i < strings[band]; i++) {
            leds[88 - i] = CRGB(red, green, blue);
            leds[i + 89] = CRGB(red, green, blue);
          }
        } 
        else if (band == 7) {
          for (int i = 0; i < strings[band]; i++) {
            leds[120 - i] = CRGB(red, green, blue);
            leds[i + 121] = CRGB(red, green, blue);
          }
        } 
        else if (band == 9) {
          for (int i = 0; i < strings[band]; i++) {
            leds[152 - i] = CRGB(red, green, blue);
            leds[i + 153] = CRGB(red, green, blue);
          }
        }
      }

      FastLED.show();

      if (dist <= dist_lim && prev_dist > dist_lim) {
        start = clock();
        Serial.println("clock was apdated in the end of mode 0");
      }
      else if (dist <= dist_lim && prev_dist <= dist_lim && (clock() - start) <= 1000) {
        sense_summ += (int) (dist/((double)dist_lim)*100.0);
        sense_cnt++;
      }
      else if (dist <= dist_lim && prev_dist <= dist_lim && (clock() - start) > 1000) {
        mode = 1;
        gate = 0;
        sense = sense_summ / sense_cnt;
        sense_summ = 0;
        sense_cnt = 0;
        FastLED.clear();
      }
      else if (dist > dist_lim) {
        sense_summ = 0;
        sense_cnt = 0;
      }
    }
    prev_dist = dist;
  }

  if (mode == 1) {
    dist = get_dist();
    if (gate == 0 && dist > dist_lim && prev_dist <= dist_lim) {
      start = clock();
      //Serial.println("clock was apdated in the beginning of mode 1");
    } else if (gate == 0 && dist > dist_lim && prev_dist > dist_lim && (clock() - start) > 1000) {
      gate = 1;
    }

    Serial.print(dist);
    Serial.print("cm");
    Serial.println();
    Serial.println(clock() - start);
    Serial.println(mode);
    Serial.println(sense);
    Serial.println(level);

    if ((clock() - start) > 10000) {
      start = clock() - 1100;
    }

    if (gate == 1) {
      int32_t samples_read = 0;
      for (int i = 0; i < SAMPLES; i++) {
        int32_t sample = 0;
        size_t bytes_read = 0;
        i2s_channel_read(rx_handle, &sample, sizeof(sample), &bytes_read, portMAX_DELAY);
        vReal[i] = (double)sample / 100000.0 * 2.0;
        vImag[i] = 0;
      }

      FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

      double bands_ampl[BANDS] = { 0 };
      int bands_weight[BANDS] = { 0 };

      for (int i = 0; i < (SAMPLES / 2); i++) {
        double freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;

        if (freq <= 200 || freq > 3500) {
          bands_ampl[0] += vReal[i];
          bands_weight[0]++;
        }
      }
      FastLED.clear();

      if (bands_weight[0] > 0) {
        bands_ampl[0] /= bands_weight[0];
      }

      level = constrain((int)(bands_ampl[0] / 100.0 * sense / 100), 0, 20);
      int cur_led = 0;

      for (int line = 10; line > 1; line--) {
        for (int led = 1; led < 15; led += 2) {
          for (int i = 0; i < 3; i++) {
            leds_logic[led_finder(line, led)][i] = leds_logic[led_finder(line - 1, led)][i];
            leds_logic[led_finder(10 - line + 1, led + 1)][i] = leds_logic[led_finder(10 - line + 2, led + 1)][i];
          }
        }
      }

      if (leds_logic[led_finder(1, 1)][0] == 0 && level >= 5) {
        for (int led = 1; led < 15; led += 2) {
          cur_led = led_finder(1, led);
          leds_logic[cur_led][0] = 123;
          leds_logic[cur_led][1] = 49;
          leds_logic[cur_led][2] = 2;

          cur_led = led_finder(10, led + 1);
          leds_logic[cur_led][0] = 51;
          leds_logic[cur_led][1] = 3;
          leds_logic[cur_led][2] = 110;
        }
      }

      else {
        for (int led = 1; led < 15; led += 2) {
          for (int i = 0; i < 3; i++) {
            leds_logic[led_finder(1, led)][i] = 0;
            leds_logic[led_finder(10, led + 1)][i] = 0;
          }
        }
      }

      for (int line = 1; line < 11; line++) {
        for (int led = 1; led < 15; led++) {
          cur_led = led_finder(line, led);
          leds[cur_led] = CRGB((int)(leds_logic[cur_led][0] * (level + 1) / 21), (int)(leds_logic[cur_led][1] * (level + 1) / 21), (int)(leds_logic[cur_led][2] * (level + 1) / 21));
        }
      }

      FastLED.show();

      if (dist <= dist_lim && prev_dist > dist_lim) {
        start = clock();
        Serial.println("clock was apdated in the end of mode 1");
      } 
      else if (dist <= dist_lim && prev_dist <= dist_lim && (clock() - start) <= 1000) {
        sense_summ += (int) (dist/((double)dist_lim)*100.0);
        sense_cnt++;
      }
      else if (dist <= dist_lim && prev_dist <= dist_lim && (clock() - start) > 1000) {
        mode = 0;
        gate = 0;
        sense = sense_summ / sense_cnt;
        sense_summ = 0;
        sense_cnt = 0;
        for (int i = 0; i < NUM_LEDS; i++) {
          for (int j = 0; j < 3; j++) {
            leds_logic[i][j] = 0;
          }
        }
        FastLED.clear();
      }
      else if (dist > dist_lim) {
        sense_summ = 0;
        sense_cnt = 0;
      }
    }
    prev_dist = dist;
  }

  delay(10);
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

long get_dist() {
  long duration, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  cm = microsecondsToCentimeters(duration);

  return cm;
}

int led_finder(int line, int led) {
  int found_led = 0;
  int leds_in_line = 16;
  int line_state = 1;
  for (int i = 1; i < line; i++) {
    found_led += leds_in_line;
    if (line_state == 0) {
      leds_in_line = 16;
      line_state = 1;
    } else if (line_state == 1) {
      leds_in_line = 18;
      line_state = 2;
    } else if (line_state == 2) {
      leds_in_line = 16;
      line_state = 3;
    } else if (line_state == 3) {
      leds_in_line = 14;
      line_state = 0;
    }
  }

  if (leds_in_line == 16) {
    led++;
  } else if (leds_in_line == 18) {
    led += 2;
  }

  if (line % 2 == 1) {
    found_led += led;
  } else {
    found_led += (leds_in_line - led + 1);
  }

  return found_led - 1;
}
