#include <driver/i2s_std.h>
#include "arduinoFFT.h"
#include "math.h"

#define SAMPLES 256
#define SAMPLING_FREQUENCY 10000

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

  for(int i=1; i<(SAMPLES/2); i++) {
    Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
    Serial.print(", ");
    Serial.println(vReal[i], 1);
  }

  delay(5);

}
