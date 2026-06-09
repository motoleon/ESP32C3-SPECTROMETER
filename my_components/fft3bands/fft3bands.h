#pragma once

#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include <driver/i2s.h>
#include <cmath>
#include <vector>

namespace fft3bands {

class FFTSensor : public esphome::sensor::Sensor, public esphome::PollingComponent {
 public:
  FFTSensor() : PollingComponent(120) {}

  void set_sample_rate(uint32_t sr) { this->sample_rate_ = sr; }
  void set_ws_pin(int pin) { this->ws_pin_ = pin; }
  void set_sck_pin(int pin) { this->sck_pin_ = pin; }
  void set_sd_pin(int pin) { this->sd_pin_ = pin; }
  void set_fft_size(uint16_t fft_size) { this->fft_size_ = fft_size; }
  void set_Ganancia(uint32_t Ganancia) { this->Ganancia_ = Ganancia; }
  void set_band_frequencies(const std::vector<float> &freqs) {
    this->band_frequencies_ = freqs;
    this->band_values_.assign(freqs.size(), 0.0f);
  }

  size_t get_band_count() const { return this->band_values_.size(); }
  float get_band(size_t index) const {
    if (index >= this->band_values_.size()) return 0.0f;
    return this->band_values_[index];
  }
  float get_band_frequency(size_t index) const {
    if (index >= this->band_frequencies_.size()) return 0.0f;
    return this->band_frequencies_[index];
  }
  float get_vu_level() const { return this->vu_level_; }

  std::vector<float> tabla_maximos;
  std::vector<float> tabla_minimos;
  std::vector<float> tabla_scale;
  std::vector<float> tabla_floor;
  
  void setup() override {
    ESP_LOGI("fft3bands", "Inicializando I2S real para FFT");
    ESP_LOGI("fft3bands", "update_interval efectivo: %u ms", this->get_update_interval());
    ESP_LOGI("fft3bands", "sample_rate=%u fft_size=%u Ganancia=%u", this->sample_rate_, this->fft_size_, this->Ganancia_);
    ESP_LOGI("fft3bands", "band_count=%u", (unsigned) this->band_frequencies_.size());
    for (size_t i = 0; i < this->band_frequencies_.size(); i++) {
      ESP_LOGI("fft3bands", "band[%u]=%.1f Hz", (unsigned) i, this->band_frequencies_[i]);
    }

    if (this->band_frequencies_.empty()) {
      this->band_frequencies_ = {100.0f, 1000.0f, 5000.0f, 10000.0f};
      this->band_values_.assign(this->band_frequencies_.size(), 0.0f);
    }

    this->samples_i2s_.assign(this->fft_size_, 0);
    this->re_.assign(this->fft_size_, 0.0f);
    this->im_.assign(this->fft_size_, 0.0f);

    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate = this->sample_rate_;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = 4;
    i2s_config.dma_buf_len = 256;
    i2s_config.use_apll = true;
    i2s_config.tx_desc_auto_clear = false;
    i2s_config.fixed_mclk = 0;

    i2s_pin_config_t pin_config = {};
    pin_config.bck_io_num = this->sck_pin_;
    pin_config.ws_io_num = this->ws_pin_;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num = this->sd_pin_;

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
    if (err != ESP_OK) {
      ESP_LOGE("fft3bands", "Error instalando I2S: %d", err);
      return;
    }

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
      ESP_LOGE("fft3bands", "Error configurando pines I2S: %d", err);
      return;
    }

    err = i2s_zero_dma_buffer(I2S_NUM_0);
    if (err != ESP_OK) {
      ESP_LOGW("fft3bands", "No se pudo limpiar DMA I2S: %d", err);
    }
	
	tabla_maximos.assign(this->band_frequencies_.size(), 0.0f);
	tabla_minimos.assign(this->band_frequencies_.size(), 335596928.0f);
	tabla_scale.assign(this->band_frequencies_.size(), 0.0f);
	tabla_floor.assign(this->band_frequencies_.size(), 0.0f);
	
	for (size_t idx = 0; idx < this->band_frequencies_.size(); idx++) {
		tabla_scale[idx] = mic_scale_(this->band_frequencies_[idx]);
		tabla_floor[idx] = mic_floor_(this->band_frequencies_[idx]);
	}
  }

  uint32_t last_log_ms_{0};
  
  // despues de un analisis de maximos y minimos, bajo estas frecuencias, con el INMP441
  // he obtenido una tabla de suelo y una tabla de escala
  // tomando la frecuencia 1250 como referencia de escala 1:1
  static constexpr  float FREQ_POINTS[11] = {
	  50.0f, 160.0f, 315.0f, 630.0f,
	  1250.0f, 2500.0f, 5000.0f, 10000.0f,
	  12500.0f, 16000.0f, 20000.0f
  };
  
  // this look Up Table is used to normalize the buckets against each other. Visually makes the higher frequencies appear to be more equal to the lower frequencies. 
  static constexpr  float NORMALIZACION[64] = {0.0006637301302, 0.0006793553648, 0.0006966758032, 0.0007158753602, 0.0007371579043, 0.0007607494216, 0.0007869004159, 0.0008158885684, 0.0008480216863, 0.0008836409716, 0.0009231246432, 0.0009668919541, 0.001015407642, 0.001069186866, 0.001128800673, 0.001194882066, 0.001268132722, 0.001349330446, 0.001439337425, 0.001539109388, 0.001649705751, 0.00177230087, 0.001908196507, 0.002058835652, 0.002225817851, 0.002410916183, 0.002616096095, 0.002843536264, 0.003095651737, 0.003375119574, 0.00368490727, 0.004028304269, 0.004408956893, 0.004830907057, 0.005298635188, 0.005817107803, 0.006391830243, 0.00702890513, 0.007735097169, 0.008517904978, 0.009385640709, 0.01034751831, 0.01141375137, 0.01259566156, 0.01390579885, 0.01535807478, 0.01696791017, 0.01875239887, 0.02073048926, 0.02292318547, 0.02535377038, 0.02804805287, 0.03103464187, 0.03434525011, 0.03801503091, 0.04208295139, 0.04659220631, 0.05159067664, 0.05713143806, 0.06327332449, 0.07008155284, 0.07762841548, 0.08599404787, 0.09526727952};

  static constexpr  float SCALE_POINTS[11] = {
	  0.050f, 0.322f, 0.396f, 0.648f,
	  1.000f, 1.163f, 1.240f, 2.048f,
	  2.698f, 3.373f, 4.151f
  };
  
  static constexpr  float FLOOR_POINTS[11] = {
	  30000.0f, 30000.0f, 20000.0f, 8000.0f,
	  2000.0f, 2000.0f, 2000.0f, 2000.0f,
	  2000.0f, 2000.0f, 2000.0f
  };    
  
  float interp_log_scale_(float freq, float f1, float s1, float f2, float s2) {
	  if (freq <= f1) return s1;
	  if (freq >= f2) return s2;
	  float x  = log10f(freq);
	  float x1 = log10f(f1);
	  float x2 = log10f(f2);
	  float t  = (x - x1) / (x2 - x1);
	  return s1 + t * (s2 - s1);
  }

  float mic_scale_(float freq) {
	  const int N = sizeof(FREQ_POINTS) / sizeof(FREQ_POINTS[0]);
	  if (freq <= FREQ_POINTS[0])     return SCALE_POINTS[0];
	  if (freq >= FREQ_POINTS[N - 1]) return SCALE_POINTS[N - 1];

	  for (int i = 0; i < N - 1; ++i) {
		if (freq <= FREQ_POINTS[i + 1]) {
		  return interp_log_scale_(freq,
								   FREQ_POINTS[i],     SCALE_POINTS[i],
								   FREQ_POINTS[i + 1], SCALE_POINTS[i + 1]);
		}
	  }
	  return SCALE_POINTS[N - 1];
  }  

  float mic_floor_(float freq) {
	  const int N = sizeof(FREQ_POINTS) / sizeof(FREQ_POINTS[0]);
	  if (freq <= FREQ_POINTS[0])     return FLOOR_POINTS[0];
	  if (freq >= FREQ_POINTS[N - 1]) return FLOOR_POINTS[N - 1];

	  for (int i = 0; i < N - 1; ++i) {
		if (freq <= FREQ_POINTS[i + 1]) {
		  return interp_log_scale_(freq,
								   FREQ_POINTS[i],     FLOOR_POINTS[i],
								   FREQ_POINTS[i + 1], FLOOR_POINTS[i + 1]);
		}
	  }
	  return FLOOR_POINTS[N - 1];
  } 

  void update() override {
    if (this->fft_size_ < 2 || (this->fft_size_ & (this->fft_size_ - 1)) != 0) {
      ESP_LOGE("fft3bands", "fft_size invalido: %u", this->fft_size_);
      return;
    }
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_0, this->samples_i2s_.data(), this->samples_i2s_.size() * sizeof(int32_t), &bytes_read, pdMS_TO_TICKS(50));
    if (err != ESP_OK || bytes_read != this->samples_i2s_.size() * sizeof(int32_t)) {
      ESP_LOGW("fft3bands", "Lectura I2S incompleta: err=%d bytes=%u esperados=%u", err, (unsigned) bytes_read, (unsigned) (this->samples_i2s_.size() * sizeof(int32_t)));
      return;
    }

    for (size_t i = 0; i < this->fft_size_; i++) {
      this->re_[i] = (float) (this->samples_i2s_[i] >> 8);
      this->im_[i] = 0.0f;
    }

    float mean = 0.0f;
    for (size_t i = 0; i < this->fft_size_; i++) mean += this->re_[i];
    mean /= this->fft_size_;
    for (size_t i = 0; i < this->fft_size_; i++) this->re_[i] -= mean;

    // vumeter
    float sumsq = 0.0f;
    for (size_t i = 0; i < this->fft_size_; i++) sumsq += this->re_[i] * this->re_[i];
    float rms = sqrtf(sumsq / this->fft_size_);

    float vu_raw = (rms - 2000.0f) / 100000.0f;
    if (vu_raw < 0.0f) vu_raw = 0.0f;
    if (vu_raw > 1.0f) vu_raw = 1.0f;
    this->vu_level_ = this->vu_level_ * 0.80f + vu_raw * 0.20f;  
    // vumeter
	
    for (size_t i = 0; i < this->fft_size_; i++) {
      float w = 0.54f - 0.46f * cosf((2.0f * 3.14159265f * i) / (this->fft_size_ - 1)); // ventana Hamming
      this->re_[i] *= w;
    }

    fft_(this->re_.data(), this->im_.data(), this->fft_size_); // analisis FFT

    const float bin_width = (float) this->sample_rate_ / (float) this->fft_size_;
    const int max_bin = (int) this->fft_size_ / 2 - 3;

    if (this->band_values_.size() != this->band_frequencies_.size()) {
      this->band_values_.assign(this->band_frequencies_.size(), 0.0f);
    }

    std::vector<float> tabla_en_crudo(this->band_frequencies_.size(), 0.0f);
	
	auto mag_bin = [&](int b) -> float { return sqrtf(this->re_[b] * this->re_[b] + this->im_[b] * this->im_[b]); }; // ComplexToMagnitude de todos los bin
	
    for (size_t idx = 0; idx < this->band_frequencies_.size(); idx++) {  // bucle de frecuencias de usuario
      float freq = this->band_frequencies_[idx];
      int bin = (int) (freq / bin_width + 0.5f);  // miramos que bin corresponde a esta frecuencia
      // if (bin < 2) bin = 2;
      // if (bin > max_bin) bin = max_bin;

      //auto mag_bin = [&](int b) -> float { return sqrtf(this->re_[b] * this->re_[b] + this->im_[b] * this->im_[b]); }; // ComplexToMagnitude de todos los bin
      //float mag = (mag_bin(bin - 1) + mag_bin(bin) + mag_bin(bin + 1)) / 3.0f;
	  
	  //if (bin < 1) bin = 1;
      //float mag = (mag_bin(bin - 1) + mag_bin(bin)) / 2.0f;
      float mag = mag_bin(bin);
	  
      tabla_en_crudo[idx] = mag;
      if (mag > tabla_maximos[idx]) tabla_maximos[idx] = mag;
	  if (mag < tabla_minimos[idx]) tabla_minimos[idx] = mag;
			
      //float noise_floor = 1500.0f;
	  //noise_floor = mic_floor_(freq);
      //mag = mag - mic_floor_(freq);
	  mag = mag - tabla_floor[idx];
      if (mag < 0.0f) mag = 0.0f;
  
	  //float scale = 20000.0f;
      //scale = mic_scale_(freq);
      //float n = mag * mic_scale_(freq) * this->Ganancia_;  // factor de ganancia para que se vea mejor en la pantalla
	  float n = mag * tabla_scale[idx] * this->Ganancia_;  // factor de escala y ganancia para que se vea mejor en la pantalla
	  n = n / 107927744.0f; // valor 1 de escala de referencia en la tabla de frecuencia 1250
      if (n < 0.0f) n = 0.0f;
      if (n > 1.0f) n = 1.0f;

      float smooth = 0.25f;
      //if (freq <= 120.0f) smooth = 0.20f;
      this->band_values_[idx] = this->band_values_[idx] * (1.0f - smooth) + n * smooth;
    }

    if (!this->band_values_.empty()) {
      this->publish_state(this->band_values_[0]);
    }
	
	uint32_t now = esphome::millis();  // log cada x tiempo en ms
	if (now - this->last_log_ms_ >= 500) {
	  this->last_log_ms_ = now;

	  std::string msg;
	  for (size_t idx = 0; idx < this->band_frequencies_.size(); idx++) {
		char buf[64];
		snprintf(buf, sizeof(buf), "| %.0fHz=%.3f %.0f(%.0f %.0f)", this->band_frequencies_[idx], this->band_values_[idx], tabla_en_crudo[idx], tabla_minimos[idx], tabla_maximos[idx]);
		msg += buf;
	  }
	  ESP_LOGI("fft3bands", "VU=%.2f rms=%.1f bin_width=%.2f %s", this->vu_level_, rms, bin_width, msg.c_str());
	}	
  }

 protected:
  uint32_t sample_rate_{22050};
  uint16_t fft_size_{256};
  uint32_t Ganancia_{200};
  int ws_pin_{4};
  int sck_pin_{3};
  int sd_pin_{10};

  std::vector<int32_t> samples_i2s_;
  std::vector<float> re_;
  std::vector<float> im_;
  std::vector<float> band_frequencies_;
  std::vector<float> band_values_;
  float vu_level_{0.0f};

  void fft_(float *re, float *im, int n) {
    int j = 0;
    for (int i = 0; i < n; i++) {
      if (i < j) {
        float tr = re[i];
        re[i] = re[j];
        re[j] = tr;
        float ti = im[i];
        im[i] = im[j];
        im[j] = ti;
      }
      int m = n >> 1;
      while (j >= m && m >= 2) {
        j -= m;
        m >>= 1;
      }
      j += m;
    }

    for (int len = 2; len <= n; len <<= 1) {
      float ang = -2.0f * 3.14159265f / len;
      float wlen_cos = cosf(ang);
      float wlen_sin = sinf(ang);

      for (int i = 0; i < n; i += len) {
        float w_cos = 1.0f;
        float w_sin = 0.0f;
        for (int k = 0; k < len / 2; k++) {
          int u = i + k;
          int v = i + k + len / 2;

          float vr = re[v] * w_cos - im[v] * w_sin;
          float vi = re[v] * w_sin + im[v] * w_cos;

          re[v] = re[u] - vr;
          im[v] = im[u] - vi;
          re[u] = re[u] + vr;
          im[u] = im[u] + vi;

          float next_cos = w_cos * wlen_cos - w_sin * wlen_sin;
          float next_sin = w_cos * wlen_sin + w_sin * wlen_cos;
          w_cos = next_cos;
          w_sin = next_sin;
        }
      }
    }
  }
};

}  // namespace fft3bands
