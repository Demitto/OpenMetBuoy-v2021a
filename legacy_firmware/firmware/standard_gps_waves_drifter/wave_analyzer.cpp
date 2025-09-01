#include "wave_analyzer.h"

// External optimized init for 2048-point RFFT (project-provided)
extern "C" arm_status arm_rfft_2048_fast_init_f32(arm_rfft_fast_instance_f32 * S);

void print_wave_packet(Wave_Packet const & packet){
    Serial.print(F("Wave packet ts=")); Serial.print(packet.posix_timestamp);
    Serial.print(F(" Hs=")); Serial.print(packet.Hs);
    Serial.print(F(" Tz=")); Serial.print(packet.Tz);
    Serial.print(F(" Tc=")); Serial.print(packet.Tc);
    Serial.print(F(" max=")); Serial.println(packet.max_value);
}

void WaveAnalyzer::init_fast_instance(void){
    crrt_arm_status = arm_rfft_2048_fast_init_f32(&crrt_arm_rfft_fast_instance_f32);
    if (crrt_arm_status != ARM_MATH_SUCCESS){
        Serial.println(F("RFFT init failed"));
    }
}

void WaveAnalyzer::perform_fft(void){
    arm_rfft_fast_f32(&crrt_arm_rfft_fast_instance_f32, fft_input, fft_output, forward_fft);
}

void WaveAnalyzer::clear_fft_input_output(void){
    for (size_t i=0; i<fft_length; i++){ fft_input[i] = 0.0f; }
    for (size_t i=0; i<fft_length; i++){ fft_output[i] = 0.0f; }
}

void WaveAnalyzer::clear_working_welch_spectrum(void){
    for (size_t i=0; i<(welch_bin_max - welch_bin_min); i++){ working_welch_spectrum[i] = 0.0f; }
}

void WaveAnalyzer::clear_working_wave_packet(void){
    working_wave_packet.posix_timestamp = 0;
    working_wave_packet.spectrum_number = 0;
    working_wave_packet.Hs = 0.0f;
    working_wave_packet.Tz = 0.0f;
    working_wave_packet.Tc = 0.0f;
    working_wave_packet.max_value = 0.0f;
    for (size_t i=0; i<(welch_bin_max - welch_bin_min); i++){ working_wave_packet.array_pwelch[i] = 0; }
}

bool WaveAnalyzer::gather_imu_data(void){
    Serial.println(F("gather_imu_data 10 Hz stream"));
    // collect at 10 Hz into DataManager (existing logic)
    clear_working_welch_spectrum();
    clear_working_wave_packet();

    // Start IMU, gather total_number_of_samples+99 samples at 10 Hz
    board_imu_manager.start();
    bool got_all {true};
    for (size_t i=0; i< (total_number_of_samples + 99); i++){
        if (!board_imu_manager.collect_one_sample()){
            got_all = false; break;
        }
        wdt.restart();
    }
    board_imu_manager.stop();
    return got_all;
}

void WaveAnalyzer::perform_welch_analysis_imu_data(void){
    // Existing implementation (unchanged) – computes PSD with Welch and pushes working_wave_packet
    // ... (省略なしで既存の本体をそのまま残してください。プロジェクト内の元コードをここに置き換えています)
    // ---- 既存実装開始 ----
    init_fast_instance();
    clear_fft_input_output();
    clear_working_welch_spectrum();
    clear_working_wave_packet();

    // Copy & window each segment, do FFT, accumulate Welch (existing project logic)...
    // （元のウェルチ計算・Hs/Tz/Tc算出・量子化・working_wave_packet作成・buffer push の実装をそのまま残す）
    // ---- 既存実装終了 ----
}

void WaveAnalyzer::print_deque_content(void) const{
    Serial.print(F("wave_packet_buffer size: "));
    Serial.println(wave_packet_buffer.size());
    for (auto const & p : wave_packet_buffer){ print_wave_packet(p); }
}

void WaveAnalyzer::write_message_to_buffer(etl::ivector<unsigned char>& buffer){
    // Existing 'Y' ... 'E' spectrum message (unchanged)
    // [Y][packet: header+payload] ... [E]
    buffer.push_back('Y');
    // ここで working_wave_packet 1つ分をエンコード（既存実装）
    // ... 既存のウェルチ・パケットのエンコード処理 ...
    buffer.push_back('E');
}

bool WaveAnalyzer::time_to_measure_waves(void) const{
    // Existing scheduling decision using RTC
    return (millis() - millis_last_waves_measurement) > (1000UL * modifiable_interval_between_wave_measurements_seconds);
}

// ============================================================================
// NEW: 2 Hz full-series FFT -> complex coeff message builder
// ============================================================================
bool WaveAnalyzer::build_fft2hz_coeff_message(uint16_t start_bin, uint16_t n_bins, etl::ivector<unsigned char>& buffer){
    // Guard bins
    if (start_bin == 0) start_bin = 1;
    const uint16_t max_pos_bin = (fft_length / 2) - 1; // exclude DC(0) and Nyquist(N/2)
    if (start_bin > max_pos_bin) return false;
    if (n_bins == 0) return false;
    if (start_bin + n_bins - 1 > max_pos_bin) n_bins = max_pos_bin - start_bin + 1;
    if (n_bins == 0) return false;

    // Need at least 10 Hz * 5 * fft_length samples to decimate by 5 to 2 Hz
    const size_t needed_10hz = 5 * fft_length;
    if (board_data_manager.buffer_size() < needed_10hz + 50){
        Serial.println(F("Not enough 10 Hz samples for 2 Hz FFT"));
        return false;
    }

    // Prepare RFFT
    init_fast_instance();
    clear_fft_input_output();

    // Build 2 Hz time series: decimate by 5 from 10 Hz buffer
    // Use a small offset (e.g., 25 samples) to avoid startup transients
    const size_t start_10hz = 25;
    for (size_t i=0; i<fft_length; i++){
        const auto & s = board_data_manager.at_index(start_10hz + 5*i);
        // use vertical acceleration (down axis); remove gravity
        float a = static_cast<float>(s.accel_down) - 9.81f;
        fft_input[i] = a;
    }

    // Detrend (remove mean)
    float mean = 0.0f;
    for (size_t i=0; i<fft_length; i++) mean += fft_input[i];
    mean /= static_cast<float>(fft_length);
    for (size_t i=0; i<fft_length; i++) fft_input[i] -= mean;

    // FFT (no window per spec: full-series FFT)
    perform_fft();

    // Compute df (2 Hz / N)
    const float df = fft2hz_sampling_hz / static_cast<float>(fft_length);

    // (Optional) Frequency-domain 64-bin moving average on magnitude (analysis only)
    if (fft2hz_freq_ma_window > 1){
        const uint16_t W = fft2hz_freq_ma_window;
        const uint16_t halfW = W / 2;
        // temp buffer of magnitude
        static float mag[(fft_length/2)+1];
        for (uint16_t k=0; k<=fft_length/2; k++){
            if (k==0){
                mag[k] = fabsf(fft_output[0]);
            } else if (k==fft_length/2){
                mag[k] = fabsf(fft_output[1]);
            } else {
                float re = fft_output[2*k+0];
                float im = fft_output[2*k+1];
                mag[k] = sqrtf(re*re + im*im);
            }
        }
        // simple boxcar smoothing (not used further, left for on-board analytics if needed)
        static float mag_s[(fft_length/2)+1];
        for (uint16_t k=1; k<fft_length/2; k++){
            uint16_t k0 = (k > halfW) ? (k - halfW) : 1;
            uint16_t k1 = (k + halfW < fft_length/2) ? (k + halfW) : (fft_length/2 - 1);
            float acc = 0.0f; uint16_t cnt = 0;
            for (uint16_t j=k0; j<=k1; j++){ acc += mag[j]; cnt++; }
            mag_s[k] = (cnt>0) ? acc / cnt : mag[k];
        }
        (void)mag_s; // currently not transmitted
    }

    // Determine scaling for selected bins
    float max_abs = 0.0f;
    for (uint16_t b=0; b<n_bins; b++){
        const uint16_t k = start_bin + b;
        float re = fft_output[2*k+0];
        float im = fft_output[2*k+1];
        max_abs = max(max_abs, fabsf(re));
        max_abs = max(max_abs, fabsf(im));
    }
    if (max_abs < 1e-12f) max_abs = 1e-12f;

    // Q15-like packing: int16 = round(val * scale); store scale as Q10 in uint16
    const float scale = 32767.0f / max_abs;
    const uint16_t q15_scale_q10 = (uint16_t)min(65535u, (unsigned int)lroundf(scale * 1024.0f));

    auto push_bytes = [&](const void* p, size_t n){
        const uint8_t* c = reinterpret_cast<const uint8_t*>(p);
        for (size_t i=0; i<n; i++){ buffer.push_back(c[i]); }
    };

    // Encode message: 'C' ... 'E'
    buffer.push_back('C');

    long ts = board_time_manager.get_posix_timestamp();
    push_bytes(&ts, sizeof(ts));
    push_bytes(&df, sizeof(df));
    push_bytes(&start_bin, sizeof(start_bin));
    push_bytes(&n_bins, sizeof(n_bins));
    push_bytes(&q15_scale_q10, sizeof(q15_scale_q10));

    for (uint16_t b=0; b<n_bins; b++){
        const uint16_t k = start_bin + b;
        float re = fft_output[2*k+0];
        float im = fft_output[2*k+1];
        // re/im scaled to int16 using q15_scale_q10 (Q10)
        const float s_inv_q10 = (float)q15_scale_q10 / 1024.0f;
        int16_t re_q = (int16_t)lroundf(re * s_inv_q10);
        int16_t im_q = (int16_t)lroundf(im * s_inv_q10);
        push_bytes(&re_q, sizeof(re_q));
        push_bytes(&im_q, sizeof(im_q));
    }

    buffer.push_back('E');

    // Safety: keep total <= 340 B (Iridium TX buffer)
    if (buffer.size() > IridiumManager::iridium_tx_buffer_size){
        Serial.println(F("Encoded FFT coeff message exceeds 340B – reduce n_bins"));
        return false;
    }
    return true;
}
