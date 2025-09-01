#ifndef WAVE_ANALYZER_H
#define WAVE_ANALYZER_H

#include "Arduino.h"

#include "arm_math.h"
#include "ard_supers/avr/dtostrf.h"

#include "etl.h"
#include "etl/vector.h"
#include "etl/deque.h"

#include "params.h"
#include "imu_manager.h"
#include "data_manager.h"
#include "watchdog_manager.h"
#include "time_manager.h"
#include "helpers.h"

// Wave spectrum packet (existing)
struct Wave_Packet{
    long posix_timestamp;
    unsigned int spectrum_number;
    float Hs;
    float Tz;
    float Tc;
    float max_value;
    uint16_t array_pwelch[welch_bin_max - welch_bin_min];
};

void print_wave_packet(Wave_Packet const & packet);

class WaveAnalyzer{
    public:
        void gather_and_analyze_wave_data(void);

        // Gather needed IMU data blocks at 10 Hz into DataManager
        bool gather_imu_data(void);

        // Existing: perform Welch analysis and push packet
        void perform_welch_analysis_imu_data(void);

        // Wave spectrum packets buffer (existing W* message)
        etl::deque<Wave_Packet, size_wave_packet_buffer> wave_packet_buffer;

        void print_deque_content(void) const;

        // Existing: write Welch spectrum message into buffer ('Y'... 'E')
        void write_message_to_buffer(etl::ivector<unsigned char>& buffer);

        // Scheduling helper
        bool time_to_measure_waves(void) const;

        // ---------------------------------------------------------------------
        // NEW: build a single Iridium message containing complex FFT coeffs
        // from the 2 Hz full-series FFT. This encodes:
        // 'C' [posix:int32][df:float32][start_bin:uint16][n_bins:uint16][q15_scale_q10:uint16] [ (re:int16, im:int16) * n_bins ] 'E'
        //
        // Constraints:
        //  - Uses existing fft_length (=2048) for CMSIS RFFT
        //  - Decimate 10 Hz -> 2 Hz (factor 5), take first 2048 samples
        //  - Apply 64-bin moving average on |FFT| (analysis only; not transmitted)
        //  - Pack up to 80 bins per message (fits 340B)
        //
        // Returns true if encoded and appended to 'buffer'.
        bool build_fft2hz_coeff_message(uint16_t start_bin, uint16_t n_bins, etl::ivector<unsigned char>& buffer);

        // Recommended defaults (used by IridiumManager):
        static constexpr uint16_t fft2hz_total_bins_to_send = 100;   // k=1..100
        static constexpr uint16_t fft2hz_max_bins_per_msg  = 80;     // to fit 340B
        static constexpr uint16_t fft2hz_freq_ma_window    = 64;     // 64-bin moving average (analysis only)
        static constexpr float    fft2hz_sampling_hz       = 2.0f;   // target FS after decimation

    private:
        // ------------------------------------------------------------------------------------------
        // FFT utilities
        void init_fast_instance(void);
        void perform_fft(void);
        void clear_fft_input_output(void);
        void clear_working_welch_spectrum(void);
        void clear_working_wave_packet(void);

        // ------------------------------------------------------------------------------------------
        // FFT properties (RFFT fast instance)
        static constexpr uint8_t forward_fft  = 0;
        static constexpr uint8_t backward_fft = 1;

        arm_rfft_fast_instance_f32 crrt_arm_rfft_fast_instance_f32;
        arm_status                 crrt_arm_status;

        // FFT buffers (length = params.h::fft_length = 2048)
        float32_t fft_input[fft_length];
        // Output layout (CMSIS RFFT): [FFT(0), FFT(N/2), Re(1), Im(1), Re(2), Im(2), ...]
        float32_t fft_output[fft_length];

        // Welch working buffers (existing)
        float32_t working_welch_spectrum[welch_bin_max - welch_bin_min];
        Wave_Packet working_wave_packet;

        unsigned int spectrum_number {0};
};

extern WaveAnalyzer board_wave_analyzer;

#endif
