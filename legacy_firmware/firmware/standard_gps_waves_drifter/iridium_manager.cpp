#include "iridium_manager.h"

unsigned long millis_last_callback {0};
unsigned long millis_iridium_callback_reset {0};
constexpr unsigned long callback_printout_freq_ms {2000UL};

void IridiumManager::clear_buffers(void){
    iridium_rx_buffer.clear();
    iridium_tx_buffer.clear();
    rx_message_available = false;
}

bool IridiumManager::has_received_new_message(void){
    return rx_message_available;
}

bool ISBDCallback(){
    wdt.restart();
    if (millis() - millis_last_callback > callback_printout_freq_ms){
        Serial.print(F("-"));
        millis_last_callback = millis();
    }
    return true;
}

#ifdef ISBD_DIAGNOSTICS
  void ISBDConsoleCallback(IridiumSBD *device, char c){ Serial.write(c); }
  void ISBDDiagsCallback(IridiumSBD *device, char c){ Serial.write(c); }
#endif

bool IridiumManager::send_receive_message(unsigned long timeout_cap_charging_seconds){
    last_message_went_through = false;

    Serial.println(F("iridium attempt send receive message"));
    print_vector_uc(iridium_tx_buffer);

    wdt.restart();
    turn_gnss_off();

    // Charge supercaps
    Serial.println(F("charge supercaps"));
    pinMode(superCapChgEN, OUTPUT);
    digitalWrite(superCapChgEN, HIGH);
    delay(2000);
    wdt.restart();

    bool charge_ready {false};
    for (int i=0; i<timeout_cap_charging_seconds; i++){ Serial.print(F("-")); }
    Serial.println();
    delay(10);

    for (unsigned long tstart=millis(); (!charge_ready) && (millis()-tstart < 1000UL * timeout_cap_charging_seconds);){
      charge_ready = (bool)digitalRead(superCapPGOOD);
      delay(1000);
      wdt.restart();
      Serial.print("-");
    }
    Serial.println();

    if (!charge_ready){
        Serial.println(F("failed to charge supercaps, abort transmission"));
        turn_iridium_off();
        send_receive_last_went_through = false;
        return false;
    }

    Serial.println(F("supercaps charged"));
    delay(1000);
    wdt.restart();

    // Power on Iridium
    turn_iridium_on();
    wdt.restart();
    delay(1000);
    wdt.restart();

    iridium_serial.begin(19200);
    delay(1000);
    wdt.restart();
    iridium_sbd.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
    wdt.restart();
    millis_last_callback = millis();

    if (iridium_sbd.begin() != ISBD_SUCCESS){
        Serial.println(F("failed to start the iridium modem!"));
        turn_iridium_off();
        send_receive_last_went_through = false;
        return false;
    } else {
        Serial.println(F("iridium modem started"));
    }
    wdt.restart();

    iridium_sbd.useMSSTMWorkaround(true);

    Serial.println(F("attempt iridium transmission"));
    for (int i=0; i<150; i++){ Serial.print(F("-")); }
    Serial.println();
    delay(10);

    // copy vectors to raw buffers
    rx_buffer_amount = iridium_rx_buffer_size;
    iridium_rx_buffer.clear();

    tx_buffer_amount = iridium_tx_buffer.size();
    for (size_t i=0; i<iridium_tx_buffer.size(); i++){
        iridium_tx_raw_buffer[i] = iridium_tx_buffer[i];
    }

    millis_iridium_callback_reset = millis();
    iridium_sbd.adjustSendReceiveTimeout(timeout_attempt_transmit_seconds);
    int transmission_status = iridium_sbd.sendReceiveSBDBinary(
      iridium_tx_raw_buffer, tx_buffer_amount,
      iridium_rx_raw_buffer, rx_buffer_amount
    );

    Serial.println();
    Serial.print(F("transmission status [0 is success]: ")); Serial.println(transmission_status);
    Serial.print(F("rx_buffer_amount [271 is nothing received]: ")); Serial.println(rx_buffer_amount);

    if (rx_buffer_amount != iridium_rx_buffer_size){
        Serial.print(F("received iridium command: "));
        for (size_t i=0; i<rx_buffer_amount; i++){
            Serial.print(iridium_rx_raw_buffer[i], HEX); Serial.print(F(" "));
            iridium_rx_buffer.push_back(iridium_rx_raw_buffer[i]);
        }
        Serial.println();
        rx_message_available = true;
    } else {
        Serial.println(F("no command received"));
    }

    wdt.restart();
    Serial.print("Messages remaining to be retrieved: ");
    Serial.println(iridium_sbd.getWaitingMessageCount());

    // Apply reboot / instructions if any
    reboot_if_requested_through_iridium();
    read_apply_iridium_instructions();

    // Cleanup and power down
    if (transmission_status != ISBD_SUCCESS){
        Serial.println(F("iridium transmission failed"));
        delay(500);
        wdt.restart();

        iridium_sbd.clearBuffers(ISBD_CLEAR_MO);
        wdt.restart();
        iridium_sbd.sleep();
        wdt.restart();

        turn_iridium_off();
        wdt.restart();
        iridium_serial.end();
        wdt.restart();

        send_receive_last_went_through = false;
        return false;
    } else {
        Serial.println(F("iridium transmission successful"));
        last_message_went_through = true;
    }

    iridium_sbd.clearBuffers(ISBD_CLEAR_MO);
    wdt.restart();
    iridium_sbd.sleep();
    wdt.restart();

    turn_iridium_off();
    wdt.restart();
    iridium_serial.end();
    wdt.restart();

    send_receive_last_went_through = true;
    return true;
}

void IridiumManager::attempt_transmit_gps_fixes(size_t min_nbr_messages){
    bool gnss_iridium_went_through {false};

    if (gnss_manager.gps_fixes_buffer.size() >= min_nbr_messages){
      attempt_tried_sending = true;
      clear_buffers();
      size_t nbr_of_fixes_in_message = gnss_manager.write_message_to_buffer(iridium_tx_buffer);
      wdt.restart();

      Serial.print(F("attempting to transmit "));
      Serial.print(nbr_of_fixes_in_message);
      Serial.println(F(" fixes in iridium message"));

      gnss_iridium_went_through = send_receive_message();
      if (gnss_iridium_went_through){
        gnss_manager.clear_number_sent_fixes(nbr_of_fixes_in_message);
      }
      wdt.restart();

      Serial.print(F("number of fixes left in buffer: "));
      Serial.println(gnss_manager.gps_fixes_buffer.size());
    }
    else{
      attempt_tried_sending = false;
      Serial.print(F(" only "));
      Serial.print(gnss_manager.gps_fixes_buffer.size());
      Serial.print(F(" fixes, transmit a message when at least "));
      Serial.println(min_nbr_messages);
    }
}

#ifndef DISABLE_ALL_THERMISTOR
void IridiumManager::attempt_transmit_thermistors_packets(size_t min_nbr_packets){
    bool thermistors_data_went_through {false};

    if (board_thermistors_manager.thermistors_packets_buffer.size() >= min_nbr_packets){
      attempt_tried_sending = true;
      clear_buffers();
      size_t nbr_of_packets_in_message = board_thermistors_manager.write_message_to_buffer(iridium_tx_buffer);
      wdt.restart();

      Serial.print(F("attempting to transmit "));
      Serial.print(nbr_of_packets_in_message);
      Serial.println(F(" packets in iridium message"));

      thermistors_data_went_through = send_receive_message();
      if (thermistors_data_went_through){
        board_thermistors_manager.clear_number_sent_packets(nbr_of_packets_in_message);
      }
      wdt.restart();

      Serial.print(F("number of packets left in buffer: "));
      Serial.println(board_thermistors_manager.thermistors_packets_buffer.size());
    }
    else{
      attempt_tried_sending = false;
      Serial.print(F(" only "));
      Serial.print(board_thermistors_manager.thermistors_packets_buffer.size());
      Serial.print(F(" packets, transmit a message when at least "));
      Serial.println(min_nbr_packets);
    }
}
#endif

void IridiumManager::attempt_transmit_wave_spectra(){
    bool wave_spectrum_went_through {false};

    if (board_wave_analyzer.wave_packet_buffer.size() > 0){
        attempt_tried_sending = true;
        Serial.println(F("there are available wave packets to transmit"));

        clear_buffers();
        board_wave_analyzer.write_message_to_buffer(iridium_tx_buffer);

        wave_spectrum_went_through = send_receive_message();

        if (wave_spectrum_went_through){
            Serial.println(F("wave spectrum went through, pop_back"));
            board_wave_analyzer.wave_packet_buffer.pop_back();
        } else {
            Serial.println(F("transmission failed, try again later"));
        }
        wdt.restart();
    }
    else{
        attempt_tried_sending = false;
        Serial.println(F("no wave package to transmit"));
    }
}

// ============================================================================
// NEW: 2 Hz complex FFT coefficients (100 bins) → 2 messages (80 + 20)
// ============================================================================
void IridiumManager::attempt_transmit_fft2hz_coeffs_100bins(void){
    attempt_tried_sending = true;
    Serial.println(F("attempt_transmit_fft2hz_coeffs_100bins"));

    // Ensure we have fresh IMU data buffered at 10 Hz
    if (!board_wave_analyzer.gather_imu_data()){
        Serial.println(F("IMU gather failed – abort FFT coeff transmission"));
        attempt_tried_sending = false;
        return;
    }

    // First message: bins 1..80
    clear_buffers();
    if (board_wave_analyzer.build_fft2hz_coeff_message(1, WaveAnalyzer::fft2hz_max_bins_per_msg, iridium_tx_buffer)){
        (void)send_receive_message();
    } else {
        Serial.println(F("failed to build first FFT coeff message"));
        attempt_tried_sending = false;
        return;
    }

    // Second message: bins 81..100 (20 bins)
    clear_buffers();
    if (board_wave_analyzer.build_fft2hz_coeff_message(1 + WaveAnalyzer::fft2hz_max_bins_per_msg,
                                                       WaveAnalyzer::fft2hz_total_bins_to_send - WaveAnalyzer::fft2hz_max_bins_per_msg,
                                                       iridium_tx_buffer)){
        (void)send_receive_message();
    } else {
        Serial.println(F("failed to build second FFT coeff message"));
        // still keep attempt_tried_sending = true (we tried)
    }
}

void IridiumManager::reboot_if_requested_through_iridium(void){
    Serial.println(F("check if iridium requested reboot"));

    if (rx_message_available){
        Serial.println(F("message available"));

        if (iridium_rx_buffer.size() >= 4){
            if (iridium_rx_buffer[0] == 'B' && iridium_rx_buffer[1] == 'O' && iridium_rx_buffer[2] == 'O' && iridium_rx_buffer[3] == 'T'){
                Serial.println(F("BOOT message, reboot!"));
                while (true) {;}
            }
        } else {
            Serial.println(F("too short to be a reboot"));
        }
    }
}

// --- parsing helpers for incoming commands (unchanged) ---
int IridiumManager::read_value_from_command(size_t rx_ind_start){
    char crrt_buff[3]; crrt_buff[0]='0'; crrt_buff[1]='1'; crrt_buff[2]='\0';
    crrt_buff[0] = iridium_rx_buffer[rx_ind_start+0];
    crrt_buff[1] = iridium_rx_buffer[rx_ind_start+1];
    int res = atoi(crrt_buff);
    return res;
}

void IridiumManager::read_apply_iridium_instructions(void) {
  Serial.println(F("read apply iridium instructions"));

  size_t crrt_ind {0};
  while (iridium_rx_buffer.size() - crrt_ind >= 7){
   Serial.print(F("parse from start ind: ")); Serial.print(crrt_ind); Serial.print(F(" content ASCII ")); Serial.println(iridium_rx_buffer[crrt_ind]);

   if (iridium_rx_buffer[crrt_ind+0] == '$' && iridium_rx_buffer[crrt_ind+6] == ';'){
    Serial.println(F("valid message separators..."));

    int command_value = read_value_from_command(crrt_ind+4);
    Serial.print(F("command_value: ")); Serial.println(command_value);

    if (iridium_rx_buffer[crrt_ind+1] == 'G' && iridium_rx_buffer[crrt_ind+2] == 'F' && iridium_rx_buffer[crrt_ind+3] == 'Q'){
       Serial.println(F("this is a GFQ message"));
       if (command_value <= 6*4){
        Serial.println(F("command value ok, apply it."));
        modifiable_interval_between_gnss_measurements_seconds = 15 * 60 * command_value;
       } else { Serial.println(F("command value is above threshold; ignore")); }
    }

    if (iridium_rx_buffer[crrt_ind+1] == 'W' && iridium_rx_buffer[crrt_ind+2] == 'F' && iridium_rx_buffer[crrt_ind+3] == 'Q'){
       Serial.println(F("this is a WFQ message"));
       if (command_value <= 6*2){
        Serial.println(F("command value ok, apply it."));
        modifiable_interval_between_wave_measurements_seconds = 30 * 60 * command_value;
       } else { Serial.println(F("command value is above threshold; ignore")); }
    }

    if (iridium_rx_buffer[crrt_ind+1] == 'T' && iridium_rx_buffer[crrt_ind+2] == 'F' && iridium_rx_buffer[crrt_ind+3] == 'Q'){
       Serial.println(F("this is a TFQ message"));
       if (command_value <= 6*2){
        Serial.println(F("command value ok, apply it."));
        modifiable_interval_between_thermistors_measurements_seconds = 30 * 60 * command_value;
       } else { Serial.println(F("command value is above threshold; ignore")); }
    }

    if (iridium_rx_buffer[crrt_ind+1] == 'G' && iridium_rx_buffer[crrt_ind+2] == 'M' && iridium_rx_buffer[crrt_ind+3] == 'L'){
       Serial.println(F("this is a GML message"));
       if (command_value >= 1 && command_value <= 10){
        Serial.println(F("command value ok, apply it."));
        modifiable_min_nbr_GPS_fix_per_message = command_value;
       } else { Serial.println(F("command value is outside range; ignore")); }
    }

    crrt_ind += 7;
   }
   else{
     crrt_ind += 1;
   }
  }
}

bool IridiumManager::last_communication_was_successful(void) const { return send_receive_last_went_through; }
bool IridiumManager::last_attempt_tried_sending(void) const { return attempt_tried_sending; }

IridiumManager iridium_manager;
