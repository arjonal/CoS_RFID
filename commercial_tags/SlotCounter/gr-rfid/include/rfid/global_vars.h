/* -*- c++ -*- */
/* 
 * Developed by Laura Arjona, 
 * adapted from Nikos Kargas https://github.com/nikosl21/Gen2-UHF-RFID-Reader
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RFID_GLOBAL_VARS_H
#define INCLUDED_RFID_GLOBAL_VARS_H

#include <rfid/api.h>
#include <map>
#include <sys/time.h>

namespace gr {
  namespace rfid {

    enum STATUS             {RUNNING, TERMINATED};
    enum GEN2_LOGIC_STATUS  {SEND_QUERY, SEND_ACK, SEND_QUERY_REP, IDLE, SEND_CW_ACK,SEND_CW_QUERY, START, SEND_QUERY_ADJUST, SEND_NAK_QR, SEND_NAK_Q, POWER_DOWN}; 
    enum GATE_STATUS        {GATE_OPEN, GATE_CLOSED, GATE_SEEK_RN16, GATE_SEEK_EPC};  
    enum DECODER_STATUS     {DECODER_DECODE_RN16, DECODER_DECODE_EPC};
    

    struct READER_STATS
    {
      int n_queries_sent;

      int cur_inventory_round;
      int cur_slot_number;

      int max_slot_number;
      int max_inventory_round;

      int n_epc_correct;

       int n_0;
      int n_1;
      int n_k;

      int tn_k;  
      int tn_1;
      int tn_0;
      int tQA;
      int tQR;
      int tQ;

      float output_energy;
      
      int VAR_Q;   
      int Qant; 
      float Qfp; 
      int Qupdn;

      float it_timer;

      float th;
      float TIR_th;
      float TIR_exp;
      int stop;

      std::vector<int>  unique_tags_round;
       std::map<int,int> tag_reads;    

      struct timeval start, end; 
    };

    struct READER_STATE
    {
      STATUS               status;
      GEN2_LOGIC_STATUS   gen2_logic_status;
      GATE_STATUS         gate_status;
      DECODER_STATUS       decoder_status;
      READER_STATS         reader_stats;



      std::vector<float> magn_squared_samples; // used for sync
      int n_samples_to_ungate; // used by the GATE and DECODER block
    };

    // CONSTANTS (READER CONFIGURATION)

    //const int MAX_INVENTORY_ROUND = 64;
    const int MAX_NUM_QUERIES     = 2048;     // Stop after MAX_NUM_QUERIES have been sent


    // valid values for Q
    const int Q_VALUE [16][4] =  
    {
        {0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1}, 
        {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1}, 
        {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
        {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}
    };  

    const float C = 0.3;
    const bool P_DOWN = false;

    // Duration in us
    const int CW_D         = 250;    // Carrier wave
    const int P_DOWN_D     = 2000;    // power down
    const int T1_D         = 240;    // Time from Interrogator transmission to Tag response (250 us) // default 240
    const int T2_D         = 480;    // Time from Tag response to Interrogator transmission. Max value = 20.0 * T_tag = 500us  // default 480
    const int PW_D         = 12;      // Half Tari 
    const int DELIM_D       = 12;      // A preamble shall comprise a fixed-length start delimiter 12.5us +/-5%
    const int TRCAL_D     = 200;    // BLF = DR/TRCAL => 40e3 = 8/TRCAL => TRCAL = 200us
    const int RTCAL_D     = 72;      // 6*PW = 72us

    const int NUM_PULSES_COMMAND = 5;       // Number of pulses to detect a reader command
    const int NUMBER_UNIQUE_TAGS = 60;      // Stop after NUMBER_UNIQUE_TAGS have been read 


    // Number of bits
    const int PILOT_TONE          = 12;  // Optional
    const int TAG_PREAMBLE_BITS  = 6;   // Number of preamble bits
    const int RN16_BITS          = 17;  // Dummy bit at the end
    const int EPC_BITS            = 129;  // PC + EPC + CRC16 + Dummy = 6 + 16 + 96 + 16 + 1 = 135
    const int QUERY_LENGTH        = 22;  // Query length in bits
    
    const int T_READER_FREQ = 40e3;     // BLF = 40kHz
    const float TAG_BIT_D   = 1.0/T_READER_FREQ * pow(10,6); // Duration in us
    const int RN16_D        = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
    const int EPC_D          = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;
    // Query command 
    const int QUERY_CODE[4] = {1,0,0,0};
    const int M[2]          = {0,0};
    const int SEL[2]         = {0,0}; // All
    const int SESSION[2]     = {0,0}; //S0
    const int TARGET         = 0; //Inventoried flag A
    const int TREXT         = 0;
    const int DR            = 0;


    //Slots duration
    const int T_pr = DELIM_D+ 2*PW_D + RTCAL_D + TRCAL_D; //Preamble duration in us
    const int T_FSY = DELIM_D+ 2*PW_D + RTCAL_D; //Frame Sync duration in us
    const float Rdr = 2/(3*2*PW_D* pow(10,-6)); //Reader data rate (bps)

    const float Tack = T_FSY + (18/Rdr)* pow(10,6) ; //in us
    const float TQR  = T_FSY + (4/Rdr) * pow(10,6) ; //in us
    const float TQA  = T_FSY + (9/Rdr) * pow(10,6) ; //in us
    const float TQ   = T_pr  + (22/Rdr)* pow(10,6) ; //in us

    const float Tsk = T1_D + RN16_D + T2_D + Tack + T1_D + EPC_D + T2_D;  //Duration of single/collision slot in us
    const float Ti  = T1_D + RN16_D + T2_D; //Duration of idle slot in us

    const int NAK_CODE[8]   = {1,1,0,0,0,0,0,0};

    // ACK command
    const int ACK_CODE[2]   = {0,1};

    // QueryAdjust command
    const int QADJ_CODE[4]   = {1,0,0,1};

    // 110 Increment by 1, 000 unchanged, 011 decrement by 1
    const int Q_UPDN[3][3]  = { {1,1,0}, {0,0,0}, {0,1,1} };

    // FM0 encoding preamble sequences
    const int TAG_PREAMBLE[] = {1,1,0,1,0,0,1,0,0,0,1,1};

    // Gate block parameters
    const float THRESH_FRACTION = 0.75;     
    const int WIN_SIZE_D         = 250; 

    // Duration in which dc offset is estimated
    const int DC_SIZE_D         = 120;

    // Global variable
    extern READER_STATE * reader_state;
    extern void initialize_reader_state();

    const float E_th = 0.03;


  } // namespace rfid
} // namespace gr

#endif /* INCLUDED_RFID_GLOBAL_VARS_H */

