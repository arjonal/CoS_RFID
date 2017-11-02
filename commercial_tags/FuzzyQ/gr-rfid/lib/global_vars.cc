/* -*- c++ -*- */
/* 
 * Developed by Laura Arjona, adapted from Nikos Kargas https://github.com/nikosl21/Gen2-UHF-RFID-Reader
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "rfid/global_vars.h"

#include <iostream>
namespace gr {
  namespace rfid {
    
    READER_STATE * reader_state;

    void initialize_reader_state()
    {
      reader_state = new READER_STATE;
      reader_state-> reader_stats.n_queries_sent = 0;
      reader_state-> reader_stats.n_epc_correct = 0;

      std::vector<int>  unique_tags_round;
       std::map<int,int> tag_reads;    

      reader_state-> status           = RUNNING;
      reader_state-> gen2_logic_status= START;
      reader_state-> gate_status       = GATE_SEEK_RN16;
      reader_state-> decoder_status   = DECODER_DECODE_RN16;

      reader_state-> reader_stats.cur_inventory_round = 1;
      reader_state-> reader_stats.cur_slot_number     = 1;

      reader_state-> reader_stats.n_k = 0; //Number of collision slots per frame
      reader_state-> reader_stats.n_1 = 0; //Number of success slots per frame
      reader_state-> reader_stats.n_0 = 0; //Number of idle slots per frame

      reader_state-> reader_stats.th = 0.0;
      reader_state-> reader_stats.TIR_th = 0.0;
      reader_state-> reader_stats.TIR_exp = 0.0;
      reader_state-> reader_stats.stop = 1;

      reader_state-> reader_stats.tn_k = 0; //Total Number of collision slots 
      reader_state-> reader_stats.tn_1 = 0; //Total Number of success slots 
      reader_state-> reader_stats.tn_0 = 0; //Total Number of idle slots 
      reader_state-> reader_stats.tQA = 0; //Total Number of QA sent
      reader_state-> reader_stats.tQR = 0; //Total Number of QR sent
      reader_state-> reader_stats.tQ = 1; //Total Number of Q sent
      reader_state-> reader_stats.it_timer = 0.0;

      reader_state-> reader_stats.Qupdn = 1; //posible values: 0,1,2
      reader_state-> reader_stats.output_energy = 0;
      
      reader_state-> reader_stats.VAR_Q = 2; //Initial Q value 
      reader_state-> reader_stats.Qant = 2; 

      reader_state-> reader_stats.pointer_slot = 2;
      // For the initial pointer slot round(L/9)
      // Qo = 2 -> pointer_o = 1
      // Qo = 3 -> pointer_o = 2
      // Qo = 4 -> pointer_o = 2
      // Qo = 5 -> pointer_o = 4
      // Qo = 6 -> pointer_o = 7
      // Qo = 7 -> pointer_o = 14

      //VALORES A EVALUAR 
      //en todos evaluar Q=4, porque esl a recom. del estandar y luego ademas probar con q-1,1,q+1 (como tenemos en el Nova Chapter)
      // 16 tags -> Qo = 3,4,5
      // 32 tags -> Qo = 3,4,5,6
      // 64 tags -> Qo = 3,4,5,6,7
    


      gettimeofday (&reader_state-> reader_stats.start, NULL);
    }
  } /* namespace rfid */
} /* namespace gr */

