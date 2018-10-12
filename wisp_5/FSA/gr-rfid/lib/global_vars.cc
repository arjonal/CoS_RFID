/* -*- c++ -*- */
/* 
 * Copyright 2018 <Laura Arjona Aguilera (laura.arjona@deusto.es)>.
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

      reader_state-> reader_stats.output_energy = 0;

      reader_state-> reader_stats.tn_k = 0; //Total Number of collision slots 
      reader_state-> reader_stats.tn_1 = 0; //Total Number of success slots 
      reader_state-> reader_stats.tn_0 = 0; //Total Number of idle slots 
      reader_state-> reader_stats.tQA = 0; //Total Number of QA sent
      reader_state-> reader_stats.tQ = 0; //Total Number of Q sent
      reader_state-> reader_stats.tQR = 0; //Total Number of QR sent
      reader_state-> reader_stats.sensor_read = 0;
      

      reader_state-> reader_stats.it_timer = 0.0;


      reader_state-> reader_stats.th = 0.0;
      reader_state-> reader_stats.TIR_th = 0.0;
      reader_state-> reader_stats.TIR_exp = 0.0;
      reader_state-> reader_stats.stop = 1;


      //reader_state-> reader_stats.Qupdn = 1; //posible values: 0,1,2

      std::vector<int>  unique_tags_round;
       std::map<int,int> tag_reads; 
       std::vector<float> RN16_bits_handle;  
       std::vector<float> RN16_bits_read; 

      reader_state-> status           = RUNNING;
      reader_state-> gen2_logic_status= START;
      reader_state-> gate_status       = GATE_SEEK_RN16;
      reader_state-> decoder_status   = DECODER_DECODE_RN16;

      reader_state-> reader_stats.max_slot_number = pow(2,FIXED_Q);

      reader_state-> reader_stats.cur_inventory_round = 1;
      reader_state-> reader_stats.cur_slot_number     = 1;

      //gettimeofday (&reader_state-> reader_stats.start, NULL);
    }
  } /* namespace rfid */
} /* namespace gr */

