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
#include "reader_impl.h"
#include "rfid/global_vars.h"
#include <sys/time.h>

namespace gr {
  namespace rfid {

    reader::sptr
    reader::make(int sample_rate, int dac_rate)
    {
      return gnuradio::get_initial_sptr
        (new reader_impl(sample_rate,dac_rate));
    }

    /*
     * The private constructor
     */
    reader_impl::reader_impl(int sample_rate, int dac_rate)
      : gr::block("reader",
              gr::io_signature::make( 1, 1, sizeof(float)),
              gr::io_signature::make( 1, 1, sizeof(float)))
    {

      GR_LOG_INFO(d_logger, "Block initialized");

      sample_d = 1.0/dac_rate * pow(10,6);

      // Number of samples for transmitting

      n_data0_s = 2 * PW_D / sample_d;
      n_data1_s = 4 * PW_D / sample_d;
      n_pw_s    = PW_D    / sample_d;
      n_cw_s    = CW_D    / sample_d;
      n_delim_s = DELIM_D / sample_d;
      n_trcal_s = TRCAL_D / sample_d;

      GR_LOG_INFO(d_logger, "Number of samples data 0 : " << n_data0_s);
      GR_LOG_INFO(d_logger, "Number of samples data 1 : " << n_data1_s);
      GR_LOG_INFO(d_logger, "Number of samples cw : "     << n_cw_s);
      GR_LOG_INFO(d_logger, "Number of samples delim : "  << n_delim_s);
      GR_LOG_INFO(d_logger, "Number of slots : "          << std::pow(2,reader_state->reader_stats.VAR_Q));

      // CW waveforms of different sizes
      n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16
      n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag
     
      n_p_down_s     = (P_DOWN_D)/sample_d;  

      p_down.resize(n_p_down_s);        // Power down samples
      cw_query.resize(n_cwquery_s);      // Sent after query/query rep
      cw_ack.resize(n_cwack_s);          // Sent after ack

      std::fill_n(cw_query.begin(), cw_query.size(), 1);
      std::fill_n(cw_ack.begin(), cw_ack.size(), 1);

      GR_LOG_INFO(d_logger, "Carrier wave after a query transmission in samples : "     << n_cwquery_s);
      GR_LOG_INFO(d_logger, "Carrier wave after ACK transmission in samples : "        << n_cwack_s);

      // Construct vectors (resize() default initialization is zero)
      data_0.resize(n_data0_s);
      data_1.resize(n_data1_s);
      cw.resize(n_cw_s);
      delim.resize(n_delim_s);
      rtcal.resize(n_data0_s + n_data1_s);
      trcal.resize(n_trcal_s);

      // Fill vectors with data
      std::fill_n(data_0.begin(), data_0.size()/2, 1);
      std::fill_n(data_1.begin(), 3*data_1.size()/4, 1);
      std::fill_n(cw.begin(), cw.size(), 1);
      std::fill_n(rtcal.begin(), rtcal.size() - n_pw_s, 1); // RTcal
      std::fill_n(trcal.begin(), trcal.size() - n_pw_s, 1); // TRcal

      // create preamble
      preamble.insert( preamble.end(), delim.begin(), delim.end() );
      preamble.insert( preamble.end(), data_0.begin(), data_0.end() );
      preamble.insert( preamble.end(), rtcal.begin(), rtcal.end() );
      preamble.insert( preamble.end(), trcal.begin(), trcal.end() );

      // create framesync
      frame_sync.insert( frame_sync.end(), delim.begin() , delim.end() );
      frame_sync.insert( frame_sync.end(), data_0.begin(), data_0.end() );
      frame_sync.insert( frame_sync.end(), rtcal.begin() , rtcal.end() );
      
      // create query rep
      query_rep.insert( query_rep.end(), frame_sync.begin(), frame_sync.end());
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );

      // create nak
      nak.insert( nak.end(), frame_sync.begin(), frame_sync.end());
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
    }

    void reader_impl::gen_query_bits()
    {
      int num_ones = 0, num_zeros = 0;

      query_bits.resize(0);
      query_bits.insert(query_bits.end(), &QUERY_CODE[0], &QUERY_CODE[4]);
      query_bits.push_back(DR);
      query_bits.insert(query_bits.end(), &M[0], &M[2]);
      query_bits.push_back(TREXT);
      query_bits.insert(query_bits.end(), &SEL[0], &SEL[2]);
      query_bits.insert(query_bits.end(), &SESSION[0], &SESSION[2]);
      query_bits.push_back(TARGET);
    
      query_bits.insert(query_bits.end(), &Q_VALUE[reader_state->reader_stats.VAR_Q][0], &Q_VALUE[reader_state->reader_stats.VAR_Q][4]);
      crc_append(query_bits);
    }


    void reader_impl::gen_ack_bits(const float * in)
    {
      ack_bits.resize(0);
      ack_bits.insert(ack_bits.end(), &ACK_CODE[0], &ACK_CODE[2]);
      ack_bits.insert(ack_bits.end(), &in[0], &in[16]);
    }
  
    void reader_impl::gen_query_adjust_bits()
    {
      query_adjust_bits.resize(0);
      query_adjust_bits.insert(query_adjust_bits.end(), &QADJ_CODE[0], &QADJ_CODE[4]);
      query_adjust_bits.insert(query_adjust_bits.end(), &SESSION[0], &SESSION[2]);
      query_adjust_bits.insert(query_adjust_bits.end(), &Q_UPDN[reader_state-> reader_stats.Qupdn][0], &Q_UPDN[reader_state-> reader_stats.Qupdn][3]);
    }


    /*
     * Our virtual destructor.
     */
    reader_impl::~reader_impl()
    {

    }

    void reader_impl::print_results()
    {
      std::cout << "\n --------------------------" << std::endl;
      std::cout << "| Number of Q+QR+QA sent : " << reader_state->reader_stats.n_queries_sent - 1 << std::endl;
      std::cout << "| Number of QA sent : " << reader_state->reader_stats.tQA << std::endl;
      std::cout << "| Number of QR sent : " << reader_state->reader_stats.tQR << std::endl;
      std::cout << "| Number of Q sent : " << reader_state->reader_stats.tQ << std::endl;
      std::cout << "| Current Inventory round : "          << reader_state->reader_stats.cur_inventory_round << std::endl;
      std::cout << " -------------------------------------------------"            << std::endl;

      std::cout << "| Correctly decoded EPC : "  <<  reader_state->reader_stats.n_epc_correct     << std::endl;
      std::cout << "| Number of unique tags : "  <<  reader_state->reader_stats.tag_reads.size() << std::endl;

      std::map<int,int>::iterator it;

      std::cout << " ----------------------------------------------"            << std::endl;
      std::cout << " ------------ **************************   ------------"    << std::endl;
      std::cout << "| Throughtput : "  <<  reader_state->reader_stats.th     << std::endl;
      std::cout << "| TIR theoretic : "  <<  reader_state-> reader_stats.TIR_th     << std::endl;
      std::cout << "| TIR experimental : "  <<  reader_state-> reader_stats.TIR_exp << std::endl;



    }

    void
    reader_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = 0;
    }

    int
    reader_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      const float *in = (const float *) input_items[0];
      float *out =  (float*) output_items[0];
      std::vector<float> out_message; 
      int n_output;
      int consumed = 0;
      int written = 0;

      consumed = ninput_items[0];
  
      switch (reader_state->gen2_logic_status)
      {
        

        case START:
          GR_LOG_INFO(d_debug_logger, "START");
          memcpy(&out[written], &cw_ack[0], sizeof(float) * cw_ack.size() );
          written += cw_ack.size();

          gettimeofday (&reader_state-> reader_stats.start, NULL);//start timer

          reader_state->gen2_logic_status = SEND_QUERY;    
          break;

        case POWER_DOWN:
          GR_LOG_INFO(d_debug_logger, "POWER DOWN");
          memcpy(&out[written], &p_down[0], sizeof(float) * p_down.size() );
          written += p_down.size();
          reader_state->gen2_logic_status = START;    
          break;

        case SEND_NAK_QR:
          GR_LOG_INFO(d_debug_logger, "SEND NAK");
          memcpy(&out[written], &nak[0], sizeof(float) * nak.size() );
          written += nak.size();
          memcpy(&out[written], &cw[0], sizeof(float) * cw.size() );
          written+=cw.size();
          reader_state->gen2_logic_status = SEND_QUERY_REP;    
          break;

        case SEND_NAK_Q:
          GR_LOG_INFO(d_debug_logger, "SEND NAK");
          memcpy(&out[written], &nak[0], sizeof(float) * nak.size() );
          written += nak.size();
          memcpy(&out[written], &cw[0], sizeof(float) * cw.size() );
          written+=cw.size();
          reader_state->gen2_logic_status = SEND_QUERY;    
          break;

        case SEND_QUERY:
         
          gen_query_bits();

          // -------------SET NEW FRAME SIZE --------------------------------------

          reader_state->reader_stats.n_k = 0;
          reader_state->reader_stats.n_1 = 0;
          reader_state->reader_stats.n_0 = 0;


          GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);
          //std::cout << "| Total slots : "  <<  pow(2,reader_state-> reader_stats.VAR_Q)    << std::endl;

          reader_state->reader_stats.n_queries_sent +=1;  
          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;

          memcpy(&out[written], &preamble[0], sizeof(float) * preamble.size() );
          written+=preamble.size();
   
          for(int i = 0; i < query_bits.size(); i++)
          {
            if(query_bits[i] == 1)
            {
              memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
              written+=data_1.size();
            }
            else
            {
              memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
              written+=data_0.size();
            }
          }
 
          reader_state->gen2_logic_status = SEND_CW_QUERY; 
    
          break;

        case SEND_ACK:

          GR_LOG_INFO(d_debug_logger, "SEND ACK");
          if (ninput_items[0] == RN16_BITS - 1)
          {

            // Controls the other two blocks
            reader_state->decoder_status = DECODER_DECODE_EPC;
            reader_state->gate_status    = GATE_SEEK_EPC;

            gen_ack_bits(in);
          
            // Send FrameSync
            memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
            written += frame_sync.size();

            for(int i = 0; i < ack_bits.size(); i++)
            {
              if(ack_bits[i] == 1)
              {
                memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
                written += data_1.size();
              }
              else  
              {
                memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
                written += data_0.size();
              }
            }
             consumed = ninput_items[0];
            reader_state->gen2_logic_status = SEND_CW_ACK; 
          }

          break;

        case SEND_CW_ACK:

          GR_LOG_INFO(d_debug_logger, "SEND CW");
          memcpy(&out[written], &cw_ack[0], sizeof(float) * cw_ack.size() );
          written += cw_ack.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;

        case SEND_CW_QUERY:

          GR_LOG_INFO(d_debug_logger, "SEND CW");
          memcpy(&out[written], &cw_query[0], sizeof(float) * cw_query.size() );
          written+=cw_query.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;

        case SEND_QUERY_REP:

          GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);
          

          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;
          reader_state->reader_stats.n_queries_sent +=1;  

          memcpy(&out[written], &query_rep[0], sizeof(float) * query_rep.size() );
          written += query_rep.size();

          reader_state->gen2_logic_status = SEND_CW_QUERY; 
          break;
      
        case SEND_QUERY_ADJUST: 

          GR_LOG_INFO(d_debug_logger, "SEND QUERY_ADJUST");

          gen_query_adjust_bits();

          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;
          reader_state->reader_stats.n_queries_sent +=1;  

          memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
          written += frame_sync.size();

          for(int i = 0; i < query_adjust_bits.size(); i++)
          {
             
            if(query_adjust_bits[i] == 1)
            {
              memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
              written+=data_1.size();
            }
            else
            {
              memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
              written+=data_0.size();
            }
          }

          reader_state->gen2_logic_status = SEND_CW_QUERY; 
          break;

        default:
          // IDLE
        
          break;
      }
      consume_each (consumed);
      return  written;
    }


    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    void reader_impl::crc_append(std::vector<float> & q)
    {
      
       int crc[] = {1,0,0,1,0};

      for(int i = 0; i < 17; i++)
      {
        int tmp[] = {0,0,0,0,0};
        tmp[4] = crc[3];
        if(crc[4] == 1)
        {
          if (q[i] == 1)
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
          else
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
        }
        else
        {
          if (q[i] == 1)
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
          else
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
        }
        memcpy(crc, tmp, 5*sizeof(float));
      }
      for (int i = 4; i >= 0; i--)
        q.push_back(crc[i]);
    }
  } /* namespace rfid */
} /* namespace gr */

