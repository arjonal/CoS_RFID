/* -*- c++ -*- */
/* 
 * Copyright 2018 < Laura Arjona (laura.arjona@deusto.es)>. *
  
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
#include <bitset>
using namespace std;

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

      //GR_LOG_INFO(d_logger, "Block initialized");
      
      command_bits = (char *) malloc( sizeof(char) * 24);
      sample_d = 1.0/dac_rate * pow(10,6);

      // Number of samples for transmitting
      n_data0_s = 2 * PW_D / sample_d;
      n_data1_s = 4 * PW_D / sample_d;
      n_pw_s    = PW_D    / sample_d;
      n_cw_s    = CW_D    / sample_d;
      n_delim_s = DELIM_D / sample_d;
      n_trcal_s = TRCAL_D / sample_d;

      
      // CW waveforms of different sizes
      n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16
      n_cwack_s     = (1*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag
      n_cwreq_s   = (T1_D+T2_D+HANDLE_D)/sample_d;     //Handle or new rn16
      n_cwread_s   = (T1_D+T2_D+READ_D)/sample_d;     //READ
      n_p_down_s     = (1*P_DOWN_D)/sample_d;  

      p_down.resize(n_p_down_s);        // Power down samples
      cw_query.resize(n_cwquery_s);      // Sent after query/query rep
      cw_ack.resize(n_cwack_s);          // Sent after ack
      cw_start.resize(n_cw_s);          // Sent after start
      cw_req_rn16.resize(n_cwreq_s);          // Sent after Req_RN16
      cw_read.resize(n_cwread_s);          // Sent after READ


      std::fill_n(cw_query.begin(), cw_query.size(), 1);
      std::fill_n(cw_ack.begin(), cw_ack.size(), 1);
      std::fill_n(cw_start.begin(), cw_start.size(), 1);
      std::fill_n(cw_req_rn16.begin(), cw_req_rn16.size(), 1);
      std::fill_n(cw_read.begin(), cw_read.size(), 1);

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
      crc_append(query_bits,17);
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


    void reader_impl::gen_req_rn16_bits(const float * in)
    {
      req_rn16_bits.resize(0);
      req_rn16_bits.insert(req_rn16_bits.end(), &REQ_RN16_CODE[0], &REQ_RN16_CODE[8]);
      req_rn16_bits.insert(req_rn16_bits.end(), &in[0], &in[16]);
      crc16_append(req_rn16_bits,24,command_bits);
      
    }

  void reader_impl::gen_read_bits(const float * in)
    {
      read_bits.resize(0);
      read_bits.insert(read_bits.end(), &READ_CODE[0], &READ_CODE[8]);
      read_bits.insert(read_bits.end(), &MemBank[0], &MemBank[2]);
      read_bits.insert(read_bits.end(), &WordPtr[0], &WordPtr[8]);
      read_bits.insert(read_bits.end(), &Wordcount[0], &Wordcount[8]);
      read_bits.insert(read_bits.end(), &in[0], &in[16]);
      crc16_append(read_bits,42,command_bits);
      
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
      std::cout << "| Number of reader commands sent : " << reader_state->reader_stats.n_queries_sent  << std::endl;
      //std::cout << "| Current Inventory round : "          << reader_state->reader_stats.cur_inventory_round << std::endl;
      std::cout << " --------------------------"            << std::endl;

      std::cout << "| Total collision slots : "  <<  reader_state-> reader_stats.tn_k     << std::endl;
      std::cout << "| Total idle slots : "  <<  reader_state-> reader_stats.tn_0     << std::endl;
      std::cout << "| Total success slots : "  <<  reader_state-> reader_stats.tn_1     << std::endl;
      std::cout << "| total QA : "  <<  reader_state-> reader_stats.tQA     << std::endl;
      std::cout << "| total QR : "  <<  reader_state-> reader_stats.tQR     << std::endl;
      std::cout << "| total Q : "  <<  reader_state-> reader_stats.tQ     << std::endl;
      std::cout << " --------------------------"            << std::endl;

      std::cout << "| Correctly decoded EPC : "  <<  reader_state->reader_stats.n_epc_correct     << std::endl;
      std::cout << "| Number of unique tags : "  <<  reader_state->reader_stats.tag_reads.size() << std::endl;

            std::cout << "| ----------------------------------------------------------------------- " <<  std::endl;
           
            std::cout << "| TIR theoretic : "  <<  reader_state-> reader_stats.TIR_th     << std::endl;
            std::cout << "| TIR experimental : "  <<  reader_state-> reader_stats.TIR_exp << std::endl;
            //std::cout << "| Throughtput : "  <<  reader_state->reader_stats.th     << std::endl;
            //std::cout << "| ----------------------------------------------------------------------- " <<  std::endl;
            
      std::map<int,int>::iterator it;

      for(it = reader_state->reader_stats.tag_reads.begin(); it != reader_state->reader_stats.tag_reads.end(); it++) 
      {
        std::cout << std::hex <<  "| Tag ID : " << it->first << "  ";
        std::cout << "Num of reads : " << std::dec << it->second << std::endl;
      }
           
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

          reader_state->reader_stats.n_queries_sent +=1;

         //GR_LOG_INFO(d_debug_logger, "START");
          memcpy(&out[written], &cw_start[0], sizeof(float) * cw_start.size() );
          written += cw_start.size();

          
          if (reader_state-> reader_stats.n_queries_sent <10){
            reader_state->gen2_logic_status = START;    
          }
          else {
            reader_state->gen2_logic_status = SEND_QUERY;    
          }        
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

        gettimeofday (&reader_state-> reader_stats.start, NULL);//start timer

         // std::cout << " ------- QUERY -----------  " << std::endl;
            gen_query_bits();
          
          reader_state->reader_stats.n_k = 0;
          reader_state->reader_stats.n_1 = 0;
          reader_state->reader_stats.n_0 = 0;
        
          reader_state-> reader_stats.tQ += 1;
          //GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);

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
          // Send CW for Query
          reader_state->gen2_logic_status = SEND_CW_QUERY; 
 
          break;

        case SEND_ACK:

        
        //std::cout <<  "SEND ACK" << std::endl;

          //GR_LOG_INFO(d_debug_logger, "SEND ACK");
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
                //std::cout << "1 "<< std::endl;
              }
              else  
              {
                memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
                written += data_0.size();
                //std::cout << "0 "<< std::endl;
              }
            }


            consumed = ninput_items[0];
            reader_state->gen2_logic_status = SEND_CW_ACK; 
          }

          break;

        case SEND_CW_ACK:

          GR_LOG_INFO(d_debug_logger, "SEND CW - ack");
          memcpy(&out[written], &cw_ack[0], sizeof(float) * cw_ack.size() );
          written += cw_ack.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;

        case SEND_CW_QUERY:

          GR_LOG_INFO(d_debug_logger, "SEND CW - query");
          memcpy(&out[written], &cw_query[0], sizeof(float) * cw_query.size() );
          written+=cw_query.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;


        case SEND_CW_REQ:

          memcpy(&out[written], &cw_req_rn16[0], sizeof(float) * cw_req_rn16.size() );
          written += cw_req_rn16.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;
        
        case SEND_CW_READ:

          memcpy(&out[written], &cw_read[0], sizeof(float) * cw_read.size() );
          written += cw_read.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;


        case SEND_QUERY_REP:

         // std::cout << " ------- QUERY-REP  -----------  " << std::endl;
          reader_state->reader_stats.tQR +=1;


          //GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);
          
          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;
          reader_state->reader_stats.n_queries_sent +=1;  

          memcpy(&out[written], &query_rep[0], sizeof(float) * query_rep.size() );
          written += query_rep.size();
          reader_state->gen2_logic_status = SEND_CW_QUERY; 
          break;
      
        case SEND_QUERY_ADJUST:

          //std::cout << " ------- QUERY-ADJUST -----------  " << std::endl;
          reader_state->reader_stats.tQA +=1;

          gen_query_adjust_bits();
          //GR_LOG_INFO(d_debug_logger, "SEND QUERY_ADJUST");
          
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


//-----------------------------------------------------------------------------------------------------

    case SEND_REQ_RN16:      
            std::cout << " SEND REQUEST HANDLE" << std::endl;

          reader_state->decoder_status = DECODER_DECODE_HANDLE;
          reader_state->gate_status    = GATE_SEEK_HANDLE;
        
          //Transmit: command + RN16 + CRC
           gen_req_rn16_bits(in);
          
           // Send FrameSync
            memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
            written += frame_sync.size();


           for(int i = 0; i < req_rn16_bits.size(); i++)
            {
              if(req_rn16_bits[i] == 1)
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
             //---------------------------------------------------------------         
             // for (std::vector<float>::const_iterator i = req_rn16_bits.begin(); i != req_rn16_bits.end(); ++i)
             // std::cout << *i;
             // std::cout << "--BITS OF REQ COMMAND" << std::endl;
              //--------------------------------------------------------------- 

            consumed = ninput_items[0];
            reader_state->gen2_logic_status = SEND_CW_REQ; 
        
          break;

//-----------------------------------------------------------------------------------------------------
    case SEND_READ:      
         
          //std::cout << " SEND READ COMMAND" << std::endl;

          reader_state->decoder_status = DECODER_DECODE_READ;
          reader_state->gate_status    = GATE_SEEK_READ;
          reader_state->reader_stats.n_queries_sent +=1; 


          //Transmit: command + MenmBank + WordPtr + WordCount + RN + CRC16
           gen_read_bits(in);
          
           // Send FrameSync
            memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
            written += frame_sync.size();


           for(int i = 0; i < read_bits.size(); i++)
            {
              if(read_bits[i] == 1)
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
            reader_state->gen2_logic_status = SEND_CW_READ; 

          break;


 //-----------------------------------------------------------------------------------------------------

        default:
          // IDLE
        
          break;
      }
      consume_each (consumed);
      return  written;
    }

    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    void reader_impl::crc_append(std::vector<float> & q, int num_bits)
    {
      
       int crc[] = {1,0,0,1,0};

      for(int i = 0; i < num_bits; i++) //17 because length of query is 17+CRC_5
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



    void reader_impl::crc16_append(std::vector<float> & q, int num_bits,char * bits_command)
    {
      register unsigned short i, j;
      register unsigned short crc_16;
      unsigned char * data;
      //unsigned short crc16_short;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );

      int mask;
      
      //--------------------------------------------------------------
      
      for (int i =0; i < num_bits; i ++)
        {
        if (q[i] == 0)
           bits_command[i] = '0';
        else
          bits_command[i] = '1';
        }

      //--------------------------------------------------------------     

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits_command[(i * 8) + j] == '1'){
          data[i] = data[i] | mask;
        }
        mask = mask >> 1;
        }
      }
      //--------------------------------------------------------------
      crc_16 = 0xFFFF; 
      for (i=0; i < num_bytes; i++)
      {
        crc_16^=data[i] << 8;
        for (j=0;j<8;j++)
        {
          if (crc_16&0x8000)
          {
            crc_16 <<= 1;
            crc_16 ^= 0x1021;
          }
          else
            crc_16 <<= 1;
        }
      }
      crc_16 = ~crc_16;

      
    //std::cout << " ----CRC16 of Req-rn16 ------" <<   crc_16 << std::endl;
    //std::cout << " ----------"  << std::endl;

    //crc16_short = crc_16;  
    
    bitset<16> crc16_string(crc_16); 

    //std::cout << " BIT STRING OF CRn16  of Req-rn16   " <<  crc16_string<< std::endl;
    //std::cout << " ----------"  << std::endl;

    for (int i = 15; i >= 0; i--)
        q.push_back(crc16_string[i]);
    
    } //End of crc16 append function

    

  } /* namespace rfid */
} /* namespace gr */

