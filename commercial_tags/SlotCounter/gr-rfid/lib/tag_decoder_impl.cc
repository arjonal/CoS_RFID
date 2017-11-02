/* -*- c++ -*- */
/* 
 * Developed by Laura Arjona, 
 * adapted from Nikos Kargas https://github.com/nikosl21/Gen2-UHF-RFID-Reader
 *
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
#include <gnuradio/prefs.h>
#include <gnuradio/math.h>
#include <cmath>
#include <sys/time.h>
#include "tag_decoder_impl.h"
#include <iostream>
#include <fstream>
using namespace std;


namespace gr {
  namespace rfid {

    tag_decoder::sptr
    tag_decoder::make(int sample_rate)
    {

      std::vector<int> output_sizes;
      output_sizes.push_back(sizeof(float));
      output_sizes.push_back(sizeof(gr_complex));

      return gnuradio::get_initial_sptr
        (new tag_decoder_impl(sample_rate,output_sizes));
    }

    /*
     * The private constructor
     */
    tag_decoder_impl::tag_decoder_impl(int sample_rate, std::vector<int> output_sizes)
      : gr::block("tag_decoder",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::makev(2, 2, output_sizes )),
              s_rate(sample_rate)
    {


      char_bits = (char *) malloc( sizeof(char) * 128);

      n_samples_TAG_BIT = TAG_BIT_D * s_rate / pow(10,6);      
    }

    /*
     * Our virtual destructor.
     */
    tag_decoder_impl::~tag_decoder_impl()
    {

    }

    void
    tag_decoder_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        ninput_items_required[0] = noutput_items;
    }

    int tag_decoder_impl::tag_sync(const gr_complex * in , int size, int flag)
    {
      int max_index = 0;
      float max = 0,corr;
      gr_complex corr2;
      
      // Do not have to check entire vector (not optimal)
      for (int i=0; i < 1.5 * n_samples_TAG_BIT ; i++)
      {
        corr2 = gr_complex(0,0);
        corr = 0;
        // sync after matched filter (equivalent)
        for (int j = 0; j < 2 * TAG_PREAMBLE_BITS; j ++)
        {
          corr2 = corr2 + in[ (int) (i+j*n_samples_TAG_BIT/2) ] * gr_complex(TAG_PREAMBLE[j],0);
        }
        corr = std::norm(corr2);
        if (corr > max)
        {
          max = corr;
          max_index = i;
        }
      }  

      if(flag == 1)
      {
        reader_state->reader_stats.output_energy = max;
        //GR_LOG_INFO(d_logger, " Energy of received signal when RN16: " << reader_state->reader_stats.output_energy);
        
      }
      
      // Preamble ({1,1,-1,1,-1,-1,1,-1,-1,-1,1,1} 1 2 4 7 11 12)) 
      h_est = (in[max_index] + in[ (int) (max_index + n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 3*n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 6*n_samples_TAG_BIT/2)] + in[(int) (max_index + 10*n_samples_TAG_BIT/2) ] + in[ (int) (max_index + 11*n_samples_TAG_BIT/2)])/std::complex<float>(6,0);  


      // Shifted received waveform by n_samples_TAG_BIT/2
      max_index = max_index + TAG_PREAMBLE_BITS * n_samples_TAG_BIT + n_samples_TAG_BIT/2; 
      return max_index;  
    }




    std::vector<float>  tag_decoder_impl::tag_detection_RN16(std::vector<gr_complex> & RN16_samples_complex)
    {            
      // detection + differential decoder (since Tag uses FM0)
      std::vector<float> tag_bits,dist;
      float result;
      int prev = 1,index_T=0;
      
      for (int j = 0; j < RN16_samples_complex.size()/2 ; j ++ )
      {
        result = std::real( (RN16_samples_complex[2*j] - RN16_samples_complex[2*j+1])*std::conj(h_est)); 
  
        if (result>0){
          if (prev == 1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = 1;      
        }
        else
        { 
          if (prev == -1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = -1;    
        }
      }
      return tag_bits;
      //check if the last bit (dummy bit) is a 1 => if not, not send ACK?
    }


    std::vector<float>  tag_decoder_impl::tag_detection_EPC(std::vector<gr_complex> & EPC_samples_complex, int index)
    {

      
      std::vector<float> tag_bits,dist;
      float result=0;
      int prev = 1;
      
      int number_steps = 20;
      float min_val = n_samples_TAG_BIT/2.0 -  n_samples_TAG_BIT/2.0/100, max_val = n_samples_TAG_BIT/2.0 +  n_samples_TAG_BIT/2.0/100;

      std::vector<float> energy;

      energy.resize(number_steps);
      for (int t = 0; t <number_steps; t++)
      {  
        for (int i =0; i <256; i++)
        {
          energy[t]+= reader_state->magn_squared_samples[(int) (i * (min_val + t*(max_val-min_val)/(number_steps-1)) + index)];
        }

      }
     

      int index_T = std::distance(energy.begin(), std::max_element(energy.begin(), energy.end()));
      float T =  min_val + index_T*(max_val-min_val)/(number_steps-1);

      // T estimated
      T_global = T;
  
      for (int j = 0; j < 128 ; j ++ )
      {
        result = std::real((EPC_samples_complex[ (int) (j*(2*T) + index) ] - EPC_samples_complex[ (int) (j*2*T + T + index) ])*std::conj(h_est) ); 

        
         if (result>0){
          if (prev == 1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = 1;      
        }
        else
        { 
          if (prev == -1)
            tag_bits.push_back(0);
          else
            tag_bits.push_back(1);      
          prev = -1;    
        }
      }
      return tag_bits;
    }


    int
    tag_decoder_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {


      const gr_complex *in = (const  gr_complex *) input_items[0];
      float *out = (float *) output_items[0];
      gr_complex *out_2 = (gr_complex *) output_items[1]; // for debugging
      
      int written_sync =0;
      int written = 0, consumed = 0;
      int RN16_index , EPC_index;

      std::vector<float> RN16_samples_real;
      std::vector<float> EPC_samples_real;

      std::vector<gr_complex> RN16_samples_complex;
      std::vector<gr_complex> EPC_samples_complex;

      std::vector<float> RN16_bits;
      int number_of_half_bits = 0;
      int var = 0;
 
      

      std::vector<float> EPC_bits;    
      // Processing only after n_samples_to_ungate are available and we need to decode an RN16
      if (reader_state->decoder_status == DECODER_DECODE_RN16 && ninput_items[0] >= reader_state->n_samples_to_ungate)
      {        
        RN16_index = tag_sync(in,ninput_items[0],1);

        
        if (reader_state->reader_stats.output_energy >= E_th)
        {
          for (float j = RN16_index; j < ninput_items[0]; j += n_samples_TAG_BIT/2 )
          {
            number_of_half_bits++;
            int k = round(j);
            RN16_samples_complex.push_back(in[k]);

            
            if (number_of_half_bits == 2*(RN16_BITS-1))
            {
              //out_2[written_sync] = h_est;
               //written_sync ++;  
              //produce(1,written_sync);        
              break;
            }
          } 

          if (number_of_half_bits == 2*(RN16_BITS-1))
          {
              GR_LOG_INFO(d_debug_logger, "RN16 DECODED");
              RN16_bits  = tag_detection_RN16(RN16_samples_complex);

              for(int bit=0; bit<RN16_bits.size(); bit++)
              {
                out[written] =  RN16_bits[bit];
                written ++;
              }
                produce(0,written);
                reader_state->gen2_logic_status = SEND_ACK;
          }
          else  
          {  
            GR_LOG_INFO(d_logger, "RN16 NOT DECODED CORRECTLY");
            update_slot();
          }    

        }

        else // no response frome tags -- power of output signal too low
        {
          // -------------------   IDLE SLOT ------------------------------------
          reader_state->reader_stats.n_0+=1;
          reader_state->reader_stats.tn_0 +=1 ; 
          reader_state-> reader_stats.Qfp = reader_state-> reader_stats.Qfp - C;
          //Qf = max(0, Qfp-C)
          var = round(reader_state-> reader_stats.Qfp);
          if (var > 0)
          {
            if(var >15)
            {
              reader_state-> reader_stats.Qant = 15;
            }
            else
            {
              reader_state-> reader_stats.Qant = var;
            }
          }
          else
          {
            reader_state-> reader_stats.Qant =  0; 
          }
          //GR_LOG_INFO(d_logger, " Idle slot");
          update_slot();
        }

       consumed = reader_state->n_samples_to_ungate;
      } // ELSE of DECODE RN16

      else if (reader_state->decoder_status == DECODER_DECODE_EPC && ninput_items[0] >= reader_state->n_samples_to_ungate )
      {
        //After EPC message send a query rep or query
                
        EPC_index = tag_sync(in,ninput_items[0],0);

        for (int j = 0; j < ninput_items[0]; j++ )
        {
          EPC_samples_complex.push_back(in[j]);
        }


        EPC_bits   = tag_detection_EPC(EPC_samples_complex,EPC_index);


        if (EPC_bits.size() == EPC_BITS - 1)
        {
          // float to char -> use Buettner's function
          for (int i =0; i < 128; i ++)
          {
            if (EPC_bits[i] == 0)
              char_bits[i] = '0';
            else
              char_bits[i] = '1';
          }


          if(check_crc(char_bits,128) == 1)
          {
            // ---------------------- SINGLE SLOT ---------------------------------------
            reader_state->reader_stats.n_epc_correct+=1;
            reader_state->reader_stats.n_1+=1;
            reader_state->reader_stats.tn_1  +=1; 
        
            int result = 0;
            for(int i = 0 ; i < 32 ; ++i)
            {
              result += std::pow(2,31-i) * EPC_bits[80+i] ;
            }
            
            GR_LOG_INFO(d_debug_logger, "EPC CORRECTLY DECODED");
            //GR_LOG_INFO(d_logger, "EPC CORRECTLY DECODED: " << reader_state->reader_stats.n_epc_correct);
            //GR_LOG_INFO(d_logger, " Success slot");

            // Save part of Tag's EPC message (EPC[104:111] in decimal) + number of reads
            std::map<int,int>::iterator it = reader_state->reader_stats.tag_reads.find(result);
            if ( it != reader_state->reader_stats.tag_reads.end())
            {
              it->second ++;
            }
            else
            {
              reader_state->reader_stats.tag_reads[result]=1;
            }

            //GR_LOG_INFO(d_logger, "Unique Tags read: " << reader_state->reader_stats.tag_reads.size() );

            update_slot(); // no modification to current frame size
            
          }

          else
          {   
            
            //GR_LOG_INFO(d_logger, "EPC FAIL TO DECODE"); 
            reader_state->reader_stats.n_k+=1;
            reader_state->reader_stats.tn_k += 1; 
        
            //GR_LOG_INFO(d_logger, " Collision slot");
            
           // ---------------------- COLLISION SLOT ---------------------------------------

            reader_state-> reader_stats.Qfp = reader_state-> reader_stats.Qfp + C;
            //Q = min(15, Qfp+C)
            var = round(reader_state-> reader_stats.Qfp);
            if (var < 15)
            {
              if(var<0)
              {
                reader_state-> reader_stats.Qant  = 0;
              }
              else
              {
                reader_state-> reader_stats.Qant = var;
              }
            }
            else
            {
              reader_state-> reader_stats.Qant = 15; //maximum value for Q is 15
            } 

            GR_LOG_INFO(d_debug_logger, "EPC FAIL TO DECODE");  

            update_slot();
          }
        }
        else
        {
          //GR_LOG_INFO(d_logger, "CHECK ME");
          GR_LOG_EMERG(d_debug_logger, "CHECK ME");  
        }

        consumed = reader_state->n_samples_to_ungate;
      }

      consume_each(consumed);
     return WORK_CALLED_PRODUCE;
    }
///////////////////////////////////////////////////////////////////////////

    void tag_decoder_impl::performance_evaluation()
    {
        if (reader_state-> reader_stats.stop == 1)
          {
          reader_state-> reader_stats.stop = 0;
          reader_state->reader_stats.th = ((float)reader_state->reader_stats.tn_1)/((float)reader_state->reader_stats.tn_k+(float)reader_state->reader_stats.tn_1+(float)reader_state->reader_stats.tn_0);

          float it_reader = reader_state-> reader_stats.tQA* TQA + reader_state-> reader_stats.tQR * TQR + TQ;
          float it_tag = Tsk*reader_state->reader_stats.tn_k+Tsk*reader_state->reader_stats.tn_1+Ti*reader_state->reader_stats.tn_0;

          float thn = ((float)reader_state->reader_stats.tn_1)/((float)reader_state->reader_stats.tn_k+(float)reader_state->reader_stats.tn_1+(float)reader_state->reader_stats.tn_0*Ti/Tsk);
          reader_state-> reader_stats.TIR_th = reader_state->reader_stats.tag_reads.size()  /(it_reader* pow(10,-6) + it_tag* pow(10,-6));
          reader_state-> reader_stats.TIR_exp = (float)reader_state->reader_stats.tag_reads.size()/(float)((reader_state-> reader_stats.it_timer));

            std::cout << "| ----------------------------------------------------------------------- " <<  std::endl;
           
            std::cout << "| TIR theoretic : "  <<  reader_state-> reader_stats.TIR_th     << std::endl;
            std::cout << "| TIR experimental : "  <<  reader_state-> reader_stats.TIR_exp << std::endl;
            std::cout << "| Throughtput : "  <<  reader_state->reader_stats.th     << std::endl;
            std::cout << "| Total QA: " << reader_state-> reader_stats.tQA << std::endl;      
            std::cout << "| Total QR: " << reader_state-> reader_stats.tQR << std::endl;      
            std::cout << "| Total ck: " << reader_state->reader_stats.tn_k << std::endl;      
            std::cout << "| Total ci: " << reader_state->reader_stats.tn_0 << std::endl;      
            std::cout << "| Total cs: " << reader_state->reader_stats.tn_1 << std::endl;    
          
          
          //WRITE AND SAVE RESULTS INTO FILE TO PLOT LATER IN MATLAB
          ofstream myfile;

          myfile.open ("ntags_read.txt", ios::app);
          myfile << reader_state->reader_stats.tag_reads.size();
          myfile << "\n";
          myfile.close();

          myfile.open ("nepc_read.txt", ios::app);
          myfile << reader_state->reader_stats.n_epc_correct;
          myfile << "\n";
          myfile.close();

          myfile.open ("TIR_timer.txt", ios::app);
          myfile << reader_state-> reader_stats.TIR_exp;
          myfile << "\n";
          myfile.close();

          myfile.open ("TIR_formula.txt", ios::app);
          myfile << reader_state-> reader_stats.TIR_th;
          myfile << "\n";
          myfile.close();

          myfile.open ("throughput.txt", ios::app);
          myfile << reader_state-> reader_stats.th;
          myfile << "\n";
          myfile.close();
          
          myfile.open ("tQA.txt", ios::app);
          myfile << reader_state-> reader_stats.tQA;
          myfile << "\n";
          myfile.close();

          myfile.open ("tQR.txt", ios::app);
          myfile << reader_state-> reader_stats.tQR;
          myfile << "\n";
          myfile.close();

          myfile.open ("ck.txt", ios::app);
          myfile << reader_state->reader_stats.tn_k;
          myfile << "\n";
          myfile.close();

          myfile.open ("ci.txt", ios::app);
          myfile << reader_state->reader_stats.tn_0;
          myfile << "\n";

          myfile.open ("thn.txt", ios::app);
          myfile << thn;
          myfile << "\n";
          myfile.close();    
          }
      
    }

    //////////////////////////////////////////////////////////////
  
    void tag_decoder_impl::update_slot()
    {
        if(reader_state-> reader_stats.tag_reads.size() >= NUMBER_UNIQUE_TAGS && reader_state->reader_stats.cur_slot_number == pow(2,reader_state-> reader_stats.VAR_Q)) //make calculations
        //if(reader_state->reader_stats.n_queries_sent >= MAX_NUM_QUERIES)
        {
          gettimeofday (&reader_state-> reader_stats.end, NULL);

          //std::cout << "|END TIME: " << reader_state-> reader_stats.end.tv_usec*+reader_state-> reader_stats.start.tv_sec << std::endl;    

          reader_state-> reader_stats.it_timer = reader_state-> reader_stats.end.tv_sec - reader_state-> reader_stats.start.tv_sec+(reader_state-> reader_stats.end.tv_usec*pow(10,-6) - reader_state-> reader_stats.start.tv_usec*pow(10,-6));

          performance_evaluation();
        } 

      //If End of frame or Q modified(at any slot) => new frame
      if (reader_state->reader_stats.cur_slot_number == pow(2,reader_state-> reader_stats.VAR_Q) || reader_state-> reader_stats.VAR_Q != reader_state->reader_stats.Qant)
      { 
        if (reader_state->reader_stats.Qant > reader_state-> reader_stats.VAR_Q) 
        {
          reader_state-> reader_stats.Qupdn = 0; //increase Q by one
          reader_state-> reader_stats.VAR_Q = reader_state->reader_stats.Qant;
        }
        else if(reader_state->reader_stats.Qant <  reader_state-> reader_stats.VAR_Q)
        {
          reader_state-> reader_stats.Qupdn = 2; //decrease Q by one
          reader_state-> reader_stats.VAR_Q = reader_state->reader_stats.Qant;
        }
        else
        {
          reader_state-> reader_stats.Qupdn = 1;// Q unchanged
        }

        /* --- Taken from EPC specifications version 2.0.0 ---
        If a Tag receives a QueryAdjust with an UpDn value different from those specified above then it shall treat
        the command as invalid (see Table C.30). If a Tag whose Q value is 15 receives a QueryAdjust with
        UpDn=110 then it shall change UpDn to 000 prior to executing the command; likewise, if a Tag whose Q
        value is 0 receives a QueryAdjust with UpDn=011 then it shall change UpDn to 000 prior to executing the
        command.
        */

        reader_state->reader_stats.tQA +=1; 

        reader_state->reader_stats.cur_slot_number = 1;
        reader_state->reader_stats.unique_tags_round.push_back(reader_state->reader_stats.tag_reads.size());
        reader_state->reader_stats.cur_inventory_round += 1;
        reader_state->reader_stats.n_k = 0;
        reader_state->reader_stats.n_1 = 0;
        reader_state->reader_stats.n_0 = 0;

        reader_state->gen2_logic_status = SEND_QUERY_ADJUST;
      }

      else 
      {
        reader_state->reader_stats.cur_slot_number++;
        reader_state->reader_stats.tQR +=1; 
        reader_state->gen2_logic_status = SEND_QUERY_REP;
      }
    }
 
    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    int tag_decoder_impl::check_crc(char * bits, int num_bits)
    {
      register unsigned short i, j;
      register unsigned short crc_16, rcvd_crc;
      unsigned char * data;
      int num_bytes = num_bits / 8;
      data = (unsigned char* )malloc(num_bytes );
      int mask;

      for(i = 0; i < num_bytes; i++)
      {
        mask = 0x80;
        data[i] = 0;
        for(j = 0; j < 8; j++)
        {
          if (bits[(i * 8) + j] == '1'){
          data[i] = data[i] | mask;
        }
        mask = mask >> 1;
        }
      }
      rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

      crc_16 = 0xFFFF; 
      for (i=0; i < num_bytes - 2; i++)
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

      if(rcvd_crc != crc_16)
        return -1;
      else
        return 1;
    }
  } /* namespace rfid */
} /* namespace gr */

