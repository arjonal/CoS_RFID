// Developed by Laura Arjona

//FuzzyQ anti-collision protocol
    const float Q_Low[4] = {0,0,5,6};
    const float Q_Medium[4] = {5,6,8,9};  
    const float Q_High [4]  = {8,9,15,15};  
    //Inputs of FRBS
    const float SW_Low[4] = {0.0,0.0,0.5,0.7};
    const float SW_High[4]   = {0.5,0.7,1.0,1.0};  
    //Output of FRBS
    const int delta_Q_out[4]  = {0,1,2,3};

  ////////////////////////////////////////////////////////////////////////



 //Returns the value of delta_Q after evaluating the FRBS
    float tag_decoder_impl::calculate_Delta_Q(int in_Q,float in_SW)
    {
      //Fuzzification
      float mu_Q_Low    = calculate_mu(Q_Low[0],   Q_Low[1],   Q_Low[2],   Q_Low[3],(float)in_Q);
      float mu_Q_Medium = calculate_mu(Q_Medium[0],Q_Medium[1],Q_Medium[2],Q_Medium[3],(float)in_Q);
      float mu_Q_High   = calculate_mu(Q_High[0],  Q_High[1],  Q_High[2],  Q_High[3],(float)in_Q);

      float mu_SW_Low   = calculate_mu(SW_Low[0], SW_Low[1], SW_Low[2],SW_Low[3],in_SW);
      float mu_SW_High  = calculate_mu(SW_High[0],SW_High[1],SW_High[2],SW_High[3],in_SW);

      
      //Inference
      float antecedent1 = inference_function(mu_Q_Low,mu_SW_Low); //If Q is Low and SW is Low
      float rule1 = antecedent1*delta_Q_out[0]; // then delta_Q is Null (or Low?)
      //std::cout << "---- Antecedente 1 : " <<  antecedent1 << std::endl;

      float antecedent2 = inference_function(mu_Q_Low,mu_SW_High);//If Q is Low and SW is High
      float rule2 = antecedent2*delta_Q_out[3]; // then delta_Q is High
      
      //std::cout << "---- mu_Q_low : " <<  mu_Q_Low << std::endl;
      //std::cout << "---- mu_SW_high : " <<  mu_SW_High << std::endl;
      //std::cout << "---- Antecedente 2 : " <<  antecedent2 << std::endl;
      //std::cout << "---- Rule 2 : " <<  rule2 << std::endl;

      float antecedent3 = inference_function(mu_Q_Medium,mu_SW_Low); //If Q is Medium and SW is Low
      float rule3 = antecedent3*delta_Q_out[0]; // then delta_Q is Null
      //std::cout << "---- Antecedente 3 : " <<  antecedent3 << std::endl;

      float antecedent4 = inference_function(mu_Q_Medium,mu_SW_High);//If Q is Medium and SW is High
      float rule4 = antecedent4*delta_Q_out[2]; // then delta_Q is Medium
      //std::cout << "---- Antecedente 4 : " <<  antecedent4 << std::endl;

      float antecedent5 = inference_function(mu_Q_High,mu_SW_Low); //If Q is High and SW is Low
      float rule5 = antecedent5*delta_Q_out[0]; // then delta_Q is Null
      //std::cout << "---- Antecedente 5 : " <<  antecedent5 << std::endl;

      float antecedent6 = inference_function(mu_Q_High,mu_SW_High);//If Q is High and SW is High
      float rule6 = antecedent6*delta_Q_out[1]; // then delta_Q is Low
      //std::cout << "---- Antecedente 6 : " <<  antecedent6 << std::endl;


      return (rule1+rule2+rule3+rule4+rule5+rule6) / (antecedent1+antecedent2+antecedent3+antecedent4+antecedent5+antecedent6);
    }


    //Calculates the inference with an AND operation
    float tag_decoder_impl::inference_function(float a, float b)
    {
      return a*b;
    }


    //Calculates the degree of membership of a discrete input value to a System Input MF
    //Valid for trapezoidal membership functions
    float tag_decoder_impl::calculate_mu(float mf1, float mf2, float mf3, float mf4, float input_value)
    {

      
      if(input_value < mf1)
      {
        return 0.0;
      }
      else if (input_value == mf1)
      {
        if (mf1 == mf2)
        {
          return 1.0;
        } 
        else
        {
          return 0.0;
        }
      }
      else if(input_value < mf2)
      {
        if (mf2 == mf1)
        {
                return 1.0;
        }
            else
            {
                return (input_value-mf1)/(mf2-mf1);
            }
      }
      else if(input_value == mf2)
      {
        return 1.0;
      }
      else if (input_value <= mf3)
      {
        return 1.0;
      }
      else if(input_value <mf4)
      {
        if (mf3 == mf4)
        {
                return 1.0;
        }
            else
            {
                return (mf4-input_value)/(mf4-mf3);
            }
            
      }
      else if(input_value == mf4)
      {
        if (mf3 == mf4)
        {
                return  1.0;
        }
            else
            {
                return  0.0;
            }
          
      }
      else
      {
        return 0;
      }

    }

