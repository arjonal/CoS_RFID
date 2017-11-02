//Test of the Fuzzy Rule Based System (FRBS) used in FuzzyQ
//for all posible values of inputs.
//Then I will comopare vaules with those from Matalb (original implementation)

#include <iostream>
#include <cmath>


const float Q_Low[4] = {0,0,5,6};
const float Q_Medium[4] = {5,6,8,9};  
const float Q_High [4]  = {8,9,15,15};  
//Inputs of FRBS
const float SW_Low[4] = {0.0,0.0,0.5,0.7};
const float SW_High[4]   = {0.5,0.7,1.0,1.0};  
 //Output of FRBS
const int delta_Q_out[4]  = {0,1,2,3};
const float epsilon = 0.001f;

float inference_function(float a, float b);
float calculate_mu(float mf1, float mf2, float mf3, float mf4, float input_value);
bool cmpf(float A, float B, float epsilon);

int main() {
   //Test all input values
 for(int in_Q = 0; in_Q <=15; in_Q  = in_Q +1)
  {

  for(float in_SW = 0.0f; in_SW <= 1.1f; in_SW  = in_SW + 0.1f)
  {

    //Fuzzification
      float mu_Q_Low    = calculate_mu(Q_Low[0],   Q_Low[1],   Q_Low[2],   Q_Low[3],(float)in_Q);
      float mu_Q_Medium = calculate_mu(Q_Medium[0],Q_Medium[1],Q_Medium[2],Q_Medium[3],(float)in_Q);
      float mu_Q_High   = calculate_mu(Q_High[0],  Q_High[1],  Q_High[2],  Q_High[3],(float)in_Q);

      float mu_SW_Low   = calculate_mu(SW_Low[0], SW_Low[1], SW_Low[2],SW_Low[3],in_SW);
      float mu_SW_High  = calculate_mu(SW_High[0],SW_High[1],SW_High[2],SW_High[3],in_SW);
      
      //Inference
      float antecedent1 = inference_function(mu_Q_Low,mu_SW_Low); //If Q is Low and SW is Low
      float rule1 = antecedent1*delta_Q_out[1]; // then delta_Q is Null (or Low?)

      float antecedent2 = inference_function(mu_Q_Low,mu_SW_High);//If Q is Low and SW is High
      float rule2 = antecedent2*delta_Q_out[3]; // then delta_Q is High

      float antecedent3 = inference_function(mu_Q_Medium,mu_SW_Low); //If Q is Medium and SW is Low
      float rule3 = antecedent3*delta_Q_out[0]; // then delta_Q is Null
     
      float antecedent4 = inference_function(mu_Q_Medium,mu_SW_High);//If Q is Medium and SW is High
      float rule4 = antecedent4*delta_Q_out[2]; // then delta_Q is Medium

      float antecedent5 = inference_function(mu_Q_High,mu_SW_Low); //If Q is High and SW is Low
      float rule5 = antecedent5*delta_Q_out[0]; // then delta_Q is Null
     
      float antecedent6 = inference_function(mu_Q_High,mu_SW_High);//If Q is High and SW is High
      float rule6 = antecedent6*delta_Q_out[1]; // then delta_Q is Low
      
      float result = round((rule1+rule2+rule3+rule4+rule5+rule6) / (antecedent1+antecedent2+antecedent3+antecedent4+antecedent5+antecedent6));

       std::cout <<result<<std::endl;
   }
 }
}
    //Returns true if two float values are 'equal'
    bool cmpf(float A, float B, float epsilon)
    {
      return (fabs(A - B) < epsilon);
    }

    //Calculates the inference with an AND operation
    float inference_function(float a, float b)
    {
      return a*b;
    }

    //Calculates the degree of membership of a discrete input value to a System Input MF
    //Valid for trapezoidal membership functions
    float calculate_mu(float mf1, float mf2, float mf3, float mf4, float input_value)
    {
      if(input_value < mf1)
      {
        return 0.0;
      }
      else if (cmpf(input_value,mf1,epsilon))
      {
        if (cmpf(mf1,mf2,epsilon))
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
        if (cmpf(mf1,mf2,epsilon))
        {
                return 1.0;
        }
            else
            {
                return (input_value-mf1)/(mf2-mf1);
            }
      }
      else if(cmpf(input_value,mf2,epsilon))
      {
        return 1.0;
      }
      else if (input_value < mf3 || cmpf(input_value,mf3,epsilon))
      {
        return 1.0;
      }
      else if(input_value <mf4)
      {
        if (cmpf(mf3,mf4,epsilon))
        {
                return 1.0;
        }
            else
            {
                return (mf4-input_value)/(mf4-mf3);
            }
            
      }
      else if(cmpf(input_value,mf4,epsilon))
      {
        if (cmpf(mf3,mf4,epsilon))
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
