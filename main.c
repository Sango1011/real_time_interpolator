#include <24FV16KM202.h>
#DEVICE ADC=8
#device ICD=3

#use delay(clock = 32MHZ, internal = 8MHZ)
#USE RS232(UART2, BAUD = 9600, PARITY = N, BITS = 8, STOP = 1, TIMEOUT = 500))


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES CKSFSM                   //Clock Switching is enabled, fail Safe clock monitor is enabled
#FUSES NOBROWNOUT               //No brownout reset
#FUSES BORV_LOW                 //Brown-out Reset set to lowest voltage
#use fast_io(B)
#FUSES FRC_PLL

#define NUM 10       //number of stored samples
#define FACTOR 1     //interpolation factor
#define INTER  1/FACTOR   //number of time periods in interpolation 1/Factor

int1 timer_flag = 0;   //global flags
unsigned int16 samples [NUM];   //array to store samples
double interpolate[FACTOR];  //array to store interpolated values
unsigned int8 cur = 0;       //used to track the index of the stored samples
unsigned int8 index; //used for calculating interpolated values
float accumulator = 0;  //used to keep track of the value of the interpolated data
unsigned int8 out;     //used to send to the dac
int8 count = 0; //used to keep track of the interpolation 
int timer = 45536;  //used for 100 hz sample rate
//int8 state = 0;   //state machine
float x, x1;    //used to store the value to put into the sinc function
int8 t, n; //used for the sinc function
//int start, end;  //used to calculate the starting and ending values for the filter
float store [FACTOR];   //used to store the array values used forinterpolations
int8 i;     //counter

#INT_TIMER1
void  timer1_isr(void)  // used for sampling
{ 
   output_toggle (PIN_A2);       
   set_timer1(timer + get_timer1());  //set desired frequency
   timer_flag = 1; 
}

void main()
{  
   for (i = 0; i < FACTOR; i++)  //get the array of values used for the
   {                             //interpolation process
      store[i] = i * INTER;
   }
   
   // Setup ADC
   setup_adc(ADC_CLOCK_DIV_2 | ADC_TAD_MUL_4);   
   setup_adc_ports(sAN0 | VSS_VDD); 
   // Setup DAC
   setup_dac(1,DAC_REF_VDD | DAC_ON);
   setup_opamp1(OPAMP_ENABLED | OPAMP_PI_TO_DAC | OPAMP_NI_TO_OUTPUT | OPAMP_HIGH_POWER_MODE); 
   // Setup Timer 
   clear_interrupt(INT_TIMER1);
   setup_timer1(T1_INTERNAL | T1_DIV_BY_8);   
   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);
   
   while (i < NUM)   //collect the starting samples
   {
      if(timer_flag)
      {
         samples[i] = read_adc();  i++;
      }
   } 
   for(t = 0; t < FACTOR; t++)   //get the first N interpolated values
   {
      index = cur;
      for(n = 0; n < NUM; n++)
      {
         x = PI * (n - store[t]);   //used in the sinc function
         x1 = x;        //original x value
         while(abs(x) > 2*PI) //make sure it between -2PI to 2 PI
         {
           if(x > 2*PI)
              x = x - (2*PI);
           else
               x = x + (2*PI);
          }
          if(x == 0)    //sinc (0) = 1
          {
             accumulator += samples[index];
          }
          else 
          {       //sinc filter
             accumulator += samples[index]*(sin(x)/x1);
          }
          index++;
          if(index == NUM)    //circular buffer
          {      
            index = 0;
          }
      }
      interpolate[t] = (double)accumulator;  //save interpolated value
      accumulator = 0;     //reset accumulator
   } 
   timer = 45536;//55536; //200hz //45536;//100h //60536;//400 h //63536;//1kh //61536; //500h
    
  while(true)
  {
      if(count == FACTOR)
      {
         samples[cur] = read_adc(); //get new sample
         cur++; count = 0; //increase pointer and reset count
         if(cur == NUM) //circular buffer
         {
            cur = 0;
         }
         for(t = 0; t < FACTOR; t++)   //interpolate the N factors
         {
            index = cur;
            for(n = 0; n < NUM; n++)
            {
               x = PI * (n - store[t]);   //used in the sinc function
               x1 = x;        //original x value
               while(abs(x) > 2*PI) //must be between -2PI and 2 PI
               {
                  if(x > 2*PI)
                     x = x - (2*PI);
                  else
                     x = x + (2*PI);
               }
               if(x == 0)     //sinc 0 = 1
               {
                  accumulator += samples[index];
               }
               else 
               {     //sinc filter
                  accumulator += samples[index]*(sin(x)/x1);
               }
               index++;
               if(index == NUM)     //circular buffer
               {
                  index = 0;
               }
            }
           interpolate[t] = (double)accumulator;   //save value
           accumulator = 0;      //reset accumulator
         }        
      } 
      if(timer_flag)
      {
         out = (int8)(interpolate[count]);   //convert to 8 bit
         dac_write(1, out);   //send value to dac
         count++;    //increase the count
      }
  }
}

