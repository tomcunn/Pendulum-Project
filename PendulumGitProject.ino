//**************************************
//*   AI Pendulum Control Platform Interface
//*   4/6/2018
//*   Cunningham
//***************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//Hardware Assignments
#define  EncoderChannel_A  2
#define  EncoderChannel_B  3
#define  Step 12
#define  Dir  11
#define  MAX_CART_VELOCITY  30000
#define  MAX_CART_ACCELERATION  2000

#define LED_GREEN  10  
#define LED_YELLOW 9
#define LED_RED 8

#define  PGAIN 750

//Function prototypes
void  SetupTimer1(void);
void  SetupTimer0(void);
void  Encoder_ISR(void);
void  WriteStepSize(int Speed);
void  MotionVelocityControl(void);
void  MotionAccelerationControl(void);

//Create a structure for movement data
struct MovementParms
{
  signed int Position;
  signed int Velocity;
  signed int Acceleration;
  bool Direction;
};

//Create instance of three structures 
volatile struct MovementParms DesiredCart;
volatile struct MovementParms CurrentCart;
volatile struct MovementParms EncoderValue;

//Static Variables that are accessed in interrupts
volatile unsigned int abs_Current_Velocity = 0;
volatile unsigned int Counter = 0;
volatile unsigned int StepSpeed = 0;
volatile signed long timestep = 0;
volatile bool RunTask_10ms = 0;
byte SerialControlData1 = 0;
byte SerialControlData2 = 0;
bool SerialControlDirection = 0;
static bool ReturnHomeFlag = 0;
static unsigned int EncoderStableCount = 0;

byte serial_buffer[5] = {0,0,0,0,0};
int ByteCount;
static int integrator = 0;
Encoder PendulumPosition(EncoderChannel_A, EncoderChannel_B);

//**************************************
//  Interrupt used to generate the servo 
//  pulses
//  The lower the address the higher is the priority level
//  Vector Address 12 
//**************************************
ISR(TIMER1_COMPA_vect)  
{   
   if(StepSpeed < 64000)
   {
     //Check the direction pin and determine the position
     if(CurrentCart.Direction == 1)
     {
        digitalWrite(Dir,1);
        CurrentCart.Position++;
     }
     else
     {
       digitalWrite(Dir,0);
       CurrentCart.Position--;
     }
  
     //Step once
     digitalWrite(Step,1);
     digitalWrite(Step,0);
   }
   //Interrupt occurs when timer matches this value
   //Larger velocity means smaller OCR1A value
   //StepSpeed is calculated using another function and then stored 
   //in a global variable to be accessed by the interrupt routine
   OCR1A = StepSpeed;
}

//**************************************
//  10 ms Task Rate
//  Creates a periodic Task 
//  Vector Address 15
//**************************************
ISR(TIMER0_COMPA_vect)
{
  //Check to see if the flag was not cleared, this will track 
  //overruns in the 10ms task
  if(RunTask_10ms)
  {
    digitalWrite(LED_BUILTIN,1);
  }
  RunTask_10ms = 1;
}

//*******************************************
// WriteStepSize
//
// Pass in a velocity and update the step size
// global variable which is read by the interrupt
// to generate the stepper pulses. Speed is in 
// the units mm/min. 
//*******************************************
void WriteStepSize(signed int Speed)
{
  //Set the direction pin based on the sign of the speed
  if(Speed > 0)
  {
     CurrentCart.Direction = 1;
  }
  else
  {
     CurrentCart.Direction = 0;
  } 
  // Conversion Factor
  //  20000 mm      1 min      360 deg     1 rev       step          4,444 step
  // ----------- x -------- x -------- x -------- x ------------ =  ---------
  //     min        60 sec      rev        120 mm     0.225 deg       sec
  //
  //   16,000,000 cycles      1 sec            3,600 cycles
  //   ----------------- x --------------- = -----------
  //       1 sec             4,444 step          
  //  16,000,000/4,444    = 3,600 clock cycles  (20000 mm/min)
  //  16,000,000/2,222    = 7,200 clock cycles  (10000 mm/min)
  //  16,000,000/666      = 24,024 clock cycles  (3000 mm/min)
  
  //For some reason the prescale set to zero causes problems, so 8 is the minimum
  //which means the clock is 16,000,000/8=2,000,000
  //  2,000,000/4,444   = 450 clock cycles  (20000 mm/min)
  //  2,000,000/2,222    = 900 clock cycles  (10000 mm/min)
  //  2,000,000/666      = 3003 clock cycles (3000 mm/min)

  //To compute position it is 
  // Conversion Factor
  //    360 deg     1 rev       step          13.33 steps
  //  -------- x -------- x ------------ =  ---------
  //     rev        120 mm     0.225 deg        mm

  
  
  //Check to prevent overflows, StepSpeed is 16 bit
  if(Speed > 700 || Speed < -700)
  {
     //This is derived from the table above using a Power trendline
     StepSpeed = (signed int)(9048739L/(abs(Speed)));
  }
  //This is like setting it to zero speed
  else
  {
     StepSpeed = 65000;
  }
}
//*******************************************
//  Configure Timer 1
//*******************************************

void SetupTimer1()
{
  //TCCRnA/B- Stands for Timer/Counter Control Registers.
  TCCR1A = 0;
  TCCR1B = 0;

  //TCNTn- Stands for Timer/Counter Register.
  TCNT1 = 0; // initialize the counter from 0

  //OCRnA/B- Stands for Output Compare Register.
  OCR1A = 65000; // sets the counter compare value

  //TIMSKn- Stands for Timer/Counter Mask In Registers.
  TCCR1B |= (1<<WGM12); // enable the CTC mode
  TCCR1B |= (1<<CS11); // 1/8 Prescale, 0 prescale was not working

  TIMSK1 |= (1<<OCIE1A); 
}

//*********************************************
//  Configure Timer 0
//*********************************************
void SetupTimer0()
{
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0  = 0; //initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 155;// = (16*10^6) / (100*1024) - 1 = 155(must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // 1024 prescaler - page 142 of datasheet
  TCCR0B |= (1 << CS00); 
  TCCR0B |= (1 << CS02);  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

//*****************************************
//  Main Setup Function
//
//
//*****************************************
void setup() 
{
  bool run_program = 0;
  //Set the pin modes
  pinMode(EncoderChannel_A,INPUT);
  pinMode(EncoderChannel_B,INPUT);
  pinMode(Step,OUTPUT);
  pinMode(Dir,OUTPUT);
  pinMode(4,INPUT);
  pinMode(PIN_A5, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
 
  //Turn on the onboard LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,1);
  
  digitalWrite(LED_GREEN, 0);
  digitalWrite(LED_RED, 1);
  digitalWrite(LED_YELLOW, 0);

  delay(50);
  
  //Wait for the switch input to start things off
  while(true)
  {
    run_program = digitalRead(PIN_A5);
    if(run_program == 1)
    {
       digitalWrite(LED_BUILTIN,0);
	     digitalWrite(LED_YELLOW, 1);
	     digitalWrite(LED_RED, 0);
       break;
    }
  }

  //Disable interrupts
  cli();
  //Setup Timers
  SetupTimer1();
  SetupTimer0();
  //enable interrupts
  sei();
  Serial.begin(115200);
  
}


//********************************************
//  Main Loop
//********************************************
void loop() 
{
  static bool ReadyForTop = 0;
  //Serial.println(EncoderValue.Position,DEC);
  if(RunTask_10ms)
  {
    EncoderValue.Position = PendulumPosition.read();
   
    DesiredCart.Acceleration = 2000;
    
    if(!ReadyForTop)
    {
      if(EncoderValue.Position > 1200)
      {
        DesiredCart.Velocity = -20000;
      }
      else
      {
        DesiredCart.Velocity = 20000;
      } 
      if(EncoderValue.Position < 3 && EncoderValue.Position > -3)
      {
        ReadyForTop = 1;
        digitalWrite(LED_GREEN, 1);
      }
    }
    if(ReadyForTop)
    {
      //Always try to get the penduum to the middle
      if(CurrentCart.Position > 0 )
      {
        EncoderValue.Position -=3;
      }
      else
      {
        EncoderValue.Position -=1;
      }
  
      integrator +=EncoderValue.Position;
        
      timestep++;
      if (!ReturnHomeFlag)
      {
         if (EncoderValue.Position > (20000 / PGAIN))
         {
           DesiredCart.Velocity = -20000;
         }
         else if (EncoderValue.Position < (-20000 / PGAIN))
         {
           DesiredCart.Velocity = 20000;
         }
         else
         {
           DesiredCart.Velocity = (EncoderValue.Position) * -PGAIN;
         }
      }
      //50 worked
      DesiredCart.Velocity += integrator * -75;
    }
    timestep++;
    Serial.print(",");
    Serial.print(timestep,DEC);
    Serial.print(",");
    Serial.print(EncoderValue.Position,DEC);
    Serial.print(",");
    Serial.print(EncoderValue.Velocity,DEC);
    Serial.print(",");
    Serial.print(CurrentCart.Position,DEC);
    Serial.print(",");
    Serial.print(CurrentCart.Velocity,DEC);
    Serial.print(",");
    Serial.print(CurrentCart.Acceleration,DEC);
    Serial.print(",");
    Serial.print(DesiredCart.Velocity ,DEC);
    Serial.print(",");
    Serial.println("");

    //ByteCount = Serial.readBytesUntil(65,serial_buffer,5);
    //Data1 is the high byte. Data2 is the low byte
    /*if(!ReturnHomeFlag)
    {  
      if(int(serial_buffer[2]) == 0x01)
      {
         DesiredCart.Velocity = ((int(serial_buffer[0]) << 8) + serial_buffer[1]);
      }
      else
      {
         DesiredCart.Velocity = -1*((int(serial_buffer[0]) << 8) + serial_buffer[1]);
      }
    }*/

    //Check the position limits and return home if required
    if(CurrentCart.Position > 8900)
    {
       //Set the return home flag
       ReturnHomeFlag = 1;
       //Override the velocity
       DesiredCart.Velocity = -5000;
    }
    else if(CurrentCart.Position < -8900)
    {
       ReturnHomeFlag = 1;
       //Override the velocity
       DesiredCart.Velocity = 5000;
    }
    
    if(ReturnHomeFlag)
    {
      if(CurrentCart.Position < 20 && CurrentCart.Position > -20)
      {
        DesiredCart.Velocity = 0;
        if(EncoderValue.Position < 100 && EncoderValue.Position > -100)
        {
          EncoderStableCount++;
        }
        if(EncoderStableCount > 1000)
        {
          ReturnHomeFlag = 0;
          EncoderStableCount = 0;
        }
      }
    }
    
    
    //Call this function to compute the current velocity,this
    //takes into account the acceleration
    MotionVelocityControl();

    //Call function to covert the velocity from mm/min to timer value
    WriteStepSize(CurrentCart.Velocity);
    //Clear the flag that gets set by the interrupt
    //This was added at end to detect overruns of the 10ms task
    RunTask_10ms = 0;
  }
}

//*********************************************
// MotionVelocityControl
//
// Uses all global structures
//*********************************************
void MotionVelocityControl(void)
{  
  //Moving in the positive direction
  if(CurrentCart.Velocity >= 0)
  
     //Determine if we are speeding up
     if(CurrentCart.Velocity < DesiredCart.Velocity)
     {
        //Check to see if we can add the acceleration without overshooting
        if(CurrentCart.Velocity < (DesiredCart.Velocity - DesiredCart.Acceleration))
        {
           CurrentCart.Velocity += DesiredCart.Acceleration;
        }
        else
        {
           CurrentCart.Velocity = DesiredCart.Velocity;
        }
     }
     //We are slowing down 
     else
     {
        //Check for overshoot
        if(CurrentCart.Velocity > (DesiredCart.Velocity + DesiredCart.Acceleration))
        {
           CurrentCart.Velocity -= DesiredCart.Acceleration;
        }
        else
        {
           CurrentCart.Velocity = DesiredCart.Velocity;
        }
     }
  //Moving in the negative direction, all the signs flip   
  else
  {
     //Determine if we are speeding up
     if(CurrentCart.Velocity > DesiredCart.Velocity)
     {
        //Calculate the timer for the next pulse
        if(CurrentCart.Velocity > (DesiredCart.Velocity + DesiredCart.Acceleration))
        {
           CurrentCart.Velocity -= DesiredCart.Acceleration;
        }
        else
        {
           CurrentCart.Velocity = DesiredCart.Velocity;
        }
     }
     //We are slowing down 
     else
     {
        //Calculate the timer for the next pulse
        if(CurrentCart.Velocity < (DesiredCart.Velocity - DesiredCart.Acceleration))
        {
           CurrentCart.Velocity += DesiredCart.Acceleration;
        }
        else
        {
           CurrentCart.Velocity = DesiredCart.Velocity;
        }
     }
  }
}

//***********************************
//  MotionAccelerationControl
//
//
//***********************************
void MotionAccelerationControl(void)
{
   //Acceleration is a signed value
   //If positive check to make sure it is not bigger than max V
   //else check the - max speed
   
   if(DesiredCart.Acceleration >= 0)
   {
      if(CurrentCart.Velocity < (MAX_CART_VELOCITY + DesiredCart.Acceleration))
      {
         CurrentCart.Velocity += DesiredCart.Acceleration; 
      }
      else
      {
        CurrentCart.Velocity = MAX_CART_VELOCITY;
      }
   }
   else
   {
      if(CurrentCart.Velocity > (-MAX_CART_VELOCITY + DesiredCart.Acceleration))
      {
         CurrentCart.Velocity += DesiredCart.Acceleration;
      }
      else
      {
         CurrentCart.Velocity = -MAX_CART_VELOCITY;
      }
   }
}


