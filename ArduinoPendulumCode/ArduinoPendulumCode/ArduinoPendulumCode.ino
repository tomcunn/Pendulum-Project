//**************************************
//*   Open Pendulum Control Platform Interface
//*   8/25/2024
//*   Cunningham
//***************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>


//Hardware Assignments
#define  LIMIT_HOME   A3
#define LED_GREEN  10  
#define LED_YELLOW 9
#define LED_RED 8
#define  DIR 11
#define  STEP 12
#define  SWITCH1 4
#define  SWITCH2 A2
#define  EncoderChannel_A  2
#define  EncoderChannel_B  3

#define  MAX_CART_VELOCITY  500  //875
#define  MAX_CART_ACCELERATION  50 //100

//Function prototypes
void  SetupTimer1(void);
void  SetupTimer0(void);

//Global Variables
uint8_t RunTask_10ms = 0;


volatile unsigned int StepSpeed = 0;

//Create and instance of the encoder module
Encoder PendulumPosition(EncoderChannel_A, EncoderChannel_B);

  //Create a structure for movement data
struct MovementParms
{
  signed int Count;
  signed int Position;
  signed int Velocity;
  signed int Acceleration;
  signed int Direction;
};

struct PendulumData
{
  //Create instance of three structures 
  volatile struct MovementParms DesiredCart;
  volatile struct MovementParms CurrentCart;
  volatile struct MovementParms EncoderValue;
};

PendulumData pd;

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
     if(pd.CurrentCart.Direction == 1)
     {
        digitalWrite(DIR,1);
        pd.CurrentCart.Count++;
     }
     else
     {
       digitalWrite(DIR,0);
       pd.CurrentCart.Count--;
     }
  
     //Step once
     digitalWrite(STEP,1);
     digitalWrite(STEP,0);
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
  RunTask_10ms = 1;
}

//*******************************************
// WriteStepSize
//
// Pass in a velocity and update the step size
// global variable which is read by the interrupt
// to generate the stepper pulses. Speed is in 
// the units mm/sec. 
//*******************************************
void WriteStepSize(signed int Speed)
{
  //Set the direction pin based on the sign of the speed
  if(Speed > 0)
  {
     pd.CurrentCart.Direction = 1;
  }
  else
  {
     pd.CurrentCart.Direction = 0;
  } 
  
  //Check to prevent overflows, StepSpeed is 16 bit
  if(Speed > 2 || Speed < -2)
  {
     //This is derived from the table above using a Power trendline
     StepSpeed = (signed int)(591733/(abs(Speed)));
  }
  //This is like setting it to zero speed
  else
  {
     StepSpeed = 65000;
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
   
   if(pd.DesiredCart.Acceleration >= 0)
   {
      if(pd.CurrentCart.Velocity < (MAX_CART_VELOCITY + pd.DesiredCart.Acceleration))
      {
         pd.CurrentCart.Velocity += pd.DesiredCart.Acceleration; 
      }
      else
      {
        pd.CurrentCart.Velocity = MAX_CART_VELOCITY;
      }
   }
   else
   {
      if(pd.CurrentCart.Velocity > (-MAX_CART_VELOCITY + pd.DesiredCart.Acceleration))
      {
         pd.CurrentCart.Velocity += pd.DesiredCart.Acceleration;
      }
      else
      {
         pd.CurrentCart.Velocity = -MAX_CART_VELOCITY;
      }
   }

   WriteStepSize(pd.CurrentCart.Velocity);
}
//*********************************************
//  SetupTimer0
//
//  Used to generate the 10 ms task rate
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

//*********************************************
// SetupTimer1
//
//  Used to generate the stepper motor pulses
//********************************************
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



//****************************************
//  Setup
//
//****************************************
void setup()
{
  //Initialize Variables
  pd.CurrentCart.Velocity = 0;
  pd.DesiredCart.Velocity = 0;
  pd.CurrentCart.Count = 0;
  pd.DesiredCart.Acceleration = 1;

  //Setup Inputs
  pinMode(LIMIT_HOME, INPUT);
  pinMode(SWITCH1, INPUT);
  pinMode(SWITCH2, INPUT);
  pinMode(EncoderChannel_A, INPUT);
  pinMode(EncoderChannel_B, INPUT);

  //Setup Outputs
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_YELLOW,OUTPUT);
  pinMode(LED_RED,OUTPUT);
  pinMode(STEP,OUTPUT);
  pinMode(DIR,OUTPUT);

  //Disable interrupts
  cli();
  
  //Setup Timers
  SetupTimer0();

  SetupTimer1();
  
  //enable interrupts
  sei();
  Serial.begin(115200);
}

//*********************************************
// Return Home
//
//********************************************
bool ReturnHome()
{
   bool home = false;

   digitalWrite(LED_RED,HIGH);
   digitalWrite(LED_GREEN,HIGH);
   
   if(pd.CurrentCart.Position > 5)
   {
      pd.DesiredCart.Velocity = -100;
   }
   else if(pd.CurrentCart.Position < -5)
   {
     pd.DesiredCart.Velocity = 100;
   }
   
   //Determine which direction to go
   if(!digitalRead(LIMIT_HOME))
   {
      pd.CurrentCart.Count = 0;
      pd.DesiredCart.Velocity = 0;
      pd.CurrentCart.Velocity = 0;
      pd.DesiredCart.Acceleration = 0;
      home = true;
   }
   WriteStepSize(pd.DesiredCart.Velocity);
   return home;
}

void PID_Loop()
{
  int error = 0;
  static int pre_error = 0;

  error = pd.EncoderValue.Count;

  //Try
  // P = 0.75
  // D = 2.0

  // P = 0.6
  // D = 4.0  Better

  // P = 0.6
  // D = 8.0 
  pd.DesiredCart.Acceleration = (int)((float)error * 0.6f) + ((float)(error - pre_error) * 8.0f) ;
  pre_error = error;  
}

void SendSerialData()
{
   // Send the three structures as binary data
   Serial.write((byte*)&pd,  sizeof(pd));
}

void loop()
{
  static int counter;
  static int previous_position;
  static bool homed = false;
  static bool homing = false;
  static bool start_control = false;

  if(RunTask_10ms)
  {
    counter++;
    if(counter > 2)
    {
      //SendSerialData();
      counter = 0;
    }
    //The Encoder has 2400 pulses per rev
    pd.EncoderValue.Count = -1*PendulumPosition.read();
    pd.EncoderValue.Position = (signed int)((float)pd.EncoderValue.Count/6.66666);
    
    //Turn the number of counts in a position in mm
    pd.CurrentCart.Position = (signed int)((float)pd.CurrentCart.Count/3.3799);

    //Serial.println(pd.EncoderValue.Position);

    digitalWrite(LED_YELLOW,HIGH);
    
    if(digitalRead(SWITCH2))
    {
      start_control = true;
      digitalWrite(LED_GREEN,HIGH);
    }
    else if(digitalRead(SWITCH1))
    {
      start_control = false;
      digitalWrite(LED_RED,HIGH);
    }


    //If both Switches are pressed return home
    if(digitalRead(SWITCH2) && digitalRead(SWITCH1))
    {
      homing = true;
      start_control = false;
    }

    //Exit the homing logic.    
    if(homed == true)
    {
      homing = false;
      homed = false;
    }

    //Return back home using this function
    if((homing==true) && (homed == false))
    {
      start_control = false;
      homed = ReturnHome();
    }
    else
    {
       if(start_control == true)
       {
        PID_Loop();
        MotionAccelerationControl();
       }
       else
       {
        pd.CurrentCart.Velocity = 0;
        WriteStepSize(pd.CurrentCart.Velocity);
       }
    }

    if(pd.EncoderValue.Position > 10 || pd.EncoderValue.Position < -10)
    {
      start_control = false;
    }

    Serial.print(homing);
    Serial.print(",");
    Serial.print(homed);
    Serial.print(",");
    Serial.println(pd.CurrentCart.Position);


    pd.EncoderValue.Acceleration = TCNT0;
    pd.EncoderValue.Velocity = 75;
    pd.EncoderValue.Direction = 30000;
    RunTask_10ms = 0;
  }
}
