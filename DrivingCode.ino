#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <TimerOne.h>

//Push Button Pins
const int buttonPin1 = 22;       // Pin 4 for the button 1
const int buttonPin2 = 24;      // Pin 3 for the button 2
const int buttonPin3 = 26;      // Pin 2 for the button 3

// Color Sorting Pins
#define s0 23        //Module pins   wiring
#define s1 25
#define s2 27
#define s3 29
#define out 31

//States
const int LANE = 200;
const int THRESHOLD = 201;
const int COLOR_DETECTION = 202;
const int SORT = 203;
const int START = 204;
const int PLANT_DRIVE = 205;
const int WALL_DRIVE = 206;
const int TURNING = 207;
const int STOP = 208;
const int END = 209;
int state = 200;

//Inputs
int clickCount1 = 0;            // Variable to store the click count
int clickCount2 = 0;
int clickCount3 = 0;
int LastclickCount1 = 5;
int LastclickCount2 = 5;
int LastclickCount3 = 5;
int lastButtonState1 = 0;    // Variable to store the last state of the button
int lastButtonState2 = 0; 
int lastButtonState3 = 0; 
int plant1 = 100;
int plant2 = 100;
int plant3 = 100;
int UNKNOWN = 100;
int YELLOW = 101;
int BLUE = 102;
int GREEN = 103;
int RED = 104;
String lane1 = "";
String lane2 = "";
String lane3 = "";

//Sorting
int   Red_0=0, Blue_0=0, Green_0=0;
int   Red_=0, Blue_=0, Green_=0;  //RGB values 
int   TYellow=0, TRed=0, TBlue=0, TGreen=0; //Total Balls of each color sorted
int   Ball = UNKNOWN;
int   FULL = 4;

//Servos
Servo ServoSort;
Servo ServoGate;
Servo ServoP3;
Servo ServoP2;
Servo ServoP1;

const byte SorterServo = 13;
const byte Lane1angle = 35;
const byte Lane2angle = 85;
const byte Lane3angle = 135;

const byte GateServo = 11;
const byte OpenGate = 100;
const byte CloseGate = 0;

const byte Plant3Servo = 4;
const byte Plant2Servo = 5;
const byte Plant1Servo = 6;
const byte OpenPlant = 90;
const byte ClosePlant = 180;

//Timer
long int TimeRem = 120000;
long int TimeCon = 0;
long int TimeStart = 0;
long int TimeEnd = 0;

//Driving
int PlantCount = 0;

// Encoder setup
const int LCHA_PIN = 18;
const int LCHB_PIN = 17;
const int RCHA_PIN = 19;
const int RCHB_PIN = 16;
volatile int Lenc_count = 0; volatile int Lenc_change = 0;
int Ltemp_count = 0;
volatile int Renc_count = 0; volatile int Renc_change = 0;
int Rtemp_count = 0;
long int speed_time = 0;
float Lspeed = 0;
float Rspeed = 0;
float desired_speed = 200;
float sampling_delay = 10;
long int pos_time = 0; float turnPos = 420; float Rpos = 0; bool turnSwitch = 0; long int turnSettle = 6000;

// Ultrasonic setup
#define TIMER_US 100                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks
const int RtrigPin = 12;
const int RechoPin = 2;
const int FtrigPin = 10;
const int FechoPin = 3;
volatile long echo_start = 0;                         // Records start of echo pulse 
volatile long echo_end = 0;    
volatile long echo_duration = 0;                      
volatile int trigger_time_count = 0;                  
volatile long range_flasher_counter = 0;              
long int ultraSpace = 69; // variable to control distance change reading

// Motor control setup
int leftMotor = 8;
int rightMotor = 9;
int In1 = 34;
int In2 = 32;
int In3 = 30;
int In4 = 28;
float Lerror = 0; float Lerror_prior = 0; float Lintegral_prior = 0; float Lintegral = 0; float Lderivative = 0; float Ldrive = 0;
float Rerror = 0; float Rerror_prior = 0; float Rintegral_prior = 0; float Rintegral = 0; float Rderivative = 0; float Rdrive = 0; float Rturn = 0;
int actual_time = 0;
float LKP = .1; float LKI = 0.03; float LKD = .5; float RKP = .1; float RKI = 0.03; float RKD = .5;
float Rscaling_factor = 7; float Lscaling_factor = 7;

//SubFSM setup
int prevstate = STOP;

//route Setup
int wall = 1;
int lap = 0;

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif
LiquidCrystal_I2C lcd(0x27,20,4);


void setup() {
  pinMode(buttonPin1, INPUT);   // Set button pin as input
  pinMode(buttonPin2, INPUT);   // Set button pin as input
  pinMode(buttonPin3, INPUT);   // Set button pin as input


  lcd.init();                      // initialize the lcd 
  lcd.backlight();

  Serial.begin(9600);          // Initialize serial communication

  pinMode(s0,OUTPUT);     //pin modes
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(out,INPUT);

  digitalWrite(s0,HIGH); //Putting S0/S1 on HIGH/HIGH levels   means the output frequency scalling is at 100% (recommended)
  digitalWrite(s1,HIGH);   //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%

  //Servo 
  ServoSort.attach(SorterServo);
  ServoGate.attach(GateServo);
  ServoP3.attach(Plant3Servo);
  ServoP2.attach(Plant2Servo);
  ServoP1.attach(Plant1Servo);
  
  ServoSort.write(Lane2angle); // Set default position
  ServoGate.write(CloseGate); // Set default position
  ServoP3.write(ClosePlant); // Set default position
  ServoP2.write(ClosePlant); // Set default position
  ServoP1.write(ClosePlant); // Set default position

  pinMode(LCHA_PIN, INPUT);
  pinMode(LCHB_PIN, INPUT);
  pinMode(RCHA_PIN, INPUT);
  pinMode(RCHB_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LCHA_PIN), Lencoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RCHA_PIN), Rencoder, RISING);
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(RtrigPin, OUTPUT); // Sets trigPin as output
  pinMode(RechoPin, INPUT); // Sets echoPin as input
  pinMode(FtrigPin, OUTPUT); // Sets trigPin as output
  pinMode(FechoPin, INPUT); // Sets echoPin as input
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt(timerIsr);                 // Attach interrupt to the timer service routine 
  attachInterrupt(digitalPinToInterrupt(2), Recho_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
  attachInterrupt(digitalPinToInterrupt(3), Fecho_interrupt, CHANGE);
}

void loop() {
  // Read the current state of the button 
  int buttonState1 = digitalRead(buttonPin1); 
  int buttonState2 = digitalRead(buttonPin2); 
  int buttonState3 = digitalRead(buttonPin3);
  switch (state) {
    case LANE:
      // Check for rising edge (button going from LOW to HIGH)
      if (buttonState1 == HIGH && lastButtonState1 == LOW) {
        clickCount1++; // Increment the click count
      }
      if (buttonState2 == HIGH && lastButtonState2 == LOW) {
        clickCount2++; // Increment the click count
      }
      if (buttonState3 == HIGH && lastButtonState3 == LOW) {
        if (plant1 == UNKNOWN || plant2 == UNKNOWN || (plant1==plant2)){
        clickCount3 =0;
        }
        else{
        clickCount3++; // Increment the click count
        }
      }

      // Update the last button state
      lastButtonState1 = buttonState1;
      lastButtonState2 = buttonState2;
      lastButtonState3 = buttonState3;

      
      if (clickCount3 == 0){
        if(clickCount1 != LastclickCount1){
          lcd.setCursor(0,0);
          lcd.print("Plant 1 ");
          if (clickCount1 == 3) {
            plant1 = RED;
            lane1 ="Red     ";
            lcd.print(lane1);
          } 
          else if (clickCount1 == 2) {
            plant1 = GREEN;
            lane1 ="Green   ";
            lcd.print(lane1);
          } 
          else if (clickCount1 == 1) {
            plant1 = BLUE;
            lane1 ="Blue    ";
            lcd.print(lane1);
          } 
          else {
              plant1 = UNKNOWN;
              lane1 ="Unknown  ";
              clickCount1 = 0; //reset click count1
              lcd.println(lane1);
            }
          LastclickCount1 = clickCount1; 
          }

        if(clickCount2 != LastclickCount2 ){
          lcd.setCursor(0,1);
          lcd.print("Plant 2 ");
          if (clickCount2 == 3) {
            plant2 = RED;
            lane2 ="Red    ";
            lcd.print(lane2);
          } 
          else if (clickCount2 == 2) {
            plant2 = GREEN;
            lane2 ="Green  ";
            lcd.print(lane2);
          } 
          else if (clickCount2 == 1) {
            plant2 = BLUE;
            lane2 ="Blue   ";
            lcd.print(lane2);
          }
           
          else {
            plant2 = UNKNOWN;
            lane2 ="Unknown  ";
            clickCount2 = 0; //reset click count1
            lcd.println(lane2);
          }
          
          LastclickCount2 = clickCount2; 
        }
      }

      if(clickCount3 != LastclickCount3 && clickCount3 != 0 && (plant1 != UNKNOWN || plant2 != UNKNOWN) && (plant1 != plant2) ){
        if (lane1 != lane2 && clickCount3 == 1){
          lcd.clear();
          lcd.print("P1:");
          if (plant1 == BLUE){
          lcd.print("B ");
          }
          else if (plant1 == GREEN){
            lcd.print("G ");
          }
          else if (plant1 == RED){
            lcd.print("R ");
          }
          lcd.print("P2:");
          if (plant2 == BLUE){
            lcd.print("B ");
          }
          else if (plant2 == GREEN){
            lcd.print("G ");
          }
          else if (plant2 == RED){
            lcd.print("R ");
          }
          if (plant1 == UNKNOWN || plant2 == UNKNOWN){
            //Welcome Screen
            lcd.clear();
          }
          else if (plant1 != BLUE && plant2 != BLUE){
            plant3 = BLUE;
          }
          else if (plant1 != GREEN && plant2 != GREEN){
            plant3 = GREEN;
          }
          else if (plant1 != RED && plant2 != RED){
            plant3 = RED;
          }
          lcd.print("P3:");
          if (plant3 == BLUE){
            lcd.print("B ");
          }
          else if (plant3 == GREEN){
            lcd.print("G ");
          }
          else if (plant3 == RED){
            lcd.print("R ");
          }
        }
        else if (clickCount3 == 2){
          clickCount3 = 0;
          state = THRESHOLD;
          lcd.clear();
          lcd.print("SORTING");
          break;
        }
        LastclickCount3 = clickCount3;
      }
      delay(100);
    break;

    case THRESHOLD:  {
      ServoP1.write(ClosePlant);
      ServoP2.write(ClosePlant);
      ServoP3.write(ClosePlant);
      TGreen = 0;
      TBlue - 0;
      TRed = 0;
      TimeEnd = millis();
      if (lap == 0){
        TimeEnd = 0;
      }
      TimeRem = TimeRem + TimeStart - TimeEnd;
      GetColors();                                     //Execute the GetColors function   to get the value of each RGB color
                                                    //Depending   of the RGB values given by the sensor we can define the color and displays it on   the monitor
        Red_0=Red_;
        Blue_0=Blue_;
        Green_0=Green_;
        state = COLOR_DETECTION;
      }break;
      
    case COLOR_DETECTION:  {
      ServoGate.write(CloseGate);
      delay(500);
      GetColors();

      if (Red_>35 && Blue_>40 && Green_>40){         //If the Red, Blue, and Green values within threshhold range, there is no ball            
        Ball = UNKNOWN;
        delay(500);                                   //2s delay
        
      }
      else if (Red_<=Blue_ && Red_<=Green_ && Red_<20 && Green_>15 && Blue_>10){      //if   Red value is the lowest one and smaller thant 23 it's likely Red
        delay(500);
        if (Red_ <= Blue_ && Red_ <= Green_ && Red_<20 && Green_ > 15 && Blue_ >10){
          Ball = RED;
          if (Ball == plant1){
          ServoSort.write(Lane1angle);
          }
          else if (Ball == plant2){
            ServoSort.write(Lane2angle);
          }
          else if (Ball == plant3){
           ServoSort.write(Lane3angle);
          }                                  //2s delay
          state = SORT;
        }
      }
      else if (Blue_<Green_ && Blue_<Red_ && Blue_<15 && Red_>15){    //Same thing for Blue
        delay(500);
        if (Blue_<Green_ && Blue_<Red_ && Blue_<15 && Red_>15){
        Ball = BLUE;
        if (Ball == plant1){
          ServoSort.write(Lane1angle);
        }
        else if (Ball == plant2){
          ServoSort.write(Lane2angle);
        }
        else if (Ball == plant3){
          ServoSort.write(Lane3angle);
        }
        state = SORT;
        }
      }
      else if (Green_<=Red_ && Red_>5 && Blue_>= 5 && Red_<25 && Green_<20){           //Green it was a little tricky,   you can do it using the same method as above (the lowest), but here I used a reflective   object
        delay(500);
        if (Green_<=Red_ && Red_>5 && Blue_>= 5 && Red_<25 && Green_<20){
        Ball = GREEN;
        if (Ball == plant1){
          ServoSort.write(Lane1angle);
        }
        else if (Ball == plant2){
          ServoSort.write(Lane2angle);
        }
        else if (Ball == plant3){
          ServoSort.write(Lane3angle);
        }                                   //2s delay
        state = SORT;
        }
      }
    }break;

    case SORT:  {
      if (Ball == UNKNOWN){
        state = COLOR_DETECTION;
      }

      else if (Ball == RED){
        TRed++;
        ServoGate.write(OpenGate);
        Ball = UNKNOWN;
        state = COLOR_DETECTION;
      }

      else if (Ball == BLUE){
        TBlue++;
        ServoGate.write(OpenGate);
        Ball = UNKNOWN;
        state = COLOR_DETECTION;
      }
      else if (Ball == GREEN){
        TGreen++;
        ServoGate.write(OpenGate);
        Ball = UNKNOWN;
        state = COLOR_DETECTION;
      }

      if (TRed == FULL || TBlue == FULL || TGreen == FULL){      
        state = START;
      }
      else{
        delay(500);                                   //2s delay
        Ball = UNKNOWN;
        state = COLOR_DETECTION;
      }
    }break;
    case START:  
      TimeStart = millis();
      state = PLANT_DRIVE;
      lcd.clear();
      lcd.print("DRIVING");
    break;
    case PLANT_DRIVE:
      if(millis()-speed_time > sampling_delay){
        actual_time = millis()-speed_time;
        Lspeed = (Lenc_count-Ltemp_count)*1000/(actual_time*Lscaling_factor);
        Rspeed = (Renc_count-Rtemp_count)*1000/(actual_time*Rscaling_factor);
        Ltemp_count = Lenc_count;
        Rtemp_count = Renc_count;
        speed_time = millis();
        Lerror = (1*desired_speed)-Lspeed;
        Lintegral = Lintegral_prior + Lerror*actual_time;
        Lderivative = (Lerror-Lerror_prior)/actual_time;
        Ldrive = LKI*Lintegral;
        Lerror_prior = Lerror;
        Lintegral_prior = Lintegral;
        Rerror = (1*desired_speed)-Rspeed;
        Rintegral = Rintegral_prior + Rerror*actual_time;
        Rderivative = (Rerror-Rerror_prior)/actual_time;
        Rdrive = RKI*Rintegral;
        Rerror_prior = Rerror;
        Rintegral_prior = Rintegral;
        if(Ldrive > 255){
          Ldrive = 255;
        }
        if(Ldrive < 0){
          Ldrive = 0;
        }
        if(Rdrive > 255){
          Rdrive = 255;
        }
        if(Rdrive < 0){
          Rdrive = 0;
        }
      }  
      TimeCon = TimeRem - millis() + TimeStart;
      if(TimeCon <= 0){
        state = END;
        lcd.clear();
        lcd.print("COMPLETED");
        break;
      }
      if((echo_duration/58) > 30 & echo_duration/58 < 90 & ultraSpace == 69){
        ultraSpace = millis();
      }
      else if(millis()-ultraSpace > 50 & ultraSpace != 69){
        ultraSpace = 69;
        if((echo_duration/58) > 30 & echo_duration/58 < 90){
          state = STOP;
          prevstate = PLANT_DRIVE;
          speed_time = millis();
        }
      }
    break;
    case WALL_DRIVE:  
      TimeCon = TimeRem - millis() + TimeStart;
      if(TimeCon <= 0){
        state = END;
        lcd.clear();
        lcd.print("COMPLETED");
        break;
      }
      if(millis()-speed_time > sampling_delay){
        actual_time = millis()-speed_time;
        Lspeed = (Lenc_count-Ltemp_count)*1000/(actual_time*Lscaling_factor);
        Rspeed = (Renc_count-Rtemp_count)*1000/(actual_time*Rscaling_factor);
        Ltemp_count = Lenc_count;
        Rtemp_count = Renc_count;
        speed_time = millis();
        Lerror = (1*desired_speed)-Lspeed;
        Lintegral = Lintegral_prior + Lerror*actual_time;
        Lderivative = (Lerror-Lerror_prior)/actual_time;
        Ldrive = LKP*Lerror + LKI*Lintegral + LKD*Lderivative;
        Lerror_prior = Lerror;
        Lintegral_prior = Lintegral;
        Rerror = (1*desired_speed)-Rspeed;
        Rintegral = Rintegral_prior + Rerror*actual_time;
        Rderivative = (Rerror-Rerror_prior)/actual_time;
        Rdrive = RKP*Rerror + RKI*Rintegral + RKD*Rderivative;
        Rerror_prior = Rerror;
        Rintegral_prior = Rintegral;
        if(Ldrive > 255){
          Ldrive = 255;
        }
        if(Ldrive < 0){
          Ldrive = 0;
        }
        if(Rdrive > 255){
          Rdrive = 255;
        }
        if(Rdrive < 0){
          Rdrive = 0;
        }
      }
      if(((echo_duration/58) < 37) & ((echo_duration/58) > 2)){
        prevstate = WALL_DRIVE;
        state = STOP;
        speed_time = millis();
      }
    break;
    case TURNING:  {
      TimeCon = TimeRem - millis() + TimeStart;
      if(TimeCon <= 0){
        state = END;
        lcd.clear();
        lcd.print("COMPLETED");
        break;
      }
      if(millis()-pos_time > sampling_delay){
      Rpos = Renc_count/Rscaling_factor;
      Rtemp_count = Renc_count;
      pos_time = millis();
      Rerror = (turnPos)-Rpos;
      Rderivative = (Rerror-Rerror_prior)/sampling_delay;
      Rturn = 10*Rerror;
      if(Rpos > (turnPos-10)){
        Rintegral = Rintegral_prior + Rerror*sampling_delay;
        Rturn = 15*Rerror + .01*Rintegral;
      }

      Rerror_prior = Rerror;
      Rintegral_prior = Rintegral;
    }
    if(Rturn < 0){
      //Rturn = abs(Rturn);
      turnSwitch = 1;
    }
    else{
      turnSwitch = 0;
    }
    if(Rturn > 255){
      Rturn = 255;
    }
    if(Rpos > turnPos){
      if(turnSettle == 6000){
        turnSettle = millis();
      }
    }
    if(millis()-turnSettle > 1000 & turnSettle != 6000){
        state = STOP;
        prevstate = TURNING;
        speed_time = millis();
        turnSettle = 6000;
      }

      //state = PLANT_DRIVE;
    }break;
    case STOP:  {
      TimeCon = TimeRem - millis() + TimeStart;
      if(TimeCon <= 0){
        state = END;
        lcd.clear();
        lcd.print("COMPLETED");
        break;
      }
      if(millis()-speed_time > 300){
      Lerror = 0; Lerror_prior = 0; Lintegral_prior = 0; Lintegral = 0; Lderivative = 0; Ldrive = 0;
      Rerror = 0; Rerror_prior = 0; Rintegral_prior = 0; Rintegral = 0; Rderivative = 0; Rdrive = 0;
      Lenc_count = 0; Lenc_change = 0; Ltemp_count = 0; Renc_count = 0; Renc_change = 0; Rtemp_count = 0; Lspeed = 0; Rspeed = 0; Rpos = 0;
      if(prevstate == PLANT_DRIVE){
        if (wall == 1){
          //insert servo code here
          ServoP1.write(OpenPlant);
          delay(500);
          digitalWrite(In1, LOW);
          digitalWrite(In2, HIGH);
          digitalWrite(In3, LOW);
          digitalWrite(In4, HIGH);
          analogWrite(leftMotor, 255);
          analogWrite(rightMotor, 255);
          delay(50);
          digitalWrite(In1, HIGH);
          digitalWrite(In2, HIGH);
          digitalWrite(In3, HIGH);
          digitalWrite(In4, HIGH);
          analogWrite(leftMotor, 255);
          analogWrite(rightMotor, 255);
          delay(1500);
          //ServoP1.write(ClosePlant);       
        }
        else if(wall == 2){
          //insert servo code here
          ServoP2.write(OpenPlant);
          delay(500);
          digitalWrite(In1, LOW);
          digitalWrite(In2, HIGH);
          digitalWrite(In3, LOW);
          digitalWrite(In4, HIGH);
          analogWrite(leftMotor, 255);
          analogWrite(rightMotor, 255);
          delay(50);
          digitalWrite(In1, HIGH);
          digitalWrite(In2, HIGH);
          digitalWrite(In3, HIGH);
          digitalWrite(In4, HIGH);
          analogWrite(leftMotor, 255);
          analogWrite(rightMotor, 255);
          delay(1500);
          //ServoP2.write(ClosePlant);          
        }
        else if(wall == 3){
          //insert servo code here
          ServoP3.write(OpenPlant);
          delay(500);
          digitalWrite(In1, HIGH);
          digitalWrite(In2, LOW);
          digitalWrite(In3, HIGH);
          digitalWrite(In4, LOW);
          analogWrite(leftMotor, 255);
          analogWrite(rightMotor, 255);
          delay(50);
          digitalWrite(In1, HIGH);
          digitalWrite(In2, HIGH);
          digitalWrite(In3, HIGH);
          digitalWrite(In4, HIGH);
          analogWrite(leftMotor, 255);
          analogWrite(rightMotor, 255);
          delay(1500);
          //ServoP3.write(ClosePlant);                     
        }
        state = WALL_DRIVE;
      }
      if(prevstate == WALL_DRIVE){
        wall++;
        state = TURNING;
        if(wall == 3){
          turnPos = 400;
        }
        else{
          turnPos = 410;
        }      
      }
      if(prevstate == TURNING | prevstate == STOP){
        if(wall == 4){
          state = WALL_DRIVE;
        }
        else if(wall == 1 | wall == 2 | wall == 3){
          state = PLANT_DRIVE;
        }
        else if(wall == 5){
          wall = 1;
          lap++;
          state = THRESHOLD;
        }
        else{
          state = PLANT_DRIVE;
        }        
      }
      prevstate = STOP;
      echo_duration = 100*58;
    }
    break;
  }
    case END:  {
    }break;
  }

  if(state == WALL_DRIVE | state == PLANT_DRIVE){
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
    analogWrite(leftMotor, Ldrive);
    analogWrite(rightMotor, Rdrive);
  }
  else if(state == TURNING){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, HIGH);
    if(turnSwitch == 1){
      digitalWrite(In3, HIGH);
      digitalWrite(In4, LOW);
    }
    else{
      digitalWrite(In3, LOW);
      digitalWrite(In4, HIGH);
    }
    analogWrite(leftMotor, 0);
    analogWrite(rightMotor, Rturn);
  }
  else{
    digitalWrite(In1, HIGH);
    digitalWrite(In2, HIGH);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, HIGH);
    analogWrite(leftMotor, 255);
    analogWrite(rightMotor, 255);
  }
}

// int RfindPos(RcurrentPos, RdesiredPos){
//       Rpos = Renc_count/Rscaling_factor;
//       Rtemp_count = Renc_count;
//       pos_time = millis();
//       Rerror = RdesiredPos-RcurrentPos;
//       Rderivative = (Rerror-Rerror_prior)/sampling_delay;
//       Rturn = 10*Rerror;
//       if(Rpos > (turnPos-10)){
//         Rintegral = Rintegral_prior + Rerror*sampling_delay;
//         Rturn = 15*Rerror + .01*Rintegral;
//       }
//       Rerror_prior = Rerror;
//       Rintegral_prior = Rintegral;
//     if(Rturn < 0){
//       turnSwitch = 1;
//     }
//     else{
//       turnSwitch = 0;
//     }
//     if(Rturn > 255){
//       Rturn = 255;
//     }
// }

void Rencoder() {
  Renc_change = readEncoder(RCHB_PIN);
  if (Renc_change != 0) {
    Renc_count = Renc_count - Renc_change;
  }
}

void Lencoder() {
  Lenc_change = readEncoder(LCHB_PIN);
  if (Lenc_change != 0) {
    Lenc_count = Lenc_count + Lenc_change;
  }
}

int result = 0;
int chA_last = 0;
int chB_last = 0;
int chA_new = 0;
int chB_new = 0;
int readEncoder(int chB) {
  result = 0;
  chB_new = digitalRead(chB);
    if ((chB_new == HIGH)) { // CW turn
      result = 1;
    }
    if ((chB_new == LOW)) { // CW turn
      result = -1;
    }
  return result;
}

void timerIsr() {
  static volatile int ultrastate = 0;                 // State machine variable
  if (!(--trigger_time_count))                   // Count to 200mS
  {                                              // Time out - Initiate trigger pulse
      trigger_time_count = TICK_COUNTS;           // Reload
      ultrastate = 1;                                  // Changing to state 1 initiates a pulse
  }

  switch(ultrastate)                                  // State machine handles delivery of trigger pulse
  {
    case 0:                                      // Normal state does nothing
        break;
    case 1:
        if(state == PLANT_DRIVE){
          digitalWrite(RtrigPin, HIGH); 
        }
        else if(state == WALL_DRIVE){
          digitalWrite(FtrigPin, HIGH);
        }
        else{
          digitalWrite(RtrigPin, LOW);
          digitalWrite(FtrigPin, LOW);
        }                                      
        ultrastate = 2;                                // and set state to 2
        break;
    
    case 2:                                      // Complete the pulse
    default:
        if(state == PLANT_DRIVE){
          digitalWrite(RtrigPin, LOW); 
        }
        else if(state == WALL_DRIVE){
          digitalWrite(FtrigPin, LOW);
        }    
        else{
          digitalWrite(RtrigPin, LOW);
          digitalWrite(FtrigPin, LOW);
        }               
        ultrastate = 0;                                // and return state to normal 0
        break;
  }                            // Flash the onboard LED distance indicator
}

void Recho_interrupt(){
  switch(digitalRead(RechoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();
      if(state == PLANT_DRIVE | state == WALL_DRIVE){
        echo_duration = echo_end - echo_start; 
      }                 
      break;
  }
}
void Fecho_interrupt(){
  switch(digitalRead(FechoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      if(state == PLANT_DRIVE | state == WALL_DRIVE){
        echo_duration = echo_end - echo_start; 
      }
      break;
  }
}

void GetColors(){    
  digitalWrite(s2,   LOW);                                           //S2/S3 levels define which set   of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH   is for green 
  digitalWrite(s3, LOW);                                           
   Red_ = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);       //here we wait   until "out" go LOW, we start measuring the duration and stops when "out" is   HIGH again, if you have trouble with this expression check the bottom of the code
   delay(20);  
  digitalWrite(s3, HIGH);                                         //Here   we select the other color (set of photodiodes) and measure the other colors value   using the same techinque
  Blue_ = pulseIn(out, digitalRead(out) == HIGH ? LOW   : HIGH);
  delay(20);  
  digitalWrite(s2, HIGH);  
  Green_ = pulseIn(out,   digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);  
}