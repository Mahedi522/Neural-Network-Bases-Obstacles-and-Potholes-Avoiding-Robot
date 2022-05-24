#include <math.h>
#include <Servo.h>

#include <SharpIR.h>
#define IRPin A0
#define model 1080
int distance_cm;

#define MotorA_in1 4
#define MotorA_in2 2
#define MotorB_in1 6
#define MotorB_in2 7
#define MotorA_PWM 3
#define MotorB_PWM 5

#define echoPin A1 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin A2 //attach pin D3 Arduino to pin Trig of HC-SR04

#define trigPin1 A3 
#define echoPin1 A4
#define trigPin2 8
#define echoPin2 9

Servo myservo;
long duration; // variable for the duration of sound wave travel
int dist; // variable for the distance measurement
int distl;
int distr;
/******************************************************************
 * Network Configuration - customized per network 
 ******************************************************************/

const int PatternCount = 3;
const int InputNodes = 4;
const int HiddenNodes = 5;
const int OutputNodes = 3;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;

const byte Input[PatternCount][InputNodes] = {
{1, 0, 1, 1},
{1, 1, 1, 1},
{0, 1, 0, 0},




}; 

const byte Target[PatternCount][OutputNodes] = {
{1,1, 1},
{0,0, 0},
{0,0, 0}


};

/******************************************************************
 * End Network Configuration
 ******************************************************************/


int i, j, p, q, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long  TrainingCycle;
float Rando;
float Error;
float Accum;


float Hidden[HiddenNodes];
float Output[OutputNodes];
float HiddenWeights[InputNodes+1][HiddenNodes];
float OutputWeights[HiddenNodes+1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes+1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes+1][OutputNodes];

SharpIR mySensor = SharpIR(IRPin, model);

void setup(){
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode (trigPin1, OUTPUT); 
  pinMode (echoPin1, INPUT);
  pinMode (trigPin2, OUTPUT); 
  pinMode (echoPin2, INPUT);
  
  pinMode(MotorA_in1, OUTPUT);
  pinMode(MotorA_in1, OUTPUT);
  pinMode(MotorB_in1, OUTPUT);
  pinMode(MotorB_in2, OUTPUT);
  pinMode(MotorA_PWM, OUTPUT);  
  pinMode(MotorB_PWM, OUTPUT);

   

  myservo.attach(12);
  myservo.write(90);
  
  randomSeed(analogRead(3));
  ReportEvery1000 = 1;
  for( p = 0 ; p < PatternCount ; p++ ) {    
    RandomizedIndex[p] = p ;
  }
}  

void loop (){
  train_nn();
  drive_nn();

}

void motorA(int percent){
  Serial.print("Strength is: ");
  Serial.println(percent);
  int maxSpeed = 90;
  int minSpeed = 10;
  int dir = 0;
  if (percent < 50) {
    dir = 0;
  }
  if (percent > 50) {
    dir = 1;
  }
  if (dir == 1) {
//  analogWrite(MotorA_PWM, 200);
  
  digitalWrite(MotorA_in1, HIGH);
  digitalWrite(MotorA_in2, LOW);
  Serial.println("Forwarding Motor A");
//    pinMode(APHASE, INPUT);
//    pinMode(APWM, INPUT);
//    pinMode(APHASE, OUTPUT);
//    pinMode(APWM, OUTPUT);
//    digitalWrite(APHASE, LOW);
    int drive = map(percent, 51, 100, 20, 180);
    //drive = constrain(drive, 0, 1023);
//    //SerialUSB.print("Driving Fore: ");
//    //SerialUSB.println(drive);
    Serial.print("drive is :");
    Serial.println(drive);
    analogWrite(MotorA_PWM, drive);
  }
  if (dir == 0) {
  //analogWrite(MotorA_PWM, 155);
  
  digitalWrite(MotorA_in1, LOW);
  digitalWrite(MotorA_in2, HIGH);
  Serial.println("Backwarding Motor A");
//    pinMode(APHASE, INPUT);
//    pinMode(APWM, INPUT);
//    pinMode(APHASE, OUTPUT);
//    pinMode(APWM, OUTPUT);
//    digitalWrite(APHASE, HIGH);
    int drive = map(percent, 49, 0, 20, 180);
  //  drive = constrain(drive, 0, 1023);
//    SerialUSB.print("Driving Back: ");
//    SerialUSB.println(drive);
    Serial.print("drive is :");
    Serial.println(drive);
    analogWrite(MotorA_PWM, drive);
  }
  if (percent == 50) {
  digitalWrite(MotorA_in1, LOW);
  digitalWrite(MotorA_in2, LOW);
  Serial.println("Stoping Motor A");
//    pinMode(APHASE, INPUT);
//    pinMode(APWM, INPUT);
  }
}
void motorB(int percent){
  int maxSpeed = 90;
  int minSpeed = 10;
  int dir = 0;
  if (percent < 50) {
    dir = 0;
  }
  if (percent > 50) {
    dir = 1;
  }
  if (dir == 1) {
//  analogWrite(MotorB_PWM, 200);

  digitalWrite(MotorB_in1, HIGH);
  digitalWrite(MotorB_in2, LOW);
  Serial.println("Forwarding Motor B");
//    pinMode(APHASE, INPUT);
//    pinMode(APWM, INPUT);
//    pinMode(APHASE, OUTPUT);
//    pinMode(APWM, OUTPUT);
//    digitalWrite(APHASE, LOW);
    int drive = map(percent, 51, 100, 20, 180);
 //   drive = constrain(drive, 0, 1023);
//    //SerialUSB.print("Driving Fore: ");
//    //SerialUSB.println(drive);
    analogWrite(MotorB_PWM, drive);
  }
  if (dir == 0) {

//  analogWrite(MotorB_PWM, 155);

  digitalWrite(MotorB_in1, LOW);
  digitalWrite(MotorB_in2, HIGH);
  Serial.println("Backwarding Motor B");
//    pinMode(APHASE, INPUT);
//    pinMode(APWM, INPUT);
//    pinMode(APHASE, OUTPUT);
//    pinMode(APWM, OUTPUT);
//    digitalWrite(APHASE, HIGH);
    int drive = map(percent, 49, 0, 20, 180);
 //   drive = constrain(drive, 0, 1023);
//    SerialUSB.print("Driving Back: ");
//    SerialUSB.println(drive);
    analogWrite(MotorB_PWM, drive);
  }
  if (percent == 50) {

  digitalWrite(MotorB_in1, LOW);
  digitalWrite(MotorB_in2, LOW);
  Serial.println("Stoping Motor B");
  
//    pinMode(APHASE, INPUT);
//    pinMode(APWM, INPUT);
  }
    Serial.println("---------------------------------------------------------------------------------------------------------");
}
//float search(void)
//  {
//    float duration = 0.00;                // Float type variable declaration 
//    float CM = 0.00;
//      
//      
//      digitalWrite(trigPin, LOW);        // Trig_pin output as OV (Logic Low-Level) 
//      delayMicroseconds(2);              // Delay for 2 us
//    
//      //Send 10us High Pulse to Ultra-Sonic Sonar Sensor "trigPin" 
//      digitalWrite(trigPin, HIGH);       // Trig_pin output as 5V (Logic High-Level)
//      delayMicroseconds(10);             // Delay for 10 us 
//    
//      digitalWrite(trigPin, LOW);        // Trig_pin output as OV (Logic Low-Level)
//  
//    
//    duration = pulseIn(echoPin, HIGH); // Start counting time, upto again "echoPin" back to Logical "High-Level" and puting the "time" into a variable called "duration" 
//   
//    CM = (duration / 58.82); //Convert dis into CM. 
//    
//   return CM;
//  }
float SonarSensor(int trigpin,int echopin){
  float duration = 0.00;
  float CM = 0.00;
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  duration = pulseIn(echopin, HIGH);
  CM = (duration*0.034)/2;
  delayMicroseconds(2);
  return CM;
}

void servoAngle(int dis){

  float Rightdis = 0.00;
  float Leftdis = 0.00;
//  if (dis>90){
    motorA(50);
    motorB(50);
    delay(300);
  
    myservo.write(5);
    delay(700);
    Rightdis = SonarSensor(trigPin, echoPin);
    delay(700);
    Serial.print("Right dis = ");
    Serial.println(Rightdis);
    myservo.write(90);
    delay(700);
    myservo.write(180);
    delay(700);
    Leftdis = SonarSensor(trigPin, echoPin);
    delay(700);
    Serial.print("Left dis = ");
    Serial.println(Leftdis);
    myservo.write(90);
    delay(700);

      if(Leftdis > Rightdis)
        {
          Serial.println("going left");
          motorB(20);
          motorA(65);
          delay(200);            
                        
        }

        else 
        {
          Serial.println("going right");
          motorA(20);
          motorB(65);
          delay(200);             
        }
  //} 
}
void train_nn(){
/******************************************************************
* Initialize HiddenWeights and ChangeHiddenWeights 
******************************************************************/

  for( i = 0 ; i < HiddenNodes ; i++ ) {    
    for( j = 0 ; j <= InputNodes ; j++ ) { 
      ChangeHiddenWeights[j][i] = 0.0 ;
      Rando = float(random(100))/100;
      HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
/******************************************************************
* Initialize OutputWeights and ChangeOutputWeights
******************************************************************/

  for( i = 0 ; i < OutputNodes ; i ++ ) {    
    for( j = 0 ; j <= HiddenNodes ; j++ ) {
      ChangeOutputWeights[j][i] = 0.0 ;  
      Rando = float(random(100))/100;        
      OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  Serial.println("Initial/Untrained Outputs: ");
  toTerminal();
/******************************************************************
* Begin training 
******************************************************************/

  for( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {    

/******************************************************************
* Randomize order of training patterns
******************************************************************/

    for( p = 0 ; p < PatternCount ; p++) {
      q = random(PatternCount);
      r = RandomizedIndex[p] ; 
      RandomizedIndex[p] = RandomizedIndex[q] ; 
      RandomizedIndex[q] = r ;
    }
    Error = 0.0 ;
/******************************************************************
* Cycle through each training pattern in the randomized order
******************************************************************/
    for( q = 0 ; q < PatternCount ; q++ ) {    
      p = RandomizedIndex[q];

/******************************************************************
* Compute hidden layer activations
******************************************************************/

      for( i = 0 ; i < HiddenNodes ; i++ ) {    
        Accum = HiddenWeights[InputNodes][i] ;
        for( j = 0 ; j < InputNodes ; j++ ) {
          Accum += Input[p][j] * HiddenWeights[j][i] ;
        }
        Hidden[i] = 1.0/(1.0 + exp(-Accum)) ;
      }

/******************************************************************
* Compute output layer activations and calculate errors
******************************************************************/

      for( i = 0 ; i < OutputNodes ; i++ ) {    
        Accum = OutputWeights[HiddenNodes][i] ;
        for( j = 0 ; j < HiddenNodes ; j++ ) {
          Accum += Hidden[j] * OutputWeights[j][i] ;
        }
        Output[i] = 1.0/(1.0 + exp(-Accum)) ;   
        OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;   
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;
      }

/******************************************************************
* Backpropagate errors to hidden layer
******************************************************************/

      for( i = 0 ; i < HiddenNodes ; i++ ) {    
        Accum = 0.0 ;
        for( j = 0 ; j < OutputNodes ; j++ ) {
          Accum += OutputWeights[i][j] * OutputDelta[j] ;
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;
      }


/******************************************************************
* Update Inner-->Hidden Weights
******************************************************************/


      for( i = 0 ; i < HiddenNodes ; i++ ) {     
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;
        for( j = 0 ; j < InputNodes ; j++ ) { 
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;
        }
      }

/******************************************************************
* Update Hidden-->Output Weights
******************************************************************/

      for( i = 0 ; i < OutputNodes ; i ++ ) {    
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;
        for( j = 0 ; j < HiddenNodes ; j++ ) {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;
          OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
        }
      }
    }

/******************************************************************
* Every 1000 cycles send data to terminal for display
******************************************************************/
    ReportEvery1000 = ReportEvery1000 - 1;
    if (ReportEvery1000 == 0)
    {
      Serial.println(); 
      Serial.println(); 
      Serial.print ("TrainingCycle: ");
      Serial.print (TrainingCycle);
      Serial.print ("  Error = ");
      Serial.println (Error, 5);

      toTerminal();

      if (TrainingCycle==1)
      {
        ReportEvery1000 = 999;
      }
      else
      {
        ReportEvery1000 = 1000;
      }
    }    


/******************************************************************
* If error rate is less than pre-determined threshold then end
******************************************************************/

    if( Error < Success ) break ;  
  }

  Serial.println ();
  Serial.println(); 
  Serial.print ("TrainingCycle: ");
  Serial.print (TrainingCycle);
  Serial.print ("  Error = ");
  Serial.println (Error, 5);

  toTerminal();

  Serial.println ();  
  Serial.println ();
  Serial.println ("Training Set Solved! ");
  Serial.println ("--------"); 
  Serial.println ();
  Serial.println ();  
  ReportEvery1000 = 1;
}

void drive_nn(){
//  SerialUSB.println("Running NN Drive Test");
//  if (Success < Error) {
//    prog_start = 0;
//    SerialUSB.println("NN not Trained");
//  }
  while (Error < Success) {
    float TestInput[] = {0, 0, 0, 0};

//    // Clears the trigPin condition
//    digitalWrite(trigPin, LOW);
//    delayMicroseconds(2);
//    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
//    digitalWrite(trigPin, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(trigPin, LOW);
//    // Reads the echoPin, returns the sound wave travel time in microseconds
//    duration = pulseIn(echoPin, HIGH);
//    // Calculating the distance
//    dist = (duration * 0.034) / 2; // Speed of sound wave divided by 2 (go and back)
//    // Displays the distance on the Serial Monitor
  
    dist = SonarSensor(trigPin, echoPin);
//    delay(2);
    distl = SonarSensor(trigPin1, echoPin1);
//    delay(2);
    distr = SonarSensor(trigPin2, echoPin2);
//    delay(2);
    Serial.print("Distance front: ");
    Serial.print(dist);
    Serial.println(" cm");

    distance_cm = mySensor.distance();
    Serial.print("DMS Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");

    int LL1 = dist;
    int LL2 = distance_cm;
    int LL3 = distl;
    int LL4 = distr;

    Serial.print("Distance left: ");
    Serial.print(LL3);
    Serial.println(" cm");
    Serial.print("Distance right: ");
    Serial.print(LL4);
    Serial.println(" cm");
    
    
    LL1 = map(LL1, 0, 200, 0, 100);
    LL2 = map(LL2, 10, 20, 0, 100);
    LL3 = map(LL3, 0, 200, 0, 100);
    LL4 = map(LL4, 0, 200, 0, 100); //0,200,15,100

    
    LL1 = constrain(LL1, 0, 100);
    LL2 = constrain(LL2, 0, 100);
    LL3 = constrain(LL3, 0, 100);
    LL4 = constrain(LL4, 0, 100);

    Serial.print("mapped sonar Front: ");
    Serial.println(LL1);
    Serial.print("mapped dsm: ");
    Serial.println(LL2);
    Serial.print("mapped sonar left: ");
    Serial.println(LL3);
    Serial.print("mapped sonar right: ");
    Serial.println(LL4);
    
    TestInput[0] = float(LL1)/100;
    TestInput[1] = float(LL2)/100;
    TestInput[2] = float(LL3)/100;
    TestInput[3] = float(LL4)/100;
    
    InputToOutput(TestInput[0], TestInput[1], TestInput[2], TestInput[3]);//INPUT to ANN to obtain OUTPUT

    int speedA = Output[0] *100;
    int speedB = Output[1] *100;
    int sr = Output[2]*100;

//    Serial.println(Output[0]);
//    Serial.println(Output[1]);
//    Serial.println(Output[2]);
      
    
    speedA = int(speedA);
    speedB = int(speedB);
    sr = int(sr);

    Serial.print("Speed Motor A: ");
    Serial.println(speedA);
    Serial.print("Speed Motor B:");
    Serial.println(speedB);
    Serial.print("Servo Angle: ");
    Serial.println(sr);
    
    motorA(speedA);
    motorB(speedB);
    if(sr <=40){
    servoAngle(sr);
    }
    delay(50);  
  }
}

void toTerminal()
{

  for( p = 0 ; p < PatternCount ; p++ ) { 
    Serial.println(); 
    Serial.print ("  Training Pattern: ");
    Serial.println (p);      
    Serial.print ("  Input ");
    for( i = 0 ; i < InputNodes ; i++ ) {
      Serial.print (Input[p][i], DEC);
      Serial.print (" ");
    }
    Serial.print ("  Target ");
    for( i = 0 ; i < OutputNodes ; i++ ) {
      Serial.print (Target[p][i], DEC);
      Serial.print (" ");
    }


/******************************************************************
* Compute hidden layer activations
******************************************************************/

    for( i = 0 ; i < HiddenNodes ; i++ ) {    
      Accum = HiddenWeights[InputNodes][i] ;
      for( j = 0 ; j < InputNodes ; j++ ) {
        Accum += Input[p][j] * HiddenWeights[j][i] ;
      }
      Hidden[i] = 1.0/(1.0 + exp(-Accum)) ;
    }

/******************************************************************
* Compute output layer activations and calculate errors
******************************************************************/

    for( i = 0 ; i < OutputNodes ; i++ ) {    
      Accum = OutputWeights[HiddenNodes][i] ;
      for( j = 0 ; j < HiddenNodes ; j++ ) {
        Accum += Hidden[j] * OutputWeights[j][i] ;
      }
      Output[i] = 1.0/(1.0 + exp(-Accum)) ; 
    }
    Serial.print ("  Output ");
    for( i = 0 ; i < OutputNodes ; i++ ) {       
      Serial.print (Output[i], 5);
      Serial.print (" ");
    }
  }
}

void InputToOutput(float In1,float In2, float In3, float In4)
{
  float TestInput[] = {0, 0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;
  TestInput[3] = In4;

  /******************************************************************
    Compute hidden layer activations
  ******************************************************************/

  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    Accum = HiddenWeights[InputNodes][i] ;
    for ( j = 0 ; j < InputNodes ; j++ ) {
      Accum += TestInput[j] * HiddenWeights[j][i] ;
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/

  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Accum = OutputWeights[HiddenNodes][i] ;
    for ( j = 0 ; j < HiddenNodes ; j++ ) {
      Accum += Hidden[j] * OutputWeights[j][i] ;
    }
    Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }
#ifdef DEBUG
  SerialUSB.print ("  Output ");
  for ( i = 0 ; i < OutputNodes ; i++ ) {
    SerialUSB.print (Output[i], 5);
    SerialUSB.print (" ");
  }
#endif
}
