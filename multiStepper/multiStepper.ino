//FOR ARDUINO UNO

#include "Stepper.h"
#include "vec3d.h"
#include "Quaternion.h"

//WString.h:#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))

const float LENGTH_A = 200.000;
const float LENGTH_B = 194.504;
const float SHLDR_OFFSET = 22.457;

vec3d toolPos;
vec3d toolDir(0.418, 0, -0.909);
float speed = 0;
float maxSpeed = 10;
float accel = 100;
float toolLength = 100;
vec3d Home;

volatile bool running = false;

Stepper stepper[5] = {
  //stp pin, change dir pin, gear ratio to output, lower angle, upper angle
  {3, 4, -80, -3, 65}, //shoulder pitch
  {5, 6, -10, 44, 157.514}, //elbow
  {7, 8, 10},//, -80, 80}, //shoulder yaw
  {9, 10, -6}, //wrist roll
  {11, 12, 3} //wrist pitch
};

void inv_kine(vec3d, vec3d, float*);
bool runSteppers(float duration);
void lerp(vec3d, vec3d);
void test();
void serialComm();

void setup() {
  pinMode(11, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("START!");

  toolPos.x = 51.444;
  toolPos.y = 0;
  toolPos.z = 23.155;

  toolDir.normalize();

  toolPos += toolDir * toolLength;
  Home = toolPos;

  float angle[5];
  inv_kine(toolPos, toolDir, angle);

  for(uint8_t i = 0; i < 5; i++){ 
    
    stepper[i].setOrigin(angle[i]);
    stepper[i].id = i;
  }

  Serial.println(F("\nReady For Input...\n  (j) manual jog\n  (t) manual tool rotation\n  (o) enter target or angle\n  (v) to change velocity\n  (a) to change acceleration\n"));
  
}


void loop() {
  serialComm();
  
}

volatile int isrCounter = 0;
volatile int isrMax = 0;

#define isrPeriod 200

bool runSteppers(float duration){
  while(running); //wait out previous run
  
  isrMax = duration / isrPeriod;
  isrCounter = 0;
  
  float angle[5];
  inv_kine(toolPos, toolDir, angle);

  for(uint8_t i = 0; i < 5; i++){ 
//    if(!stepper[i].target(angle[i], isrMax)) return false;
    stepper[i].target(angle[i], isrMax);
  }
  

cli();
  
  TCCR1A = 0;// set entire TCCR2A register to 0
  TCCR1B = 0;// same for TCCR2B
  TCNT1  = 0;//initialize counter value to 0

  OCR1A  = 399;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS21 bit for 8 prescaler
  TCCR1B |= (1 << CS11);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();
  
  running = true;
  return true;
}

ISR(TIMER1_COMPA_vect){
  //PORTB ^= (1<<3);
  for(uint8_t i = 0; i < 5; i++){ 
    stepper[i].step(isrCounter);
  }
  if(++isrCounter == isrMax){
    TIMSK1 &= ~(1 << OCIE1A);
    running = false;
  }
}

//enum Side {right, left};
//Side side = right;
vec3d prevNormal(0, -1, 0);

void inv_kine(vec3d pos, vec3d dir, float* angles){
  
  vec3d p1 = pos - dir*toolLength;
  angles[2] = atan2(p1.y, p1.x);
  
  p1.x = sqrt(p1.x * p1.x + p1.y * p1.y) + SHLDR_OFFSET;

  angles[1] = acos(
    (p1.x * p1.x + p1.z * p1.z - LENGTH_A * LENGTH_A - LENGTH_B * LENGTH_B) / 
    (2 * LENGTH_A * LENGTH_B)
  );

  angles[0] = (PI / 2) - (atan2(p1.z, p1.x) + atan((LENGTH_B * sin(angles[1])) / (LENGTH_A + LENGTH_B * cos(angles[1]))));
  
  for(uint8_t i = 0; i < 3; i++){
    angles[i] *= 180 / PI;  //convert angles to degrees
  }
  Quaternion head, q;
  head.setByVecDeg(vec3d::Y, angles[0] + angles[1] - 90);  //pitch angle
  q.setByVecDeg(vec3d::Z, angles[2]); //yaw angle
  head = q * head;
  
//  dir.println();
  dir = head.inverse() * dir;
//  dir = head * dir;
//  dir.println();Serial.println();
  
  vec3d normal = vec3d::X.cross(dir);//dir.cross(vec3d::X);
  if(normal.dot(prevNormal) < 0){
    normal *= -1;
  }
  prevNormal = normal;
//  switch(side){
//    case right:
//      if(normal.y > 0){
//        normal *= -1;
//      }
//      break;
//    case left:
//      if(normal.y < 0){
//        normal *= -1;
//      }
//      break;
//  }
  angles[3] = atan2(normal.y, normal.z);
  angles[4] = atan2(sqrt(dir.y * dir.y + dir.z * dir.z), dir.x);
  
  for(uint8_t i = 3; i < 5; i++){
    angles[i] *= 180 / PI;  //convert angles to degrees
  }
  
}



void lerp(vec3d targetPos, vec3d targetDir = toolDir){
  
  /// check if target position is in bounds of all steppers ///
  float checkAngle[5];
  
  inv_kine(targetPos, targetDir, checkAngle);
  
  for(uint8_t i = 0; i < 5; i++){
    if(!stepper[i].inBounds(checkAngle[i])){
      Serial.println(i);
      Serial.println(F("\nTARGET OUT OF BOUNDS!!!"));
      Serial.print(F("--- Angle ")); Serial.print(checkAngle[i]); Serial.print(F(" is beyond stepper "));
      Serial.print(stepper[i].id); Serial.print(F("'s bounds ("));Serial.print(stepper[i].bounds.lower);
      Serial.print(',');Serial.print(stepper[i].bounds.upper);Serial.println(F(") ---\n"));
      return;
    }
  }
  
  
  Serial.println(F("Running..."));
  
  long lastTime = micros();

  vec3d v = targetPos - toolPos;
  float dist0 = v.mag();

  enum Mode {accl, coast, deccel, special};
  Mode mode = coast;

  Quaternion rotate(toolDir, targetDir);
  vec3d dir0 = toolDir;
  float fractionPerSecond = ((maxSpeed / toolLength) * (180 / PI)) / toolDir.angleBetween(targetDir);
  float accumulator = 0;
  
  /// Main Interpolation Loop ///
  while(toolPos != targetPos || toolDir != targetDir){
    long temp = micros();
    float dt = static_cast<float>(temp - lastTime) / 1000000;
    lastTime = temp;

    
    vec3d pointer = targetPos - toolPos;
    float dist = pointer.mag();
    pointer.normalize();
    
    
    
    float nextDistCoast = dist - (speed * dt);
    float nextDistAccel = dist - 2 * (speed * dt + 0.5 * accel * dt * dt);
    float x = 0.5 * speed * speed / accel;
    bool beforeStoppingDistCoast = nextDistCoast > x;
    bool beforeStoppingDistAccel = nextDistAccel > x;
      
    mode = coast;
    if(beforeStoppingDistAccel){
      mode = accl;
      speed += accel * dt;
      if(speed > maxSpeed){ speed = maxSpeed; mode = coast;}
    }
    else if(!beforeStoppingDistCoast){
      float temp = speed;
      speed -= accel * dt;
      mode = deccel;
      if(speed <= 0.1*maxSpeed){
        speed = temp * 0.7;
        mode = special;
      }
    }
    pointer *= speed * dt;
    if(v.dot(pointer) < 0){
      toolPos = targetPos;
      runSteppers(4000);
      continue;
    }
    toolPos += pointer;
    
//    toolDir.print();
//    Serial.print(accumulator);
//    Serial.print(' ');
//    Serial.print(fractionPerSecond);
//    Serial.print(' ');
//    Serial.print(dt, 6);
    accumulator += fractionPerSecond * dt;
    
    if(accumulator >= 1) toolDir = targetDir;
    else toolDir = rotate.slerp(accumulator) * dir0;
//    toolDir.println();
    
//    for(byte i = 0; i < 5; i++){
//      Serial.print(stepper[i].getAngle(), 4);
//      Serial.print(' ');
//    }Serial.println();

    
    if(!runSteppers(4000)){
      Serial.println();
      Serial.println(F("!!!BOUNDS EXCEEDED!!!"));
      Serial.println(F("WARNING, HARD STOP: STEPS MAY BE LOST!"));
      return;
    }
  }
//  Serial.println(F("Ready!"));
    Serial.println();
}

void serialComm(){
  if(Serial.available()){
    vec3d temp;
    static float jogDistance{};
    static float dAngle{};
    switch(Serial.read()){
      case 'v':
      {
        Serial.print(F("Please enter new velocity (c to cancel):\n Current value: "));
        Serial.print(maxSpeed);Serial.println(F(" mm/s"));
        while(1){
          if(Serial.available()){
            if(Serial.peek() == 'c') {Serial.read(); break;}
            maxSpeed = Serial.parseInt();
            Serial.print(F("  New maxSpeed is: "));
            Serial.println(maxSpeed);
            break;
          }
        }
        break;
      }
      
      case 'a':
      {
        Serial.print(F("Please enter value for acceleration (c to cancel):\n Current value: "));
        Serial.print(accel);Serial.println(F(" mm/s^2"));
        while(1){
          if(Serial.available()){
            if(Serial.peek() == 'c') {Serial.read(); break;}
            accel = Serial.parseInt();
            Serial.print(F("  New acceleration is: "));
            Serial.println(accel);
            break;
          }
        }
        break;
      }
      
      case 'j':
      {
        Serial.println(F("Manual Jog (m to set distance in mm)(o to enter target coods)(c to cancel):\n w/s -> +/- x\n a/d -> +/- y\n e/q -> +/- z  "));
        Serial.print(F("Jog distance: "));Serial.print(jogDistance);Serial.println(F(" mm"));
        while(1){
          if(Serial.available()){
            if(Serial.peek() == 'c') {Serial.read(); break;}
            switch(Serial.read()){
              case 'm':
                Serial.println(F("Please enter new jog distance in mm: "));
                while(!Serial.available());
                jogDistance = Serial.parseInt();
                Serial.print(F("  Jog distance is now "));
                Serial.print(jogDistance);
                Serial.println(F(" mm"));
                break;
              case 'w':
                temp = toolPos;
                temp += vec3d(jogDistance, 0, 0);
                lerp(temp);
                break;
              case 'a':
                temp = toolPos;
                temp += vec3d(0, jogDistance, 0);
                lerp(temp);
                break;
              case 's':
                temp = toolPos;
                temp += vec3d(-jogDistance, 0, 0);
                lerp(temp);
                break;
              case 'd':
                temp = toolPos;
                temp += vec3d(0, -jogDistance, 0);
                lerp(temp);
                break;
              case 'q':
                temp = toolPos;
                temp += vec3d(0, 0, -jogDistance);
                lerp(temp);
                break;
              case 'e':
                temp = toolPos;
                temp += vec3d(0, 0, jogDistance);
                lerp(temp);
                break;
            }
            
            Serial.print(F("Tool Position: "));
            toolPos.println();
          }
        }
        break;
      }
      
      case 't':
      {
        Serial.println(F("Manual Tool Rotation (o to enter angle)(m to set dAngle in degs)(c to cancel):\n w/s -> +/- pitch\n a/d -> +/- roll"));
        Serial.print(F("Delta Angle: "));Serial.print(dAngle);Serial.println(F(" deg"));
        while(1){
          if(Serial.available()){
            if(Serial.peek() == 'c') {Serial.read(); break;}
            switch(Serial.read()){
              case 'm':
                Serial.println(F("Please enter new delta angle in deg: "));
                while(!Serial.available());
                dAngle = Serial.parseInt();
                Serial.print(F("  Delta angle is now "));
                Serial.print(dAngle);
                Serial.println(F(" deg"));
                break;
              case 'w':
                temp = toolDir;
                temp = Quaternion(vec3d::Y, dAngle) * temp;
                lerp(toolPos, temp);
                break;
              case 'a':
                temp = toolDir;
                temp = Quaternion(vec3d::X, dAngle) * temp;
                lerp(toolPos, temp);
                break;
              case 's':
                temp = toolDir;
                temp = Quaternion(vec3d::Y, -dAngle) * temp;
                lerp(toolPos, temp);
                break;
              case 'd':
                temp = toolDir;
                temp = Quaternion(vec3d::X, -dAngle) * temp;
                lerp(toolPos, temp);
                break;
            }
            vec3d normal = vec3d::X.cross(toolDir);
            float roll = (atan2(normal.y, normal.z) * 180 / PI) - 90;
            float pitch = atan2(sqrt(toolDir.y * toolDir.y + toolDir.z * toolDir.z), toolDir.x) * 180 / PI;
            Serial.print(F("Tool Head Position:\n"));
            Serial.print(F(" Pitch: ")); Serial.print(pitch); Serial.println(F(" deg"));
            Serial.print(F(" Roll: ")); Serial.print(roll); Serial.println(F(" deg"));
            
          }
        }
        break;
      }
      
      case 'o':
      {
        Serial.print(F("Target and angle entry (l to enter lerp target)(a to enter tool angle)(c to cancel):\n"));
        vec3d target;
        while(1){
          if(Serial.available()){
            if(Serial.peek() == 'c') {Serial.read(); break;}
            switch(Serial.read()){
              case 'l':
                Serial.println(F("Enter target coordinates:"));
                Serial.print(F("  x: "));
                while(!Serial.available());
                target.x = Serial.parseInt();
                Serial.println(target.x);
                
                Serial.print(F("  y: "));
                while(!Serial.available());
                target.y = Serial.parseInt();
                Serial.println(target.y);
              
                Serial.print(F("  z: "));
                while(!Serial.available());
                target.z = Serial.parseInt();
                Serial.println(target.z);
                
                Serial.println(F("Press any key to lerp to target..."));
                while(!Serial.available());
                Serial.read();
                lerp(target);
                Serial.print(F("Tool Position: "));
                toolPos.println();
                break;
              case 'a':
                Serial.println(F("Enter target angles:"));
                Serial.print(F("  pitch: "));
                while(!Serial.available());
                target.x = Serial.parseInt();
                Serial.println(target.x);
                
                Serial.print(F("  roll: "));
                while(!Serial.available());
                target.y = Serial.parseInt();
                Serial.println(target.y);
                
                Serial.println(F("Press any key to lerp to target..."));
                while(!Serial.available());
                Serial.read();

                temp = vec3d::X;
                temp = Quaternion(vec3d::X, target.y) * Quaternion(vec3d::Y, target.x) * temp;
                lerp(toolPos, temp);
                vec3d normal = vec3d::X.cross(toolDir);
                float roll = (atan2(normal.y, normal.z) * 180 / PI) - 90;
                float pitch = atan2(sqrt(toolDir.y * toolDir.y + toolDir.z * toolDir.z), toolDir.x) * 180 / PI;
                Serial.print(F("Tool Head Position:\n"));
                Serial.print(F(" Pitch: ")); Serial.print(pitch); Serial.println(F(" deg"));
                Serial.print(F(" Roll: ")); Serial.print(roll); Serial.println(F(" deg"));
                break;
            }
          }
        }
        break;
      }
      

//        
//      case 't':
//        lerp(vec3d(100, -50, 50));
//        break;
//      case 'z':
//        lerp(toolPos, vec3d::Z * -1);
//        break;
//      case 'x':
//        vec3d roll(0, -1, -0.5);
//        roll.normalize();
//        lerp(toolPos, roll);
//        break;
//      case 'y':
//        for(byte i = 0; i < 5; i++){
//          Serial.print(stepper[i].getAngle(), 6);
//          Serial.print(' ');
//        }
//        Serial.println();
//        break;
//      case 'm':
//        lerp(vec3d(100, 0, 50));
//        break;
//      case 'n':
//        lerp(Home);
//        break;      
//      case 'v':
//        int value = Serial.parseInt();
//        maxSpeed = value;
//        Serial.print(F("maxSpeed is now: "));
//        Serial.println(maxSpeed);
//        break;
//      case 'b':
//        int b = Serial.parseInt();
//        accel = b;
//        Serial.print(F("accel is now: "));
//        Serial.println(accel);
//        break;
    }
    Serial.println(F("\nReady For Input...\n  (j) manual jog\n  (t) manual tool rotation\n  (o) enter target or angle\n  (v) to change velocity\n  (a) to change acceleration\n"));
  }
}

//void test(){
//  Serial.println("Testing...");
//  int duration = 4000;
//  isrMax = duration / isrPeriod;
//  isrCounter = 0;
//  
//  float angle[5];// = stepper[1].getAngle() + 0.1;
////  stepper[1].target(angle, isrMax);
//
//  for(uint8_t i = 0; i < 5; i++){ 
//    angle[i] = stepper[i].getAngle() + 0.08;
//    stepper[i].target(angle[i], isrMax);
//  }
//
//cli();
//  
//  TCCR1A = 0;// set entire TCCR2A register to 0
//  TCCR1B = 0;// same for TCCR2B
//  TCNT1  = 0;//initialize counter value to 0
//
//  OCR1A  = 2 * isrPeriod - 1;
//  // turn on CTC mode
//  TCCR1B |= (1 << WGM12);
//  // Set CS21 bit for 8 prescaler
//  TCCR1B |= (1 << CS11);   
//  // enable timer compare interrupt
//  TIMSK1 |= (1 << OCIE1A);
//
//  sei();
//  
//  running = true;
//  Serial.println("exit 'test' function call");
//}
