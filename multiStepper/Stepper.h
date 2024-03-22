#ifndef STEPPER
#define STEPPER

#define P 1
#define D 0.1

class Stepper {
private:
  uint8_t microSteps = 16;
  const uint16_t numSteps = 200;
  
  struct Bounds {
    float upper = 0;
    float lower = 0;
    bool enabled = false;
  };
  
  uint8_t stepPin;
  uint8_t stepPort;
  uint8_t stepShift;
  
  uint8_t dirPin;
  uint8_t dirPort;
  uint8_t dirShift;
  
  float outputRatio;

  volatile long stepPos; //steps
  float origin; //deg

  volatile float period;
  volatile float accumulator;

  bool dir;
  bool armed = false;
  
public:
  uint8_t id;
  Bounds bounds;

  Stepper(){}
  
  Stepper(uint8_t step_pin, uint8_t dir_pin, float outRatio = 1, float low = 1, float up = 0){
    stepPin = step_pin;
    stepPort = stepPin / 8;
    stepShift = stepPin - stepPort * 8;
    
    dirPin = dir_pin;
    dirPort = dirPin / 8;
    dirShift = dirPin - dirPort * 8;
    
    outputRatio = outRatio;
    if(low < up){
      bounds.lower = low;
      bounds.upper = up;
      bounds.enabled = true;
    }
    
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW);
  }

  void setMicrosteps(int newStep){
    microSteps = newStep;
  }

  void setOrigin(float angle){
    origin = angle;
    stepPos = 0;//angleToStep(origin);
  }

  void getOrigin(){
    return origin;
  }
  
  bool target(float angle, int pulses){
    if(!inBounds(angle)) return false;
    armed = true;
    
    long stepDelta = angleToStep(angle) - stepPos;
    if(stepDelta == 0) armed = false;
    
    period = abs(static_cast<float>(pulses) / stepDelta);
    if(id == 1){
//      Serial.print(stepDelta);
//      Serial.print(' ');
//      Serial.print(angleToStep(angle));
//      Serial.print(' ');
//      Serial.print(stepPos);
//      Serial.print(' ');
//      Serial.println(angle);
//      Serial.print(' ');
//      Serial.println(period);
//      Serial.print("delta angle: ");
//      Serial.println(1000 * (angle - getAngle()), 4);

    }
    
    if(stepDelta < 0){
      if(dir) {
        digitalWrite(dirPin, LOW);
        dir = 0;
      }
    }
    else{
      if(!dir) {
        digitalWrite(dirPin, HIGH);
        dir = 1;
      }
    }

    accumulator = 0;
    return true;
  }

  void step(uint8_t counter){
    if(accumulator <= counter && armed){  //if the counter is greater than the acc, step and add on to the acc
      accumulator += period;
      switch(stepPort){
        case 0:
          PORTD |= (1<<stepShift);
          PORTD &= ~(1<<stepShift);
          break;
        case 1:
          PORTB |= (1<<stepShift);
          PORTB &= ~(1<<stepShift);
          break;
      }

      switch(dir){ //count the step forward or backward
        case false:
          stepPos--;
          break;
        case true:
          stepPos++;
          break;
      }
      return;
    }
  }

  long angleToStep(float angle){
    return (angle - origin) * microSteps * numSteps * outputRatio / 360;
  }

  float stepToAngle(long step){
    return origin + (step * 360) / (microSteps * numSteps * outputRatio);
  }

  long getStepPos(){
    return stepPos;
  }

  float getAngle(){
    return stepToAngle(stepPos);
  }

  bool setBounds(float up, float low){
    if(up >= low){
      bounds.upper = up;
      bounds.lower = low;
      return true;
    }
    return false;
  }

  bool inBounds(float angle){
    if(bounds.enabled == false) return true;
    if(angle >= bounds.lower && angle <= bounds.upper) return true;
    return false;
  }
  
};

#endif
