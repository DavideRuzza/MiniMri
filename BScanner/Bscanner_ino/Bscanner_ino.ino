#include <Adafruit_ADS1X15.h>
#include<Wire.h>

Adafruit_ADS1115 adc;
#include <AccelStepper.h>

#define xstep 2
#define xdir 5

#define ystep 3
#define ydir 6

#define zstep 4
#define zdir 7

#define xlim 9
#define ylim 10
#define zlim 11

#define x_step_per_mm 10./2.
#define y_step_per_mm 10./2.
#define z_step_per_mm 200/8.

#define x_max_vel 100.
#define x_acc 90.

#define y_max_vel 1500.
#define y_acc 650.

#define z_max_vel 2000.
#define z_acc 1000.

#define xmul 1
#define ymul 16
#define zmul 16

#define xdi 1
#define ydi 1
#define zdi 1

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : -1))
AccelStepper x_step = AccelStepper(AccelStepper::DRIVER, xstep, xdir);
AccelStepper y_step = AccelStepper(AccelStepper::DRIVER, ystep, ydir);
AccelStepper z_step = AccelStepper(AccelStepper::DRIVER, zstep, zdir);

#define X_MAX 15
#define Y_MAX 15
#define Z_MAX 12

#define EN 8

float x_pos = 0;
float y_pos = 0;
float z_pos = 0;

void setup() {
  Serial.begin(115200);
  pinMode(EN,OUTPUT);
  Wire.begin();
  if (!adc.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  adc.setGain(GAIN_TWO);
  
  pinMode(xlim, INPUT_PULLUP);
  pinMode(ylim, INPUT_PULLUP);
  pinMode(zlim, INPUT_PULLUP);
  
  x_step.setMaxSpeed(x_max_vel);
  x_step.setAcceleration(x_acc);
  y_step.setMaxSpeed(y_max_vel);
  y_step.setAcceleration(y_acc);
  z_step.setMaxSpeed(z_max_vel);
  z_step.setAcceleration(z_acc);
  digitalWrite(EN,HIGH);

  Serial.write(0xff);
}

void home_step(AccelStepper &stepper, int dir, int pin, bool std_val, float max_vel){
  stepper.setMaxSpeed(0.8*max_vel);
  while(digitalRead(pin)==std_val){
    stepper.move(-1*dir);
    stepper.run();
  }
}

void home_stepper(AccelStepper &stepper, int dir, int pin, bool std_val, float step_per_mm, float max_vel, float acc, float &pos){
  stepper.setCurrentPosition(0);
  long home_pos = -1*dir;
  stepper.setMaxSpeed(max_vel*0.8);
  digitalWrite(EN, LOW);
  while(digitalRead(pin)==std_val){
    stepper.moveTo(home_pos);
    stepper.run();
    home_pos=home_pos-dir;
  }

  stepper.setCurrentPosition(0);

  home_pos = -dir;
  
  while(digitalRead(pin)!=std_val){
    //Serial.println(digitalRead(pin));
    stepper.moveTo(home_pos);
    stepper.run();
    home_pos=home_pos+dir;
  }
  digitalWrite(EN, HIGH);

  stepper.setCurrentPosition(0);
  
  stepper.setMaxSpeed(max_vel);
  pos = 0;
}

float readAdc_V(){

  float volts = adc.computeVolts(adc.readADC_Differential_2_3());
  return volts; //volts to milliTesla
}

float readAdc_mT(){

  float volts = adc.computeVolts(adc.readADC_Differential_2_3());
  return volts*72.7; //volts to milliTesla
}

void write_float(float val){
  long val_int = * (long *) &val;
  Serial.write(val_int>>24&0xff);
  Serial.write(val_int>>16&0xff);
  Serial.write(val_int>>8&0xff);
  Serial.write(val_int&0xff);
  Serial.write(0x00);
}


void loop() {

  char buf[5] = {};
  int i = 0;
  while (true){
    if (Serial.available()){
      char c = Serial.read();
      if (c=='\n' && i >4){
        break;
      }
      if (i<=4){
        buf[i] = c;
        i++;
      }
    }
  }
  char cmd = buf[0];
  long val_fl = ((long)buf[1]<<24)&0xff000000 | ((long)buf[2]<<16)&0x00ff0000 | ((long)buf[3]<<8)&0x0000ff00 | (long)buf[4]&0x000000ff;
  float value = * (float *) &val_fl;
  
//  while (Serial.available() > 0) {
//    char cmd = Serial.read();
//    float value = Serial.parseFloat();
//    String end_str = Serial.readStringUntil('\n');

  switch(cmd){
    case 0x01: //move_mm x
    {
      float x_mm = value;
      x_step.move(x_mm*x_step_per_mm*xmul*xdi);
      digitalWrite(EN, LOW);
      while(x_step.run()){};
      digitalWrite(EN, HIGH);
      x_pos+=x_mm;
      Serial.write(0x01);
      write_float(x_pos);
    }break;

    case 0x02: //move_mm y
    {
      float y_mm = value;
      y_step.move(y_mm*y_step_per_mm*ymul*ydi);
      digitalWrite(EN, LOW);
      while(y_step.run()){};
      digitalWrite(EN, HIGH);
      y_pos+=y_mm;
      Serial.write(0x02);
      write_float(y_pos);
    }break;

    case 0x03: //move_mm z
    {
      float z_mm = value;
      z_step.move(z_mm*z_step_per_mm*zmul*zdi);
      digitalWrite(EN, LOW);
      while(z_step.run()){};
      digitalWrite(EN, HIGH);
      z_pos+=z_mm;
      Serial.write(0x03);
      write_float(z_pos);
    }break;

    case 0x04: // home x
    {
      home_stepper(x_step, xdi, xlim, 1, x_step_per_mm, x_max_vel, x_acc, x_pos);
      Serial.write(0x04);
      write_float(x_pos);
    }break;

    case 0x05: // home y
    {
      home_stepper(y_step, ydi, ylim, 1, y_step_per_mm, y_max_vel, y_acc, y_pos);
      Serial.write(0x05);
      write_float(y_pos);
    }break;

    case 0x06: // home z
    {
      home_stepper(z_step, zdi, zlim, 1, z_step_per_mm, z_max_vel, z_acc, z_pos);
      Serial.write(0x06);
      write_float(z_pos);
    }break;
    case 0x07: //move to x pos
    {
      float x_mm = (value-x_pos);
      x_step.move(x_mm*x_step_per_mm*xmul*xdi);
      digitalWrite(EN, LOW);
      while(x_step.run()){};
      digitalWrite(EN, HIGH);
      x_pos+=x_mm;
      Serial.write(0x07);
      write_float(x_pos);
    }break;
    
    case 0x08: //move to y pos
    {
      float y_mm = (value-y_pos);
      y_step.move(y_mm*y_step_per_mm*ymul*ydi);
      digitalWrite(EN, LOW);
      while(y_step.run()){};
      digitalWrite(EN, HIGH);
      y_pos+=y_mm;
      Serial.write(0x08);
      write_float(y_pos);
    }break;

    case 0x09: //move to z pos
    {
      float z_mm = (value-z_pos);
      z_step.move(z_mm*z_step_per_mm*zmul*zdi);
      digitalWrite(EN, LOW);
      while(z_step.run()){};
      digitalWrite(EN, HIGH);
      z_pos+=z_mm;
      Serial.write(0x09);
      write_float(z_pos);
    }break;

    case 0x0b: //Read Adc
    
    {
      float val = 0;
      for (int i = 0; i<5; i++){
        val += readAdc_mT();
      }
      val/=5;
      Serial.write(0x0b);
      write_float(val);
    }break;
    default:
    {
      Serial.write(0xff);
      write_float(0.0);
    }break;
  }
}
    /*
    if (cmd == 'M'){ // moving
      if (axis == 'X') {
        x_step.move(distance*x_step_per_mm*xmul*xdi);
        digitalWrite(EN, LOW);
        while(x_step.run()){};
        digitalWrite(EN, HIGH);
      } else if (axis=='Y') {
        y_step.move(distance*y_step_per_mm*ymul*ydi);
        digitalWrite(EN, LOW);
        while(y_step.run()){};
        digitalWrite(EN, HIGH);
      } else if (axis=='Z') {
        z_step.move(distance*z_step_per_mm*zmul*ydi);
        digitalWrite(EN, LOW);
        while(z_step.run()){};
        digitalWrite(EN, HIGH);
      }
      Serial.write(\x01);
    }
    
    else if (cmd == 'H') {// homing
      
      if (axis == 'X') {
          home_stepper(x_step, xdi, xlim, 1, x_step_per_mm, x_max_vel, x_acc, x_pos);
        } else if (axis == 'Y') {
          home_stepper(y_step, ydi, ylim, 1, y_step_per_mm, y_max_vel, y_acc, y_pos);
        } else if (axis == 'Z') {
          home_stepper(z_step, zdi, zlim, 1, z_step_per_mm, z_max_vel, z_acc, z_pos);
      }
      Serial.println(cmd);
    }
    else if (cmd == 'R'){
      float val = 0;
      for (int i = 0; i<5; i++){
        val += readAdcmT();
      }
      val/=2;
//      Serial.println(val);
      write_float(val);
//      Serial.println();
    }
    else {
      
    }
    
  }
  */
