//                    Code by Siddharthan🔥🔥🔥
#include <AFMotor.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"

#define ECHO_PIN A0
#define TRIG_PIN A1
#define IR_PIN A3
#define RELAY_PIN 2

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

Servo myservo;
int Speed = 0;
int turnState = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X
);

bool soilDetected = false;
const char* majorType = "UNKNOWN";

void rgb2hsv(float r, float g, float b, float &h, float &s, float &v) {
  float max = r;
  if (g > max) max = g;
  if (b > max) max = b;
  float min = r;
  if (g < min) min = g;
  if (b < min) min = b;
  v = max;
  float delta = max - min;
  if (max == 0) { s = 0; h = 0; return; }
  s = delta / max;
  if (delta == 0) { h = 0; return; }
  if (max == r)      h = (g - b) / delta;
  else if (max == g) h = 2 + (b - r) / delta;
  else               h = 4 + (r - g) / delta;
  h *= 60;
  if (h < 0) h += 360;
}

// ------------- Seed routines -------------
void dispenseSeed(const char* type) {
  stopMotors();
  lcd.clear();
  if(strcmp(type, "Soil Type 1")==0) {
    lcd.setCursor(0,0); lcd.print("Loamy soil");
    lcd.setCursor(0,1); lcd.print("Red cow peas");
    int startAng = 38, endAng = 0;
    for(int angle = startAng; angle != endAng; angle--) {
      myservo.write(angle); delay(10);
      if(angle == endAng) break;
    }
    delay(300);
    for(int angle = endAng; angle != startAng; angle++) {
      myservo.write(angle); delay(10);
      if(angle == startAng) break;
    }
    delay(300);
  } else if(strcmp(type, "Soil Type 2")==0) {
    lcd.setCursor(0,0); lcd.print("Sand & Clay soil");
    lcd.setCursor(0,1); lcd.print("Pepper");
    int startAng = 38, endAng = 90;
    for(int angle = startAng; angle != endAng; angle++) {
      myservo.write(angle); delay(10);
      if(angle == endAng) break;
    }
    delay(300);
    for(int angle = endAng; angle != startAng; angle--) {
      myservo.write(angle); delay(10);
      if(angle == startAng) break;
    }
    delay(300);
  } else if(strcmp(type, "Soil Type 3")==0) {
    lcd.setCursor(0,0); lcd.print("Red soil");
    lcd.setCursor(0,1); lcd.print("Green Bean");
    int startAng = 135, endAng = 180;
    for(int angle = startAng; angle != endAng; angle++) {
      myservo.write(angle); delay(10);
      if(angle == endAng) break;
    }
    delay(300);
    for(int angle = endAng; angle != startAng; angle--) {
      myservo.write(angle); delay(10);
      if(angle == startAng) break;
    }
    delay(300);
  } else {
    lcd.setCursor(0,0); lcd.print("UNKNOWN");
    lcd.setCursor(0,1); lcd.print("Check the soil!");
    // Do nothing else
  }
}



void dispenseWater() {
  digitalWrite(RELAY_PIN, LOW);
  delay(350); // Water ON
  digitalWrite(RELAY_PIN, HIGH);
  delay(1000);
}

bool isSoilType1(float h, float s, float v, float r, float g, float b) {
  return (h >= 19.4 && h <= 26.2 && s >= 0.57 && s <= 0.67 && v >= 0.46 && v <= 0.52 &&
          r >= 0.456 && r <= 0.518 && g >= 0.284 && g <= 0.309 && b >= 0.172 && b <= 0.195);
}
bool isSoilType2(float h, float s, float v, float r, float g, float b) {
  return (h >= 20.6 && h <= 34.5 && s >= 0.49 && s <= 0.65 && v >= 0.41 && v <= 0.50 &&
          r >= 0.412 && r <= 0.504 && g >= 0.289 && g <= 0.327 && b >= 0.177 && b <= 0.211);
}
bool isSoilType3(float h, float s, float v, float r, float g, float b) {
  return (h >= 20.4 && h <= 29.4 && s >= 0.53 && s <= 0.62 && v >= 0.44 && v <= 0.49 &&
          r >= 0.435 && r <= 0.488 && g >= 0.290 && g <= 0.318 && b >= 0.186 && b <= 0.205);
}
const char* classify(float h, float s, float v, float r, float g, float b) {
  if (isSoilType1(h,s,v,r,g,b)) return "Soil Type 1";
  if (isSoilType2(h,s,v,r,g,b)) return "Soil Type 2";
  if (isSoilType3(h,s,v,r,g,b)) return "Soil Type 3";
  return "UNKNOWN";
}

void forward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  
  motor1.setSpeed(200); 
  motor2.setSpeed(200);
  motor3.setSpeed(200); 
  motor4.setSpeed(200);
}
void stopMotors() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE); 
  motor3.run(RELEASE); 
  motor4.run(RELEASE);
}
void uTurnLeft() {
  lcd.clear(); 
  lcd.setCursor(0, 0); 
  lcd.print("LEFT");

  motor1.run(FORWARD); 
  motor2.run(FORWARD);
  motor3.run(BACKWARD); 
  motor4.run(BACKWARD);
  
  motor1.setSpeed(100); 
  motor2.setSpeed(100); 
  motor3.setSpeed(100); 
  motor4.setSpeed(100);
  delay(2200); 
  stopMotors();
}
void uTurnRight() {
  lcd.clear(); 
  lcd.setCursor(0, 0); 
  lcd.print("RIGHT");

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD); 
  motor4.run(FORWARD);
  
  motor1.setSpeed(100); 
  motor2.setSpeed(100); 
  motor3.setSpeed(100); 
  motor4.setSpeed(100);
  
  delay(2200); 
  stopMotors();
}
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Soil scan and seed select ONCE:
void takeInitialSoilReadingsAndSeed() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Examining the");
  lcd.setCursor(0,1);
  lcd.print("soil...");
  
  int soilType1Count = 0, soilType2Count = 0, soilType3Count = 0, unknownCount = 0;
  const int N = 3; // Only 3 readings
  for(int i=0; i<N; i++) {
    myservo.write(38); // Park servo
    // No forward or stopMotors calls, robot stays still
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    float norm = c ? c : 1;
    float avgRed = r / norm;
    float avgGreen = g / norm;
    float avgBlue = b / norm;
    float h, s, v;
    rgb2hsv(avgRed, avgGreen, avgBlue, h, s, v);
    const char* result = classify(h, s, v, avgRed, avgGreen, avgBlue);
    if (strcmp(result, "Soil Type 1") == 0) soilType1Count++;
    else if (strcmp(result, "Soil Type 2") == 0) soilType2Count++;
    else if (strcmp(result, "Soil Type 3") == 0) soilType3Count++;
    else unknownCount++;
    delay(900); // Slight delay between readings for stability
  }
  int majorCount = soilType1Count;
  majorType = "Soil Type 1";
  if (soilType2Count > majorCount) { majorCount = soilType2Count; majorType = "Soil Type 2"; }
  if (soilType3Count > majorCount) { majorCount = soilType3Count; majorType = "Soil Type 3"; }
  if (unknownCount > majorCount) {
    if (soilType1Count >= soilType2Count && soilType1Count >= soilType3Count && soilType1Count > 0) {
      majorType = "Soil Type 1";
    } else if (soilType2Count >= soilType1Count && soilType2Count >= soilType3Count && soilType2Count > 0) {
      majorType = "Soil Type 2";
    } else if (soilType3Count >= soilType1Count && soilType3Count >= soilType2Count && soilType3Count > 0) {
      majorType = "Soil Type 3";
    } else {
      majorType = "UNKNOWN";
    }
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(majorType);
  soilDetected = true;
}


void setup() {
  Serial.begin(9600); 
  myservo.attach(10);
  motor1.setSpeed(Speed); 
  motor2.setSpeed(Speed); 
  motor3.setSpeed(Speed); 
  motor4.setSpeed(Speed);
  
  pinMode(ECHO_PIN, INPUT); 
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(IR_PIN, INPUT); 
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  
  lcd.init(); 
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Plasma Plant");
  lcd.setCursor(0,1);
  lcd.print("by Siddharthan");
  if (tcs.begin()) tcs.setInterrupt(true); 
  else while (1);
}

void loop() {
  digitalWrite(13, HIGH);  // turn LED on
  delay(150);              // LED on for 150ms
  digitalWrite(13, LOW);   // turn LED off
  delay(150);     
  if (!soilDetected) {
    takeInitialSoilReadingsAndSeed();
    return;
  }
// Normal operation -- dispense seed/water, move, IR/US detect
  forward();
  delay(200);
  dispenseSeed(majorType);
  dispenseWater();

  int ir_value = digitalRead(IR_PIN);
  if (ir_value == LOW) {
    stopMotors();
    lcd.setCursor(0, 1); 
    lcd.print("Roof Detected!   ");
    while (digitalRead(IR_PIN) == LOW) { delay(10); }
    lcd.setCursor(0, 1); 
    lcd.print("                ");
    return;
  }

  long dist = readDistanceCM();
  Serial.print("Distance: "); 
  Serial.println(dist);
  if (dist <= 10) {
    stopMotors(); 
    delay(150);
    
    if (turnState == 0) { uTurnLeft(); turnState = 1; }
    else { uTurnRight(); 
    turnState = 0; }
    delay(150);
    while (readDistanceCM() <= 10) { delay(50); }
  }
}


