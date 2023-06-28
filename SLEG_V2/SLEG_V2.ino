/* PIN connections
-------------------------BEERLECADA MOTOR DRIVER DUAL-------------------------
                      |              Uno              |                   
GND                   |                               |                GND         
IN1B PWM_RIGHT yellow | pin 6                         |                VM          
IN2B DIR_RIGHT green  | pin 4                   pin 7 | green DIR_LEFT IN2A 
IN1A PWM_LEFT  yellow | pin 5                         |                SBY          
...
-------------------------------------------------------------------------------


DIR LOW Forward
DIR HIGH Backwards
*/

//How to use Motors: (TEST)
// // motor A (left) forward
// digitalWrite(MOTOR_DIR_L_PIN, LOW);
// analogWrite(MOTOR_SPEED_L_PIN, 60);
// // motor B (right) forward
// digitalWrite(MOTOR_DIR_R_PIN, LOW);
// analogWrite(MOTOR_SPEED_R_PIN, 60);
// delay(5000);
// // motor A (left) backwards
// digitalWrite(MOTOR_DIR_L_PIN, HIGH);
// analogWrite(MOTOR_SPEED_L_PIN, 195);
// // motor B (right) backwards
// digitalWrite(MOTOR_DIR_R_PIN, HIGH);
// analogWrite(MOTOR_SPEED_R_PIN, 195);
// delay(5000);

#include <SoftwareSerial.h>

// #define DEBUG_MODE

#define MOTOR_DIR_L_PIN 7
#define MOTOR_SPEED_L_PIN 6
#define MOTOR_SPEED_R_PIN 5
#define MOTOR_DIR_R_PIN 4

#define DEAD_ZONE 30
#define SPEED 100
#define MAX_SPEED 255
#define TURN_SPEED 2  //mult factor for turning


//TXD = 8 | RXD = 9
SoftwareSerial bluetoothSerial(8, 9);

//Bluetooth controll values
int btAngle = 0;
int btStrenght = 0;
int btButton = 0;

//Motor controll values
int currSpeed = 0;           //derzeitige geschwindigkeit
int currLeftSideSpeed = 0;   //derzeitige geschwindigkeit der linken räder
int currRightSideSpeed = 0;  //derzeitige geschwindigkeit der rechten räder

void setup() {
  Serial.begin(9600);
  Serial.println("Los geht's");
  bluetoothSerial.begin(9600);  //38400
  pinMode(MOTOR_DIR_L_PIN, OUTPUT);
  pinMode(MOTOR_DIR_R_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_L_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_R_PIN, OUTPUT);
}

void loop() {
  readBluetooth();
  setCurrSpeed();
  setLeftSpeed();
  setRightSpeed();

   #ifdef DEBUG_MODE
  Serial.println("CurrentSpeed: " + (String)currSpeed);
  Serial.println("Left   Speed: " + (String)currLeftSideSpeed);
  Serial.println("Right  Speed: " + (String)currRightSideSpeed);
#endif
}
//basierend auf der geschwindigkeit und der lenkung derehen sich links und recht unterschiedlich
void setCurrSpeed() {
  int x = cos(radians(btAngle)) * btStrenght;  //-100, 100
  int y = sin(radians(btAngle)) * btStrenght;  //-100, 100
  // int forward = y >= 0 ? 1 : 1;
  currSpeed = round(y * SPEED / 100);  //-128 ... 128
  Serial.println(currSpeed);
  // currLeftSideSpeed = constrain(currSpeed - (x * TURN_SPEED * forward), -MAX_SPEED, MAX_SPEED);
  // currRightSideSpeed = constrain(currSpeed + (x * TURN_SPEED * forward), -MAX_SPEED, MAX_SPEED);
  // Serial.println(currSpeed);
  // if (btAngle > 0 && btAngle < 180) {
  //   Serial.println("TOP");
  //   float diff = (90 - btAngle) * TURN_SPEED;
  //   //there are different cases: I and II

  // } else if (btAngle >= 180 && btAngle < 360) {
  //   Serial.println("BOTTOM");
  //   float diff = (270 - btAngle) * TURN_SPEED;

  // }

  // currSpeed = constrain(y * 2.55, -MAX_SPEED, MAX_SPEED);  //(int)map(y, -1, 1, -MAX_SPEED, MAX_SPEED);

  // currLeftSideSpeed = (int)constrain(map(x, -100, 100, MAX_SPEED, 0), -MAX_SPEED, MAX_SPEED);   //x * currSpeed;//currSpeed - (int)constrain(map(x, -1, 0, currSpeed, 0), 0, currSpeed);
  // currRightSideSpeed = (int)constrain(map(x, 100, -100, MAX_SPEED, 0), MAX_SPEED, -MAX_SPEED);  //x * currSpeed; //currSpeed - (int)constrain(map(x, 0, 1, currSpeed, 0), 0, currSpeed);
}
void readBluetooth() {
  /*
  If joystick to the right -> 0 degrees
  top = 90
  left = 180
  bottom =270
  */
  if (bluetoothSerial.available()) {
    String bluetoothInput = bluetoothSerial.readStringUntil('#');
  // #ifdef DEBUG_MODE
  //     Serial.println("Bluetooth input:\t" + bluetoothInput);
  // #endif
    if (bluetoothInput.length() != 7) {
      return;
    }

    String angle = bluetoothInput.substring(0, 3);
    String strength = bluetoothInput.substring(3, 6);
    String button = bluetoothInput.substring(6, 8);

    if (isDigit(angle))
      btAngle = angle.toInt();
    if (isDigit(strength))
      btStrenght = strength.toInt();
    if (isDigit(button))
      btButton = button.toInt();
  // #ifdef DEBUG_MODE
  //     Serial.println("Angle\t\t\t" + (String)btAngle);
  //     Serial.println("Strength\t\t" + (String)btStrenght);
  //     Serial.println("Button\t\t\t" + (String)btButton);
  // #endif
    bluetoothSerial.flush();
    bluetoothInput = "";
  }
}
void setLeftSpeed() {
  byte dirL = 0;
  int speedL = 0;
  MotordriverDual(currSpeed, &dirL, &speedL);
  digitalWrite(MOTOR_DIR_L_PIN, dirL);
  analogWrite(MOTOR_SPEED_L_PIN, speedL);
}
void setRightSpeed() {
  byte dirR = 0;
  int speedR = 0;
  MotordriverDual(currSpeed, &dirR, &speedR);
  digitalWrite(MOTOR_DIR_R_PIN, dirR);
  analogWrite(MOTOR_SPEED_R_PIN, speedR);

  // Serial.println(speedR);
}
void MotordriverDual(int speed, byte *realDir, int *realValue) {
  if (realDir == nullptr || realValue == nullptr) {
    return;
  }
  if (speed == 0) {
    *realDir = LOW;
    *realValue = 0;
    return;
  }

  // speed = constrain(speed, -255 + DEAD_ZONE, 255 - DEAD_ZONE);
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    *realDir = LOW;
    *realValue = speed;
  } else {
    *realDir = HIGH;
    *realValue = 255 + speed;
  }
  // *realValue = constrain(realValue, 0, 255);
}
void MotordriverDualOld(int speed, byte *realDir, int *realValue) {
  if (realDir == nullptr || realValue == nullptr) {
    return;
  }
  if (speed == 0) {
    *realDir = LOW;
    *realValue = 0;
    return;
  }

  speed = constrain(speed, -255 + DEAD_ZONE, 255 - DEAD_ZONE);

  if (speed > 0) {
    *realDir = LOW;
    *realValue = speed + DEAD_ZONE;
  } else {
    *realDir = HIGH;
    *realValue = 255 - DEAD_ZONE + speed;
  }
  // *realValue = constrain(realValue, 0, 255);
}
bool isDigit(String text) {
  for (size_t i = 0; i < text.length(); i++) {
    if (!isdigit(text.charAt(i))) {
      return false;
    }
  }
  return true;
}
