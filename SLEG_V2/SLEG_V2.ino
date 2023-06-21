#include <SoftwareSerial.h>

#define DEBUG_MODE

#define DEAD_ZONE 30
#define MAX_SPEED 255



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
}

void loop() {
  readBluetooth();
  setCurrSpeed();
  byte dirL = 0;
  int speedL = 0;
  MotordriverDual(currLeftSideSpeed, &dirL, &speedL);
  // digitalWrite();
  // analogWrite();
#ifdef DEBUG_MODE
  Serial.println("CurrentSpeed: " + (String)currSpeed);
  Serial.println("Left   Speed: " + (String)currLeftSideSpeed);
  Serial.println("Right  Speed: " + (String)currRightSideSpeed);
#endif
}
//basierend auf der geschwindigkeit und der lenkung derehen sich links und recht unterschiedlich
void setCurrSpeed() {
  double x = sin(radians(btAngle)) * btStrenght;           //-100, 100
  double y = cos(radians(btAngle)) * btStrenght;           //-100, 100
  currSpeed = constrain(y * 2.55, -MAX_SPEED, MAX_SPEED);  //(int)map(y, -1, 1, -MAX_SPEED, MAX_SPEED);

  currLeftSideSpeed = (int)constrain(map(x, -100, 100, MAX_SPEED, 0), -MAX_SPEED, MAX_SPEED);   //x * currSpeed;//currSpeed - (int)constrain(map(x, -1, 0, currSpeed, 0), 0, currSpeed);
  currRightSideSpeed = (int)constrain(map(x, 100, -100, MAX_SPEED, 0), MAX_SPEED, -MAX_SPEED);  //x * currSpeed; //currSpeed - (int)constrain(map(x, 0, 1, currSpeed, 0), 0, currSpeed);
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
#ifdef DEBUG_MODE
    Serial.println("Bluetooth input:\t" + bluetoothInput);
#endif
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
#ifdef DEBUG_MODE
    Serial.println("Angle\t\t\t" + (String)btAngle);
    Serial.println("Strength\t\t" + (String)btStrenght);
    Serial.println("Button\t\t\t" + (String)btButton);
#endif
    bluetoothSerial.flush();
    bluetoothInput = "";
  }
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

  speed = constrain(speed, -255 + DEAD_ZONE, 255 - DEAD_ZONE);

  if (speed > 0) {
    *realDir = LOW;
    *realValue = speed + DEAD_ZONE;
  } else {
    *realDir = HIGH;
    //anstatt "- abs(speed)" rechne ich "+" weil "-" und "+" = "-"
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
