#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HardwareSerial.h>

template<typename T>
struct Wheels {
  T left;
  T right;
};

HardwareSerial mySerial(1);
bool inPacket = 0;
WiFiUDP Udp;
float integral = 0;
float derivative = 0;
float lastError = 0;
// Constant storing the PWM signal pins on the esp for the wheels
// The left wheels signal is outputed at pin 33 and right at pin 14
const Wheels<int> WHEEL_PINS = { 33, 14 };

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
Servo servoRight, servoLeft;  // Servo for left and right pins defined

bool isCalibrating = false;

Wheels<int> offset = { 0, 0 };  // Offsets of wheel outputs for calibration

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                    gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
      "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
      "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  mySerial.setRxBufferSize(1024);
  mySerial.begin(115200, SERIAL_8N1, 16, 17);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  ledcSetup(0, 5000, 8);
  ledcAttachPin(, 0)

  pinMode(WHEEL_PINS.left, OUTPUT);
  pinMode(WHEEL_PINS.right, OUTPUT);

  if (!servoRight.attached()) {
    servoRight.setPeriodHertz(50);                    // standard 50 hz servo
    servoRight.attach(WHEEL_PINS.right, 1000, 2000);  // Attach the servo after it has been detatched
    servoLeft.setPeriodHertz(50);                     // standard 50 hz servo
    servoLeft.attach(WHEEL_PINS.left, 1000, 2000);    // Attach the servo after it has been detatched
  }

  WiFi.softAP("poo", "password");

  Udp.begin(8008);

}

void writeWheels(int left, int right) {
  servoLeft.write(left);
  servoRight.write(right);
}

void writeWheels(Wheels<int> dutyCycles) {
  writeWheels(dutyCycles.left, dutyCycles.right);
}

Wheels<int> getWheelsDutyCycle(GamepadPtr myGamepad) {
  int axisX = myGamepad->axisX(), axisY = myGamepad->axisY(), throttle = myGamepad->throttle();

  Wheels<int> output;

  int both = -(axisY / 4);

  if (!isCalibrating && axisX < 50 && axisX > -50 && axisY < 50 && axisY > -50) {
    output = { 0, 0 };
  } else if (axisY <= 0) {
    if (axisX > 0) {  // robot moving forward right
      int left = both - (axisX / 4);
      output.left = (left < 128) ? left : 127;
      output.right = both;
    } else {  // robot moving forward left
      int right = both + (axisX / 4);
      output.right = (right < 128) ? right : 127;
      output.left = both;
    }
  } else {
    if (axisX > 0) {  // robot moving backwards right
      int left = both + (axisX / 4);
      output.left = (left > -128) ? left : -128;
      output.right = both;
    } else {  // robot moving backwards left
      int right = both - (axisX / 4);
      output.right = (right > -128) ? right : -128;
      output.left = both;
    }
  }

  if (myGamepad->x()) {
    // If x is pressed on the controller, reset offsets and set isCalibrating to true for future calibration
    offset.left = 0;
    offset.right = 0;
    isCalibrating = true;
  } else if (isCalibrating) {
    // Once calibration is done (x isn't pressed but isCalibrating is still true), set the new offsets
    offset.left = output.left;
    offset.right = output.right;
    isCalibrating = false;
  }

  output.left += offset.left;
  output.right += offset.right;

  double scale = (!isCalibrating && axisX < 50 && axisX > -50 && axisY < 50 && axisY > -50) ? 0.5 : (double(throttle) / 2048.0) + 0.5;

  output.right *= scale;
  output.left *= scale;

  output.right += 128;
  output.left += 128;

  return output;
}

// Arduino loop function. Runs in CPU 1
void loop() {
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (!(myGamepad && myGamepad->isConnected())) continue;
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    // if (myGamepad->a()) {
    //   static int colorIdx = 0;
    //   // Some gamepads like DS4 and DualSense support changing the color LED.
    //   // It is possible to change it by calling:
    //   switch (colorIdx % 3) {
    //   case 0:
    //     // Red
    //     myGamepad->setColorLED(255, 0, 0);
    //     break;
    //   case 1:
    //     // Green
    //     myGamepad->setColorLED(0, 255, 0);
    //     break;
    //   case 2:
    //     // Blue
    //     myGamepad->setColorLED(0, 0, 255);
    //     break;
    //   }
    //   colorIdx++;
    // }

    if (myGamepad->b()) {
      // Turn on the 4 LED. Each bit represents one LED.
      static int led = 0;
      led++;
      // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
      // support changing the "Player LEDs": those 4 LEDs that usually
      // indicate the "gamepad seat". It is possible to change them by
      // calling:
      myGamepad->setPlayerLEDs(led & 0x0f);
    }

    // if (myGamepad->x()) {
    //   // Duration: 255 is ~2 seconds
    //   // force: intensity
    //   // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
    //   // rumble.
    //   // It is possible to set it by calling:
    //   myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
    // }

    // Another way to query the buttons, is by calling buttons(), or
    // miscButtons() which return a bitmask.
    // Some gamepads also have DPAD, axis and more.
    Serial.printf(
      "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: "
      "%4d, %4d, brake: %4d, throttle: %4d, misc: 0x%02x, gyro x:%6d y:%6d "
      "z:%6d, accel x:%6d y:%6d z:%6d\n",
      i,                         // Gamepad Index
      myGamepad->dpad(),         // DPAD
      myGamepad->buttons(),      // bitmask of pressed buttons
      myGamepad->axisX(),        // (-511 - 512) left X Axis
      myGamepad->axisY(),        // (-511 - 512) left Y axis
      myGamepad->axisRX(),       // (-511 - 512) right X axis
      myGamepad->axisRY(),       // (-511 - 512) right Y axis
      myGamepad->brake(),        // (0 - 1023): brake button
      myGamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
      myGamepad->miscButtons(),  // bitmak of pressed "misc" buttons
      myGamepad->gyroX(),        // Gyro X
      myGamepad->gyroY(),        // Gyro Y
      myGamepad->gyroZ(),        // Gyro Z
      myGamepad->accelX(),       // Accelerometer X
      myGamepad->accelY(),       // Accelerometer Y
      myGamepad->accelZ()        // Accelerometer Z
    );

    writeWheels(getWheelsDutyCycle(myGamepad));  // You can query the axis and other properties as well. See Gamepad.h
    // For all the available functions.
  }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //serail stuff
  if (mySerial.available() > 23) {

    unsigned int start = mySerial.read();  // first byte of packet, should be 0xFA

    if (start != 0xFA) return;

    uint8_t data[22];  // data array
    data[0] = start;
    int i = 1;

    while (i < 22) {  // loop through reading all pieces of data
      if (!mySerial.available()) continue;
      data[i] = Serial1.read();
      i++;
    }

    if(data[1] == 0 && !inPacket){
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      inPacket =1;
    }

    if (inPacket){
      Udp.write(data[0]);
      Udp.write(data[1]);
      Udp.write(data[4]);
      Udp.write(data[5]);
      Udp.write(data[8]);
      Udp.write(data[9]);
      Udp.write(data[12]);
      Udp.write(data[13]);
      Udp.write(data[16]);
      Udp.write(data[17]);
    }
    if (data[1]>=254 && inPacket){
      Udp.endPacket();
      inPacket=0;
    }

    byte rphLowByte =
        data[2];  // rph no clue what this unit is but it is translated to rpm later
    byte rphHighByte = data[3];

    float speed = float((((unsigned int)rphHighByte) << 8) | rphLowByte) /
            64.0;  // try bitshifting as int to stop overflow
    // may be an overflow issue in speed control

    if (speed < 0) {  // if speed is negative, set to 500 it is likely an overflow issue
        Serial.println("speed is negative");
        speed = 500;
    }

    float error = 350 - speed;

    integral += error;

    integral = constrain(integral, -20, 20);

    derivative = error - lastError;

    float output = kp * error + ki * integral + kd * derivative;

    output = constrain(output, 0, 255);

    lastError = error;

    analogWrite((byte)output);
  }
  // vTaskDelay(1);
  delay(150);
}