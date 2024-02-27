#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <future> 
#include <mutex> 
#include <iostream> 
#include <chrono> 

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
// #define NOTE_C5  523
// #define NOTE_CS5 554
// #define NOTE_D5  587
// #define NOTE_DS5 622
// #define NOTE_E5  659
// #define NOTE_F5  698
// #define NOTE_FS5 740
// #define NOTE_G5  784
// #define NOTE_GS5 831
// #define NOTE_A5  880
// #define NOTE_AS5 932
// #define NOTE_B5  988
#define NOTE_C5  1047
#define NOTE_CS5 1109
#define NOTE_D5  1175
#define NOTE_DS5 1245
#define NOTE_E5  1319
#define NOTE_F5  1397
#define NOTE_FS5 1480
#define NOTE_G5  1568
#define NOTE_GS5 1661
#define NOTE_A5  1760
#define NOTE_AS5 1865
#define NOTE_B5  1976
#define NOTE_C6  2093
#define NOTE_CS6 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

template <typename T>
struct Wheels {
  T left;
  T right;
};

// Constant storing the PWM signal pins on the esp for the wheels
// The left wheels signal is outputed at pin 33 and right at pin 14
const Wheels<int> WHEEL_PINS = {33, 14};
struct Note {
  unsigned int frequency;
  double duration;
};

const Wheels<int> WHEEL_PINS = {33, 14};
const int BUZZER_PIN = 32;

Note billieJeanNotes[] = {{NOTE_FS5, 0.5}, {NOTE_FS5, 0.5}, {NOTE_E5, 0.5}, {NOTE_CS5, 0.5}, {NOTE_CS5, 1}, 
                          {NOTE_FS5, 0.5}, {NOTE_FS5, 1}, {NOTE_E5, 1}, {NOTE_CS5, 2}, 
                          {NOTE_FS5, 0.5}, {NOTE_FS5, 0.5}, {NOTE_FS5, 0.5}, {NOTE_E5, 0.5}, {NOTE_CS5, 1.5},  
                          {NOTE_FS5, 0.5}, {NOTE_A5, 1}, {NOTE_B5, 1}, {NOTE_A5, 0.5}, {NOTE_GS5, 0.5}, {NOTE_FS5, 3}, 
                          {NOTE_FS5, 1}, {NOTE_FS5, 0.5}, 
                          {NOTE_CS6, 0.5}, {NOTE_B5, 1.5}, {NOTE_FS5, 0.5}, {NOTE_D5, 1}, {NOTE_CS5, 2}};

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
Servo servoRight, servoLeft; // Servo for left and right pins defined

bool isCalibrating = false;

Wheels<int> offset = {0, 0}; // Offsets of wheel outputs for calibration

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

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  pinMode(WHEEL_PINS.left, OUTPUT);
  pinMode(WHEEL_PINS.right, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  if (!servoRight.attached()) {
		servoRight.setPeriodHertz(50); // standard 50 hz servo
		servoRight.attach(WHEEL_PINS.right, 1000, 2000); // Attach the servo after it has been detatched
    servoLeft.setPeriodHertz(50); // standard 50 hz servo
    servoLeft.attach(WHEEL_PINS.left, 1000, 2000); // Attach the servo after it has been detatched
	}
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

  Wheels<int> output = {0, 0};

  if(axisX < 50 && axisX > -50 && axisY < 50 && axisY > -50) {
    return output;
  }

  int both = -(axisY / 4);

  if(axisY <= 0) {
    if(axisX > 0) { // robot moving forward right
      int left = both + (axisX / 4);
      output.left = (left < 128) ? left : 127;
      output.right = both;
    } else { // robot moving forward left
      int right = both - (axisX / 4);
      output.right = (right < 128) ? right : 127;
      output.left = both;
    }
  } else {
    if(axisX > 0) { // robot moving backwards right
      int left = both - (axisX / 4);
      output.left = (left > -128) ? left : -128;
      output.right = both;
    } else { // robot moving backwards left
      int right = both + (axisX / 4);
      output.right = (right > -128) ? right : -128;
      output.left = both;
    }
  }

  if(myGamepad->x()) {
    // If x is pressed on the controller, reset offsets and set isCalibrating to true for future calibration
    offset.left = 0;
    offset.right = 0;
    isCalibrating = true;
  } else if(isCalibrating) {
    // Once calibration is done (x isn't pressed but isCalibrating is still true), set the new offsets
    offset.left = output.left;
    offset.right = output.right;
    isCalibrating = false;
  }

  output.left += offset.left;
  output.right += offset.right;

  double scale = (double(throttle) / 2048.0) + 0.5;

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
        i,                        // Gamepad Index
        myGamepad->dpad(),        // DPAD
        myGamepad->buttons(),     // bitmask of pressed buttons
        myGamepad->axisX(),       // (-511 - 512) left X Axis
        myGamepad->axisY(),       // (-511 - 512) left Y axis
        myGamepad->axisRX(),      // (-511 - 512) right X axis
        myGamepad->axisRY(),      // (-511 - 512) right Y axis
        myGamepad->brake(),       // (0 - 1023): brake button
        myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
        myGamepad->miscButtons(), // bitmak of pressed "misc" buttons
        myGamepad->gyroX(),       // Gyro X
        myGamepad->gyroY(),       // Gyro Y
        myGamepad->gyroZ(),       // Gyro Z
        myGamepad->accelX(),      // Accelerometer X
        myGamepad->accelY(),      // Accelerometer Y
        myGamepad->accelZ()       // Accelerometer Z
    );

    writeWheels(getWheelsDutyCycle(myGamepad));// You can query the axis and other properties as well. See Gamepad.h
    // For all the available functions.
  }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  // vTaskDelay(1);
  delay(150);
}