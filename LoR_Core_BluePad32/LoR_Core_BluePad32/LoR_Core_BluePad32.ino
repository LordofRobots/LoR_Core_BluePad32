/**
 * fLoR_Core_BluePad32.ino
 * 
 * Description:
 * This project file integrates the LoR (Lord of Robots) library to control a robotic platform
 * equipped with wheels, utilizing joystick input for dynamic movement control and LED feedback. 
 * It features motion control via a wheel drive mechanism, which allows for movement 
 * . This implementation utilizes the LoR library to handle complex
 * input and output operations, including motor control and serial communication.
 * 
 * Key Features:
 * - rive Control: Enables movement using joystick inputs.
 * - Motor Speed Control: Uses a slew rate control function to manage acceleration and deceleration smoothly.
 * - LED Feedback System: Controls an Adafruit NeoPixel LED strip to provide visual feedback based on the system status.
 * - Gamepad Support: Integrates with Bluepad32 to support various gamepad controllers for intuitive control.
 * 
 * Components:
 * - Motion_Control: Processes joystick inputs to compute the target motor speeds.
 * - Set_Motor_Output: Manages individual motor speeds based on computed targets.
 * - Motor_Control: Coordinates the updating of all motors' outputs.
 * - NeoPixel_SetColour: Manages LED colors to reflect different operational statuses.
 * - Controller Connection Management: Handles the connection and disconnection events for gamepad controllers.
 * 
 * Usage:
 * To deploy this project, ensure that the LoR library is downloaded and installed in your Arduino environment.
 * The project is configured for ESP32-based controllers, leveraging multitasking capabilities to handle
 * multiple operations concurrently. Adjust the motor and pin configurations as necessary to match your hardware setup.
 * 
 * Setup:
 * - Ensure all necessary libraries are included and the hardware connections are correctly configured.
 * - For detailed library usage and additional functionality, refer to the LoR library documentation.
 * - Install ESP32 boards through board manager. Must include this link in file/preferences https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 * - Install BluePad32 boards through board manager. Must include this link in file/preferences https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
 * - The code is designed to be uploaded to an ESP32 controller.
 * - Target board: Choose the bluepad version of the "ESP32 dev module"
 *
 * Author:
 * Dave Barratt
 * Khushi Tailor
 * 
 * Date:
 * JUNE 2 2024
 */

#include <LoR.h> // Install via Library manager, by Lord of Robots

//////////////////////////////////////////////////////////////////////////
/////             Serial  config and functions                        /////
//////////////////////////////////////////////////////////////////////////

//Serial SETUP ------------------------------------------------------------
void INIT_Serial(){
  Serial.begin(115200);
  delay(200);
  Serial.println("MiniBot: System Startup...");
}

//////////////////////////////////////////////////////////////////////////
/////              Motor config and functions                        /////
//////////////////////////////////////////////////////////////////////////

//GPIO SETUP ------------------------------------------------------------
void INIT_GPIO() {
  // Set up the pins
  pinMode(LED_DataPin, OUTPUT);
  pinMode(SwitchPin, INPUT_PULLUP);
  pinMode(MotorEnablePin, OUTPUT);
  digitalWrite(MotorEnablePin, 0);


  for (int i = 0; i < 6; i++) {
    pinMode(motorPins_A[i], OUTPUT);
    pinMode(motorPins_B[i], OUTPUT);
    digitalWrite(motorPins_A[i], 0);
    digitalWrite(motorPins_B[i], 0);
  }

  delay(1000);  // to encurage startup stability

  // output preset bias
  digitalWrite(LED_DataPin, 0);
  digitalWrite(MotorEnablePin, 1);

  //Motor test tones
  Start_Tone();
}

// PWM CONFIG ------------------------------------------------------------
// configure LED PWM functionalitites
void INIT_PWM() {
  for (int i = 0; i < 6; i++) {
    ledcSetup(MOTOR_PWM_Channel_A[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR_PWM_Channel_B[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(motorPins_A[i], MOTOR_PWM_Channel_A[i]);
    ledcAttachPin(motorPins_B[i], MOTOR_PWM_Channel_B[i]);
    // ledcWrite(motorPins_A[i], 0);
    // ledcWrite(motorPins_B[i], 0);
  }
}

// STARTUP TONES ------------------------------------------------------
// Auditable Tones created in the motors. Cycle through each motor.
void Start_Tone() {
  for (int i = 0; i < 6; i++) {
    long ToneTime = millis() + 200;
    bool state = 0;
    while (millis() < ToneTime) {
      digitalWrite(motorPins_A[i], state);
      digitalWrite(motorPins_B[i], !state);
      state = !state;
      long WaitTime = micros() + (100 * (i + 1));
      while (micros() < WaitTime) {}
    }
    digitalWrite(motorPins_A[i], 0);
    digitalWrite(motorPins_B[i], 0);
    delay(50);
  }
  for (int i = 0; i < 6; i++) {  // most likely not needed
    digitalWrite(motorPins_A[i], 0);
    digitalWrite(motorPins_B[i], 0);
  }
}

// SLEW RATE ------------------------------------------------------------
//Function to handle slew rate for motor speed ramping
// Slew rate for ramping motor speed
int SLEW_RATE_MS = 200;
int SlewRateFunction(int Input_Target, int Input_Current) {
  int speedDiff = Input_Target - Input_Current;
  if (speedDiff > 0) Input_Current += min(speedDiff, SLEW_RATE_MS);
  else if (speedDiff < 0) Input_Current -= min(-speedDiff, SLEW_RATE_MS);
  Input_Current = constrain(Input_Current, -512, 512);
  return Input_Current;  //// BYPASSED
}

//MOTOR OUTPUT CONTROL --------------------------------------------------
// Function to control motor output based on input values from -512 to 512
// PWM config: 8bit @ 20,000hz
// Motor speed limits and starting speed
const int MAX_SPEED = 255;
const int MIN_STARTING_SPEED = 100;  // to overcome internal friction inside motor assembly
const int STOP = 0;
void Set_Motor_Output(int Output, int Motor_ChA, int Motor_ChB) {
  Output = constrain(Output, -512, 512);
  int Mapped_Value = map(abs(Output), 0, 512, MIN_STARTING_SPEED, MAX_SPEED);
  int A, B = 0;
  if (Output < -1) {  // Rotate Clockwise
    A = 0;
    B = Mapped_Value;
  } else if (Output > 1) {  // Rotate Counter-Clockwise
    A = Mapped_Value;
    B = 0;
  } else {  // Rotation Stop
    A = STOP;
    B = STOP;
  }
  ledcWrite(Motor_ChA, A);  //send to motor control pins
  ledcWrite(Motor_ChB, B);
}

//MOTION CONTROL ALGORITHM ---------------------------------------------
// Process joystick input and calculate motor speeds
const float TURN_RATE = 1;
int Motor_LEFT_SetValue, Motor_RIGHT_SetValue = 0;
void Motion_Control(int ForwardBackward_Axis, int TurnLeftRight_Axis) {
  int LEFT_TargetValue = 0;
  int RIGHT_TargetValue = 0;
  const int DEAD_BAND = 50;

  if (abs(ForwardBackward_Axis) < DEAD_BAND) ForwardBackward_Axis = 0;
  else {
    LEFT_TargetValue = ForwardBackward_Axis;
    RIGHT_TargetValue = ForwardBackward_Axis;
  }
  //calculate rotation values
  if (abs(TurnLeftRight_Axis) < DEAD_BAND) TurnLeftRight_Axis = 0;
  else {
    LEFT_TargetValue -= (TURN_RATE * TurnLeftRight_Axis);
    RIGHT_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
  }

  //constrain to joystick range
  LEFT_TargetValue = constrain(LEFT_TargetValue, -512, 512);
  RIGHT_TargetValue = constrain(RIGHT_TargetValue, -512, 512);

  //set motor speed through slew rate function
  SLEW_RATE_MS = 300;
  Motor_LEFT_SetValue = SlewRateFunction(LEFT_TargetValue, Motor_LEFT_SetValue);
  Motor_RIGHT_SetValue = SlewRateFunction(RIGHT_TargetValue, Motor_RIGHT_SetValue);
}

// MINIBOT 3 MOTOR DRIVE ----------------------------------------------
//send values to all motors
void DriveControl(int LEFT, int RIGHT) {
  Set_Motor_Output(LEFT, Motor_M1_A, Motor_M1_B);
  Set_Motor_Output(LEFT, Motor_M2_A, Motor_M2_B);
  Set_Motor_Output(LEFT, Motor_M3_A, Motor_M3_B);
  Set_Motor_Output(-RIGHT, Motor_M4_A, Motor_M4_B);
  Set_Motor_Output(-RIGHT, Motor_M5_A, Motor_M5_B);
  Set_Motor_Output(-RIGHT, Motor_M6_A, Motor_M6_B);
}




//////////////////////////////////////////////////////////////////////////
//                     RGB LED config and functions                     //
//////////////////////////////////////////////////////////////////////////

// SETUP & CONFIG --------------------------------------------------------
// Set a specific color for the entire NeoPixel strip
// NeoPixel Configurations
Adafruit_NeoPixel strip(LED_COUNT, LED_DataPin, NEO_GRB + NEO_KHZ800);
const uint32_t RED = strip.Color(255, 0, 0, 0);
const uint32_t GREEN = strip.Color(0, 255, 0, 0);
const uint32_t BLUE = strip.Color(0, 0, 255, 0);
const uint32_t WHITE = strip.Color(0, 0, 0, 255);
const uint32_t PURPLE = strip.Color(255, 0, 255, 0);
const uint32_t CYAN = strip.Color(0, 255, 255, 0);
const uint32_t YELLOW = strip.Color(255, 255, 0, 0);

// Neopixels Configuration
void INIT_rgbLED() {
  strip.begin();            // INITIALIZE NeoPixel strip object
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
  NeoPixel_SetColour(BLUE);
}

void NeoPixel_SetColour(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    strip.show();                                // Update strip with new contents
  }
}


//////////////////////////////////////////////////////////////////////////
//           BluePad32 controller config and functions                  //
//////////////////////////////////////////////////////////////////////////

ControllerPtr myController = nullptr;  // Define a single controller pointer
// SETUP BLUEPAD -----------------------------------------------------------------------------
void INIT_BluePad32() {
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  //BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// GAMEPAD CONNECT -----------------------------------------------------------------------------
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.println("Controller connected");
    myController = ctl;  // Assign the connected controller to the single pointer
    myController->playDualRumble(100, 100, 255, 255);
  } else {
    Serial.println("Another controller tried to connect but is rejected");
    ctl->disconnect();  // Reject the connection if another controller tries to connect
  }
}

// GAMEPAD DISCONNECT -----------------------------------------------------------------------------
void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("Controller disconnected");
    NeoPixel_SetColour(RED);
    myController = nullptr;  // Reset the controller pointer when disconnected
  }
}

// GAMEPAD SERIAL OUTPUT -----------------------------------------------------------------------------
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d BATT:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ(),       // Accelerometer Z
    ctl->battery()       // Battery level
  );
}

// GAMEPAD BATTERY -----------------------------------------------
void CheckController_Battery() {
  int Batt_Level = map(myController->battery(), 0, 255, 1, 8);

  myController->setPlayerLEDs(Batt_Level);
  if (Batt_Level < 4) {
    myController->setColorLED(200, 255, 0);
  } else if (Batt_Level < 2) {
    myController->setColorLED(255, 0, 0);
    //myController->playDualRumble(100, 100, 100, 100);
  } else {
    myController->setColorLED(0, 255, 0);
  }
}

// GAME PAD COMPATIBILITY -----------------------------------------------
void processGamePad() {
  if (myController->isGamepad()) {
    dumpGamepad(myController);
    NeoPixel_SetColour(GREEN);
    //CheckController_Battery();
  } else {
    Serial.println("Unsupported controller");
    NeoPixel_SetColour(YELLOW);
  }
}

///////////////////////////////////////////////////////////////////////
//                          Main SETUP                               //
///////////////////////////////////////////////////////////////////////

// Set up pins, LED PWM functionalities and begin PS4 controller, Serial and Serial2 communication
void setup() {
  INIT_Serial();
  INIT_rgbLED();
  LoR.begin();  // Initialize the LoR library
  INIT_BluePad32();
  INIT_GPIO();
  INIT_PWM();
  NeoPixel_SetColour(RED);
  Serial.println("MiniBot: CORE System Ready! ");
}


///////////////////////////////////////////////////////////////////////
//                          Main SETUP                               //
///////////////////////////////////////////////////////////////////////

void loop() {

  BP32.update();
  if (myController && myController->isConnected()) {
    processGamePad();
    Motion_Control(myController->axisY(), myController->axisRX());  // Joystick control
    DriveControl(Motor_LEFT_SetValue, Motor_RIGHT_SetValue);
  }
  else {  //Stop/Standby

    DriveControl(STOP, STOP);
  }
}