// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
//
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************\

#include <Arduino.h>

#define SERIAL_DEBUG 0
#if SERIAL_DEBUG
#define SERIAL_BAUD 9600 //115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#else
#define SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#endif

#define START_FRAME 0xABCD // [-] Start frme definition for reliable serial communication
// #define TIME_SEND 100 // [ms] Sending time interval
//#define SPEED_MAX 1024 // [-] Maximum speed for testing
// #define SPEED_STEP 20 // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define FORWARD_SPEED_MAX 1000
#define BACKWARD_SPEED_MAX -300
#define THROTTLE_FILTER_ACCELERATION 0.022 // Lower value means slower acceleration
#define THROTTLE_FILTER_BRAKE 0.03
#define THROTTLE_FILTER_NEUTRAL 0.08
#define THROTTLE_HISTERESIS 30

#define TX_PIN PD1 // D1
#define RX_PIN PD0 // D0
#define THROTTLE_PIN PD3
#define BRAKE_PIN PD2

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
float throttle = 0;
float filterValue = 0;
float throttleFiltered = 0;
int throttleFilteredInt = 0;
uint8_t isThrottlePedalPressed = 0;
uint8_t isBrakePedalPressed = 0;

typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  Serial.begin(SERIAL_BAUD);
  delay(1000);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  Serial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (Serial.available())
  {
    incomingByte = Serial.read();                                       // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return;
  }

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      // Serial.print("1: ");   Serial.print(Feedback.cmd1);
      // Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      // Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
      // Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
      // Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
      // Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
      // Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

// unsigned long iTimeSend = 0;
// int iTest = 0;
// int iStep = SPEED_STEP;

// void loop(void)
// {
//   unsigned long timeNow = millis();

//   // Check for new received data
//   //Receive();

//   // Send commands
//   // if (iTimeSend > timeNow) return;
//   // iTimeSend = timeNow + TIME_SEND;
//   delay(50);
//   Send(0, iTest);
//   //Send(1, iTest);

//   // Calculate test command signal
//   iTest += iStep;

//   // invert step if reaching limit
//   if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
//     iStep = -iStep;

//   // Blink the LED
//   digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
// }

void loop(void)
{
  unsigned long timeNow = millis();
  isThrottlePedalPressed = !digitalRead(THROTTLE_PIN);
  isBrakePedalPressed = !digitalRead(BRAKE_PIN);

  if (isBrakePedalPressed == 1) {
    throttle = BACKWARD_SPEED_MAX;
    filterValue = THROTTLE_FILTER_BRAKE;
  } else if (isThrottlePedalPressed == 1) {
    throttle = FORWARD_SPEED_MAX;
    filterValue = THROTTLE_FILTER_ACCELERATION;
  } else {
    throttle = 0;
    filterValue = THROTTLE_FILTER_NEUTRAL;
  }

  throttleFiltered = throttleFiltered * (1 - filterValue) + throttle * filterValue;
  throttleFilteredInt = int(throttleFiltered);

  if (abs(throttleFilteredInt) < THROTTLE_HISTERESIS) {
    throttleFilteredInt = 0;
  }

  #if SERIAL_DEBUG
  Serial.print("Gas: ");
  Serial.print(isThrottlePedalPressed);
  Serial.print(", Brake: ");
  Serial.print(isBrakePedalPressed);
  Serial.print(", Throttle: ");
  Serial.print(throttle);
  Serial.print(", ThrottleFiltered: ");
  Serial.println(throttleFilteredInt);
  #else
  Send(0, throttleFilteredInt);
  #endif

  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
  delay(10);
}
