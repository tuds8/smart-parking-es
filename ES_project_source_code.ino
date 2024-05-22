#include <Servo.h>

//#define GPIO_ENABLE_W1TS 0x60000304

// Pins for ultrasonic sensor
const int echoPin = D5;
const int trigPin = D3;

// Pin for button
const int buttonPin = D2;

// Pins for 7-segment display shift registers
const int dataPin = D8;
const int latchPin = D7;
const int clockPin = D6;

// Variables for distance calculation
long duration;
double distance;
double lastDistance;

// Create a Servo object
Servo servo;

// Threshold distance in cm
const double thresholdDistance = 6.0;

// Parking space variables
int totalParkingSpots = 106; // Set initial number of parking spots
int availableParkingSpots = totalParkingSpots;
bool carEntering = true; // State to check if a car is entering or leaving

// Variables for 7-segment display
byte digit;

// Segment variables for displaying numbers
byte numbers[] = {
  B00111111, // 0
  B00000110, // 1
  B01011011, // 2
  B01001111, // 3
  B01100110, // 4
  B01101101, // 5
  B01111101, // 6
  B00000111, // 7
  B01111111, // 8
  B01101111  // 9
};

// Variables for non-blocking delay
unsigned long previousMillis = 0;
const long interval = 2000; // 2 seconds

// Servo state
bool servoOpen = false;
bool initialDelay = false; // We set a delay of 2 seconds after each car that passes the gate in order to prevent accidental gate closes (i.e. if the ultrasonic sensor has a spike in distance measurement we do not close the gate in the first 2 seconds)

// Successive car tracking variables
int successiveCarCount = 0; // Keeps track of how many cars have gone through consecutively
unsigned long lastCarTime = 0;
const long successiveInterval = 5000; // 5 seconds
const long extendedOpenTime = 40000; // 40 seconds
unsigned long extendedOpenStartTime = 0;

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT); // D3 -> GPIO0: GPIO_ENABLE_W1TS |= 1 << 0;
  pinMode(echoPin, INPUT); // D5 -> GPIO14: GPIO_ENABLE_W1TC |= 1 << 14;

  // Initialize button pin
  pinMode(buttonPin, INPUT_PULLUP); // Button is pulled up
  // D2 -> GPIO4: GPIO_ENABLE_W1TC |= 1 << 4;
  // D2 -> GPIO4: GPIO_IN |= 1 << 4; to pull it up

  // Attach the servo on digital pin D1
  servo.attach(D1); // D1 -> GPIO5: GPIO_ENABLE_W1TS |= 1 << 5;

  // Set servo to minimum position at the start
  servo.write(0);

  // Set the pins used by the shift registers to output
  pinMode(latchPin, OUTPUT); // D7 -> GPIO13: GPIO_ENABLE_W1TS |= 1 << 13;
  pinMode(clockPin, OUTPUT); // D6 -> GPIO12: GPIO_ENABLE_W1TS |= 1 << 12;
  pinMode(dataPin, OUTPUT); // D8 -> GPIO15: GPIO_ENABLE_W1TS |= 1 << 15;

  // Initialize lastDistance
  lastDistance = measureDistance();
}

void loop() {
  // Read the button state
  readButtonState();

  unsigned long currentMillis = millis();

  // Check if the initial delay period is not active
  if (!initialDelay) {
    distance = measureDistance(); // Get the distance

    // Print the distance to the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Check if the distance is less than the threshold (car is in front of the gate)
    if (distance <= thresholdDistance) {
      handleServoLogic(currentMillis);
    }
  } 
  // Else if the initial delay period is active
  else {
    // Check if the initial delay has passed
    if (currentMillis - previousMillis >= interval) { // Otherwise we wait for the delay period to pass
      distance = measureDistance();

      // Print the distance to the Serial Monitor during extended open time
      if (servoOpen) {
        Serial.print("Distance during extended open time: ");
        Serial.print(distance);
        Serial.println(" cm");
      }

      handleExtendedServoLogic(currentMillis);
    }
  }

  // Check if the interval between successive cars has exceeded the limit and that the extended servo open time has passed
  if ((successiveCarCount > 0) && (currentMillis - lastCarTime > successiveInterval) && !(currentMillis - extendedOpenStartTime < extendedOpenTime)) {
    successiveCarCount = 0;
    Serial.println("Resetting successive car count due to timeout.");
  }

  // Refresh the display continuously
  updateDisplay();

  // Short delay before the next loop iteration
  delay(1);
}

void readButtonState() {
  if (digitalRead(buttonPin) == LOW) {
    // Debounce the button
    delay(50);
    if (digitalRead(buttonPin) == LOW) {
      // Toggle carEntering state
      carEntering = !carEntering;
      // Wait for the button to be released
      while (digitalRead(buttonPin) == LOW);
    }
  }
}

double measureDistance() {
  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, HIGH); // Send a HIGH pulse from the transmitter
  delayMicroseconds(10); // For 10 microseconds
  digitalWrite(trigPin, LOW); // Stop the high pulse by sending a LOW one

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH); // Receive on the echo pin (corresponding to the receiver) a HIGH pulse and the travel time (from the transmitter to the object and back to the receiver)

  // Calculate the distance
  return duration * 0.034 / 2; // Using the formula distance = travel time * speed of sound / 2
}

void handleServoLogic(unsigned long currentMillis) {
  // Check if a car is entering and there are available spots, or if a car is leaving and there are spots to free up
  if ((carEntering && availableParkingSpots > 0) || (!carEntering && availableParkingSpots < totalParkingSpots)) {
    // If the servo is not already open
    if (!servoOpen) {
      // Move servo to maximum position (raise barrier)
      servo.write(180);
      previousMillis = currentMillis; // Record the current time for delay tracking
      servoOpen = true; // Set the state of the servo to open
      initialDelay = true; // After raising the barrier we activate the initial delay period to ensure minimum time for the car to pass
      successiveCarCount++; // Increase the number of consecutive cars that have passed
      lastCarTime = currentMillis; // Update the variable that keeps track of the last moment a car has passed the gate
      Serial.print("Successive Car Count: ");
      Serial.println(successiveCarCount);

      // Check if the successive car count has reached the threshold to trigger the extended open time
      if (successiveCarCount >= 3) {
        extendedOpenStartTime = currentMillis; // Record the start time of the extended open period
        Serial.println("Extended open time triggered.");
      }
    } else {
      // If the servo is already open during the extended open time
      if (successiveCarCount >= 3 && (currentMillis - extendedOpenStartTime < extendedOpenTime)) {
        checkAndUpdateParkingSpots(); // Check and update the number of available parking spots
      }
    }
  }
}

void handleExtendedServoLogic(unsigned long currentMillis) {
  // If the distance is greater than the threshold (no car in front of the gate)
  if (distance > thresholdDistance) {
    // Check if the conditions for the extended open servo state are met
    if (successiveCarCount >= 3 && (currentMillis - extendedOpenStartTime < extendedOpenTime)) {
      checkAndUpdateParkingSpots(); // Update parking spots if necessary
      previousMillis = currentMillis; // Reset the timer for the extended open period
      Serial.println("Keeping servo open during extended time.");
    } else {
      // Move servo back to minimum position (lower barrier)
      servo.write(0);
      servoOpen = false;
      initialDelay = false;

      // Reset successive car count if the extended time has passed
      if (successiveCarCount >= 3 && currentMillis - extendedOpenStartTime >= extendedOpenTime) {
        successiveCarCount = 0;
        Serial.println("Resetting successive car count after extended open time.");
      }

      // Update the number of available parking spots
      updateParkingSpots();
    }
  } else {
    // Update the number of available parking spots during the extended open time
    if (servoOpen && successiveCarCount >= 3 && (currentMillis - extendedOpenStartTime < extendedOpenTime)) {
      checkAndUpdateParkingSpots();
    }

    // Reset the timer if the car is still present
    previousMillis = currentMillis;
  }
}

void checkAndUpdateParkingSpots() {
  // Check for transition from a state in which a car was present to a state in which no car is present
  if (distance > thresholdDistance && lastDistance <= thresholdDistance) {
    updateParkingSpots();
    lastCarTime = millis(); // Record the time
    Serial.print("Distance during extended time: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  lastDistance = distance; // Update last distance variable with the current distance measurement
}

void updateParkingSpots() {
  // Only update when there is at least 1 available parking spot
  if (carEntering && availableParkingSpots > 0) {
    availableParkingSpots--;
  } 
  // Only update when the number of cars that want to leave the parking is below the total number of parking spaces
  else if (!carEntering && availableParkingSpots < totalParkingSpots) {
    availableParkingSpots++;
  }
}

void updateDisplay() {
  // Static variable to keep track of the current digit position being updated
  static int digitPosition = 1;

  // Calculate the values of each digit position
  int thousands = availableParkingSpots / 1000;              // Extract thousands place
  int hundreds = (availableParkingSpots % 1000) / 100;       // Extract hundreds place
  int tens = (availableParkingSpots % 100) / 10;             // Extract tens place
  int ones = availableParkingSpots % 10;                     // Extract ones place

  // Switch statement to update the display for the current digit position
  switch (digitPosition) {
    case 1:
      // If the thousands place is non-zero, display it; otherwise, clear the digit
      if (thousands != 0) displayDigit(1, thousands); else clearDigit(1);
      break;
    case 2:
      // If the hundreds place is non-zero, display it; if it is zero but thousands is non-zero, display zero; otherwise, clear the digit
      if (hundreds != 0 || (hundreds == 0 && thousands != 0)) displayDigit(2, hundreds); else clearDigit(2);
      break;
    case 3:
      // If the tens place is non-zero, display it; if it is zero but hundreds or thousands is non-zero, display zero; otherwise, clear the digit
      if (tens != 0 || (tens == 0 && hundreds != 0) || (tens == 0 && thousands != 0)) displayDigit(3, tens); else clearDigit(3);
      break;
    case 4:
      // Always display the ones place
      displayDigit(4, ones);
      break;
  }

  // Increment the digit position for the next update
  digitPosition++;
  // Reset the digit position to 1 if it exceeds 4
  if (digitPosition > 4) digitPosition = 1;
}

void displayDigit(int digitPosition, int number) {
  // Determine which digit position to activate based on the input parameter
  switch(digitPosition) {
    case 1: digit = B00001110; break; // Activate the first digit (thousands place)
    case 2: digit = B00001101; break; // Activate the second digit (hundreds place)
    case 3: digit = B00001011; break; // Activate the third digit (tens place)
    case 4: digit = B00000111; break; // Activate the fourth digit (ones place)
  }

  // Send the digit position and the number to be displayed to the shift register
  digitalWrite(latchPin, LOW);                // Begin the transmission
  shiftOut(dataPin, clockPin, MSBFIRST, digit);  // Shift out the digit position
  shiftOut(dataPin, clockPin, MSBFIRST, numbers[number]); // Shift out the number to be displayed
  digitalWrite(latchPin, HIGH);               // End the transmission
}

void clearDigit(int digitPosition) {
  // Determine which digit position to activate based on the input parameter
  switch(digitPosition) {
    case 1: digit = B00001110; break; // Select the first digit (thousands place)
    case 2: digit = B00001101; break; // Select the second digit (hundreds place)
    case 3: digit = B00001011; break; // Select the third digit (tens place)
    case 4: digit = B00000111; break; // Select the fourth digit (ones place)
  }

  // Clear the digit at the specified position
  digitalWrite(latchPin, LOW);                // Begin the transmission
  shiftOut(dataPin, clockPin, MSBFIRST, digit);  // Shift out the digit position
  shiftOut(dataPin, clockPin, MSBFIRST, 0);     // Shift out 0 to clear the digit
  digitalWrite(latchPin, HIGH);               // End the transmission
}

