// Define CPU frequency
#define F_CPU 80000000L // ESP8266 runs at 80MHz

// Base address for GPIO
#define GPIO_BASE 0x60000300

// GPIO register addresses
#define GPIO_OUT (*(volatile uint32_t *)(GPIO_BASE + 0x00))
#define GPIO_OUT_W1TS (*(volatile uint32_t *)(GPIO_BASE + 0x04))
#define GPIO_OUT_W1TC (*(volatile uint32_t *)(GPIO_BASE + 0x08))
#define GPIO_ENABLE (*(volatile uint32_t *)(GPIO_BASE + 0x0C))
#define GPIO_ENABLE_W1TS (*(volatile uint32_t *)(GPIO_BASE + 0x10))
#define GPIO_ENABLE_W1TC (*(volatile uint32_t *)(GPIO_BASE + 0x14))
#define GPIO_IN (*(volatile uint32_t *)(GPIO_BASE + 0x18))

// Timer register addresses
#define TIMER_BASE 0x60000600
#define FRC1_LOAD (*(volatile uint32_t *)(TIMER_BASE + 0x0))
#define FRC1_COUNT (*(volatile uint32_t *)(TIMER_BASE + 0x4))
#define FRC1_CTRL (*(volatile uint32_t *)(TIMER_BASE + 0x8))
#define FRC1_INT (*(volatile uint32_t *)(TIMER_BASE + 0xC))

// Pin numbers
#define ECHO_PIN 14    // GPIO14 (D5 on NodeMCU)
#define TRIG_PIN 0     // GPIO0  (D3 on NodeMCU)
#define BUTTON_PIN 4   // GPIO4  (D2 on NodeMCU)
#define SERVO_PIN 5    // GPIO5  (D1 on NodeMCU)
#define DATA_PIN 15    // GPIO15 (D8 on NodeMCU)
#define LATCH_PIN 13   // GPIO13 (D7 on NodeMCU)
#define CLOCK_PIN 12   // GPIO12 (D6 on NodeMCU)

// Constants for timing and distance calculation
#define US_TO_CM 0.034 / 2

// Variables for distance calculation
volatile long duration;
volatile double distance;
volatile double lastDistance;

// Threshold distance in cm
const double thresholdDistance = 6.0;

// Parking space variables
int totalParkingSpots = 6; // Set initial number of parking spots
int availableParkingSpots = totalParkingSpots;
bool carEntering = true; // State to check if a car is entering or leaving

// 7-segment display numbers
byte numbers[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};
byte digit;

// Variables for non-blocking delay
unsigned long previousMillis = 0;
const long interval = 2000; // 2 seconds

// Servo state
bool servoOpen = false;
bool initialDelay = false; // Delay of 2 seconds after each car passes to prevent accidental gate closure

// Successive car tracking variables
int successiveCarCount = 0; // Keeps track of how many cars have gone through consecutively
unsigned long lastCarTime = 0;
const long successiveInterval = 5000; // 5 seconds
const long extendedOpenTime = 40000; // 40 seconds
unsigned long extendedOpenStartTime = 0;

// Define pulse width constants for servo
#define MIN_PULSE_WIDTH 544  // Minimum pulse width in microseconds
#define MAX_PULSE_WIDTH 2400 // Maximum pulse width in microseconds

// Timer functions
void initTimer() {
    // Disable timer
    FRC1_CTRL = 0;

    // Set the prescaler to 16 (80 MHz / 16 = 5 MHz)
    FRC1_CTRL = (1 << 6) | (1 << 2);

    // Set the load value to the maximum (23-bit counter)
    FRC1_LOAD = 0x7FFFFF;

    // Enable the timer
    FRC1_CTRL = (1 << 7) | (1 << 6) | (1 << 2);
}

unsigned long getMicros() {
    // Read the current timer count and convert to microseconds
    return (0x7FFFFF - FRC1_COUNT) / 5;
}

void customDelayMicroseconds(unsigned int us) {
    unsigned long start = getMicros();
    while ((getMicros() - start) < us);
}

void delay(unsigned long ms) {
    while (ms--) {
        customDelayMicroseconds(1000);
    }
}

unsigned long getMillis() {
    return getMicros() / 1000;
}

// Function to set servo position
void setServoPosition(int degrees) {
    // Ensure degrees are within the valid range (0 to 180)
    if (degrees < 0) {
        degrees = 0;
    } else if (degrees > 180) {
        degrees = 180;
    }

    // Map the degrees to the pulse width (1000 to 2000 microseconds)
    // Calculate pulse width using linear interpolation
    uint16_t pulse_width = MIN_PULSE_WIDTH + (uint16_t)((uint32_t)degrees * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180);

    // Convert pulse width to ticks (16-bit value)
    uint16_t ticks = pulse_width * 5; // Since the timer operates at 5 MHz (80 MHz / 16 prescaler)

    // Set the pulse width
    FRC1_LOAD = ticks;
}

void setup() {
    // Initialize timer
    initTimer();

    // Set pin modes (enable as output or input)
    GPIO_ENABLE_W1TS |= (1 << TRIG_PIN) | (1 << SERVO_PIN) | (1 << DATA_PIN) | (1 << LATCH_PIN) | (1 << CLOCK_PIN);
    GPIO_ENABLE_W1TC |= (1 << ECHO_PIN) | (1 << BUTTON_PIN);

    // Initialize servo to minimum position
    setServoPosition(0);

    // Initialize lastDistance
    lastDistance = measureDistance();
}

void loop() {
    // Read the button state
    readButtonState();

    unsigned long currentMillis = getMillis();

    // Check if the initial delay period is not active
    if (!initialDelay) {
        distance = measureDistance(); // Get the distance

        // Check if the distance is less than the threshold (car is in front of the gate)
        if (distance <= thresholdDistance) {
            handleServoLogic(currentMillis);
        }
    } else {
        // Check if the initial delay has passed
        if (currentMillis - previousMillis >= interval) {
            distance = measureDistance();

            handleExtendedServoLogic(currentMillis);
        }
    }

    // Check if the interval between successive cars has exceeded the limit and that the extended servo open time has passed
    if ((successiveCarCount > 0) && (currentMillis - lastCarTime > successiveInterval) && !(currentMillis - extendedOpenStartTime < extendedOpenTime)) {
        successiveCarCount = 0;
    }

    // Refresh the display continuously
    updateDisplay();

    // Short delay before the next loop iteration
    delay(1);
}

void readButtonState() {
    if (!(GPIO_IN & (1 << BUTTON_PIN))) {
        // Debounce the button
        delay(50);
        if (!(GPIO_IN & (1 << BUTTON_PIN))) {
            carEntering = !carEntering;
            while (!(GPIO_IN & (1 << BUTTON_PIN)));
        }
    }
}

double measureDistance() {
    // Clear the trigPin by setting it LOW
    GPIO_OUT_W1TC |= 1 << TRIG_PIN;
    customDelayMicroseconds(2);

    // Trigger the ultrasonic sensor
    GPIO_OUT_W1TS |= 1 << TRIG_PIN;
    customDelayMicroseconds(10);
    GPIO_OUT_W1TC |= 1 << TRIG_PIN;

    // Read the echoPin, returns the sound wave travel time in microseconds
    duration = measurePulse(ECHO_PIN);

    // Calculate the distance
    return duration * US_TO_CM;
}

long measurePulse(int pin) {
    unsigned long start = getMicros();
    while (!(GPIO_IN & (1 << pin))) {
        if (getMicros() - start > 1000000) return 0; // Timeout after 1 second
    }
    start = getMicros();
    while (GPIO_IN & (1 << pin)) {
        if (getMicros() - start > 1000000) return 0; // Timeout after 1 second
    }
    return getMicros() - start;
}

void handleServoLogic(unsigned long currentMillis) {
    // Check if a car is entering and there are available spots, or if a car is leaving and there are spots to free up
    if ((carEntering && availableParkingSpots > 0) || (!carEntering && availableParkingSpots < totalParkingSpots)) {
        // If the servo is not already open
        if (!servoOpen) {
            // Move servo to maximum position (raise barrier)
            setServoPosition(180);
            previousMillis = currentMillis; // Record the current time for delay tracking
            servoOpen = true; // Set the state of the servo to open
            initialDelay = true; // After raising the barrier we activate the initial delay period to ensure minimum time for the car to pass
            successiveCarCount++; // Increase the number of consecutive cars that have passed
            lastCarTime = currentMillis; // Update the variable that keeps track of the last moment a car has passed the gate

            // Check if the successive car count has reached the threshold to trigger the extended open time
            if (successiveCarCount >= 3) {
                extendedOpenStartTime = currentMillis; // Record the start time of the extended open period
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
        } else {
            // Move servo back to minimum position (lower barrier)
            setServoPosition(0);
            servoOpen = false;
            initialDelay = false;

            // Reset successive car count if the extended time has passed
            if (successiveCarCount >= 3 && currentMillis - extendedOpenStartTime >= extendedOpenTime) {
                successiveCarCount = 0;
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
        lastCarTime = getMillis(); // Record the time
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
        case 1: digit = 0x0E; break;
        case 2: digit = 0x0D; break;
        case 3: digit = 0x0B; break;
        case 4: digit = 0x07; break;
    }

    // Send the digit position and the number to be displayed to the shift register
    GPIO_OUT_W1TC |= 1 << LATCH_PIN;                // Begin the transmission
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, digit);  // Shift out the digit position
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, numbers[number]); // Shift out the number to be displayed
    GPIO_OUT_W1TS |= 1 << LATCH_PIN;               // End the transmission
}

void clearDigit(int digitPosition) {
    // Determine which digit position to activate based on the input parameter
    switch(digitPosition) {
        case 1: digit = 0x0E; break;
        case 2: digit = 0x0D; break;
        case 3: digit = 0x0B; break;
        case 4: digit = 0x07; break;
    }

    // Clear the digit at the specified position
    GPIO_OUT_W1TC |= 1 << LATCH_PIN;                // Begin the transmission
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, digit);  // Shift out the digit position
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);     // Shift out 0 to clear the digit
    GPIO_OUT_W1TS |= 1 << LATCH_PIN;               // End the transmission
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
    for (uint8_t i = 0; i < 8; i++) {
        if (bitOrder == LSBFIRST) {
            if (val & 1) {
                GPIO_OUT_W1TS |= 1 << dataPin;
            } else {
                GPIO_OUT_W1TC |= 1 << dataPin;
            }
            val >>= 1;
        } else {
            if (val & 0x80) {
                GPIO_OUT_W1TS |= 1 << dataPin;
            } else {
                GPIO_OUT_W1TC |= 1 << dataPin;
            }
            val <<= 1;
        }
        GPIO_OUT_W1TS |= 1 << clockPin;
        GPIO_OUT_W1TC |= 1 << clockPin;
    }
}

