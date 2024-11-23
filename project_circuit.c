// Motor Pins
#include <Servo.h>
#define BUTTON_PIN 8
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6
#define SCRUB_MOTOR_PIN1 3
#define SCRUB_MOTOR_PIN2 4
#define SERVO_PIN 11
#define MOTOR_SPEED 255
#define MOTOR_STOP 0
// Cleaning solution volume tank pins
#define CSV_LED_PIN 13
#define CSV_WARNING_LED_PIN 12
// Ultrasonic Sensor Pins
#define TRIGGER_PIN 10
#define ECHO_PIN 9
// Ultrasonic Sensor Tolerance Value
// sensor deviates by +-25 on average
// need tolerance to create stability in measuring the same area multiple times
#define US_TOLERANCE 25
// using integers, less precision but values more stable
// for logic based on comparison in loop
int distance;
int time;
int initial_distance;
// cleaning solution tank initial volume
int cleaning_solution_amount = 50;
// Servo to rotate counter cleaner
Servo servoMotor;

enum ServoPosition
{
    POSITION_0 = 0,
    POSITION_90 = 90,
    POSITION_180 = 180
};

int get_next_servo_position(int currentPosition)
{
    switch (currentPosition)
    {
    case POSITION_0:
        return POSITION_90;
    case POSITION_90:
        return POSITION_180;
    case POSITION_180:
        return POSITION_0;
    default:
        return POSITION_0;
    }
}
// set initial position of servo
static int servoPosition = POSITION_0;
// DRY principle here.. We measure the distance multiple times
// lets make a common function that returns a float.
int measure_distance()
{
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    // Read the echo time in microseconds
    time = pulseIn(ECHO_PIN, HIGH);
    // speed of sound is 340 m/s
    // which is 29 ms per cm
    distance = time / 29 / 2;
    Serial.print("Measured Distance: ");
    Serial.println(distance);
    return distance;
}

void run_right_scrubber()
{
    // SCRUB_MOTOR_PIN1 - spins cleaning disc right side
    Serial.println("Right Cleaning Disc Spinning");
    analogWrite(SCRUB_MOTOR_PIN1, MOTOR_SPEED);
    analogWrite(SCRUB_MOTOR_PIN2, MOTOR_STOP);
    delay(1000);
}

void run_left_scrubber()
{
    // SCRUB_MOTOR_PIN2 - spins cleaning disc left side
    Serial.println("Left Cleaning Disc Spinning");
    analogWrite(SCRUB_MOTOR_PIN2, MOTOR_SPEED);
    analogWrite(SCRUB_MOTOR_PIN1, MOTOR_STOP);
    delay(1000);
}

void stop_scrubbers()
{
    // Stop spinning cleaning discs both sides
    Serial.println("Cleaning Discs Stop Spinning");
    analogWrite(SCRUB_MOTOR_PIN2, MOTOR_STOP);
    analogWrite(SCRUB_MOTOR_PIN1, MOTOR_STOP);
}

void counter_cleaner_check_counter_distance()
{
    // Check distance at the beginning of every loop
    // added delays to stabilize measurements
    delay(500);
    distance = measure_distance();
    delay(500);
    Serial.print("Initial Distance: ");
    Serial.println(initial_distance);
}

void counter_cleaner_move_next_position()
{
    // Move forward at full speed for 500 ms
    Serial.println("Move to next position");
    analogWrite(MOTOR_PIN1, MOTOR_SPEED);
    analogWrite(MOTOR_PIN2, MOTOR_STOP);
    delay(500);
    // stop movement
    analogWrite(MOTOR_PIN1, MOTOR_STOP);
}

void counter_cleaner_find_next_position()
{
    // Rotate in 90 degree increments until finding
    // a safe direction to move
    Serial.println("EDGE OR OBJECT DETECTED");
    servoPosition = get_next_servo_position(servoPosition);
    servoMotor.write(servoPosition);
    distance = measure_distance();
}

void counter_cleaner_apply_cleaning_solution()
{
    // We don't have a pump or sprayer in tinkercad
    // plus voltage limitations etc..
    // simulating checking tank volume and applying solution
    // with LED lights
    if (cleaning_solution_amount > 0)
    {
        Serial.println("Apply Cleaning Solution");
        digitalWrite(CSV_LED_PIN, HIGH);
        delay(2000);
        digitalWrite(CSV_LED_PIN, LOW);
        // subtract volume of solution used from tank
        cleaning_solution_amount -= 10;
    }
    else
    {
        while (cleaning_solution_amount <= 0)
        {
            int button_position = digitalRead(BUTTON_PIN);
            if (button_position == HIGH)
            {
                digitalWrite(CSV_WARNING_LED_PIN, LOW);
                Serial.println("Refilling operation begins..");
                cleaning_solution_amount = 200;
            }
            Serial.println("NO CLEANING SOLUTION REMAINING!!!");
            digitalWrite(CSV_WARNING_LED_PIN, HIGH);
            delay(200);
            digitalWrite(CSV_WARNING_LED_PIN, LOW);
        }
    }

    Serial.print("Cleaning Solution Units: ");
    Serial.println(cleaning_solution_amount);
}

void counter_cleaner_run_cleaning_procedure()
{
    counter_cleaner_apply_cleaning_solution();
    run_right_scrubber();
    run_left_scrubber();
    stop_scrubbers();
}

void setup()
{
    // Button pin
    // simulates user refilling cleaning solution tank
    pinMode(BUTTON_PIN, INPUT);
    // Cleaning solution dispenser/tank volume pin setup
    // simulating an input/output with led lights
    pinMode(CSV_LED_PIN, OUTPUT);
    pinMode(CSV_WARNING_LED_PIN, OUTPUT);
    // Forward Movement Motor control pin setup
    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);
    // Rotational Servo Motor control
    servoMotor.attach(SERVO_PIN);
    // set initial position
    servoMotor.write(servoPosition);
    // Cleaning Scrubber Motor control pin setup
    pinMode(SCRUB_MOTOR_PIN1, OUTPUT);
    pinMode(SCRUB_MOTOR_PIN2, OUTPUT);
    // Ultrasonic sensor pin setup
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    // Initialize serial communication
    Serial.begin(9600);
    // Set initial distance to counter top
    initial_distance = measure_distance();
    Serial.print("Initial Distance: ");
    Serial.println(initial_distance);
}

void loop()
{
    if (Serial.available() > 0)
    {
        String command_from_swarm = Serial.readString();

        while (command_from_swarm == "Counter Cleaner ON!")
        {
            counter_cleaner_check_counter_distance();
            if ((distance == initial_distance) ||
                ((distance) == (initial_distance + US_TOLERANCE)) ||
                ((distance) == (initial_distance - US_TOLERANCE)))
            {
                String next_command = Serial.readString();
                if (next_command == "Counter Cleaner OFF!")
                {
                    command_from_swarm = "";
                }
                counter_cleaner_run_cleaning_procedure();
                counter_cleaner_move_next_position();
            }
            else
            {
                String next_command = Serial.readString();
                if (next_command == "Counter Cleaner OFF!")
                {
                    command_from_swarm = "";
                }
                counter_cleaner_find_next_position();
            }
        }
    }
    delay(500);
}
