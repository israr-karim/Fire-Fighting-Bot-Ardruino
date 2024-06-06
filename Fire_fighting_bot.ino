#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

Servo myservo;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // Initialize MLX90614 object

const int fireSensor2Pin = A1;
const int relayPin = 8;    // Define the pin connected to the relay
const int MotorA_in1 = 2; // Define the pins for Motor A
const int MotorA_in2 = 3;
const int MotorB_in3 = 4; // Define the pins for Motor B
const int MotorB_in4 = 5;

bool motorMoved = false; // Flag to track if the motor has moved

void setup() {
  myservo.attach(A3);
  myservo.write(90); // Set servo to 90-degree position
  Serial.begin(9600);
  
  pinMode(fireSensor2Pin, INPUT);
  pinMode(relayPin, OUTPUT);     // Set relay pin as output
  digitalWrite(relayPin, HIGH);
  pinMode(MotorA_in1, OUTPUT);   // Set motor pins as output
  pinMode(MotorA_in2, OUTPUT);
  pinMode(MotorB_in3, OUTPUT);
  pinMode(MotorB_in4, OUTPUT);

  mlx.begin();  // Initialize MLX90614 sensor
}

void loop() {
  bool fire2Value = digitalRead(fireSensor2Pin) == LOW;

  if (fire2Value) {
    Serial.println("Fire detected!");
    
    // Move motor forward only the first time it detects flame
    if (!motorMoved) {
      Serial.println("Moving motor forward.");
      digitalWrite(MotorA_in1, HIGH);
      digitalWrite(MotorA_in2, LOW);
      digitalWrite(MotorB_in3, HIGH);
      digitalWrite(MotorB_in4, LOW);
      delay(200); // Move motor forward for half a second
      digitalWrite(MotorA_in1, LOW);
      digitalWrite(MotorA_in2, LOW);
      digitalWrite(MotorB_in3, LOW);
      digitalWrite(MotorB_in4, LOW);
      
      motorMoved = true; // Set flag to true
    }

    // Activate servo gradually
    Serial.println("Activating servo rotation.");
    for (int pos = 160; pos >= 0; pos -= 1) {
      myservo.write(pos);
      delay(15);
    }
    
    // Activate pump simultaneously with servo
    Serial.println("Activating pump.");
    digitalWrite(relayPin, LOW);
    
    // Check fire sensor value again
    fire2Value = digitalRead(fireSensor2Pin) == LOW;

    // If fire is still detected, run the loop again
    while (fire2Value) {
      // Activate servo and pump again
      Serial.println("Activating servo rotation and pump again.");
      for (int pos = 160; pos >= 0; pos -= 1) {
        myservo.write(pos);
        delay(15);
      }
      digitalWrite(relayPin, LOW);

      // Check fire sensor value again
      fire2Value = digitalRead(fireSensor2Pin) == LOW;
    }

    // Turn off servo
    Serial.println("Turning off servo.");
    myservo.write(90);

    // Turn off pump
    Serial.println("Turning off pump.");
    digitalWrite(relayPin, HIGH);

    Serial.println("No fire detected. Turning off servo and pump.");
  }

  // Print temperature from MLX90614 sensor
  float objTempC = mlx.readObjectTempC(); // Read object temperature in Celsius
  Serial.print("Object Temperature (C): ");
  Serial.println(objTempC);

  // Delay before next iteration
  delay(2000);
}
