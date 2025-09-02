#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
// Create instances for MPU6050 and ADXL345
Adafruit_MPU6050 mpu;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


int AU = 0;

const int MQ3_PIN = A0;        // MQ3 sensor connected to analog pin A0
const int BUZZER_PIN = 5;      
int ALCOHOL_THRESHOLD = 235;  // 160  230 

const int MQ6_PIN = A1;         // MQ6 sensor connected to analog pin A2
int CNG_THRESHOLD = 255;  // Define this threshold based on calibration

//const int TRIG_PIN = 9;           // Ultrasonic sensor trigger connected to D0
//const int ECHO_PIN = 10;           // Ultrasonic sensor echo connected to D1
//const int OBSTACLE_THRESHOLD = 5; // Distance threshold in centimeters

const int IR_SENSOR_PIN = 4; 

// Threshold for sudden acceleration change (m/s^2)
#define ACCIDENT_THRESHOLD 15.0
// Previous acceleration values
float prevAccelX = 0;
float prevAccelY = 0;
float prevAccelZ = 0;


#define VIBRATION_PIN A2  // Define the analog pin connected to the vibration sensor
#define THRESHOLD 1000   // 230 Define a vibration threshold (adjust based on your sensor)

const int tempsensorPin = A3; // Pin connected to the output of LM35
float temperature;

void buzz(){

          digitalWrite(BUZZER_PIN, HIGH);         // Turn on the buzzer
            delay(500);
        digitalWrite(BUZZER_PIN, LOW);          // Turn off the buzzer//
  
  
  }




void setup() {
  Serial.begin(9600);           // Initialize serial communication for monitoring
  pinMode(MQ3_PIN, INPUT);      // Set MQ3 pin as input
  
  pinMode(MQ6_PIN, INPUT);       // Set MQ6 pin as input

//  pinMode(TRIG_PIN, OUTPUT);       // Set trigger pin as output
//  pinMode(ECHO_PIN, INPUT);        // Set echo pin as input

  pinMode(IR_SENSOR_PIN, INPUT); // Set IR sensor pin as input

  pinMode(VIBRATION_PIN, INPUT); // Set the vibration pin as input



  // Initialize I2C ACCELERATION AND GYRO
  Wire.begin();  // Starts the I2C bus
  // Initialize MPU6050 gyroscope
  if (!mpu.begin()) {
//    Serial.println("Could not find MPU6050 sensor!");    
    while (1);
  }
//  Serial.println("MPU6050 initialized.");
  // Initialize ADXL345 accelerometer
  if (!accel.begin()) {
//    Serial.println("Could not find ADXL345 sensor!");
    while (1);
  }
  
  accel.setRange(ADXL345_RANGE_16_G); // Set range for higher sensitivity

//  Serial.println("ADXL345 initialized.");

  pinMode(BUZZER_PIN, OUTPUT);  // Set buzzer pin as output
  
}








//
//int i = 0;


void loop() {

//
//    
  int alcoholLevel = analogRead(MQ3_PIN);  // Read the analog value from the MQ3 sensor
  int cngLevel = analogRead(MQ6_PIN);  // Read the analog value from the MQ6 sensor

//
//if(i == 7){
//
//  ALCOHOL_THRESHOLD = alcoholLevel + 10;
//  CNG_THRESHOLD =  cngLevel + 20;
//  
//  }
//
//
//
//i = i + 1;



//Serial.print("Alchol threshold: ");
//Serial.print(ALCOHOL_THRESHOLD);
//Serial.print(" Alcholol value: ");
//Serial.println(alcoholLevel);

//  long duration, distance;
//  // Trigger the ultrasonic sensor
//  digitalWrite(TRIG_PIN, LOW);
//  delayMicroseconds(2);
//  digitalWrite(TRIG_PIN, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG_PIN, LOW);
//  // Read the duration of the echo pulse
//  duration = pulseIn(ECHO_PIN, HIGH);
//  // Calculate distance in centimeters
//  distance = duration * 0.034 / 2;

  int irState = digitalRead(IR_SENSOR_PIN);  // Read the state from the IR sensor

  int vibrationValue = analogRead(VIBRATION_PIN);

  // Read data from MPU6050 (Gyroscope)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Calculate tilt angle using accelerometer data
  float GaccelX = a.acceleration.x;
  float GaccelY = a.acceleration.y;
  float GaccelZ = a.acceleration.z;
  // Calculate tilt angle in degrees (based on atan2)
  float tiltAngle = atan2(sqrt(GaccelX * GaccelX + GaccelY * GaccelY), GaccelZ) * 180.0 / PI;


  
   // Read data from ADXL345 (Accelerometer)
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);
  
  // Current acceleration readings
  float accelX = accelEvent.acceleration.x;
  float accelY = accelEvent.acceleration.y;
  float accelZ = accelEvent.acceleration.z;
  // Calculate change in acceleration
  float deltaAccelX = abs(accelX - prevAccelX);
  float deltaAccelY = abs(accelY - prevAccelY);
  float deltaAccelZ = abs(accelZ - prevAccelZ);
  // Combine changes to determine overall sudden change
  float totalDeltaAccel = sqrt(deltaAccelX * deltaAccelX + deltaAccelY * deltaAccelY + deltaAccelZ * deltaAccelZ);




  

  int tempsensorValue = analogRead(tempsensorPin); // Read the sensor
  temperature = tempsensorValue * (5.0 / 1023.0) * 100; // Convert to Celsius



//  
////
//  Serial.println("");
//  Serial.println("");
//  Serial.print("Alcohol Level: ");
//  Serial.print(alcoholLevel); // Print the value to Serial Monitor
//  Serial.print("  ALCOHOL_THRESHOLD  ");
//  Serial.print(ALCOHOL_THRESHOLD);
//  Serial.println("");
//  Serial.print("CNG Level: ");
//  Serial.print(cngLevel); // Print the value to Serial Monitor
//  Serial.print("  CNG_THRESHOLD  ");
//  Serial.print(CNG_THRESHOLD);
//  Serial.println("");
////  Serial.print("Distance: ");
////  Serial.print(distance);
//  Serial.println("");

//  // Print the vibration value to the Serial Monitor
//  Serial.print("Vibration Value: ");
// Serial.println(vibrationValue);
//  
//  // Print values for debugging
//  Serial.print("Gyro Sendor X: "); Serial.print(accelX);
//  Serial.print("\t Y: "); Serial.print(accelY);
//  Serial.print("\t Z: "); Serial.print(accelZ);
//  Serial.print("\t Tilt Angle: "); Serial.println(tiltAngle);
//  
//  // Debugging output
//  Serial.print("Accel X: "); Serial.print(accelX);
//  Serial.print("\t Y: "); Serial.print(accelY);
//  Serial.print("\t Z: "); Serial.print(accelZ);
//  Serial.print("\t Total Delta Accel: "); Serial.println(totalDeltaAccel);
//
// Serial.print("Temperature: ");
//  Serial.print(temperature);
//  Serial.println(" °C");
//  








  

  // Check if alcohol level is above the threshold
  if (alcoholLevel > ALCOHOL_THRESHOLD) {
//    Serial.println("Alcohol Detected!");
      buzz();
    AU = 1;
    //message to relatives about alchol level
    
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // Turn off the buzzer
 }


  // Check if CNG level is above the threshold
  if (cngLevel > CNG_THRESHOLD) {
//    Serial.println("CNG Detected!");
      AU = 1;
      buzz();
  
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // Turn off the buzzer
  }






//      if (distance  < 10) {
//        
//       // Blink the LED
//       AU = 2; // breaak applied
//                  // Wait for 500 milliseconds
//      } else if(distance < 20) {
//
//        AU = 1; //speed slow
//
//      }














  // Check if an obstacle is detected
  if (irState == LOW) {  // Assuming LOW indicates an obstacle (depends on sensor type)
//    Serial.println("  Obstacle Detected!");
    AU = 1;
      buzz();
  
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // Turn off the buzzer if no obstacle
  }


    if (tiltAngle < 143 || tiltAngle > 170) {
        AU = 1;
        //messafe to relatives emergency services
      
      buzz();

    } else {
//        Serial.println("No Tilt detected.");
    }



  // Check if sudden change exceeds threshold
  if (totalDeltaAccel > ACCIDENT_THRESHOLD) {

    //mssg to relatives and emergency services
    AU = 1;    
      buzz();

  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
  }
  // Update previous acceleration values
  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;



//  if (vibrationValue > THRESHOLD) {
//        AU = 1;
//        //messafe to relatives
//      
//      buzz();
//  }



      if (temperature > 100) {
       // Blink the LED
       AU = 1;
       //message to relatives and emergency services        
      buzz();
      } else {
        digitalWrite(BUZZER_PIN, LOW);  // Ensure the LED is off if temp <= 20
      }

//||  vibrationValue > THRESHOLD

if(temperature > 50     ||  totalDeltaAccel > ACCIDENT_THRESHOLD  || (tiltAngle < 143 || tiltAngle > 170)  || irState == LOW ||  cngLevel > CNG_THRESHOLD ||
alcoholLevel > ALCOHOL_THRESHOLD){
  AU = 1;
  }else {
    AU = 0;
 }

Serial.println(AU); 
//Serial.println(temperature); 

  delay(100); // Delay for half a second before taking the next reading
}
