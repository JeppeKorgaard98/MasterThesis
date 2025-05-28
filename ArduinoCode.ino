#include <AccelStepper.h>
#include <Statistic.h> // Statistics
#include <RunningMedian.h> // Statistics

#include <LiquidCrystal.h>

#include <Keyboard.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>


// Pins
const int stepPin = 7;
const int dirPin = 4;
const int encoderCLK = 10;
const int encoderDT = 11;
const int encoderButton = 12;
const int runButton = 13;
const int enablePin = 2;


#define BUTTON_PIN 9
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
statistic::Statistic<float, uint32_t, true> myStats;

// Statistics stuff (Barometer etc)
double CurrentMax = 0;
double CurrentReading = 0;
int counter = 0;
int initialRead = -1;
RunningMedian samples = RunningMedian(10);
int delayVal = 10;


const int maxSpeed = 400;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// Encoder vars
int lastCLK = HIGH;
int encoderPos = 25; // Initial speed is (4*encoderPos) 
int lastSpeed = 0; 
bool motorDirection = true;

// Debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;


void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}




void setup() {
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  pinMode(encoderButton, INPUT_PULLUP);
  pinMode(runButton, INPUT_PULLUP);
  
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(1000);

  
   lastSpeed = map(encoderPos, 0, 100, 0, maxSpeed);
   stepper.setSpeed(motorDirection ? lastSpeed : -lastSpeed);

  Serial.begin(9600);
  pinMode(enablePin, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(enablePin, HIGH);
  initialRead = digitalRead(BUTTON_PIN);

  bool buttonPress = false;
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();




// Set 1/8 microstepping
}

void loop() {

  int input = digitalRead(BUTTON_PIN);
  if (digitalRead(BUTTON_PIN) != initialRead){
    initialRead = !initialRead;
    getSensorReadings();
    }

  
  // Read rotary encoder
  int currentCLK = digitalRead(encoderCLK);
  if (currentCLK != lastCLK && currentCLK == LOW) {
    if (digitalRead(encoderDT) != currentCLK) {
      encoderPos-=2;  // Clockwise
    } else {
      encoderPos+=2;  // Counterclockwise
    }

    // Optional: Clamp speed range
    if (encoderPos < 2) encoderPos = 2;
    if (encoderPos > 100) encoderPos = 100;

    lastSpeed = map(encoderPos, 0, 100, 0, maxSpeed);
    stepper.setSpeed(motorDirection ? lastSpeed : -lastSpeed);

    Serial.print("Speed: ");
    Serial.println(lastSpeed);
  }
  lastCLK = currentCLK;

  static bool lastEncoderButton = HIGH;
  bool currentEncoderButton = digitalRead(encoderButton);
  if (lastEncoderButton == HIGH && currentEncoderButton == LOW && (millis() - lastDebounceTime > debounceDelay)) {
    lastDebounceTime = millis();
    motorDirection = !motorDirection;
    stepper.setSpeed(motorDirection ? lastSpeed : -lastSpeed);
    Serial.print("Direction toggled: ");
  }
  lastEncoderButton = currentEncoderButton;

  // Run the motor ONLY if runButton is held
  if (digitalRead(runButton) == LOW) {
    digitalWrite(enablePin, LOW);
    stepper.runSpeed();  // Non-blocking
  }
  else{
    digitalWrite(enablePin, HIGH);
    }
}

    void getSensorReadings(){
      double readings[10];
      sensors_event_t event;


      myStats.clear();
      samples.clear();
      for (int i=0; i< 10; i++){
        bmp.getEvent(&event);
        if (event.pressure)
        {
        /* Display atmospheric pressue in Pa */
        CurrentReading = (event.pressure - 1035.8) * -100;
        myStats.add(CurrentReading);
        samples.add(CurrentReading);
        readings[i] = CurrentReading;
        delay(100);
        }
      
     
      }
      
        Serial.print("Average");
        Serial.println(myStats.average());

      
}
