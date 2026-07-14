const int motor_axisX_up = 9;    // Arduino digital pin for hbridge - IN4 -
const int motor_axisX_down = 6;  // Arduino digital pin for hbridge - IN3

const int motor_axisY_left = 5;   // Arduino digital pin for hbridge - IN4 -
const int motor_axisY_right = 3;  // Arduino digital pin for hbridge - IN3

const int solarPanelsensor_PanelsPin = A4;

//DEBUG VARIABLES////////
bool debug_verbose = true;
bool isLabConditions = true;
/////////////////////////


int run_time_DCmovement = 30;  //ms of movement for small action of the DC motors
int incomingValue = 0;         // for incoming serial data; used for debugging and controlling movement from IDE

int dutyCycleMin_Y = 35;                                      //Minimum motor speed to overcome friction
int dutyCycleMin_X = 28;                                      //Minimum motor speed for up down movements
int dutyCycleMax = 50;                                        //Max allowed motor speed
const int MIN_SPEED_Y = map(dutyCycleMin_Y, 0, 100, 0, 255);  // Minimum motor speed to overcome friction
const int MIN_SPEED_X = map(dutyCycleMin_X, 0, 100, 0, 255);  // Minimum motor speed to overcome friction
const int MAX_SPEED = map(dutyCycleMax, 0, 100, 0, 255);      // Maximum motor speed

///MOVING AVERAGE VARS////////////////////////////////////
const int numReadings = 8;  // Number of values to average
int readings[numReadings];  // Array to store the values
int index = 0;              // Index of the current reading
int total = 0;              // Running total of the values
int average = 0;            // The average
//////////////////////////////////////////////////////////

int lastMeasuredValue = 0;
int maxMeasuredValue = 0;

void setup() {
  Serial.begin(9600);

  pinMode(motor_axisX_up, OUTPUT);
  pinMode(motor_axisX_down, OUTPUT);
  pinMode(motor_axisY_left, OUTPUT);
  pinMode(motor_axisY_right, OUTPUT);

  pinMode(solarPanelsensor_PanelsPin, INPUT);

  //motors_check();//Up, Down, Left,Right check on startup

  for (int i = 0; i < numReadings; i++) {  //initialize to zero the vector
    readings[i] = 0;
  }
  delay_with_read(200);  //initializes with actual values the readings and stabilizes moving average

  //sweep360_forMaxValue(); Do a 360 sweep to find the max luminosity

  maxMeasuredValue = 455;  //since it was measured with the 360, to avoid wasting time at each bootup, this 
  //was set manually
  delay(5000);
  Serial.println(average);
}


void loop() {
  //if it sees any change in luminosity more than 7% max luminosity, 
  // reset max (it can only improve from here on)
  //then start re-calibrating position to maximize light
  float percentageOfMaxLuminosity = float(average) / float(maxMeasuredValue) * 100;
  if (percentageOfMaxLuminosity < 93) {
    maxMeasuredValue = average;
    move_axisY();
    //move_axisX();
  }
  delay_with_read(4000);//this would be set to a more realistic amount of time (like 1h) in
  //a real scenario
}


//used as a firs
void sweep360_forMaxValue() {
  //left 180°
  analogWrite(motor_axisY_left, MIN_SPEED_Y);
  delay_and_saveMax(4400);
  analogWrite(motor_axisY_left, 0);
  delay(400);  // pause for stabilization after stop to avoid jerking

  //right 360°
  analogWrite(motor_axisY_right, MIN_SPEED_Y);
  delay_and_saveMax(8800);
  analogWrite(motor_axisY_right, 0);
  delay(400);  // pause for stabilization after stop to avoid jerking

  //back to 0°
  analogWrite(motor_axisY_left, MIN_SPEED_Y);
  delay_with_read(4400);
  analogWrite(motor_axisY_left, 0);
  delay(400);  // pause for stabilization after stop to avoid jerking
}


void move_axisY() {
  //move right (following normal sun rotation with respect to ground)
  //this means rotating towards left/clockwise with respect of set
  //axis
  delay_with_read(200);
  analogWrite(motor_axisY_left, MIN_SPEED_Y);
  //wait and check path until wrong direction
  delay_and_checkPath();
  //stop the motor
  analogWrite(motor_axisY_left, 0);
  delay(100);  //brief pause to avoid too much jerking

  //abnormal case where sun moves "backwards". Basically for testing left/right movements
  if (isLabConditions == true) {
    delay_with_read(200);
    analogWrite(motor_axisY_right, MIN_SPEED_Y);
    //wait and check path until wrong direction
    delay_and_checkPath();
    //stop the motor
    analogWrite(motor_axisY_right, 0);
    delay(100);  //brief pause to avoid too much jerking

    //move back a bit to previous best position
    analogWrite(motor_axisY_left, MIN_SPEED_Y);
    delay(150);
    analogWrite(motor_axisY_left, 0);  //stop
  }
  delay_with_read(200);  //wait 50ms and sample more values for average

  if (debug_verbose == true) {  //if verbose true, also send it to serial
    Serial.print("Final value axis Y: ");
    Serial.println(average);
  }
  maxMeasuredValue=average;
}

void move_axisX() {
  //move up
  delay_with_read(200);
  analogWrite(motor_axisX_up, MIN_SPEED_X);
  //wait and check path until wrong direction
  delay_and_checkPath();
  //stop the motor
  analogWrite(motor_axisX_up, 0);
  delay_with_read(300);  //pause to avoid too much jerking and set average value

  analogWrite(motor_axisX_down, MIN_SPEED_X);
  //wait and check path until wrong direction
  delay_and_checkPath();
  //stop the motor
  analogWrite(motor_axisX_down, 0);
  delay(100);  //brief pause to avoid too much jerking

  //move back a bit to previous best position
  analogWrite(motor_axisX_up, MIN_SPEED_X);
  delay(250);
  analogWrite(motor_axisX_up, 0);  //stop

  delay_with_read(200);  //wait 50ms and sample more values for average

  if (debug_verbose == true) {  //if verbose true, also send it to serial
    Serial.print("Final value axis X: ");
    Serial.println(average);
  }
  maxMeasuredValue=average;//reset the max to check for any changes in light later
}


//used only for the sweep function. It keeps measuring the max value while turning the 
//flower around the Y axis, for an amount of time circa equivalent to the degrees indicated
//in the sweep function
void delay_and_saveMax(unsigned long timeMs) {
  unsigned long startTime = millis();  // Record the start time
  while (millis() - startTime < timeMs) {
    // Run the function repeatedly, send it to serial if verbose on
    int adc_avgRead = get_averageRead(solarPanelsensor_PanelsPin);
    if (adc_avgRead > maxMeasuredValue + 1) {
      maxMeasuredValue = adc_avgRead;
      if (debug_verbose == true) {
        Serial.print("New max value found: ");
        Serial.println(maxMeasuredValue);  //if verbose true, also send it to serial
      }                                    //this will stop when reached timeMs (this way it keeps updating the moving average)
    }
  }
}

//does a delay while continuesly reeding the ADC and updating the average value.
void delay_with_read(unsigned long timeMs) {
  unsigned long startTime = millis();  // Record the start time
  while (millis() - startTime < timeMs) {
    // Run the function repeatedly, send it to serial if verbose on
    int adc_avgRead = get_averageRead(solarPanelsensor_PanelsPin);  //run it to update the moving average
    if (debug_verbose == true) {
      Serial.print("Average ADC val: ");
      Serial.println(adc_avgRead);  //if verbose true, also send it to serial
    }                               //this will stop when reached timeMs (this way it keeps updating the moving average)
  }
}

//more of a "fake" delay, since it concentrates more on the changing of values
//than doing an actual delay. But it manages how long the motors are turned on
//in a direction, while continuesly calculating the moving average and checking
//for the termination condition
void delay_and_checkPath() {
  lastMeasuredValue = average;
  bool isRightPath = true;
  while (isRightPath) {
    // Run the function repeatedly, send it to serial if verbose on

    int adc_avgRead = get_averageRead(solarPanelsensor_PanelsPin);

    if (adc_avgRead < (lastMeasuredValue - 3)) {
      isRightPath = false;

      if (debug_verbose == true) {
        Serial.print("Wrong path: ");
        Serial.println(average);
      }

      //if luminosity is more or less stall or stable, keep going
      //or if luminosity is actually getting better then update the last measured value (and also the max if necessary)
    } else {

      if (debug_verbose == true) {
        Serial.print("Right path: ");
        Serial.println(adc_avgRead);
      }

      //set new threshold for relative position
      if (adc_avgRead >= lastMeasuredValue + 1) {
        lastMeasuredValue = adc_avgRead;  

        if (lastMeasuredValue > maxMeasuredValue + 1) {  //set new max
          maxMeasuredValue = lastMeasuredValue;
          if (debug_verbose == true) {
            Serial.print("New Max!: ");
            Serial.println(average);
          }
        }
      }
    }
  }
}


//rolling average implementation to smooth the values a bit and remove some noise.
//values then oscilate between +1 and -1 circa. 
//the delay_and_checkPath function uses this to verify if the value of the luminosity is descending
int get_averageRead(int pin) {
  float newReading = analogRead(pin);
  total = total - readings[index];// Subtract the oldest reading from the total
  readings[index] = newReading;// Add the new reading to the array
  total = total + readings[index];// Add the new reading to the total
  index = (index + 1) % numReadings;// Move to the next position in the array
  average = float(total) / numReadings;// Calculate the average
  return round(average);
}

//simple motors check to debug movement.
void motors_check() {
  int run_time = 250;  //ms of movement

  //Up
  analogWrite(motor_axisX_up, MIN_SPEED_X);
  delay(run_time);
  analogWrite(motor_axisX_up, 0);  //STOP
  delay(1000);

  //Down
  analogWrite(motor_axisX_down, MIN_SPEED_X);
  delay(run_time);
  analogWrite(motor_axisX_down, 0);  //STOP
  delay(1000);

  // Move left
  analogWrite(motor_axisY_left, MIN_SPEED_Y);
  delay(run_time);
  analogWrite(motor_axisY_left, 0);  //STOP
  delay(1000);

  // Move right
  analogWrite(motor_axisY_right, MIN_SPEED_Y);
  delay(run_time);
  analogWrite(motor_axisY_right, 0);  //STOP
  delay(100);
}

//simple function used initially to verify the values measured by the ADC
void serialPlot_solarPanelSignal() {
  if (debug_verbose != true) {  //use the serial only if the debug verbos is not enabled
    Serial.print("Tension val: ");
    Serial.println(get_averageRead(solarPanelsensor_PanelsPin));
  }
}