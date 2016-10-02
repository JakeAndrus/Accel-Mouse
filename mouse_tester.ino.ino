#include <Wire.h>
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <math.h>
#include <Mouse.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
            
//assign pins
const int pushbutton = 11;
const int vibration1 = 10;
const int blue       = 5;
const int green      = 17;
const int vibration2 = 8;
const int calButton  = 13;
const int red        = 14;

const int mouseDelay = 12;

const int yAxis      = 1;
const int xAxis      = 0;

const int filterSamples = 9;

float smoothArrayAccelX[filterSamples];
float smoothArrayAccelY[filterSamples];
float smoothArrayAccelZ[filterSamples];
float smoothArrayGyroX[filterSamples];
float smoothArrayGyroY[filterSamples];

float rawAccelX,    rawAccelY,    rawAccelZ,            smoothAccelX,       smoothAccelY,   smoothAccelZ;
float rawGyroX,     rawGyroY,     smoothGyroX,          smoothGyroY;
float xAngle,       yAngle; 
float xLinearAccel, yLinearAccel, xSmoothLinearAccel,   ySmoothLinearAccel; 
float xVelocity,    yVelocity,    xSmoothVelocity,      ySmoothVelocity;

int x_count = 0, y_count = 0, x_sign_count = 0, y_sign_count = 0;
int x_initial_sign, y_initial_sign, x_sign, y_sign;

unsigned long previous_time_x   = 0;
unsigned long previous_time_y   = 0;
unsigned long previous_time_xx  = 0;
unsigned long previous_time_yy  = 0;

float xGyroOffset = 0;         
float yGyroOffset = 0;         
float xAccelOffset = 0;        
float yAccelOffset = 0;        
float zAccelOffset = 0;

float g                           = 9.8;
float current_val                 = 0.0;
float current_ang                 = 0.0;

bool printVelocityX               = 1;
bool printVelocityY               = 0;
bool printAngleX                  = 1;
bool printAngleY                  = 0;
bool printGravity                 = 0;
bool printCalibrateProcess        = 0;
bool printCalibrateTotal          = 0;
bool printMouseMoveX              = 0;
bool printMouseMoveY              = 0;
bool printArrayData               = 1;

bool toggle                       = 0;
bool axis                         = 0;

float smoothArrays[5][filterSamples] = {
  {smoothArrayAccelX[filterSamples]},
  {smoothArrayAccelY[filterSamples]},
  {smoothArrayAccelZ[filterSamples]},
  {smoothArrayGyroX[filterSamples]},
  {smoothArrayGyroY[filterSamples]}
};

float rawDataArray[5]{
  rawAccelX,
  rawAccelY,
  rawAccelZ,
  rawGyroX,
  rawGyroY
};

float smoothDataArray[5]{
  smoothAccelX,
  smoothAccelY,
  smoothAccelZ,
  smoothGyroX,
  smoothGyroY
};

float dataOffsetArray[5]{
  xAccelOffset,
  yAccelOffset,
  zAccelOffset,
  xGyroOffset,
  yGyroOffset
};

int signAndCountArray[4][2]{
  {x_count, y_count},
  {x_initial_sign, y_initial_sign},
  {x_sign, y_sign}
};

void calibrate();

void setup(void) { 
  Serial.begin(9600);
  Serial.println("test");
  
  //pin setups
  pinMode(pushbutton, INPUT);
  pinMode(vibration1, INPUT);
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(calButton, INPUT);
  pinMode(green, OUTPUT);

  if(!accel.begin())
    {
      /* There was a problem detecting the ADXL345 ... check your connections */
      Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
      while(1);
    }
    if(!mag.begin())
    {
      /* There was a problem detecting the LSM303 ... check your connections */
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      while(1);
    }
    if(!gyro.begin())
    {
      /* There was a problem detecting the L3GD20 ... check your connections */
      Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    Mouse.begin();

    calibrate();
}

void mouseMove(float y_velocity, float x_velocity);                                                                                                       //take velocity and move the mouse
float digitalSmooth(float rawData, float *senseSmoothArray);                                                                                              //smooth sensor data
float Angle(int axis, float smoothGyro, float primarySmoothAccel, float otherSmoothAccel, float smoothAccelz, float previous_angle);    //take gyro data and accelerometer data and find current angle
float linearAccel(float current_ang, float current_val);                                                                                                  //factor out acceleration due to gravity given current angle
float Velocity(int axis, float linearAccel, float previous_velocity);                                                                                     //integrate linear acceleration to find velocity
bool mouseOn();                                                                                                                                           //checks to see whether button has been toggled on or off to turn on mouse

void loop() 
{
  
  
  if(digitalRead(calButton) == HIGH)
  {
    calibrate();
  }
  digitalWrite(blue, toggle);
  sensors_event_t event;
     
  accel.getEvent(&event);
  rawDataArray[0] = event.acceleration.x;
  rawDataArray[1] = event.acceleration.y;
  rawDataArray[2] = event.acceleration.z;
  
  gyro.getEvent(&event);
  rawDataArray[3] = event.gyro.x;
  rawDataArray[4] = event.gyro.y;

    //smooth all sensor data
  for(int i = 0; i < 5; i++){
    smoothDataArray[i] = (digitalSmooth(rawDataArray[i], smoothArrays[i]))- dataOffsetArray[i];
    if(printArrayData){
      Serial.print("smoothDataArray=");  Serial.print(smoothDataArray[i]);             Serial.print("\t");
      Serial.print("rawDataArray=");     Serial.print(rawDataArray[i]);                Serial.print("\t");
      Serial.print("smoothArrays=");     Serial.print(smoothArrays[i][5]); Serial.print("\t");
    };
  };

    //find other movement values
  xAngle          = Angle(0,  smoothDataArray[3], smoothDataArray[1], smoothDataArray[0], smoothDataArray[2], xAngle);
  yAngle          = Angle(1, -smoothDataArray[4], smoothDataArray[0], smoothDataArray[1], smoothDataArray[2], yAngle);      //Y axis gyro data must be switched to match accelerometer angle data
  
  xLinearAccel    = linearAccel(yAngle, smoothDataArray[0]);
  yLinearAccel    = linearAccel(xAngle, smoothDataArray[1]);

  xVelocity       = Velocity(0, xLinearAccel, xVelocity);
  yVelocity       = Velocity(1, yLinearAccel, yVelocity);

  //xSmoothVelocity = digitalSmooth(xVelocity, smoothArrayVelocityX);
  //ySmoothVelocity = digitalSmooth(yVelocity, smoothArrayVelocityY);
  
  //mouseClick(MOUSE_LEFT,   vibration1);                 //left click
  //mouseClick(MOUSE_RIGHT,  vibration2);                 //right click
  //mouseClick(MOUSE_MIDDLE, button3   );                 //middle click
 
  if(mouseOn())
  {
    mouseMove(yVelocity, xVelocity);
  }  
  delay(mouseDelay);
  Serial.println();
}

bool mouseOn()                                            //checks to see whether button has been toggled on or off to turn on mouse
{
  if(digitalRead(pushbutton) == HIGH)
  {
    toggle = !toggle;
    delay(200);
  }
  return toggle;                                          //return true or false
}

void mouseClick(char mousebutton, int sensor)             //click mouse button based on button and sensor parameters
{
  if(digitalRead(sensor) == 1)
  {
    Mouse.click(mousebutton);
    Serial.println(mousebutton +" pressed");
    digitalWrite(blue, 1);
    delay(300);
  }else{
    digitalWrite(blue, 0);
  }
}

void calibrate()
{
  sensors_event_t event;

  digitalWrite(red, 1);

  if(printCalibrateTotal)
  {
    Serial.print("xAccelOffset = ");Serial.print(xAccelOffset); Serial.println();
    Serial.print("yAccelOffset = ");Serial.print(yAccelOffset); Serial.println();
    Serial.print("zAccelOffset = ");Serial.print(zAccelOffset); Serial.println();
    Serial.print("xGyroOffset = "); Serial.print(xGyroOffset);  Serial.println();
    Serial.print("yGyroOffset = "); Serial.print(yGyroOffset);  Serial.println(); 
  }
  
  float xAccelTotal = 0;
  float yAccelTotal = 0;
  float xGyroTotal  = 0;
  float yGyroTotal  = 0;
  float zAccelTotal = 0;
  
  for(int i = 0; i < 30; i++)
  {
    accel.getEvent(&event);
  
    xAccelTotal += event.acceleration.x;  if(printCalibrateProcess){Serial.print(" xAccel = "); Serial.print(event.acceleration.x); Serial.print(" xAccelTotal = ");Serial.print(xAccelTotal);}
    yAccelTotal += event.acceleration.y;  if(printCalibrateProcess){Serial.print(" yAccel = "); Serial.print(event.acceleration.y); Serial.print(" yAccelTotal = ");Serial.print(yAccelTotal);}
    zAccelTotal += event.acceleration.z;  if(printCalibrateProcess){Serial.print(" zAccel = "); Serial.print(event.acceleration.z); Serial.print(" zAccelTotal = ");Serial.print(zAccelTotal);}
    
    gyro.getEvent(&event);

    xGyroTotal  += event.gyro.x;          if(printCalibrateProcess){Serial.print(" xGyro = "); Serial.print(event.gyro.x); Serial.print(" xGyroTotal = ");Serial.print(xGyroTotal);}
    yGyroTotal  += event.gyro.y;          if(printCalibrateProcess){Serial.print(" yGyro = "); Serial.print(event.gyro.y); Serial.print(" yGyroTotal = ");Serial.print(yGyroTotal);}
    delay(100);
    Serial.println();
  }
  
  dataOffsetArray[0] = xAccelTotal/30;
  dataOffsetArray[1] = yAccelTotal/30;
  dataOffsetArray[2] = 0;
  dataOffsetArray[3] = xGyroTotal/30;
  dataOffsetArray[4] = yGyroTotal/30;
  
  float potentialAccelOffsetZ = zAccelTotal/30;
  
  if(printCalibrateTotal)
  {
    Serial.print("xAccelOffset = ");Serial.print(dataOffsetArray[0]); Serial.println();
    Serial.print("yAccelOffset = ");Serial.print(dataOffsetArray[1]); Serial.println();
    Serial.print("zAccelOffset = ");Serial.print(dataOffsetArray[2]); Serial.println();
    Serial.print("xGyroOffset = "); Serial.print(dataOffsetArray[3]);  Serial.println();
    Serial.print("yGyroOffset = "); Serial.print(dataOffsetArray[4]);  Serial.println(); 
  }
  
  digitalWrite(red, 0);
  digitalWrite(green, 1);
  delay(100);
  digitalWrite(green, 0);
  delay(100);
  digitalWrite(green, 1);
  delay(100);
  digitalWrite(green, 0);
  delay(100);
  digitalWrite(green, 1);
  delay(100);
  digitalWrite(green, 0);
  
}

float linearAccel(float current_ang, float current_val)
{                                 
  float gravity_accel = g*sin(current_ang);
  
  float linear_accel = (current_val - gravity_accel);
  
  if(printGravity)
  {
    Serial.print("linear accel = ");  Serial.print(linear_accel); Serial.print("\t");
    Serial.print("current_ang = ");   Serial.print(current_ang);  Serial.print("\t");
    Serial.print("gravity_accel = "); Serial.print(gravity_accel);Serial.print("\t");
  }
  return linear_accel;
}

float Angle(int axis, float smoothGyro, float primarySmoothAccel, float otherSmoothAccel, float smoothAccelz, float previous_angle)
{
  float degreeAccelAngle, AccelAngle, degreeAngle, gyroAngle, dt, angle;
  unsigned long current_time;
  
  current_time = micros();
  
  if(axis == 0)
  {
    dt = (current_time - previous_time_x);
    previous_time_x = current_time;
    //Serial.print("current time  angle = ");Serial.print(current_time);Serial.print("previous time angle = ");Serial.print(previous_time_xx);Serial.print(" dt angle = ");Serial.println(dt);
  }else{
    dt = (current_time - previous_time_y);
    previous_time_y = current_time;
  }
  angle = previous_angle + (smoothGyro*dt)/1000000;
  
  gyroAngle = angle * 57.2958;
  
  AccelAngle = atan2(primarySmoothAccel, smoothAccelz);
  
  angle = .98 * angle + .02 * AccelAngle;
  
  degreeAccelAngle = AccelAngle * 57.2958;
  
  degreeAngle = angle * 57.2958;
  
  if(printAngleX && axis == 0)
  {
    Serial.print("smoothGyro x= ");        Serial.print(smoothGyro);   Serial.print("\t");   Serial.print("accel x= ");           Serial.print(primarySmoothAccel); Serial.print("\t");
    Serial.print("gyro angle x= ");        Serial.print(gyroAngle);    Serial.print("\t");   Serial.print("accel angle x= ");     Serial.print(degreeAccelAngle);   Serial.print("\t");
    Serial.print("angle x= ");             Serial.print(degreeAngle);  Serial.print("\t");   Serial.print("accel z= ");           Serial.print(smoothAccelz);       Serial.print("\t");
  }else if(printAngleY && axis == 1){
    Serial.print("smoothGyro y= ");        Serial.print(smoothGyro);   Serial.print("\t");   Serial.print("accel y= ");           Serial.print(primarySmoothAccel); Serial.print("\t");
    Serial.print("gyro angle y= ");        Serial.print(gyroAngle);    Serial.print("\t");   Serial.print("accel angle y= ");     Serial.print(degreeAccelAngle);   Serial.print("\t");
    Serial.print("angle y= ");             Serial.print(degreeAngle);  Serial.print("\t");   Serial.print("accel z= ");           Serial.print(smoothAccelz);       Serial.print("\t");
  }
  return angle;
}

float Velocity(int axis, float linearAccel, float previous_velocity)
{
  unsigned long current_time;
  float dt, velocity;
  int sign, count, sign_count, initial_sign;
  bool reset;
  
  current_time = micros();
  if(axis == 0)
  {
    dt = (current_time - previous_time_xx);
    previous_time_xx = current_time;
    count = x_count;
    sign = x_sign;
    initial_sign = x_initial_sign;
  }else{
    dt = (current_time - previous_time_yy);
    previous_time_yy = current_time;
    count = y_count;
    sign = y_sign;
    initial_sign = y_initial_sign;
  }
  
  if(abs(linearAccel) > .5)
  {
    velocity = previous_velocity + ((linearAccel * dt)/1000000);
    count = 0;
  }else{
    count += 1;
    velocity = previous_velocity;
    if(count >= 3)
    {
      velocity = 0;
      count = 0;
    }
  }
  if(previous_velocity == 0 && !(velocity == 0))
  {
    initial_sign = (velocity > 0) - (velocity < 0);
  }
  
  sign = (velocity > 0) - (velocity < 0);
    
  if(printVelocityX && axis == 0)
  {
    Serial.print("x smoothAccel = ");   Serial.print(smoothAccelX); Serial.print("\t");
    Serial.print("x linear_accel = ");  Serial.print(linearAccel);  Serial.print("\t");
    Serial.print("x velocity = ");      Serial.print(velocity);     Serial.print("\t");
    Serial.print("x count = ");         Serial.print(count);        Serial.print("\t");
  }else if(printVelocityY && axis == 1){
    Serial.print("y smoothAccel = ");   Serial.print(smoothAccelY); Serial.print("\t");
    Serial.print("y linear_accel = ");  Serial.print(linearAccel);  Serial.print("\t");
    Serial.print("y velocity = ");      Serial.print(velocity);     Serial.print("\t");
    Serial.print("y count = ");         Serial.print(count);        Serial.print("\t");
  }
  if(axis == 0)
  {
    x_count = count;
    x_sign = sign;
    x_initial_sign = initial_sign;
  }else if(axis == 1){
    y_count = count;
    y_sign = sign;
    y_initial_sign = initial_sign;
  }
  return velocity;
}

// Paul Badger 2007

float digitalSmooth(float rawData, float *senseSmoothArray)
{
  int j;
  float k, temp, top, bottom;
  float total;
  static int i;
  static float sorted[filterSamples];
  bool done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  senseSmoothArray[i] = rawData;                 // input new data into the oldest slot

  //Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = senseSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  for (j = 0; j < (filterSamples); j++){    // print the array to debug
    //Serial.print(sorted[j]); 
    //Serial.print("   "); 
  }
  //Serial.println();


  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
     //Serial.print(sorted[j]); 
     //Serial.print("   "); 
  }

  //Serial.println();
  //Serial.print("average = ");
  //Serial.println(total/k);
  return total / k;    // divide by number of samples
}

const int maxXvelocity = 5;
const int maxYvelocity = 5;
const int minXvelocity = -5;
const int minYvelocity = -5;

const int XSign = 1;
const int YSign = 1;

const int MaxMouseMovement = 50;

void mouseMove(float y_velocity, float x_velocity)
{ 
  if(printMouseMoveX)
  {
    Serial.print("x_velocity = ");Serial.print(x_velocity); Serial.print("\t");
  }
  if(printMouseMoveY)
  {
    Serial.print("y_velocity = ");Serial.print(y_velocity); Serial.print("\t");
  }
  if(toggle && (x_initial_sign == x_sign)){
    Mouse.move(0, -50*x_velocity);
  }
  if(toggle && (y_initial_sign == y_sign)){
    Mouse.move(-50*y_velocity, 0);
  }
}







