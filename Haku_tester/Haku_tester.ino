#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <RF24.h>
#include <printf.h>



const int E1 = 4;
const int M1 = 5;
const int sw1 = 2; // bottom switch
const int sw2 = 3; // top switch
const int hall2 = A1; // hall effect sensor that indicates the float reached the surface
const int hall1 = A0; // hall effect sensor that indicates the float reached the bottom
bool surfaced = false; // surfaced and seabed is used to indicate whether the float has surfaced or reached the seabed so that vertical profiling can be completed
bool seabed = false;
bool bottom = false;
bool top = false;
unsigned int seconds;
unsigned long startTime = 0;
unsigned long previousTime;
int profileCounter;
int collected;
int original;
RF24 radio(9, 10);
const byte address[6] = "00001";
MS5837 sensor;

const int DEPTH_ARRAY_SIZE = 8;
float depthData[5][DEPTH_ARRAY_SIZE] = {0.0};

struct dataPacket //test
{
  char team_no[5];
  int seconds;
  // float depth[DEPTH_ARRAY_SIZE];
  float depth;
  float pressure;
  //float altitude;
  int profileno;
  int dataCollected;
};

void setup() {

  Serial.begin(9600);

  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(sw1, INPUT_PULLUP); // Use INPUT_PULLUP to enable the internal pull-up resistor
  pinMode(sw2, INPUT_PULLUP);
  pinMode(A0, INPUT); // Use INPUT_PULLUP to enable the internal pull-up resistor
  
  radio.begin();
  radio.setDataRate(RF24_2MBPS);
  radio.enableAckPayload();
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_HIGH); // Set power level (RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX)
  
  
  Serial.println("Starting");

  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  // .init sets the sensor model for us but we can override it if required.
  // Uncomment the next line to force the sensor model to the MS5837_30BA.
  sensor.setModel(MS5837::MS5837_30BA);

  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)


}




void motor_down()
{
 bool sw1_stat = digitalRead(sw1);
 //Serial.println(sw1_stat);
 if (sw1_stat == LOW)
 {
  digitalWrite(M1, HIGH); //turns on motor
  digitalWrite(E1, LOW); //sets direction
  Serial.println("moving");
 }
 else if (sw1_stat == HIGH)
 {
  digitalWrite(M1, LOW); // turns off motor
  digitalWrite(E1,LOW);
  Serial.println("stop");
 }

}

void motor_up()
{
  bool sw2_stat = digitalRead(sw2);
  if (sw2_stat == LOW) 
   {
    digitalWrite(M1, HIGH); //turns on motor
    digitalWrite(E1, HIGH);
    Serial.println("motor is upper"); //sets direction
   }
  else if (sw2_stat == HIGH)
   {
    digitalWrite(M1, LOW); // turns off motor
    digitalWrite(E1, LOW);
    Serial.println("stop");
   }
}


void VP_down() {
  bool hall1_stat = digitalRead(hall1);
  Serial.println(hall1_stat);
  if (hall1_stat == HIGH && bottom == false)// bed reached
  {
    surfaced = false;
    seabed = true;
    bottom = true;
    top = false;
    Serial.println("seabed reached");
  }
  else if (hall1_stat == LOW && seabed == false) // reaching bed
  {
    bottom = false;
    seabed = false;
    Serial.println("down we go");
  }
  if (seabed == false)
    motor_down();
    Serial.println("motor_down");
}

void VP_up() {
  bool hall2_stat = digitalRead(hall2);
  Serial.println(hall2_stat);
  if (hall2_stat == HIGH && top == false)// surface reached
  {
    surfaced = true;
    top = true;
    bottom = false;
    seabed = false;
    Serial.println("surfaced reached");
  }
  else if (hall2_stat == LOW && surfaced == false)// surface reaching
  {
    top = false;
    Serial.println("going up");
  }
  if (surfaced == false)
    motor_up();
    Serial.println("motor_up");
}

void countSeconds() {
  if (startTime == 0) {
    startTime = millis(); // Set the initial start time
  }

  unsigned long currentTime = millis(); // Get the current time in milliseconds
  unsigned long elapsedTime = currentTime - startTime; // Calculate the elapsed time

  // Calculate the number of seconds
  seconds = elapsedTime / 1000;
}
// void depthCollect(dataPacket &packet)
// {

//   // static unsigned int i = 0; // Declare i as static
//   unsigned long initialTime = millis();
//   if (initialTime - previousTime >= 500)
//   {
//     previousTime = initialTime;
//     sensor.read();
//     float data = sensor.depth();
//     // Serial.print("data = ");
//     // Serial.println(data);
//     // packet.depth[packet.dataCollected] = data;
//     depthData[(int) packet.dataCollected / DEPTH_ARRAY_SIZE][packet.dataCollected % DEPTH_ARRAY_SIZE] = data;

//     packet.dataCollected ++;

//     // Serial.println(packet.depth[i]);

//     // packet.dataCollected ++;
//     // if(packet.dataCollected == 6){
//     //   packet.depth[0] = packet.depth[5];
//     //   for(int i = 1; i < 5; i ++){
//     //     packet.depth[i] = 0.0;
//     //   }
//     //   packet.dataCollected = 1;
//     // }
//   }
// }

void verticalProfile()
{
 int intial = seconds;
 if (seabed == false) {//if it is not in the seabed it will go down
    VP_down();
    Serial.println("going down");
  }
 else if (surfaced == false) { //if its not on the surface it will go up
    VP_up();
    Serial.println("going up");
  }
  profileCounter= profileCounter + 1;
 
}

// void reset() {
//   for(int i = 0; i < DEPTH_ARRAY_SIZE; i ++){
//     depthData[(int) i / DEPTH_ARRAY_SIZE][i % DEPTH_ARRAY_SIZE] = 0.0;
//   }
// }

void loop() {
  char dataSent;
  countSeconds();
  sensor.read();
  dataPacket data;
  strcpy(data.team_no, "EX05");
  data.profileno = profileCounter;
  data.seconds = seconds;
  //data.altitude = sensor.altitude(); //in meters above sea level
  data.depth = sensor.depth();
  data.pressure = sensor.pressure(); // in mbar

  Serial.println(data.seconds);
  if (data.seconds >= 10)
  {
   verticalProfile();
   Serial.println("commencing vertical profile");
   Serial.print("profile no = ");
   Serial.println(data.profileno);
   original = data.seconds;
   delay(1000);
  }
  else if (profileCounter > 0 && original - data.seconds >= 15)
  {
   verticalProfile();

  }
  if (data.depth <= 1)
  {
    radio.write(&data, sizeof(data));
    Serial.println("sending");
   
  }

  Serial.print("Pressure: ");
  Serial.print(sensor.pressure());
  Serial.println(" mbar");

  Serial.print("Temperature: ");
  Serial.print(sensor.temperature());
  Serial.println(" deg C");

  Serial.print("Depth: ");
  Serial.print(sensor.depth());
  Serial.println(" m");

  Serial.print("Altitude: ");
  Serial.print(sensor.altitude());
  Serial.println(" m above mean sea level");

  delay(1000);
}


  //float tempDepth = sensor.depth();
  // Serial.print("data = ");

  // Print depth array
    // Serial.print("data collected: ");
    // Serial.println(data.dataCollected);
  //   for(int i = 0; i < 5 * DEPTH_ARRAY_SIZE; i ++){
  //     Serial.print(depthData[(int) i / DEPTH_ARRAY_SIZE][i % DEPTH_ARRAY_SIZE]);
  //     Serial.print(" ");
  //   }
  //   Serial.println("");
  
  // // Serial.println(data.pressure);

  // if (sensor.depth() <= 1){
  //   float depth[8];
  //   for(int i = 0; i < 8; i ++){
  //     depth[i] = depthData[0][i];
  //   }
  //   float depth2[8];
  //   for(int i = 0; i < 8; i ++){
  //     depth2[i] = depthData[1][i];
  //   }
  //   float depth3[8];
  //   for(int i = 0; i < 8; i ++){
  //     depth3[i] = depthData[2][i];
  //   }
  //   float depth4[8];
  //   for(int i = 0; i < 8; i ++){
  //     depth4[i] = depthData[3][i];
  //   }
  //   float depth5[8];
  //   for(int i = 0; i < 8; i ++){
  //     depth5[i] = depthData[4][i];
  //   }
  //   radio.write(&depth, sizeof(depth));
  //   delay(1000);
  //   radio.write(&depth2, sizeof(depth2));
  //   delay(1000);
  //   radio.write(&depth3, sizeof(depth3));
  //   delay(1000);
  //   radio.write(&depth4, sizeof(depth4));
  //   delay(1000);
  //   radio.write(&depth5, sizeof(depth5));
  //   // radio.write(&data, sizeof(data));
  //   delay(1000);
  //   radio.write(&data, sizeof(data));
  // }
  //  else 
  //  {
  //   Serial.println("not in range");

  //  }


  // data.previousProfileno = data.profileno;
  // if (Serial.available()) 
  //  {
  //   String input = Serial.readString();
  //   input.trim();
  //  }
  // if (seabed == false) {//if it is not in the seabed it will go down
  //   VP_down();
  // }
  // else if (surfaced == false) { //if its not on the surface it will go up
  //   VP_up();
  // }
  // delay(1000);

  //  radio.read(&dataSent, sizeof(dataSent));
   //Serial.print("data received = ");
   //Serial.println(dataSent);
  //  if (dataReceived == "w")
  //   {
  //     verticalProfile();
  //   }
  //  else if (dataReceived == "s")
  //   {
  //     motor_down();
  //   }
  //  else if (dataReceived == "a")
  //   {
  //     motor_up();
  //   }
  //  switch (dataSent)
  //  {
  //   case 'w':
  //    {
  //     verticalProfile();
  //     break;
  //    }
  //    case 's':
  //    {
  //      motor_down();
  //      break;
  //    }
  //    case 'a':
  //    {
  //      motor_up();
  //      break;
  //    }
  //  }
