#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001";

struct dataPacket
{
  char team_no[5];
  int seconds;
  // float depth[6];
  float pressure;
  //float altitude;
  int profileno;
  int dataCollected;
};

void setup() {

  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_2MBPS);
  radio.enableAckPayload();
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
}

void loop() {
  if (Serial.available()) 
  {  
    char dataSent = Serial.read();  // Read the incoming character
    Serial.print("data sent: ");
    Serial.println(dataSent);  // Print the received character
    radio.write(&dataSent, sizeof(dataSent));
  }
  if (radio.available()) {
    dataPacket data;
    float depthData[5][8];
    float depth[8];
    float depth2[8];
    float depth3[8];
    float depth4[8];
    float depth5[8];

    radio.read(&depth, sizeof(depth));
    Serial.print("Depth: ");
    for (int i = 0; i < 8; i++){
     Serial.print(depth[i]);
     Serial.print(" ");
    }

    delay(1000);

    radio.read(&depth2, sizeof(depth2));
    for (int i = 0; i < 8; i++){
     Serial.print(depth2[i]);
     Serial.print(" ");
    }

     delay(1000);

    radio.read(&depth3, sizeof(depth3));
    for (int i = 0; i < 8; i++){
     Serial.print(depth3[i]);
     Serial.print(" ");
    }

    delay(1000);

    radio.read(&depth4, sizeof(depth4));
    for (int i = 0; i < 8; i++){
     Serial.print(depth4[i]);
     Serial.print(" ");
    }

    delay(1000);

    radio.read(&depth5, sizeof(depth5));
    for (int i = 0; i < 8; i++){
     Serial.print(depth5[i]);
     Serial.print(" ");
    }
    Serial.println("");


    // radio.read(&depthData, sizeof(depthData));
    // Serial.print(", Depth: ");
    // for (int i = 0; i < 40; i++){
    //  Serial.print(depthData[(int) i / 8][i % 8]);
    //  Serial.print(" ");
    // }

    // Serial.println("");

    delay(1000);
    radio.read(&data, sizeof(data));
    Serial.print("Received - Team Number: ");
    Serial.print(data.team_no);
    Serial.print(", Seconds: ");
    Serial.print(data.seconds);
    Serial.print(", Pressure: ");
    Serial.print(data.pressure);
    // Serial.print("number of profile = ");
    // Serial.print(data.profileno);
    Serial.println(" ");

    // Serial.print("data collected: ");
    // Serial.print(data.dataCollected);

    delay(5000);
  }
  else {
    Serial.println("No data available");
    delay(1000);
  }
}