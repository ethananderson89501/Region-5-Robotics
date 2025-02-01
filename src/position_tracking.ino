
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SERIAL_PORT Serial

#define WIRE_PORT Wire // Your desired Wire port.
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; //create an ICM_20948_I2C object

unsigned long start_time;
unsigned long current_time;
unsigned long elapsed_time;
double position[2] = {0, 0};
double velocity[2] = {0, 0};
double acceleration[2] = {0, 0};

void setup()
{

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {


    myICM.begin(WIRE_PORT, AD0_VAL);


    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  start_time = micros();
}

void loop()
{

  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(300);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }

  current_time = micros();
  elapsed_time = current_time - start_time;
  start_time = current_time;

  acceleration[0] = myICM.accX();
  acceleration[1] = myICM.accY();

  position[0] = position[0] + velocity[0]*elapsed_time + acceleration[0]*elapsed_time*elapsed_time;
  position[1] = position[1] + velocity[1]*elapsed_time + acceleration[1]*elapsed_time*elapsed_time;

  velocity[0] = velocity[0] + acceleration[0]*elapsed_time;
  velocity[1] = velocity[1] + acceleration[1]*elapsed_time;
}