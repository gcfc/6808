/* get_data.ino
   6.808 Mobile and Sensor Computing Final Project
   Authors: George Chen, Daniel Kuang, Xu Zeng

   Human posture / gait sensing system using 11 Adafruit BNO055 sensors enabled by
   Adafruit TCA9548A Multiplexer

   Move around a little bit to calibrate it before publishing data
*/

#include <Wire.h>
#include <Adafruit_Sensor.h> // in Libraries, search "Adafruit Unified Sensor"
#include <Adafruit_BNO055.h> // in Libraries, search "Adafruit BNO055"
#include <utility/imumaths.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70
#define abs(x) ((x)>0?(x):-(x))



// initiate IMU objects
Adafruit_BNO055 bno  = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29);

// constant resources
const double pi                = 3.141592653589793238462643383;
const float  L_arm             = 0.225; // shoulder to elbow, or elbow to wrist (m)
const int    num_buses         = 2;     // currently only using SD/SC 0 and 1
const int    num_total_sensors = 11;

// variable declarations
int num_sensors;
float eul_x, eul_z;

float x[num_total_sensors],   y[num_total_sensors],  z[num_total_sensors];    // = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float dx[num_total_sensors], dy[num_total_sensors], dz[num_total_sensors];

float   phi[num_total_sensors],   phi_prev[num_total_sensors],   phi_offset[num_total_sensors];
float theta[num_total_sensors], theta_prev[num_total_sensors], theta_offset[num_total_sensors];

bool calibrated[num_total_sensors];
int    negate_x[num_total_sensors] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // either 1 or -1
int    negate_y[num_total_sensors] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // either 1 or -1
char   out[500];

bool  validating[num_total_sensors], validated[num_total_sensors];
int   validate_timer;
float init_x[num_total_sensors], init_y[num_total_sensors];
float  min_x[num_total_sensors],  max_x[num_total_sensors],  min_y[num_total_sensors],  max_y[num_total_sensors];
bool  printed;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("TCA MUX Test begins...");

  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;

      uint8_t data;
      if (!twi_writeTo(addr, &data, 0, 1, 1)) {
        Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
        num_sensors++;
      }
    }
  }
  Serial.print("TCA MUX Test complete! Total sensors found: "); Serial.println(num_sensors);

  Serial.println("Orientation Sensor Test begins...");

  for (uint8_t t = 0; t < num_buses; t++) {

    int i = t + 2;
    if (t == 0) {
      i = t;
    }
    else if (t == 1) {
      i = t + 1;
    }

    tcaselect(t);
    /* Initialise the sensor */
    if (!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }
    Serial.print("Orientation Sensor Test complete: "); Serial.println(i);

    if (t == 0 || t == 1) {
      i++;
      /* Initialise the sensor */
      if (!bno2.begin())
      {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 at 0x29 detected ... Check your wiring or I2C ADDR at 0x29!");
        while (1);
      }
      Serial.print("Orientation Sensor Test complete: "); Serial.println(i);
    }
  }
  delay(1000);

  bno.setExtCrystalUse(true); // use external crystal for better accuracy, must be configured immediately before get_event (because of low-level hardware reasons)
}

void loop()
{
  Serial.print(micros()); Serial.print("\t");
  for (uint8_t b = 0; b < num_buses; b++) {
    tcaselect(b);

    // internal variables: b - buses, i - sensors
    int i = b + 2;
    if (b == 0) {
      i = b;
    }
    else if (b == 1) {
      i = b + 1;
    }

    sensorOperations(&bno, i);

    if (b == 0 || b == 1) {
      i++;
      sensorOperations(&bno2, i);
    }

  }
  Serial.println();
  if (all_validated() && !printed) {
    Serial.println();
    Serial.print(negate_x[0]); Serial.print("\t"); Serial.println(negate_y[0]);
    Serial.print(init_x[0]); Serial.print("\t"); Serial.print(min_x[0]); Serial.print("\t"); Serial.println(max_x[0]);
    Serial.print(init_y[0]); Serial.print("\t"); Serial.print(min_y[0]); Serial.print("\t"); Serial.println(max_y[0]);

    Serial.print(negate_x[1]); Serial.print("\t"); Serial.println(negate_y[1]);
    Serial.print(init_x[1]); Serial.print("\t"); Serial.print(min_x[1]); Serial.print("\t"); Serial.println(max_x[1]);
    Serial.print(init_y[1]); Serial.print("\t"); Serial.print(min_y[1]); Serial.print("\t"); Serial.println(max_y[1]);
    delay(1000);
    printed = true;
    Serial.println("\n=========== ALL VALIDATED! ===========");
  }

}
