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


// initiate IMU objects
Adafruit_BNO055 bno  = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29);

// constant resources
const double pi        = 3.141592653589793238462643383;
const float  L1        = 0.225; // shoulder to elbow (m)
const float  L2        = 0.25;  // elbow to wrist (m)
const int    num_buses = 2;     // currently only using SD/SC 0 and 1

// variable declarations
int num_sensors;
float eul_x, eul_z;
float phi, theta, phi_prev, theta_prev;
float x, y, z, x_prev, y_prev, z_prev, dx, dy, dz;
float phi_offset[11];   // = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float theta_offset[11]; // = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
bool calibrated[11];
char out[500];

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
    tcaselect(t);
    /* Initialise the sensor */
    if (!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }

    Serial.println("Orientation Sensor Test complete!");
  }
  delay(1000);

  bno.setExtCrystalUse(true); // use external crystal for better accuracy, must be configured immediately before get_event (because of low-level hardware reasons)
}

void loop()
{
  for (uint8_t t = 0; t < 1; t++) { // TODO: i < num_sensors
    tcaselect(t);

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    eul_x = event.orientation.x;    // deg
    eul_z = event.orientation.z;    // deg
    Serial.print(eul_x); Serial.print("\t"); Serial.print(eul_z); Serial.print("\t");

    // angles in the rest of the code are in radians
    theta =  eul_z * pi / 180.0 - theta_offset[t];
    phi   =  eul_x * pi / 180.0 - theta_offset[t];

    // if not calibrated, calibrate and set angles = 0
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if (!calibrated[t]) {
      if (mag == 3) {
        phi_offset[t] = phi;
        theta_offset[t] = theta;
        calibrated[t] = true;
        sprintf(out, "Sensor %d calibrated successfully!", t);
        Serial.println(out);
      }
      else {
        printCalibrationStatus();
      }
    }

    if (all_calibrated()) {


      dx = L1 * sin(phi) * cos(theta) - L1 * sin(phi_prev) * cos(theta_prev);
      dy = L1 * sin(phi) * sin(theta) - L1 * sin(phi_prev) * sin(theta_prev);
      dz = -(L1 * cos(phi) - L1 * cos(phi_prev));

//      dx = L1 * cos(theta) - L1 * cos(theta_prev);
//      dy = L1 * sin(theta) - L1 * sin(theta_prev);

      x += dx; y += dy; z += dz;

      theta_prev = theta;
      phi_prev = phi;
      x_prev = x;
      y_prev = y;
      z_prev = z;

      Serial.print(x); Serial.print("\t"); Serial.print(y); Serial.print("\t"); Serial.print(z); Serial.print("\t \t");
    }
  }
  Serial.println();

}
