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
const double pi        = 3.141592653589793238462643383;
const float  L1        = 0.225; // shoulder to elbow (m)
const float  L2        = 0.25;  // elbow to wrist (m)
const int    num_buses = 2;     // currently only using SD/SC 0 and 1

// variable declarations
int num_sensors;
float eul_x, eul_z;

float x[11],  y[11],  z[11];    // = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float dx[11], dy[11], dz[11];

float phi[11],     phi_prev[11],   phi_offset[11];
float theta[11], theta_prev[11], theta_offset[11];

bool calibrated[11];
int negate_x[11] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // either 1 or -1
int negate_y[11] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // either 1 or -1
char out[500];

bool validating[11], validated[11];
int validate_timer;
float init_x[11], init_y[11];
float min_x[11], max_x[11], min_y[11], max_y[11];
bool printed;

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

    Serial.print("Orientation Sensor Test complete: "); Serial.println(t + 1);
  }
  delay(1000);

  bno.setExtCrystalUse(true); // use external crystal for better accuracy, must be configured immediately before get_event (because of low-level hardware reasons)
}

void loop()
{
  Serial.print(micros()); Serial.print("\t");
  for (uint8_t t = 0; t < num_sensors; t++) {
    tcaselect(t);

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    eul_x = event.orientation.x;    // deg
    eul_z = event.orientation.z;    // deg
    //    Serial.print((int)eul_x); Serial.print("\t"); Serial.print((int)eul_z); Serial.print("\t");

    // angles in the rest of the code are in radians
    theta[t] =  eul_z * pi / 180.0;//- theta_offset[t];
    phi[t]   =  eul_x * pi / 180.0;//- theta_offset[t];

    // if not calibrated, calibrate and set angles = 0
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if (!calibrated[t]) {
      if (mag == 3) {
        phi_offset[t] = phi[t];
        theta_offset[t] = theta[t];
        calibrated[t] = true;
        sprintf(out, "Sensor %d calibrated successfully!", t);
        Serial.println(out);
      }
      else {
        printCalibrationStatus();
      }
    }

    if (all_calibrated()) {
      dx[t] = negate_x[t] * (L1 * sin(phi[t]) * cos(theta[t]) - L1 * sin(phi_prev[t]) * cos(theta_prev[t])) ;
      dy[t] = negate_y[t] * (L1 * sin(phi[t]) * sin(theta[t]) - L1 * sin(phi_prev[t]) * sin(theta_prev[t])) ;
      dz[t] = -(L1 * cos(phi[t]) - L1 * cos(phi_prev[t]));
      x[t] += dx[t]; y[t] += dy[t]; z[t] += dz[t];
      theta_prev[t] = theta[t];
      phi_prev[t] = phi[t];

      //        Serial.print(dx[t]); Serial.print("\t"); Serial.print(dy[t]); Serial.print("\t"); Serial.print(dz[t]); Serial.print("\t \t");
      Serial.print(x[t]); Serial.print("\t"); Serial.print(y[t]); Serial.print("\t"); Serial.print(z[t]); Serial.print("\t \t");

      // janky validation code, wave forearm only
      if (!validated[t]) {
        if (!validating[t]) {
          validate_timer = millis();
          init_x[t] = x[t];
          init_y[t] = y[t];
          validating[t] = true;
        }
        if (millis() - validate_timer < 10000) {
          min_x[t] = (x[t] < min_x[t]) ? x[t] : min_x[t];
          min_y[t] = (y[t] < min_y[t]) ? y[t] : min_y[t];
          max_x[t] = (x[t] > max_x[t]) ? x[t] : max_x[t];
          max_y[t] = (x[t] > max_y[t]) ? y[t] : max_y[t];
        }
        else {
          negate_x[t] = (abs(init_x[t] - min_x[t]) > abs(init_x[t] - max_x[t])) ? 1 : -1;
//          negate_y[t] = (abs(init_y[t] - max_y[t]) > abs(init_y[t] - min_y[t])) ? 1 : -1;
          validated[t] = true;
        }
      }
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
    delay(20000);
    printed = true;
    Serial.println("\n=========== ALL VALIDATED! ===========");
  }

}
