float to_first_quad(float degree) {
  if (90 <= degree && degree <= 180) {
    return 180.0 - degree;
  } else if (180 <= degree && degree <= 270) {
    return degree - 180.0;
  } else if (270 <= degree && degree <= 360) {
    return 360.0 - degree;
  } else if (degree < 0) {
    return to_first_quad(360.0+degree);
  }
}


void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

bool all_calibrated() {
  for (int i = 0; i < num_sensors; i++) {
    if (!calibrated[i]) {
      return false;
    }
  }
  return true;
}

bool all_validated() {
  for (int i = 0; i < num_sensors; i++) {
    if (!validated[i]) {
      return false;
    }
  }
  return true;
}

void printCalibrationStatus() {
  Serial.print("Not calibrated: ");
  for (int i = 0; i < num_sensors; i++) { 
    if (!calibrated[i]) {
      Serial.print(i); Serial.print(" ");
    }
  }
  Serial.println();
}

void sensorOperations(Adafruit_BNO055* my_bno, int i) {
  /* Get a new sensor event */
    sensors_event_t event;
    my_bno->getEvent(&event);
    uint8_t system, gyro, accel, mag = 0;
    my_bno->getCalibration(&system, &gyro, &accel, &mag);

    eul_x = event.orientation.x;    // deg
    eul_z = event.orientation.z;    // deg
    //    Serial.print((int)eul_x); Serial.print("\t"); Serial.print((int)eul_z); Serial.print("\t");


    // angles in the rest of the code are in radians
    theta[i] =  eul_z * pi / 180.0;//- theta_offset[i];
    phi[i]   =  eul_x * pi / 180.0;//- theta_offset[i];

    // if not calibrated, calibrate and set angles = 0
    if (!calibrated[i]) {
      if (mag == 3) {
        phi_offset[i] = phi[i];
        theta_offset[i] = theta[i];
        calibrated[i] = true;
        sprintf(out, "Sensor %d calibrated successfully!", i);
        Serial.println(out);
      }
      else {
        printCalibrationStatus();
      }
    }

    if (all_calibrated()) {
      dx[i] = negate_x[i] * (L_arm * sin(phi[i]) * cos(theta[i]) - L_arm * sin(phi_prev[i]) * cos(theta_prev[i])) ;
      dy[i] = negate_y[i] * (L_arm * sin(phi[i]) * sin(theta[i]) - L_arm * sin(phi_prev[i]) * sin(theta_prev[i])) ;
      dz[i] = -(L_arm * cos(phi[i]) - L_arm * cos(phi_prev[i]));
      x[i] += dx[i]; y[i] += dy[i]; z[i] += dz[i];
      theta_prev[i] = theta[i];
      phi_prev[i] = phi[i];

      //        Serial.print(dx[i]); Serial.print("\t"); Serial.print(dy[i]); Serial.print("\t"); Serial.print(dz[i]); Serial.print("\t \t");
      Serial.print(x[i]); Serial.print("\t"); Serial.print(y[i]); Serial.print("\t"); Serial.print(z[i]); Serial.print("\t \t");

      // janky validation code, raise arm to determine negation sign
      if (!validated[i]) {
        if (!validating[i]) {
          validate_timer = millis();
          init_x[i] = x[i];
          init_y[i] = y[i];
          validating[i] = true;
        }
        if (millis() - validate_timer < 10000) {
          min_x[i] = (x[i] < min_x[i]) ? x[i] : min_x[i];
          min_y[i] = (y[i] < min_y[i]) ? y[i] : min_y[i];
          max_x[i] = (x[i] > max_x[i]) ? x[i] : max_x[i];
          max_y[i] = (x[i] > max_y[i]) ? y[i] : max_y[i];
        }
        else {
          negate_x[i] = (abs(init_x[i] - min_x[i]) > abs(init_x[i] - max_x[i])) ? 1 : -1;
//          negate_y[i] = (abs(init_y[i] - max_y[i]) > abs(init_y[i] - min_y[i])) ? 1 : -1; // if i == 1, negate_y[i] = 1
          validated[i] = true;
        }
      }
    }
}
