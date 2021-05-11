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

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.print(z);
}
