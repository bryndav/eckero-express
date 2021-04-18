void
readIMU() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(iax, iay, iaz);
    ax = iax;
    ay = iay;
    az = iaz;
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(igx, igy, igz);       // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD
    gx = (igx * (3.14159 / 180));
    gy = (igy * (3.14159 / 180));
    gz = (igz * (3.14159 / 180));
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(imx, imy, imz);
    mx = imx;
    my = imy;
    mz = imz;
  }

  deltat = filter.deltatUpdate(); //this have to be done before calling the filter update

  //filter.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  filter.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
  //filter.update(igx, igy, igz, iax, iay, iaz, imx, imy, imz);  //else use the magwick, it is slower but more accurate

  pitch = filter.getPitch();
  roll = filter.getRoll();
  imu_heading = filter.getYaw();
}
