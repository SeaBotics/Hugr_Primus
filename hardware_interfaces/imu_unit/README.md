# IMU-enhet

Denne mappen inneholder kode og konfigurasjon for IMU-en som brukes til å bestemme orienteringen til Hugr Primus.

## Innhold
- ROS2-drivere for IMU (f.eks. Bosch, Xsens, BNO085)
- Kalibrering av akselerometer, gyro og magnetometer
- Sensorfusjon (eks. Madgwick eller EKF)
- Publisering på `sensor_msgs/Imu`
- Bruk i sammenheng med GNSS i posisjonsestimering
