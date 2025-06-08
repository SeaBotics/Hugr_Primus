# Posisjonssystem (GNSS/RTK)

Denne mappen inneholder grensesnitt og konfigurasjon for GNSS- og RTK-enheten som gir Hugr Primus global posisjonering.

## Innhold
- GNSS-drivere (f.eks. NMEA-navsat, u-blox)
- RTK-basert korreksjon (f.eks. RTCM via NTRIP eller egen base)
- ROS2-integrasjon med `sensor_msgs/NavSatFix`
- Kobling mot sensorfusjon (f.eks. IMU/ekstern estimator)
- Feilhåndtering ved tap av signal eller RTK-korreksjon
