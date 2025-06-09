# Navigasjon

Denne mappen inneholder koden som styrer autonom bevegelse av Hugr Primus. Navigasjonsmodulen skal sikre at fartøyet kan utføre oppdrag som å navigere fra A til B, unngå hindringer, og gjennomføre autodocking – helt uten manuell styring.

## Hovedkomponenter

- **Waypoint-navigasjon:** Fartøyet følger en forhåndsdefinert rute basert på GPS-koordinater.
- **Path following:** Kontrollalgoritmer som holder kursen mellom waypoints.
- **Obstacle avoidance:** Dynamisk unngåelse av hindringer ved hjelp av LIDAR, kamera eller andre sensorer.
- **Autodocking:** System for å kunne navigere til en spesifikk posisjon (kai/bøye) og gjennomføre presis dokking.
- **State machine:** Overordnet logikk som styrer hvilken tilstand båten er i (idle, navigerer, docking, nødstopp, osv.)
- **Sensor-integrasjon:** Posisjon og orientering basert på GNSS og IMU-data.
