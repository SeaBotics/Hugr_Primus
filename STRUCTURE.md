# Hugr Primus – Innholdsfortegnelse

Oversikt over prosjektmappene i Hugr Primus og hva de inneholder.

## Dokumentasjon
- [`docs/`](docs/) – Teknisk dokumentasjon, designskisser og prosjektbeskrivelser.

## Hardware Interfaces
Grensesnitt for all fysisk maskinvare brukt på båten.
- [`hardware_interfaces/camera_system/`](hardware_interfaces/camera_system/) – Kameraoppsett og ROS-drivere.
- [`hardware_interfaces/positioning/`](hardware_interfaces/positioning/) – GNSS og RTK-posisjonering.
- [`hardware_interfaces/imu_unit/`](hardware_interfaces/imu_unit/) – IMU-integrasjon og orienteringsdata.
- [`hardware_interfaces/propulsion_control/`](hardware_interfaces/propulsion_control/) – Styring av thrustere og ESC-er.
- [`hardware_interfaces/power_management/`](hardware_interfaces/power_management/) – Batteri- og strømovervåkning.
- [`hardware_interfaces/modem_interface/`](hardware_interfaces/modem_interface/) – 5G-modem-overvåkning og signalstyrke.

## Programvaremoduler
- [`communication/`](communication/) – Kommunikasjon mellom båt og kontrollstasjon.
- [`control_station/`](control_station/) – GUI, manuell styring, RViz-oppsett.
- [`navigation/`](navigation/) – Waypointkjøring, docking, og hindringsunngåelse.
- [`perception/`](perception/) – Behandling av sensorinformasjon (kamera og LIDAR).
- [`simulation/`](simulation/) – Full simulering av båten i Gazebo.
- [`system_launch/`](system_launch/) – ROS2 launchfiler for å starte hele eller deler av systemet.
- [`utils/`](utils/) – Nytteskript og engangsverktøy.
- [`mission_planning/`](mission_planning/) – Oppdragsbeskrivelser og automatisering.
- [`data_logging/`](data_logging/) – Logging og opptak av drift og sensordata.
- [`diagnostics/`](diagnostics/) – Diagnostikk og helsetilstand for alle systemer.
- [`boat_description/`](boat_description/) – URDF/Xacro-modell av båten for RViz/Gazebo.
