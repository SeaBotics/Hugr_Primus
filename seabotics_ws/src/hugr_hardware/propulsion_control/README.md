# Propulsjonskontroll

Denne mappen inneholder kode og grensesnitt for å styre thrustere på Hugr Primus via ESC-er.

## Innhold
- ROS2-node for å sende thrust-kommandoer
- Protokoll for kommunikasjon med mikrokontroller (UART, USB, CAN)
- Mapping mellom thrust-verdi og PWM/DShot-signal
- Kalibreringsdata og motorparametere
- Fail-safe ved tap av kommando eller kritisk feil
