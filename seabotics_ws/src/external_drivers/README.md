# External Drivers

Denne mappen inneholder tredjepartsdrivere som integreres i prosjektet, men som ikke er utviklet internt.

## Struktur
Hver ekstern driver legges som en egen ROS2-pakke i denne katalogen, med sin egen `CMakeLists.txt` og `package.xml`.

## Retningslinjer
- Bruk **offisielle eller vedlikeholdte** drivere når de finnes.
- Ikke endre originalkoden direkte — bruk `git submodule` hvis mulig.
- Hver driver skal kunne bygges via `colcon build` sammen med resten av workspace.
- Dokumentér eventuelle lokale tilpasninger i en egen `NOTES.md`.

## Innhold
- `ros2_xsens_mti_driver`: ROS 2-driver for Xsens MTi-680G og lignende enheter (DEMCON).

