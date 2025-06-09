# Manual Control

Dette delsystemet skal håndtere manuell styring av Hugr Primus fra kontrollstasjonen. Det skal muliggjøre direkte kontroll av fartøyets bevegelser via en fysisk kontroller (Xbox/DualShock), og støtte for å bytte mellom manuell og autonom modus.

## Innhold som skal implementeres

- **Joystick Input System**  
  System som leser input fra en fysisk kontroller og oversetter til thrust- og styringskommandoer.

- **Kontrollmodusbryter**  
  System for å veksle mellom autonom styring og manuell styring. Dette skal kunne styres fra kontrollstasjonen.

- **Thrusterkobling**  
  Koble joystickinput til faktiske thrustere via fartøyets eksisterende thruster-grensesnitt.

- **Failsafe / Timeout**  
  Hvis manuell input forsvinner, skal systemet kunne gå tilbake til autonom eller stoppe fartøyet.

## Maskinvare

- Xbox- eller DualShock-kontroller koblet til kontrollstasjonen (USB eller Bluetooth)

## Programvare

- Input-system på kontrollstasjon (f.eks. pygame, ROS2 joy, eller lavnivå HID)
- Kommunikasjon til fartøy via 5G/UDP/TCP/ROS2
- Thruster-interface på fartøyet som kan ta imot manuelle kommandoer

## Fremtidige filer (forslag)

- `joystick_input/` – modul for kontroller-input
- `control_switcher/` – logikk for autonomi/manual bryter
- `manual_thruster_interface/` – kobling mot motorer
