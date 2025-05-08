# Station Météo

## Description

Ce projet implémente une station météo basée sur la carte **NUCLEO-F446RE** utilisant FreeRTOS. Trois tâches principales assurent :

* **Tmesure** : lecture et conversion de la température via un capteur LM75B sur un Arduino Shield.
* **Thorloge** : gestion de l'heure et de la date avec le module RTC DS3231.
* **Taffichage** : affichage des valeurs sur un écran LCD 16×2 via un module I2C.

Tous les périphériques sont connectés sur le même bus I2C.

## Matériel

* **Carte** : STM32 NUCLEO-F446RE
* **Shield Arduino** : Accessory Shield Waveshare
* **Affichage** : LCD 16×2 + Module I²C
* **Câblage** : SDA, SCL, VCC (5V), GND

**Remarque** : I2C1 fonctionne en mode Fast (400 kHz) avec résistances de pull-up.

## Installation et compilation

1. Clonez le dépôt :

   ```bash
   git clone https://github.com/votre-utilisateur/station-meteo.git
   cd station-meteo
   ```
2. Ouvrez le projet dans STM32CubeIDE.
3. Générez le code via STM32CubeMX (assurez-vous d'activer FreeRTOS et I2C1 avec PB8/PB9).
4. Compilez et programmez la carte.

## Liens et références

* STM32 Nucleo-64 boards User Manual : [UM1724](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
* Accessory Shield Waveshare User Manual : [Shield](https://www.waveshare.com/w/upload/7/76/Accessory-Shield-UserManual.pdf)
* Capteur LM75B Data Sheet : [LM75B](https://www.mouser.com/ds/2/302/LM75B-89456.pdf)
* Module RTC DS3231 Data Sheet : [DS3231](https://www.analog.com/media/en/technical-documentation/data-sheets/ds3231.pdf)
