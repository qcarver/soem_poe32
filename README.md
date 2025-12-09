# ESP32-PoE SOEM Project
<img src="docs/images/ESP32-POE.jpg" alt="ESP32-PoE-ISO Board" style="width:50%;height:auto;" />

ESP-IDF project for Olimex ESP32-PoE-ISO with EtherCAT master (SOEM).

## Hardware
- Olimex ESP32-PoE-ISO board
- LAN8710A PHY

## Configuration
- PHY: LAN87xx, address 0
- RMII clock: Internal APLL on GPIO 17
- MDC: GPIO 23, MDIO: GPIO 18
- PHY Power: GPIO 12

## Build
This project uses the [Espressif-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) for the ESP32 target and also the [SOEM project](https://github.com/OpenEtherCATsociety/SOEM) developed by the Open Ethercat Society as a library. The first step is to clone the library and then cmake install it. There are directions in the [Building SOEM](https://docs.rt-labs.com/soem/tutorials/building.html) below. This project uses SOEM built with [Option 2](https://docs.rt-labs.com/soem/tutorials/building.html#option-2-build-and-install-soem-separately). After SOEM is built and installed, continue with the directions in the code block text below.
```bash
source ~/esp/v5.2.1/esp-idf/export.sh # or however you init your esp-idf toolchain 
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## SOEM Documentation
[Introduction](https://docs.rt-labs.com/soem/index.html)

[EtherCAT Overview](https://docs.rt-labs.com/soem/explanations/ethercat.html)

### Tutorials
[Building SOEM](https://docs.rt-labs.com/soem/tutorials/building.html)

[Writing an application](https://docs.rt-labs.com/soem/tutorials/application.html)

### How-to Guides
[Mailbox cyclic handler](https://docs.rt-labs.com/soem/howtos/cyclic_mailbox.html)

[Distributed clocks](https://docs.rt-labs.com/soem/howtos/dc.html)

[ENI parser](https://docs.rt-labs.com/soem/howtos/eni_parser.html)

[Ethtool](https://docs.rt-labs.com/soem/howtos/ethtool.html)

### Reference
[SOEM API](https://docs.rt-labs.com/soem/reference/api_reference.html)

[Glossary](https://docs.rt-labs.com/soem/reference/glossary.html)