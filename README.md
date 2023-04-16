# Airhorn RISC-V

Airhorn is a project to make airtags beep. It is designed to be used on an [ESP32-C3](https://www.espressif.com/en/products/socs/esp32-c3), which is based on the RISC-V architecture.

## Working

The program works by listening for BLE advertisements. When it gets an airtag's advertisement (in the disconnected state), it will attempt to connect to it, then plan a sound by writing the the corresponding GATT characteristic.
