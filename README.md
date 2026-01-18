# ESP32 BLE KVM Bridge

A project that allows you to use your PC (Windows) keyboard and mouse to control another device (smartphone, tablet, laptop) via Bluetooth Low Energy (BLE), using an ESP32 as a bridge.

## How it works

1. **Host (PC)**: A Python script captures raw input from your mouse and keyboard (using the Windows Raw Input API).
2. **Bridge (Serial)**: The data is sent to the ESP32 via a serial port (USB).
3. **ESP32 (BLE)**: The microcontroller processes the data and sends it as standard HID reports over Bluetooth Low Energy.
4. **Target**: The end device sees the ESP32 as a physical Bluetooth keyboard and mouse.

## Features

- **Bluetooth LE Connection**: No cables between the bridge and the target device.
- **Low Latency**: Optimized data transfer.
- **Precision**: Uses Raw Input API to ensure 1:1 mouse movement without Windows system acceleration.
- **Easy Switching**: Quickly toggle between local and remote control with a single key (KVM style).
- **Compatibility**: Works with Android, iOS (iPhone/iPad), macOS, Windows, Linux, and any system supporting BLE HID.

## Requirements

### Hardware
- **ESP32 Board** (ESP32-WROOM, ESP32-C3, ESP32-S3, or any other compatible with ESP-IDF and BLE).
- USB cable (for power and communication with the PC).

### Software
- **Host System**: Windows 10/11 (required due to the use of WinAPI libraries in the Python script).
- **Python**: Version 3.8 or newer.
- **ESP-IDF**: For firmware compilation (version 5.x recommended).

## Installation

### 1. ESP32 Firmware

1. Ensure you have the ESP-IDF environment installed.
2. Configure the project:
   ```bash
   idf.py set-target esp32  # Set your model, e.g., esp32, esp32s3, esp32c3
   idf.py menuconfig
   ```
   *Under `Application`, you can change the Bluetooth device name (default: **ESP32**).*

3. Build and flash the firmware:
   ```bash
   idf.py build
   idf.py -p COMx flash monitor
   ```
   *(Replace COMx with the appropriate port, e.g., COM3)*

### 2. Host Software (PC)

1. Install the required Python libraries:
   ```bash
   pip install pynput pyserial
   ```

2. (Optional) Adjust settings in `main.py` (class `BridgeConfig`):
   - `port`: ESP32 COM port (usually detected automatically).
   - `baudrate`: Transmission speed (default 115200).
   - `sensitivity`: Mouse sensitivity multiplier.

## Usage

1. Connect the ESP32 to your computer.
2. Pair the target device (e.g., phone) with the ESP32 via Bluetooth:
   - Search for a device named **ESP32** (or whatever you set in the config).
   - Connect.
3. Run the bridge script on your PC:
   ```bash
   python main.py
   ```
4. **Control**:
   - The script will connect to the ESP32.
   - Press the **PAUSE / BREAK** key to toggle remote mode.
   - In remote mode, your mouse and keyboard control the Bluetooth device.
   - Press **PAUSE / BREAK** again to return to controlling your PC.

## Project Structure

- `main.py` - Host script capturing input and sending it to the ESP32.
- `main/` - ESP32 firmware source code (C/C++).
    - `src/app_main.c` - Main BLE application logic.
    - `src/uart_proto.c` - UART communication protocol handling.
    - `src/ble_hid_api.c` - HID over GATT profile implementation.
