# dump1090-custom

A custom implementation of an ADS-B (Automatic Dependent Surveillance–Broadcast) decoder in C, designed to process Mode S messages from aircraft transponders. This project includes MQTT integration for publishing decoded aircraft data.

## Features
- Decodes ADS-B messages from raw Mode S signals
- Extracts aircraft position, speed, and other relevant data
- Publishes decoded data to an MQTT broker
- Modular code structure for easy extension

## Requirements
- C compiler (e.g., gcc, clang)
- CMake (for build configuration)
- MQTT C client library (e.g., Eclipse Paho)

## Building
1. Clone the repository:
   ```sh
   git clone <repo-url>
   cd dump1090-custom
   ```
2. Build with CMake:
   ```sh
   mkdir build && cd build
   cmake ..
   make
   ```

## Usage
Run the compiled binary (e.g., `custom_sdr`) to start decoding and publishing ADS-B data:
```sh
./custom_sdr
```

## File Structure
- `main.c` - Main decoding and MQTT logic
- `types.h` - Data structures for decoded messages
- `mqtt0client.h` - MQTT client wrapper functions
- `lib/uthash.h` - Hash table utility (uthash)
- `CMakeLists.txt` - Build configuration

## References
- [ADS-B Decoding Guide](docs/ADS-B%20Decoding%20Guide.pdf) - Learn about ADS-B decoding concepts and message formats
- [Eclipse Paho MQTT C Client](https://www.eclipse.org/paho/)

## License
MIT License
