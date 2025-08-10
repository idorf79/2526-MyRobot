# Prototypes

## Development environment for ESP32 controllers

Use Visual Studio Code to install the ESP-IDF extension (ID: espressif.esp-idf-extension)

Make sure to have an `esp` directory in your "Home": ```mkdir ~/esp/```

During setting up the ESP-IDF Extension

1. Select `Advanced`
Use the following settings:

Download server: Github
Select ESP-IDF version: v5.5(release version)
ESP-IDF container directory: /home/<username>/esp
ESP-IDF Tools directory: /home/<username>/.espressif
Python version: /usr/bin/python3

2. Press "Configure Tools"

This will install ESP-IDF and the needed tools. Also it will create a virtual python environment for building the ESP code.

It will take a while to download and install.

3. Press "Download Tools¨


## External Libraries from Uncle Rus

For the ESP32 some external libraries are used from https://github.com/UncleRus/esp-idf-lib.

These can be "installed" by:

```
cd ~/esp/
git clone -b 0.9.4 https://github.com/UncleRus/esp-idf-lib.git
```

Note: A specific version is used here!

Make sure the `CMakeLists.txt` on the project level reflects the correct path to the libraries. For example:

```
set(EXTRA_COMPONENT_DIRS /home/fste/esp/esp-idf-lib/components)
```