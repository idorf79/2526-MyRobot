# Prototyping

This folder contains a ESP32 project which is used to protoype different sensors and activators.

In the `main` folder an `other` folder exist with different implementations of `main.c`.
Copying the contents of these files into `main.c` will give different functionality.

Prototypes

| C file | Description | Remarks|
|--------|-------------|---------|
| blink_main.c | Blinks a LED| GPIO pin for LED is configured via SDK config
| gyro_main.c | Reads a MPU6050 over I2C | Only seems to read one axis of the rotation sensor. I2C pins and address are hardcoded. |
| gyro2_main.c | Read MPU6050 over I2C | Uses Uncle Rus Library to read the data. Seems to be working. I2C pins and address are hardcoded. |


