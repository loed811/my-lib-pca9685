PCA9685 I2C Driver for STM32

A lightweight, portable, and robust C library for the NXP PCA9685 16-channel, 12-bit PWM FM+ I2C-bus LED controller.

While this library includes a ready-made Hardware Abstraction Layer (HAL) for STM32 (using the standard STM32 HAL), the core logic is hardware-agnostic and can be easily ported to other platforms (ESP32, AVR, PIC, etc.) by modifying the HAL file.

⚡ Features

    Complete Control: Full access to all 16 PWM channels.

    Precision: Set Duty Cycle and Phase Delay using percentage (0.0% - 100.0%).

    Group Control: Set PWM or ON/OFF states for all LEDs simultaneously.

    Frequency Management: Easy API to read/write the prescaler to set PWM frequency (e.g., 50Hz for Servos).

    Power Management: Sleep mode and Restart logic implementation.

    Output Configuration: Support for Open-Drain/Totem-Pole and Inverted logic.

    Addressing: Full support for Sub-Addresses and All Call I2C addresses for daisy-chaining.

📂 Project Structure

Plaintext

.
├── LICENSE                 # MIT License
├── README.md               # Documentation
├── pca9685_i2c.c           # Core Driver Logic (Hardware Agnostic)
├── pca9685_i2c.h           # Core Definitions & API
├── pca9685_i2c_hal.c       # Hardware Implementation (STM32 HAL)
└── pca9685_i2c_hal.h       # HAL Function Prototypes

🛠 Integration

    Copy Files: Copy the .c files to your project's Src folder and .h files to your Inc folder.

    Include Header: Add #include "pca9685_i2c.h" to your main.c.

    STM32 Configuration:

        Ensure your I2C peripheral is initialized in CubeMX.

        This driver assumes hi2c1 is the handle. If you use a different instance (e.g., hi2c2), update the extern declaration in pca9685_i2c_hal.c.

💻 Usage Examples

1. Initialization

Initialize the device struct and register the I2C address.
C

#include "pca9685_i2c.h"

// Define the device handle
pca9685_dev_t pca_dev;

void user_pca_init() {
    // 1. Initialize the low-level HAL (checks I2C handle)
    if (pca9685_i2c_hal_init() != PCA9685_OK) {
        // Handle Error
    }

    // 2. Register the device
    // Params: Device Struct, I2C Addr, AllCall Addr, SubAddr1, SubAddr2, SubAddr3
    pca9685_i2c_register(&pca_dev, I2C_ADDRESS_PCA9685, I2C_ALL_CALL_ADDRESS_PCA9685, 
                         I2C_SUB_ADDRESS_1_PCA9685, I2C_SUB_ADDRESS_2_PCA9685, I2C_SUB_ADDRESS_3_PCA9685);

    // 3. Reset the device to default state
    pca9685_i2c_reset();
    
    // 4. Configure Output Driver (optional, defaults are usually fine)
    pca9685_output_t out_conf = {
        .outdrv = PCA9685_OUTPUT_TOTEM_POLE,
        .outne  = PCA9685_OUTPUT_LOW,
        .och    = PCA9685_CH_ONSTOP,
        .invrt  = PCA9685_OUTPUT_NOTINVERT
    };
    pca9685_i2c_output_init(pca_dev, out_conf);
    
    // 5. Enable Auto-Increment (Recommended for block writes)
    pca9685_i2c_autoincrement(pca_dev, PCA9685_AUTOINCR_ON);
}

2. Setting PWM Frequency (e.g., 50Hz for Servos)

The PCA9685 must be in sleep mode to change the prescaler.
C

void set_frequency_50hz() {
    // Put device to sleep
    pca9685_i2c_sleep_mode(pca_dev, PCA9685_MODE_SLEEP);

    // Set Prescaler for 50Hz (using internal 25MHz clock)
    pca9685_i2c_write_pre_scale(pca_dev, 50.0, 25000000.0);

    // Wake up
    pca9685_i2c_sleep_mode(pca_dev, PCA9685_MODE_NORMAL);
    
    // Restart to resume PWM
    pca9685_i2c_restart(pca_dev);
}

3. Controlling Channels

You can set channels individually or all at once. Values are in percentages (0.0 to 100.0).
C

void control_leds() {
    // Set Channel 0 to 50% Duty Cycle, 0% Delay
    pca9685_i2c_led_pwm_set(pca_dev, 0, 50.0f, 0.0f);

    // Set Channel 1 to 25% Duty Cycle, 10% Phase Delay
    pca9685_i2c_led_pwm_set(pca_dev, 1, 25.0f, 10.0f);

    // Turn Channel 2 Fully ON (No PWM)
    pca9685_i2c_led_set(pca_dev, 2, PCA9685_LED_ON);

    // Turn ALL Channels OFF
    pca9685_i2c_all_led_set(pca_dev, PCA9685_LED_OFF);
}

🔄 Porting to other platforms

To use this library on platforms other than STM32 (e.g., ESP32, Arduino, AVR):

    Keep pca9685_i2c.c and pca9685_i2c.h exactly as they are.

    Modify pca9685_i2c_hal.c:

        Update pca9685_i2c_hal_init() to initialize your specific I2C hardware.

        Replace HAL_I2C_Master_Transmit and HAL_I2C_Master_Receive inside pca9685_i2c_hal_write and read with your platform's I2C functions.

        Update pca9685_i2c_hal_ms_delay with your platform's delay function (e.g., vTaskDelay for FreeRTOS).
        
MIT License

Copyright (c) 2025 loed811

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
