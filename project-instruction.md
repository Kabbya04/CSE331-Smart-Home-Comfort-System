# Smart Room Comfort Index Project Instructions

This document provides a comprehensive guide to building the Smart Room Comfort Index system using an STM32F103C8T6 "Blue Pill" board. The system monitors environmental conditions, evaluates a comfort index, and provides visual feedback and automated control.

### Required Hardware

1.  STM32F103C8T6 (Blue Pill)
2.  HC-SR04 Ultrasonic Sensor
3.  PIR Motion Sensor
4.  SG90 Servo Motor
5.  DHT11 Temperature & Humidity Sensor
6.  I2C 0.96" OLED Display (128x64)
7.  Red LED, Green LED, Yellow LED (1 each)
8.  Resistors (3x 220-330 Ohm for LEDs)
9.  Breadboard and Jumper Wires
10. ST-Link V2 programmer

### Connection Diagram

**Please read the critical warning below the table before you proceed.**

| Component | Component Pin | STM32 Pin | Description |
| :--- | :--- | :--- | :--- |
| **Power** | 5V | `5V` | Main 5V power rail for peripherals |
| **Power** | GND | `GND` | Common Ground for all components |
| **DHT11 Sensor** | VCC | `3.3V` | |
| | GND | `GND` | |
| | DATA | `PA0` | Data line for temperature/humidity |
| **HC-SR04 Sensor**| VCC | `5V` | |
| | GND | `GND` | |
| | Trig | `PA3` | Trigger pin to start a measurement |
| | Echo | `PA2` | Echo pin to read the pulse duration |
| **PIR Sensor** | VCC | `5V` | |
| | GND | `GND` | |
| | OUT | `PA4` | **Interrupt** signal for motion |
| **SG90 Servo** | V+ (Red) | `5V` | |
| | GND (Brown) | `GND` | |
| | Signal (Orange)| `PA1` | PWM signal for controlling position |
| **OLED Display** | VCC | `3.3V` | |
| | GND | `GND` | |
| | SCL | `PB6` | I2C Clock Line |
| | SDA | `PB7` | I2C Data Line |
| **Green LED (Comfort)**| Anode (+) | `PC13` | HIGH comfort indicator |
| **Yellow LED (Comfort)**| Anode (+) | `PC14` | MEDIUM comfort indicator |
| **Red LED (Comfort)** | Anode (+) | `PC15` | LOW comfort indicator |
| **Red LED (PIR)** | Anode (+) | `PA10` | Lights up when PIR detects presence |
| **Green LED (Ultrasonic)**| Anode (+) | `PA11` | Blinks when an object is close |
| **All LEDs** | Cathode (-) | `GND` | Connect via a 220-330Ω resistor to GND |
---
### The Role of the I2C Pull-Up Resistors

The two resistors you added are called **I2C pull-up resistors**. Their single, critical job is to ensure the communication signals between the STM32 and the OLED display are clear and stable.

#### The Reason (Why They Are Necessary)

The I2C communication protocol uses a special type of signaling called "open-drain." In simple terms, this means that both the STM32 and the OLED can only pull the communication lines **LOW** (to 0V/Ground). They have no ability to push the lines back **HIGH** (to 3.3V).

Think of it like a rope tied to the floor. You can easily pull the rope down, but you can't *push* it back up. To get it back up, you need a spring or a rubber band pulling it towards the ceiling.

In our circuit, **the pull-up resistor is that spring**. It constantly and gently pulls the line towards the HIGH (3.3V) state.

Without these resistors, when a device lets go of the line, it just "floats" in an undefined electrical state. This creates the "noise" and communication errors you were seeing, as the signals for '1's and '0's become corrupted.

#### The Configuration (How They Are Connected)

This is why the specific connection is so important:

*   One resistor connects the **SCL (clock) line** to the **3.3V** power line.
*   The other resistor connects the **SDA (data) line** to the **3.3V** power line.

This setup ensures that whenever no device is actively pulling the lines down to signal a '0', they reliably snap back up to a clean '1' state, making the communication successful.

---
### Step 1: STM32CubeIDE Initial Setup

This project assumes you have already installed STM32CubeIDE and can flash a basic program to your Blue Pill board.

1.  Start a new STM32 Project and select your MCU: `STM32F103C8T6`.
2.  Go to **System Core -> RCC**. For **High Speed Clock (HSE)**, select **Crystal/Ceramic Resonator**.
3.  Go to the **Clock Configuration** tab and set the **HCLK (MHz)** to **72**. Press Enter and let the tool resolve the new configuration.

### Step 2: Configuring Peripherals & Pins

In the **Pinout & Configuration** tab, set up the timers, I2C, and GPIO pins.

1.  **Timer for Delays (TIM1):**
    *   Click **Timers -> TIM1**.
    *   Set **Clock Source** to **Internal Clock**.
    *   Under the **Configuration -> Parameter Settings** tab, set the **Prescaler** to `71`.

2.  **Timer for Servo PWM (TIM2):**
    *   Click **Timers -> TIM2**.
    *   Set **Clock Source** to **Internal Clock**.
    *   Set **Channel 1** to **PWM Generation CH1**.
    *   Under the **Configuration -> Parameter Settings** tab, set **Prescaler** to `143` and **Counter Period** to `9999`.

3.  **I2C for OLED Display (I2C1):**
    *   Click **Connectivity -> I2C1**.
    *   For **Mode**, select **I2C**.

4.  **GPIO Pin Configuration:**
    *   `PA0` (DHT11): Set as **GPIO_Input**.
    *   `PA1` (Servo): This will be automatically configured as `TIM2_CH2` or a similar timer output pin.
    *   `PA2` (Echo): Set as **GPIO_Input**.
    *   `PA3` (Trig): Set as **GPIO_Output**.
    *   `PA10` (PIR LED): Set as **GPIO_Output**.
    *   `PA11` (Ultrasonic LED): Set as **GPIO_Output**.
    *   `PC13`, `PC14`, `PC15` (Comfort LEDs): Set each as **GPIO_Output**.

5.  **PIR Interrupt Configuration (Crucial Step):**
    *   On the chip diagram, left-click pin **`PA4`** and select **`GPIO_EXTI4`**. This sets the pin as an external interrupt line.
    *   Now, on the left-side panel, go to **System Core -> NVIC**.
    *   Find the row for **"EXTI line4 interrupt"** and **check the box** in the "Enabled" column.

6.  Save the `.ioc` file to generate the code.

---

### Step 3: Required Libraries

This project requires a library for the SSD1306 OLED display.

1.  Download the library files: `ssd1306.c`, `ssd1306.h`, `fonts.c`, and `fonts.h`. You can find these easily on GitHub or from the Micropeta reference site.
2.  In your STM32CubeIDE project explorer, place the `.c` files in the **Core/Src** folder.
3.  Place the `.h` files in the **Core/Inc** folder.

---

### Step 4: Adding the Code (Updated)

Open `main.c` and add the following code snippets to the marked `USER CODE` sections.

#### Add Includes and Private Variables
```c
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h" // Required for abs() function
#include "ssd1306.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
// DHT11 variables
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_0
uint8_t RHI, RHD, TCI, TCD, SUM;
float temperature = 0.0;
float humidity = 0.0;

// HC-SR04 variables
#define TRIG_PORT GPIOA
#define TRIG_PIN GPIO_PIN_3
#define ECHO_PORT GPIOA
#define ECHO_PIN GPIO_PIN_2
uint32_t pMillis;
uint32_t val1 = 0;
uint32_t val2 = 0;
uint16_t distance = 0;
uint16_t previous_distance = 0; // For movement detection

// PIR Sensor Pin
#define PIR_PORT GPIOA
#define PIR_PIN GPIO_PIN_4
volatile uint8_t occupancy = 0; // Use volatile for interrupt-modified variables

// OLED display buffer
char oled_buf[24];
char comfort_level[10]; // To store comfort level string
/* USER CODE END PV */
```

#### Add Helper Functions
```c
/* USER CODE BEGIN 0 */
void microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DHT11_Start(void) {
    uint8_t Response = 0;
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
    HAL_Delay(20);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
    microDelay(30);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
    microDelay(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        microDelay(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Response = 1;
    }
    pMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > HAL_GetTick());
    return Response;
}

uint8_t DHT11_Read(void) {
    uint8_t i = 0, j;
    for (j = 0; j < 8; j++) {
        pMillis = HAL_GetTick();
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > HAL_GetTick());
        microDelay(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) i &= ~(1 << (7 - j));
        else i |= (1 << (7 - j));
        pMillis = HAL_GetTick();
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > HAL_GetTick());
    }
    return i;
}
/* USER CODE END 0 */
```

#### Initialize Peripherals
```c
/* USER CODE BEGIN 2 */
  // --- OLED STABILITY FIX ---
  HAL_Delay(100);
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);
  HAL_Delay(100);
  // --- END OF FIX ---

  // Initialize OLED
  SSD1306_Init();

  // Start Timers
  HAL_TIM_Base_Start(&htim1); // For microsecond delay
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // For Servo Motor

  // Set initial servo position to closed (0 degrees)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 250);

  // Initial pin state for HC-SR04
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
/* USER CODE END 2 */
```

#### Add the PIR Interrupt Callback Function
This special function runs automatically whenever the PIR sensor detects motion. Place it in the `USER CODE BEGIN 4` section.
```c
/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI Line Detection Callback.
  * @param  GPIO_Pin: The Pin that triggered the interrupt.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if the interrupt came from our PIR sensor pin (PA4)
  if (GPIO_Pin == PIR_PIN)
  {
    // Update the global occupancy variable.
    // A small delay helps to debounce the sensor signal.
    HAL_Delay(50); 
    occupancy = HAL_GPIO_ReadPin(PIR_PORT, PIR_PIN);
  }
}
/* USER CODE END 4 */
```

#### Main Application Logic (The `while(1)` loop)
```c
/* USER CODE BEGIN WHILE */
  while (1) {
    // 1. Read DHT11 and Ultrasonic sensors (PIR is now handled by interrupts)
    if (DHT11_Start()) {
        RHI = DHT11_Read(); RHD = DHT11_Read();
        TCI = DHT11_Read(); TCD = DHT11_Read();
        SUM = DHT11_Read();
        if (RHI + RHD + TCI + TCD == SUM) {
            temperature = (float)TCI + (float)(TCD/10.0);
        }
    }

    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    microDelay(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    pMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 10 > HAL_GetTick());
    val1 = __HAL_TIM_GET_COUNTER(&htim1);
    pMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    val2 = __HAL_TIM_GET_COUNTER(&htim1);
    distance = (val2 - val1) * 0.034 / 2;

    // 2. Evaluate comfort and set original LED state
    if (temperature <= 27.0) {
        strcpy(comfort_level, "High");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    } else if (temperature > 27.0 && temperature <= 32.0) {
        strcpy(comfort_level, "Medium");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    } else {
        strcpy(comfort_level, "Low");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }

    // 3. Control Servo (Easy-to-test logic)
    if (temperature <= 27.0 && occupancy) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 750); // Open servo
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 250); // Close servo
    }
    
    // 4. OLED code (kept in place)
    SSD1306_Clear();
    sprintf(oled_buf, "Temp: %.1f C", temperature);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(oled_buf, &Font_7x10, 1);
    sprintf(oled_buf, "Comfort: %s", comfort_level);
    SSD1306_GotoXY(0, 12);
    SSD1306_Puts(oled_buf, &Font_7x10, 1);
    sprintf(oled_buf, "Presence: %s", occupancy ? "Yes" : "No");
    SSD1306_GotoXY(0, 24);
    SSD1306_Puts(oled_buf, &Font_7x10, 1);
    if (abs(distance - previous_distance) > 3 && distance < 200) {
        sprintf(oled_buf, "Movement: Yes");
    } else {
        sprintf(oled_buf, "Movement: No");
    }
    SSD1306_GotoXY(0, 36);
    SSD1306_Puts(oled_buf, &Font_7x10, 1);
    HAL_Delay(1);
    SSD1306_UpdateScreen();
    
    // 5. Control the Red LED for PIR presence detection
    if (occupancy) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); 
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }

    // 6. Control the Green LED for Ultrasonic distance
    if (distance < 15) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    }

    previous_distance = distance;

    HAL_Delay(250);
  }
/* USER CODE END WHILE */
```