/*************************************************************************************************************************************
 * @file huzzah32_pins.h
 * @author Doug Elliott
 * @brief Include file that defines the GPIO pins on an Adafriuit Huzzah32 development board 
 * @version 0.0.1
 * @date 2020-04-21
 * @copyright Copyright (c) 2020
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * Version YYYY-MM-DD Description
 * ------- ---------- ----------------------------------------------------------------------------------------------------------------
 * 0.0.1   2020-04-21 Include file created. Comments about the pins in markdown table format  
 *************************************************************************************************************************************
 * # Table of Huzzah32 Pin Assignments for TWIPe Robot
 * | Phy | GPIO | In,Out, |                                                                                       |
 * | Pin | Pin  | Param   |  Function                                                                             |
 * |:---:|:----:|:--------|:--------------------------------------------------------------------------------------|
 * |  1  |  -   | -       | Reset |
 * |  2  |  -   | -       | 3.3 Volts |
 * |  3  |  -   | -       | No connection |
 * |  4  |  -   | -       | CPU Ground |
 * |  5  |  26  | -       | was n/c, but rewire: DRV1-fault, to avoid ESP onboard LED
 * |  6  |  25  | -       | ADC Aq. MOSFET controlled voltage diver input - reads modified battery level |
 * |  7  |  34  | -       | No connection |
 * |  8  |  39  | -       | INT pin on IMU - generates an interrupt |
 * |  9  |  36  | -       | No connection |
 * | 10  |   4  |   I     | Software readable pushbutton input, to COM position on 4th amber pushbutton |
 * | 11  |   5  |   O     | MOSFET Enable reading of battery level on ADC = physical pin 6. Inverted - LOW enables it |
 * | 12  |  18  |   O     | DRV1-STEP |
 * | 13  |  19  |   O     | DRV1-DIR |
 * | 14  |  16  |   O     | DRV1-ENA. Powers up with weak pulldown enabled. Top DRV motor driver socket, closest to IMU |
 * | 15  |  17  |   P     | IC2-IMU Pin 4, SDA with 2.2K pullup to 3.3V |
 * | 16  |  21  |   P     | IC2-IMU Pin 3, SCL with 2.2K pullup to 3.3V |
 * | 17  |  23  |   O     | software controlled LED, +ve LED input on 4th pushbutton (orange) |
 * | 18  |  22  |   P     | I2C-LCD Pin 2 SDA with 4.7K pullup to 5V |
 * | 19  |  14  |   P     | I2C-LCD Pin 1 SCL with 4.7K pullup to 5V |
 * | 20  |  32  |   I     | DRV2-FAULT Input, Pin 10 (used to be VDD on old A4988) |
 * | 21  |  15  |   O     | DRV2-DIR, Pin 8 |
 * | 22  |  33  |   O     | DRV2-STEP, Pin 7 |
 * | 23  |  27  |   O     | DRV2-ENA, Pin 1 |
 * | 24  |  12  | -       | N/C. Using this pin seems to prevent software download in some circumstances |
 * | 25  |  13  | -       | DRV1-FAULT Input. Pin 10 (used to be VDD on old A4988) Conflicts with onboard red LED? | no longer used
 * | 26  |      | -       | USB, source of 5V for IC2-LCD pullups, LCD header |
 * | 27  |      | -       | Enable, n/c |
 * | 28  |      | -       | LiPo Battery socket, n/c |
 *************************************************************************************************************************************/
#define gp_SWR_BUTTON 4               // software readable pushbutton
#define gp_SWC_LED 23                 // software controlled LED. HIGH or TRUE = illuminated

#define gp_DRV1_ENA 16                // Enable for DRV1 - controls 12V consumption, maybe
#define gp_DRV1_STEP 18               // CPU output for STEP command 
#define gp_DRV1_DIR 19                // CPU output for DIRection control
#ifdef avoidEspLed                    // if DRV1 was rewired so fault uses cpu pin 5, gpio 26
#define gp_DRV1_FAULT 26              // rewired so DRV1 fault line goes to cpu pin 5, gpio 26
#else
#define gp_DRV1_FAULT 13              // CPU input for fault indicator. Conflicts with onboard red LED?
#endif

#define gp_DRV2_ENA 27                // Enable for DRV1 - controls 12V consumption, maybe
#define gp_DRV2_STEP 33               // CPU output for STEP command 
#define gp_DRV2_DIR 15                // CPU output for DIRection control
#define gp_DRV2_FAULT 32              // CPU input to read status from DRV2

#define gp_I2C_IMU_SDA 17             // used as input parameter to library routines
#define gp_I2C_IMU_SCL 21             // used as input parameter to library routines
#define gp_IMU_INT 39                 // INTerrupt pin on IMU 

#define gp_I2C_LCD_SDA 22             // used as input parameter to library routines 
#define gp_I2C_LCD_SCL 14             // used as input parameter to library routines 

#define gp_MOSFET_ENA 5               // CPU output that enables the battery voltage divider feeding ADC
#define gp_ADC_VOLTS 25               // Analog input that measures scaled battery voltage level

#define gp_CPU_LED 13                 // red LED on ESP32 board, next connector, conflicts with DRV1_FAULT? 