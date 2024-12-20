/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include "TMC4671_controller.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
  0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,           // USAGE (Joystick)
  0xA1, 0x01,           // COLLECTION (Application)

	//================================Input Report======================================//
	0x09, 0x01,             // Usage (Pointer)
	// Wheel report
	0x85, 0x01,             // Report ID (1)
	0xA1, 0x00,             // Collection (Physical)
		// 1 Axis for steering wheel
		0x09, 0x30,             // Usage (X) for wheel only
		0x16, 0x00, 0x80,       // Logical Minimum (-32768)
		0x26, 0xFF, 0x7F,       // Logical Maximum (32767)
		0x75, 0x10,             // Report Size (16)
		0x95, 0x01,             // Report Count (1)
		0x81, 0x02,             // Input (Data,Var,Abs)
	0xc0,                   // End Collection (Physical)

	// PID State Report
	0x05, 0x0F,          // Usage Page (Physical Interface)
	0x09, 0x92,          // Usage (PID State Report)
	0xA1, 0x02,          // Collection Datalink (Logical)
		0x85, 0x02,             // Report ID (2)
		0x09, 0x9F,             // USAGE (Device Paused)
		0x09, 0xA0,             // USAGE (Actuators Enabled)
		0x09, 0xA4,             // USAGE (Safety Switch)
		0x09, 0xA5,             // USAGE (Actuator Override Switch)
		0x09, 0xA6,             // USAGE (Actuator Power)
		0x09, 0x94,    			// USAGE (Effect Playing)
		0x15, 0x00,             // Logical Minimum (0)
		0x25, 0x01,             // Logical Maximum (1)
		0x35, 0x00,             // Physical Minimum (0)
		0x45, 0x01,             // Physical Maximum (1)
		0x75, 0x01,             // Report Size (1)
		0x95, 0x05,             // Report Count (5)
		0x81, 0x02,    			// Input (Variable)
		0x95, 0x03,    			// Report Count 3
		0x81, 0x03,    			// Input (Constant, Variable)
	0xC0,                // End Collection Datalink (Logical) (OK)

	//================================OutputReport======================================//
	// Set Effect Report
	0x09, 0x21,    		 // Usage (Set Effect Report)
	0xA1, 0x02,    		 // Collection Datalink (Logical)
		0x85, 0x01,    			// Report ID (1)

		0x09, 0x22,    			// Usage (Effect Block Index)
		0x15, 0x01,    			  // Logical Minimum (1)
		0x25, 0x28,    			  // Logical Maximum (40)
		0x35, 0x01,    			  // Physical Minimum (1)
		0x45, 0x28,    			  // Physical Maximum (40)
		0x75, 0x08,    			  // Report Size 8
		0x95, 0x01,    			  // Report Count 1
		0x91, 0x02,    			  // Output (Variable)

		0x09, 0x25,    			// Usage Effect Type
		0xA1, 0x02,    			  // Collection Datalink (logical)
			0x09, 0x26,             // Usage (ET Constant Force)
			0x09, 0x27,             // Usage (ET Ramp)
			0x09, 0x30,             // Usage (ET Square)
			0x09, 0x31,             // Usage (ET Sine)
			0x09, 0x32,             // Usage (ET Triangle)
			0x09, 0x33,             // Usage (ET Sawtooth Up)
			0x09, 0x34,             // Usage (ET Sawtooth Down)
			0x09, 0x40,             // Usage (ET Spring)
			0x09, 0x41,             // Usage (ET Damper)
			0x09, 0x42,             // Usage (ET Inertia)
			0x09, 0x43,             // Usage (ET Friction)
			0x15, 0x01,             // Logical Minimum (1)
			0x25, 0x0B,             // Logical Maximum (11)
			0x35, 0x01,             // Physical Minimum (1)
			0x45, 0x0B,             // Physical Maximum (11)
			0x75, 0x08,             // Report Size (8)
			0x95, 0x01,             // Report Count (1)
			0x91, 0x00,             // Output (Data)
	   	0xC0,            	      // End Collection Datalink (Logical)

		0x09, 0x50,             // Usage (Duration)
		0x09, 0x54,             // Usage (Trigger Repeat Interval)
		0x09, 0x51,             // Usage (Sample Period)
		0x15, 0x00,                 // Logical Minimum (0)
		0x26, 0xFF, 0x7F,           // Logical Maximum (32767)
		0x35, 0x00,                 // Physical Minimum (0)
		0x46, 0xFF, 0x7F,           // Physical Maximum (32767)
		0x66, 0x03, 0x10,           // Unit (4099)
		0x55, 0xFD,                 // Unit Exponent (253)
		0x75, 0x10,                 // Report Size (16)
		0x95, 0x04,         		// Report Count (4)
		0x91, 0x02,         		// Output (Variable)
		0x55, 0x00,         		// Unit Exponent (0)
		0x66, 0x00, 0x00,    		// Unit (0)

		0x09, 0x52,         	// Usage (Gain)
		0x15, 0x00,         		// Logical Minimum (0)
		0x26, 0xFF, 0x00,    		// Logical Maximum (255)
		0x35, 0x00,         		// Physical Minimum 0
		0x46, 0x10, 0x27,    		// Physical Maximum (10000)
		0x75, 0x08,         		// Report Size (8)
		0x95, 0x01,         		// Report Count (1)
		0x91, 0x02,         		// Output (Variable)

		0x09, 0x53,         	// Usage Trigger Button
		0x15, 0x01,         		// Logical Minimum (1)
		0x25, 0x08,         		// Logical Maximum (8)
		0x35, 0x01,         		// Physical Minimum (1)
		0x45, 0x08,         		// Physical Maximum (8)
		0x75, 0x08,         		// Report Size (8)
		0x95, 0x01,         		// Report Count (1)
		0x91, 0x02,         		// Output (Variable)

		0x09, 0x55,       		// Usage (Axes Enable)
		0xA1, 0x02,       		  // Collection Datalink (logical)
			0x05, 0x01,    			// Usage (Page Generic Desktop)
			0x09, 0x30,    			// Usage (X)
			0x15, 0x00,    			  // Logical Minimum (0)
			0x25, 0x00,    			  // Logical Maximum (0)
			0x75, 0x01,    			  // Report Size (1)
			0x95, 0x01,    			  // Report Count (1)
			0x91, 0x02,    			  // Output (Variable)
		0xC0,        		 	  // End Collection Datalink (logical)

		0x05, 0x0F,    		    // Usage Page (Physical Interface)
		0x09, 0x56,    		      // Usage (Direction Enable)
		0x95, 0x01,    		    	// Report Count (1)
		0x91, 0x02,    		    	// Output (Variable)
		0x95, 0x06,    		    	// Report Count (6)
		0x91, 0x03,    		    	// Output (Constant, Variable)
		0x09, 0x57,    			  // Usage (Direction)
		0xA1, 0x02,    				// Collection Datalink (logical)
			0x0B, 0x01, 0x00, 0x0A, 0x00,   // Usage (Ordinals: Instance 1)
			0x66, 0x14, 0x00,           	// Unit (20)
			0x15, 0x00,                     // Logical Minimum (0)
			0x27, 0xA0, 0x8C, 0x00, 0x00,   // Logical Maximum (36000)
			0x35, 0x00,                     // Physical Minimum (0)
			0x47, 0xA0, 0x8C, 0x00, 0x00,   // Physical Maximum (36000)
			0x66, 0x00, 0x00,               // Unit (0)
			0x75, 0x10,                   	// Report Size (16)
			0x95, 0x01,                   	// Report Count (1)
			0x91, 0x02,                   	// Output (Variable)
			0x55, 0x00,                   	// Unit Exponent (0)
			0x66, 0x00, 0x00,               // Unit (0)
	    0xC0,                        // End Collection Datalink (logical)

	    0x05, 0x0F,        		// Usage Page (Physical Interface)
	    0x09, 0x58,        		  // Usage (Type Specific Block Offset)
	    0xA1, 0x02,        		    // Collection (Logical)
			0x0B, 0x01, 0x00, 0x0A, 0x00,   // Usage (Ordinals:Instance 1)
			0x26, 0xFD, 0x7F, 			  	// Logical Maximum (32765) ; 32KB RAM or ROM max.
			0x75, 0x10,     				// Report Size (16)
			0x95, 0x01,     				// Report Count (1)
			0x91, 0x02,     				// Output (Data,Var,Abs)
	    0xC0,              			// End Collection (logical)
	0xC0,                 // End Collection Datalink (logical)

	// Set Envelope Report
	0x09, 0x5A,    		  // Usage (Set Envelope Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
	   	0x85, 0x02,             // Report ID (2)

	   	0x09, 0x22,           	// Usage (Effect Block Index)
	   	0x15, 0x01,           	  // Logical Minimum (1)
	   	0x25, 0x28,           	  // Logical Maximum (40)
	   	0x35, 0x01,           	  // Physical Minimum (1)
	   	0x45, 0x28,           	  // Physical Maximum (40)
	   	0x75, 0x08,           	  // Report Size (8)
	   	0x95, 0x01,           	  // Report Count (1)
	   	0x91, 0x02,           	  // Output (Variable)
	   	
		0x09, 0x5B,           	// Usage (Attack Level)
	   	0x09, 0x5D,           	// Usage (Fade Level)
	 	0x16, 0x00, 0x00,         // Logical Minimum (0)
	 	0x26, 0xFF, 0x7F,         // Logical Maximum (32767)
	 	0x36, 0x00, 0x00,         // Physical Minimum (0)
	 	0x46, 0xFF, 0x7F,         // Physical Maximum (32767)
	 	0x75, 0x10,               // Report Size (16)
	 	0x95, 0x02,               // Report Count (2)
	    0x91, 0x02,         	  // Output (Variable)

		0x09, 0x5C,         	// Usage (Attack Time)
		0x09, 0x5E,         	// Usage (Fade Time)
	 	0x66, 0x03, 0x10,  		  // Unit (1003h) English Linear, Seconds
	 	0x55, 0xFD,        		  // Unit Exponent (FDh) (X10^-3 ==> Milisecond)
	 	0x27, 0xFF, 0x7F, 0x00, 0x00,    // Logical Maximum (4294967295)
	 	0x47, 0xFF, 0x7F, 0x00, 0x00,    // Physical Maximum (4294967295)
	 	0x75, 0x20,         	  // Report Size (32)
	 	0x91, 0x02,         	  // Output (Variable)
	 	0x45, 0x00,         	  // Physical Maximum (0)
		0x66, 0x00, 0x00,   	  // Unit (0)
		0x55, 0x00,         	  // Unit Exponent (0)
	0xC0,            	  // End Collection Datalink (logical)

	// Set Condition Report
	0x09, 0x5F,    		  // Usage (Set Condition Report)
	0xA1, 0x02,           // Collection Datalink (logical)
		0x85, 0x03,    			// Report ID (3)

		0x09, 0x22,    			// Usage (Effect Block Index)
		0x15, 0x01,    			  // Logical Minimum (1)
		0x25, 0x28,    			  // Logical Maximum (40)
		0x35, 0x01,    			  // Physical Minimum (1)
		0x45, 0x28,    			  // Physical Maximum (40)
		0x75, 0x08,    			  // Report Size (8)
		0x95, 0x01,    			  // Report Count (1)
		0x91, 0x02,    			  // Output (Variable)

		0x09, 0x23,    			// Usage (Parameter Block Offset)
		0x15, 0x00,    			  // Logical Minimum (0)
		0x25, 0x03,    			  // Logical Maximum (3)
		0x35, 0x00,    			  // Physical Minimum (0)
		0x45, 0x03,    			  // Physical Maximum (3)
		0x75, 0x06,    			  // Report Size (6)
		0x95, 0x01,    			  // Report Count (1)
		0x91, 0x02,    			  // Output (Variable)

		0x09, 0x58,    			// Usage (Type Specific Block Off...)
		0xA1, 0x02,    			  // Collection Datalink (logical)
			0x0B, 0x01, 0x00, 0x0A, 0x00,   // Usage (Ordinals: Instance 1)
			0x75, 0x02,                    	// Report Size (2)
			0x95, 0x01,                   	// Report Count (1)
			0x91, 0x02,                   	// Output (Variable)
		0xC0,	         		  // End Collection Datalink (logical)
		0x16, 0x00, 0x80,    	  // Logical Minimum (-32767)
		0x26, 0xff, 0x7f,    	  // Logical Maximum (32767)
		0x36, 0x00, 0x80,    	  // Physical Minimum (-32767)
		0x46, 0xff, 0x7f,    	  // Physical Maximum (32767)

		0x09, 0x60,         	// Usage (CP Offset)
		0x75, 0x10,         	  // Report Size (16)
		0x95, 0x01,         	  // Report Count (1)
		0x91, 0x02,         	  // Output (Variable)
		0x36, 0x00, 0x80,    	  // Physical Minimum  (-32768)
		0x46, 0xff, 0x7f,    	  // Physical Maximum  (32767)

		0x09, 0x61,         	// Usage (Positive Coefficient)
		0x09, 0x62,         	// Usage (Negative Coefficient)
		0x95, 0x02,        	 	  // Report Count (2)
		0x91, 0x02,         	  // Output (Variable)
	 	0x16, 0x00, 0x00,    	  // Logical Minimum 0
	 	0x26, 0xff, 0x7f,	  	  // Logical Maximum  (32767)
	 	0x36, 0x00, 0x00,    	  // Physical Minimum 0
		0x46, 0xff, 0x7f,    	  // Physical Maximum  (32767)

		0x09, 0x63,         	// Usage (Positive Saturation)
		0x09, 0x64,         	// Usage (Negative Saturation)
		0x75, 0x10,         	  // Report Size (16)
		0x95, 0x02,         	  // Report Count (2)
		0x91, 0x02,         	  // Output (Variable)

		0x09, 0x65,         	// Usage (Dead Band)
		0x46, 0xff, 0x7f,    	  // Physical Maximum (32767)
		0x95, 0x01,          	  // Report Count (1)
		0x91, 0x02,          	  // Output (Variable)
	0xC0,    // End Collection Datalink (logical)

	// Set Periodic Report
	0x09, 0x6E,    		  // Usage (Set Periodic Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
		0x85, 0x04,             // Report ID (4)

		0x09, 0x22,             // Usage (Effect Block Index)
		0x15, 0x01,               // Logical Minimum (1)
		0x25, 0x28,               // Logical Maximum (40)
		0x35, 0x01,               // Physical Minimum (1)
		0x45, 0x28,               // Physical Maximum (40)
		0x75, 0x08,               // Report Size (8)
		0x95, 0x01,               // Report Count (1)
		0x91, 0x02,               // Output (Variable)

		0x09, 0x70,             // Usage (Magnitude)
		0x16, 0x00, 0x00,         // Logical Minimum (0)
		0x26, 0xff, 0x7f,    	  // Logical Maximum (32767)
		0x36, 0x00, 0x00,         // Physical Minimum (0)
		0x26, 0xff, 0x7f,    	  // Logical Maximum (32767)
		0x75, 0x10,               // Report Size (16)
		0x95, 0x01,               // Report Count (1)
		0x91, 0x02,               // Output (Variable)

		0x09, 0x6F,             // Usage (Offset)
		0x16, 0x00, 0x80,    	  // Logical Minimum (-32767)
		0x26, 0xff, 0x7f,    	  // Logical Maximum (32767)
		0x36, 0x00, 0x80,    	  // Physical Minimum (-32767)
		0x46, 0xff, 0x7f,    	  // Physical Maximum (32767)
		0x95, 0x01,               // Report Count (1)
		0x75, 0x10,               // Report Size (16)
		0x91, 0x02,               // Output (Variable)

		0x09, 0x71,             // Usage (Phase)
		0x66, 0x14, 0x00,             // Unit (14h) (Eng Rotation, Degrees)
		0x55, 0xFE,                   // Unit Exponent FEh (X10^-2)
		0x15, 0x00,                   // Logical Minimum (0)
		0x27, 0x9F, 0x8C, 0x00, 0x00, // Logical Maximum (35999)
		0x35, 0x00,                   // Physical Minimum (0)
		0x47, 0x9F, 0x8C, 0x00, 0x00, // Physical Maximum (35999)
		0x75, 0x10,                   // Report Size (16)
		0x95, 0x01,                   // Report Count (1)
		0x91, 0x02,                   // Output (Variable)

		0x09, 0x72,             // Usage (Period)
		0x15, 0x01,                   // Logical Minimum (1)
		0x27, 0xFF, 0x7F, 0x00, 0x00, // Logical Maximum (32K)
		0x35, 0x01,                   // Physical Minimum (1)
		0x47, 0xFF, 0x7F, 0x00, 0x00, // Physical Maximum (32K)
		0x66, 0x03, 0x10,             // Unit (1003h) (English Linear, Seconds)
		0x55, 0xFD,                   // Unit Exponent FDh (X10^-3 ==> Milisecond)
		0x75, 0x20,                   // Report Size (32)
		0x95, 0x01,                   // Report Count (1)
		0x91, 0x02,                   // Output (Variable)
		0x66, 0x00, 0x00,       // Unit (0)
		0x55, 0x00,             // Unit Exponent (0)
	0xC0,     	    	  // End Collection Datalink (logical)

	// Set Constant Force Report
	0x09, 0x73,    		  // Usage (Set Constant Force Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
		0x85, 0x05,       		// Report ID (5)

		0x09, 0x22,         	// Usage (Effect Block Index)
		0x15, 0x01,         		// Logical Minimum (1)
		0x25, 0x28,         		// Logical Maximum (40)
		0x35, 0x01,         		// Physical Minimum (1)
		0x45, 0x28,         		// Physical Maximum (40)
		0x75, 0x08,         		// Report Size (8)
		0x95, 0x01,         		// Report Count (1)
		0x91, 0x02,         		// Output (Variable)

		0x09, 0x70,         	// Usage (Magnitude)
		0x16, 0x00, 0x80,    		// Logical Minimum (-32767)
		0x26, 0xff, 0x7f,    		// Logical Maximum (32767)
		0x36, 0x00, 0x80,    		// Physical Minimum (-32767)
		0x46, 0xff, 0x7f,    		// Physical Maximum (32767)
		0x75, 0x10,         		// Report Size (16)
		0x95, 0x01,         		// Report Count (1)
		0x91, 0x02,         		// Output (Variable)
	0xC0,			      // End Collection Datalink (logical)

	// Set Ramp Force Report
	0x09, 0x74,    		  // Usage (Set Ramp Force Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
		0x85, 0x06,         	// Report ID (6)

		0x09, 0x22,         	// Usage (Effect Block Index)
		0x15, 0x01,         		// Logical Minimum (1)
		0x25, 0x28,         		// Logical Maximum (40)
		0x35, 0x01,         		// Physical Minimum (1)
		0x45, 0x28,         		// Physical Maximum (40)
		0x75, 0x08,         		// Report Size (8)
		0x95, 0x01,         		// Report Count (1)
		0x91, 0x02,         		// Output (Variable)
		0x09, 0x75,         	// Usage (Ramp Start)
		0x09, 0x76,         	// Usage (Ramp End)
		0x16, 0x00, 0x80,    		// Logical Minimum (-32767)
		0x26, 0xff, 0x7f,    		// Logical Maximum (32767)
		0x36, 0x00, 0x80,    		// Physical Minimum (-32767)
		0x46, 0xff, 0x7f,    		// Physical Maximum (32767)
		0x75, 0x10,         		// Report Size (16)
		0x95, 0x02,         		// Report Count (2)
		0x91, 0x02,         		// Output (Variable)
	0xC0,			      // End Collection Datalink (logical)

	// Effect Operation Report
	0x05, 0x0F,   		  // Usage Page (Physical Interface)
	0x09, 0x77,   		  // Usage (Effect Operation Report)
	0xA1, 0x02,   		  // Collection Datalink (logical)
		0x85, 0x0A,    			// Report ID (10)

		0x09, 0x22,    			// Usage (Effect Block Index)
		0x15, 0x01,    			  	// Logical Minimum (1)
		0x25, 0x28,    			  	// Logical Maximum (40)
		0x35, 0x01,    			  	// Physical Minimum (1)
		0x45, 0x28,    			  	// Physical Maximum (40)
		0x75, 0x08,    			  	// Report Size (8)
		0x95, 0x01,    			  	// Report Count (1)
		0x91, 0x02,    			  	// Output (Variable)

		0x09, 0x78,    			// Usage (Effect Operation)
		0xA1, 0x02,    			  // Collection Datalink (logical)
			0x09, 0x79,    			// Usage (Op Effect Start)
			0x09, 0x7A,    			// Usage (Op Effect Start Solo)
			0x09, 0x7B,    			// Usage (Op Effect Stop)
			0x15, 0x01,    			  // Logical Minimum (1)
			0x25, 0x03,    			  // Logical Maximum (3)
			0x75, 0x08,    			  // Report Size (8)
			0x95, 0x01,    			  // Report Count (1)
			0x91, 0x00,    			  // Output
		0xC0,		        	  // End Collection Datalink (logical)

		0x09, 0x7C,         	// Usage (Loop Count)
		0x15, 0x00,         		// Logical Minimum (0)
		0x26, 0xFF, 0x00,    		// Logical Maximum (255)
		0x35, 0x00,         		// Physical Minimum 0
		0x46, 0xFF, 0x00,    		// Physical Maximum (255)
		0x91, 0x02,         		// Output (Variable)
	0xC0,		    	  // End Collection Datalink (logical)

	// PID Block Free Report
	0x09, 0x90,    		  // Usage (PID Block Free Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
		0x85, 0x0B,    			// Report ID (11)

		0x09, 0x22,    			// Usage (Effect Block Index)
		0x15, 0x01,    				// Logical Minimum (1)
		0x25, 0x28,    				// Logical Maximum (40)
		0x35, 0x01,    				// Physical Minimum (1)
		0x45, 0x28,    				// Physical Maximum (40)
		0x75, 0x08,    				// Report Size (8)
		0x95, 0x01,    				// Report Count (1)
		0x91, 0x02,    				// Output (Variable)
	0xC0,	    		  // End Collection Datalink (logical)

	// PID Device Control
	0x09, 0x96,    		  // Usage (PID Device Control)
	0xA1, 0x02,    		  // Collection Datalink (logical)
		0x85, 0x0C,    		  	// Report ID (12)
		0x09, 0x97,    			// Usage (DC Enable Actuators)
		0x09, 0x98,    			// Usage (DC Disable Actuators)
		0x09, 0x99,    			// Usage (DC Stop All Effects)
		0x09, 0x9A,    			// Usage (DC Device Reset)
		0x09, 0x9B,    			// Usage (DC Device Pause)
		0x09, 0x9C,    			// Usage (DC Device Continue)
		0x15, 0x01,    				// Logical Minimum (1)
		0x25, 0x06,    				// Logical Maximum (6)
		0x75, 0x01,    				// Report Size (1)
		0x95, 0x08,    				// Report Count (8)
		0x91, 0x02,    				// Output (Variable)
	0xC0,			      // End Collection Datalink (logical)

	// Device Gain Report
	0x09, 0x7D,    		  // Usage (Device Gain Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
	   0x85, 0x0D,         		// Report ID (13)

	   0x09, 0x7E,         		// Usage (Device Gain)
	   0x15, 0x00,         			// Logical Minimum (0)
	   0x26, 0xFF, 0x00,    		// Logical Maximum (255)
	   0x35, 0x00,         			// Physical Minimum (0)
	   0x46, 0x10, 0x27,    		// Physical Maximum (10000)
	   0x75, 0x08,         			// Report Size (8)
	   0x95, 0x01,         			// Report Count (1)
	   0x91, 0x02,         			// Output (Variable)
	0xC0,                 // End Collection Datalink (logical)

	//=========================================FeatureReport======================================//
	// Create New Effect Report
	0x09, 0xAB,    		  // Usage (Create New Effect Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
		0x85, 0x11,    			// Report ID (17)

		0x09, 0x25,    			// Usage (Effect Type)
		0xA1, 0x02,    			  // Collection Datalink (logical)
			0x09, 0x26,             // Usage (ET Constant Force)
			0x09, 0x27,             // Usage (ET Ramp)
			0x09, 0x30,             // Usage (ET Square)
			0x09, 0x31,             // Usage (ET Sine)
			0x09, 0x32,             // Usage (ET Triangle)
			0x09, 0x33,             // Usage (ET Sawtooth Up)
			0x09, 0x34,             // Usage (ET Sawtooth Down)
			0x09, 0x40,             // Usage (ET Spring)
			0x09, 0x41,             // Usage (ET Damper)
			0x09, 0x42,             // Usage (ET Inertia)
			0x09, 0x43,             // Usage (ET Friction)
			0x25, 0x0B,    				// Logical Maximum (11)
			0x15, 0x01,    				// Logical Minimum (1)
			0x35, 0x01,    				// Physical Minimum (1)
			0x45, 0x0B,    				// Physical Maximum (11)
			0x75, 0x08,    				// Report Size (8)
			0x95, 0x01,    				// Report Count (1)
			0xB1, 0x00,    				// Feature (Data)
		0xC0,         			  // End Collection Datalink (logical)

		0x05, 0x01,         	// Usage Page (Generic Desktop)
		0x09, 0x3B,         	// Usage Reserved (Byte count)
		0x15, 0x00,         		// Logical Minimum (0)
		0x26, 0xFF, 0x01,    		// Logical Maximum (511)
		0x35, 0x00,         		// Physical Minimum (0)
		0x46, 0xFF, 0x01,    		// Physical Maximum (511)
		0x75, 0x0A,         		// Report Size (10)
		0x95, 0x01,         		// Report Count (1)
		0xB1, 0x02,         		// Feature (Variable)
		0x75, 0x06,         		// Report Size (6)
		0xB1, 0x01,         		// Feature (Constant)
	0xC0,    			  // End Collection Datalink (logcial)

	// Block Load Report
	0x05, 0x0F,    		  // Usage Page (Physical Interface)
	0x09, 0x89,    		  // Usage (Block Load Report)
	0xA1, 0x02,      		// Collection Datalink (logical)
		0x85, 0x12,    			// Report ID (18)

		0x09, 0x22,    			// Usage (Effect Block Index)
		0x25, 0x28,    				// Logical Maximum (40)
		0x15, 0x01,    				// Logical Minimum (1)
		0x35, 0x01,    				// Physical Minimum (1)
		0x45, 0x28,    				// Physical Maximum (40)
		0x75, 0x08,    				// Report Size (8)
		0x95, 0x01,    				// Report Count (1)
		0xB1, 0x02,    				// Feature (Variable)

		0x09, 0x8B,    			// Usage (Block Load Status)
		0xA1, 0x02,    			  // Collection Datalink (logical)
			0x09, 0x8C,    			// Usage (Block Load Success)
			0x09, 0x8D,    			// Usage (Block Load Full)
			0x09, 0x8E,    			// Usage (Block Load Error)
			0x15, 0x01,    				// Logical Minimum (1)
			0x25, 0x03,    				// Logical Maximum (3)
			0x35, 0x01,    				// Physical Minimum (1)
			0x45, 0x03,    				// Physical Maximum (3)
			0x75, 0x08,    				// Report Size (8)
			0x95, 0x01,    				// Report Count (1)
			0xB1, 0x00,    				// Feature (Data)
		0xC0,                     // End Collection Datalink (logical)

		0x09, 0xAC,               // Usage (Pool available)
		0x15, 0x00,               		// Logical Minimum (0)
		0x27, 0xFF, 0xFF, 0x00, 0x00,   // Logical Maximum (65535)
		0x35, 0x00,                   	// Physical Minimum (0)
		0x47, 0xFF, 0xFF, 0x00, 0x00,   // Physical Maximum (65535)
		0x75, 0x10,                   	// Report Size (16)
		0x95, 0x01,                   	// Report Count (1)
		0xB1, 0x00,                   	// Feature (Data)
	0xC0,    			  // End Collection Datalink (logical)

	// PID Pool Report
	0x09, 0x7F,    		  // Usage (PID Pool Report)
	0xA1, 0x02,    		  // Collection Datalink (logical)
		0x85, 0x13,             // Report ID (19)

		0x09, 0x80,             // Usage (RAM Pool size)
		0x75, 0x10,                   // Report Size (16)
		0x95, 0x01,                   // Report Count (1)
		0x15, 0x00,                   // Logical Minimum (0)
		0x35, 0x00,                   // Physical Minimum (0)
		0x27, 0xFF, 0xFF, 0x00, 0x00, // Logical Maximum (65535)
		0x47, 0xFF, 0xFF, 0x00, 0x00, // Physical Maximum (65535)
		0xB1, 0x02,                   // Feature (Variable)

		0x09, 0x83,             // Usage (Simultaneous Effects Max)
		0x26, 0xFF, 0x00,             // Logical Maximum (255)
		0x46, 0xFF, 0x00,             // Physical Maximum (255)
		0x75, 0x08,                   // Report Size (8)
		0x95, 0x01,                   // Report Count (1)
		0xB1, 0x02,                   // Feature (Variable)

		0x09, 0xA9,             // Usage (Device Managed Pool)
		0x09, 0xAA,             // Usage (Shared Parameter Blocks)
		0x75, 0x01,                   // Report Size (1)
		0x95, 0x02,                   // Report Count (2)
		0x15, 0x00,                   // Logical Minimum (0)
		0x25, 0x01,                   // Logical Maximum (1)
		0x35, 0x00,                   // Physical Minimum (0)
		0x45, 0x01,                   // Physical Maximum (1)
		0xB1, 0x02,                   // Feature (Variable)
		0x75, 0x06,                   // Report Size (6)
		0x95, 0x01,                   // Report Count (1)
		0xB1, 0x03,                   // Feature (Constant, Variable)
	0xC0, 				  // End Collection Datalink (logical)
  0xC0    				// END_COLLECTION (Application)
  /* USER CODE END 0 */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t* report_buffer);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t* report_buffer)
{
  /* USER CODE BEGIN 6 */

  /* Start next USB packet transfer once data processing is completed */
  if (USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS) != (uint8_t)USBD_OK)
  {
    return -1;
  }

//  if(HAL_GPIO_ReadPin(LED_ERR_GPIO_Port, LED_ERR_Pin) == GPIO_PIN_SET){
// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
//  }
//  else {
// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
//  }

//  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report_buffer, 0x08);

//  memcpy(buffer, report_buffer, 0x08);
  if (report_buffer[0] == 0x01)  // Zakładamy, że 0x01 to komenda dla obrotu w lewo
  {
//	  tmc4671.setMoveAngleFlag(true, 90);
	  HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
  }
  else if (report_buffer[0] == 0x02)  // Zakładamy, że 0x02 to komenda dla obrotu w prawo
  {
//	  tmc4671.setMoveAngleFlag(true, -90);
	  HAL_GPIO_TogglePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin);
  }

//  buffer[0] = 0x01;
//  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buffer, 0x08);

  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
