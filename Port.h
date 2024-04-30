/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Marwan
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* PORT Module Id */
#define PORT_MODULE_ID    (124U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Dio Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/*Port pre-Compile configuration Header file*/
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Dio_Cfg.h and Dio.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Dio_Cfg.h and Dio.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Dio_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for PORT Init */
#define PORT_INIT_SID                       (uint8)0x00

/* Service ID for PORT set pin direction */
#define PORT_SET_PIN_DIRECTION_SID          (uint8)0x01

/* Service ID for PORT refresh port direction */
#define PORT_REFRESH_PORT_DIRECTION_SID     (uint8)0x02

/* Service ID for PORT GetVersionInfo */
#define PORT_GET_VERSION_INFO_SID           (uint8)0x03

/* Service ID for PORT set pin mode */
#define PORT_SET_PIN_MODE_SID               (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                    (uint8)0x0A
/* DET code to report Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE       (uint8)0x0B
/* DET code to report API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG                 (uint8)0x0C
/* DET code to report API Port_SetPinModeservice called with Invalid Pin mode */
#define PORT_E_PARAM_INVALID_MODE           (uint8)0x0D
/* DET code to report API Port_SetPinModeservice called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE            (uint8)0x0E
/* DET code to report API service called without module initialization */
#define PORT_E_UNINIT                       (uint8)0x0F
/* DET code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER                (uint8)0x10
/*
 * If Det is enabled and the Port Driver module has
detected an error, the desired functionality shall be skipped and the requested
service shall return without any action
 */
/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Type definition for Port_PinType used for the symbolic name of a Port Pin */
typedef uint8 Port_PinType;
/*
 * Type definition for Port_PortType used for the symbolic name of a Port
 * (Does not exist in AUTOSAR 4.0.3 PORT SWS Document)
 */
typedef uint8 Port_PortType;
/* Type definition for Port_PinModeType used with the function call to set pin mode */
typedef uint8 Port_PinModeType;
/* Type definition for Port_PinDirectionType used for for defining the direction of a Port Pin */
typedef enum
{
    PORT_PIN_IN,                        /* Sets port pin as input */
    PORT_PIN_OUT                        /* Sets port pin as output */
}Port_PinDirectionType;
/* Description: Enum to hold internal resistor type for PIN
 * (Does not exist in AUTOSAR 4.0.3 PORT SWS Document)
 */
typedef enum
{
    INTERNAL_RESISTOR_OFF,              /* Turn off internal resistor feature*/
    INTERNAL_RESISTOR_PULL_UP,          /* Turn ON Internal pull up resistor  */
    INTERNAL_RESISTOR_PULL_DOWN         /* Turn ON Internal pull down resistor  */
}Port_InternalResistorType;
/* Description: Enum to hold Pin level value
 * (Does not exist in AUTOSAR 4.0.3 PORT SWS Document)
 */
typedef enum
{
    PORT_PIN_LEVEL_LOW,
    PORT_PIN_LEVEL_HIGH
}Port_PortPinLevelValueType;
/* Description: Enum to hold Pin modes
 */
typedef enum
{
    PORT_PIN_MODE_DIO,                  /* DIO mode*/
    PORT_PIN_MODE_UART,                 /* UART mode */
    PORT_PIN_MODE_JTAG,                 /* JTAG mode */
    PORT_PIN_MODE_SSI0,                  /* SSI mode */
    PORT_PIN_MODE_SSI1,                  /* SSI mode */
    PORT_PIN_MODE_SSI2,                  /* SSI mode */
    PORT_PIN_MODE_SSI3,                  /* SSI mode */
    PORT_PIN_MODE_I2C,                  /*SSI mode */
    PORT_PIN_MODE_CAN,                  /*CAN mode */
    PORT_PIN_MODE_PWM0,                 /*PWM Module 0 mode */
    PORT_PIN_MODE_PWM1,                 /*PWM Module 1 mode */
    PORT_PIN_MODE_ADC,                  /*ADC mode */
    PORT_PIN_MODE_QEI,                  /*QEI mode */
    PORT_PIN_MODE_GPT,                  /*GPT mode */
    PORT_PIN_MODE_USB,                  /*USB mode */
    PORT_PIN_MODE_NMI,                  /*NMI mode */
    PORT_PIN_MODE_ANALOG_COMPARATOR,    /*Analog Comparator mode */
    PORT_PIN_MODE_CORE,                 /*CORE (Trace Clock/Data) mode */
    PORT_PIN_MODE_ICU,                  /*ICU mode */
}PortPinMode;
/* Description: Structure to configure each individual PIN:
 *  1. PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *  2. Number of the pin in the PORT.
 *  3. Direction of pin --> INPUT or OUTPUT
 *  4. Internal resistor --> Disable, Pull up or Pull down in case of input pin
 *  5. Initial value --> holds the initial value ONLY in case of output pin
 *  6. Pin mode --> holds pin mode to be configured (DIO, ADC, UART, etc)
 *  7. Pin mode changeable --> holds the boolean value that determines weather pin mode
 *     could be changed after initialization or not
 *  8. Pin direction changeable holds the boolean value that determines weather
 *     pin direction could be changed after initialization or not
 */
typedef struct
{
    /* PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5 */
    Port_PortType port_id;
    /* Number of the pin in the PORT */
    Port_PinType pin_id;
    /* Direction of pin --> INPUT or OUTPUT */
    Port_PinDirectionType pin_direction;
    /* Internal resistor --> Disable, Pull up or Pull down in case of input pin */
    Port_InternalResistorType pin_internal_resistor;
    /* Initial value --> holds the initial value ONLY in case of output pin */
    Port_PortPinLevelValueType pin_initial_value;
    /* Pin mode --> holds pin mode to be configured (DIO, ADC, UART, etc) */
    Port_PinModeType pin_initial_mode;
    /* Pin mode changeable --> holds the boolean value that determines weather pin mode
     * could be changed after initialization or not
     */
    boolean pin_mode_changeable;
    /* Pin direction changeable holds the boolean value that determines weather
     * pin direction could be changed after initialization or not
     */
    boolean pin_direction_changeable;
}Port_ConfigPinType;
/* Data Structure required for initializing the Port Driver */
typedef struct
{
    Port_ConfigPinType Port_pins_configurations[PORT_CONFIGURED_PINS];
}Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module,
*               Function to Setup the microcontroller's pins configuration:
*              - Setup the pin as defined mode
*              - Setup the direction of the pin
*              - Setup the internal resistor for i/p pin
*              - Setup the initial value for the o/p pin
************************************************************************************/
void Port_Init(
 const Port_ConfigType* ConfigPtr
);
/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number
*                  Direction - Port Pin Direction (input/output)
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction (If Pin direction is set as changeable)
************************************************************************************/
#if(PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(
 Port_PinType Pin,
 Port_PinDirectionType Direction
);
#endif
/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes all port pins direction
************************************************************************************/
void Port_RefreshPortDirection(
 void
);
/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo -  Pointer to where to store the version information of this module
* Return value: None
* Description: Returns the version information of this module
*              - Module ID
*              - Vendor ID
*              - Vendor specific version numbers
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(
 Std_VersionInfoType* versioninfo
);
#endif
/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin  - Port Pin ID number
*                  Mode - New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode (If Pin mode is set as changeable)
************************************************************************************/
#if(PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(
 Port_PinType Pin,
 Port_PinModeType Mode
);
#endif


/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/* PCTL register masks */
#define PCTL_4BIT_SHIFT                  4
#define PCTL_MASK                        0x0000000F
#define PCTL_UART_VALUE                  0x00000001
#define PCTL_SSI3_VALUE                  0x00000001
#define PCTL_SSI_VALUE                   0x00000002
#define PCTL_I2C_VALUE                   0x00000003
#define PCTL_CAN_ADD_VALUE               0x00000003
#define PCTL_CAN_MAIN_VALUE              0x00000008
#define PCTL_PWM0_VALUE                  0x00000004
#define PCTL_PWM1_VALUE                  0x00000005
#define PCTL_QEI_VALUE                   0x00000006
#define PCTL_GPT_VALUE                   0x00000007
#define PCTL_USB_VALUE                   0x00000008
#define PCTL_NMI_VALUE                   0x00000008
#define PCTL_CORE_VALUE                  0x0000000E

#define ZERO                             0

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
