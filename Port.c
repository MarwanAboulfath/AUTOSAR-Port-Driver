 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Marwan
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"



#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

/************************************
 *   Private Functions Prototypes   *
 ************************************/

/* PORT_PIN_MODE_DIO */
STATIC Std_ReturnType Port_SetModeDIO(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_UART */
STATIC Std_ReturnType Port_SetModeUART(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_JTAG */
STATIC Std_ReturnType Port_SetModeJTAG(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_SSI */
STATIC Std_ReturnType Port_SetModeSSI(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_I2C */
STATIC Std_ReturnType Port_SetModeI2C(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_CAN */
STATIC Std_ReturnType Port_SetModeCAN(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_PWM0 */
STATIC Std_ReturnType Port_SetModePWM0(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_PWM1 */
STATIC Std_ReturnType Port_SetModePWM1(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_ADC and PORT_PIN_MODE_ANALOG_COMPARATOR */
STATIC Std_ReturnType Port_SetModeANALOG(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_QEI */
STATIC Std_ReturnType Port_SetModeQEI(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_GPT */
STATIC Std_ReturnType Port_SetModeGPT(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_USB */
STATIC Std_ReturnType Port_SetModeUSB(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_NMI */
STATIC Std_ReturnType Port_SetModeNMI(
uint8 CurrentPin,
volatile uint32* port_pointer
);
/* PORT_PIN_MODE_CORE */
STATIC Std_ReturnType Port_SetModeCORE(
uint8 CurrentPin,
volatile uint32* port_pointer
);



STATIC const Port_ConfigPinType * Port_PortPins = NULL_PTR;             /* Store address of first element in structure to use for later in set_mode and Set_direction APIs */
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;                        /* Store Port state Initialized / Uninitialized */

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
)
{
    uint8 Port_CurrentPin = ZERO;                                 /* Variable to loop from pin 0 to pin 42 */
    volatile uint32 * Port_Ptr = NULL_PTR;                 /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (ConfigPtr == NULL_PTR)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
    }
    else
#endif

    {
    Port_PortPins = ConfigPtr->Port_pins_configurations;       /* Store the address of first element of configuration struct passed to Port_init function */
    for(Port_CurrentPin = ZERO; Port_CurrentPin < PORT_CONFIGURED_PINS; Port_CurrentPin++)
    {
            switch(Port_PortPins[Port_CurrentPin].port_id)
            {
                case  PORT_PORTA: Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                  break;
                case  PORT_PORTB: Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                  break;
                case  PORT_PORTC: Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                  break;
                case  PORT_PORTD: Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                  break;
                case  PORT_PORTE: Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                  break;
                case  PORT_PORTF: Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                  break;
            }

            if( ((Port_PortPins[Port_CurrentPin].port_id == PORT_PORTD) && (Port_PortPins[Port_CurrentPin].pin_id == PORT_PIN7)) || ((Port_PortPins[Port_CurrentPin].port_id == PORT_PORTF) && (Port_PortPins[Port_CurrentPin].pin_id == PORT_PIN0)) ) /* PD7 or PF0 */
                {
                    *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                }
                else if( (Port_PortPins[Port_CurrentPin].port_id == PORT_PORTC) && (Port_PortPins[Port_CurrentPin].pin_id <= PORT_PIN3) ) /* PC0 to PC3 */
                {
                    /* Do Nothing ...  this is the JTAG pins */
                }
                else
                {
                    /* Do Nothing ... No need to unlock the commit register for this pin */
                }

            if(Port_PortPins[Port_CurrentPin].pin_direction == PORT_PIN_IN)        /* Check for pin direction by accessing pin direction element in the struct */
            {
                /* Clear bit corresponding to pin location on the port to set as INPUT pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);

                if(Port_PortPins[Port_CurrentPin].pin_internal_resistor == INTERNAL_RESISTOR_PULL_UP)          /* Check for pin internal resistor by accessing pin internal resistor element in the struct */
                {
                    /* Set bit corresponding to pin location on the port to enable pull up resistance on pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);
                }
                else if (Port_PortPins[Port_CurrentPin].pin_internal_resistor == INTERNAL_RESISTOR_PULL_DOWN)  /* Check for pin internal resistor by accessing pin internal resistor element in the struct */
                {
                    /* Set bit corresponding to pin location on the port to enable pull down resistance on pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);
                }
                else    /* Pin Internal resistor == INTERNAL_RESISTOR_OFF */
                {
                    /* Clear bits corresponding to pin location on the port to disable pull up and pull down resistance on pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);
                }
            }
            else if (Port_PortPins[Port_CurrentPin].pin_direction == PORT_PIN_OUT)  /* Check for pin direction by accessing pin direction element in the struct */
            {
                /* Set bit corresponding to pin location on the port to set as OUTPUT pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);
                /* Check for pin initial value by accessing pin initial value element in the struct */
                if (Port_PortPins[Port_CurrentPin].pin_initial_value == PORT_PIN_LEVEL_LOW)
                {
                    /* Clear bit corresponding to pin location on the port to set Initial output value on pin to LOW */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);
                }
                else if (Port_PortPins[Port_CurrentPin].pin_initial_value == PORT_PIN_LEVEL_HIGH)
                {
                    /* Set bit corresponding to pin location on the port to set Initial output value on pin to HIGH */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET) , Port_PortPins[Port_CurrentPin].pin_id);
                }
                else
                {
                    /* Do nothing */
                }
            }
            else
            {
                /* Do nothing */
            }

            /////////////////////////*    SET MODE    *//////////////////////
            switch(Port_PortPins[Port_CurrentPin].pin_initial_mode)
            {
                case PORT_PIN_MODE_DIO:  Port_SetModeDIO(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_UART: Port_SetModeUART(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_JTAG: Port_SetModeJTAG(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_SSI0:
                case PORT_PIN_MODE_SSI1:
                case PORT_PIN_MODE_SSI2:
                case PORT_PIN_MODE_SSI3: Port_SetModeSSI(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_I2C:  Port_SetModeI2C(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_CAN:  Port_SetModeCAN(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_PWM0: Port_SetModePWM0(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_PWM1: Port_SetModePWM1(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_ADC:
                case PORT_PIN_MODE_ANALOG_COMPARATOR: Port_SetModeANALOG(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_QEI:  Port_SetModeQEI(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_ICU:
                case PORT_PIN_MODE_GPT:  Port_SetModeGPT(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_USB:  Port_SetModeUSB(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_NMI:  Port_SetModeNMI(Port_CurrentPin, Port_Ptr);
                                         break;
                case PORT_PIN_MODE_CORE: Port_SetModeCORE(Port_CurrentPin, Port_Ptr);
                                         break;
            }
    }
    Port_Status = PORT_INITIALIZED;
    }
}
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
)
{
    volatile uint32 * Port_Base_Ptr = NULL_PTR;                   /* point to the required Port Registers base address */
    boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if Port is initialized */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* Do nothing */
    }
    /* Check number of pin is within valid range (0-42) */
    if(Pin >= PORT_CONFIGURED_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* Do nothing */
    }
    /* Check if pin direction is set as unchangeable */
    if(Port_PortPins[Pin].pin_direction_changeable == FALSE)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /* Do nothing */
    }
#endif

    if (error == FALSE)
    {
        /* Select required Pin's Port address */
        switch(Port_PortPins[Pin].port_id)
        {
        case  0: Port_Base_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                break;
        case  1: Port_Base_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                break;
        case  2: Port_Base_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                break;
        case  3: Port_Base_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                break;
        case  4: Port_Base_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                break;
        case  5: Port_Base_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                break;
        }
        /* Check for pin direction */
        if (Direction == PORT_PIN_IN)
        {
            /* Clear bit corresponding to pin location on the port to set as INPUT pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Pin].pin_id);
        }
        else if(Direction == PORT_PIN_OUT)
        {
            /* Set bit corresponding to pin location on the port to set as OUTPUT pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Pin].pin_id);
        }
        else
        {
            /* Do nothing */
        }

    }
    else
    {
        /* Do nothing */
    }
}
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
)
{
    volatile uint32 * Port_Base_Address_Ptr = NULL_PTR;                   /* point to the required Port Registers base address */
    boolean error = FALSE;
    uint8 Port_CurrentPinNum = ZERO;                                 /* Variable to loop from pin 0 to pin 42 */

#if(PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if Port is initialized */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* Do nothing */
    }
#endif

    if (error == FALSE)
    {
        /* Loop on all port pins */
        for (Port_CurrentPinNum = ZERO; Port_CurrentPinNum < PORT_CONFIGURED_PINS; Port_CurrentPinNum++)
        {
            /* Check if pin direction is set to unchangeable */
           if (Port_PortPins[Port_CurrentPinNum].pin_direction_changeable == FALSE)
           {
               /* Select required Pin's Port address */
               switch(Port_PortPins[Port_CurrentPinNum].port_id)
               {
               case  0: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
               break;
               case  1: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
               break;
               case  2: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
               break;
               case  3: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
               break;
               case  4: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
               break;
               case  5: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
               break;
               }
               /* Check pin's direction */
               if (Port_PortPins[Port_CurrentPinNum].pin_direction == PORT_PIN_IN)
               {
                   /* Clear bit corresponding to pin location on the port to set as INPUT pin */
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Port_CurrentPinNum].pin_id);
               }
               else if(Port_PortPins[Port_CurrentPinNum].pin_direction == PORT_PIN_OUT)
               {
                   /* Set bit corresponding to pin location on the port to set as OUTPUT pin */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Port_CurrentPinNum].pin_id);
               }
               else
               {
                   /* Do nothing */
               }
           }
           else
           {
               /* Do nothing */
           }
        }
    }
    else
    {
        /* Do nothing */
    }
}
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
)
{
    boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if Port is initialized */
    if(Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* Do nothing */
    }
    /* Check if version info pointer is null pointer */
    if(versioninfo == NULL_PTR)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
        error = TRUE;
    }
    else
    {
        /* Do nothing */
    }
#endif

    if (error == FALSE)
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
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
)
{
    volatile uint32 * Port_Address_Ptr = NULL_PTR;                   /* point to the required Port Registers base address */
    boolean error = FALSE;
    Std_ReturnType Mode_Status = E_NOT_OK;                           /* Store status of mode change */

    #if(PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if Port is initialized */
        if (Port_Status == PORT_NOT_INITIALIZED)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
            error = TRUE;
        }
        else
        {
            /* Do nothing */
        }
        /* Check number of pin is within valid range (0-42) */
        if(Pin >= PORT_CONFIGURED_PINS)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
            error = TRUE;
        }
        else
        {
            /* Do nothing */
        }
        /* Check if pin mode is set as unchangeable */
        if(Port_PortPins[Pin].pin_mode_changeable == FALSE)
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
            error = TRUE;
        }
        else
        {
            /* Do nothing */
        }
    #endif

        if (error == FALSE)
        {
            /* Select pins port base address */
            switch(Port_PortPins[Pin].port_id)
            {
            case  PORT_PORTA: Port_Address_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                              break;
            case  PORT_PORTB: Port_Address_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                              break;
            case  PORT_PORTC: Port_Address_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                              break;
            case  PORT_PORTD: Port_Address_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                              break;
            case  PORT_PORTE: Port_Address_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                              break;
            case  PORT_PORTF: Port_Address_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                              break;
             }
            /* select new required mode */
            switch(Mode)
            {
            case PORT_PIN_MODE_DIO:  Mode_Status = Port_SetModeDIO(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_UART: Mode_Status = Port_SetModeUART(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_JTAG: Mode_Status = Port_SetModeJTAG(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_SSI0:
            case PORT_PIN_MODE_SSI1:
            case PORT_PIN_MODE_SSI2:
            case PORT_PIN_MODE_SSI3: Mode_Status = Port_SetModeSSI(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_I2C:  Mode_Status = Port_SetModeI2C(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_CAN:  Mode_Status = Port_SetModeCAN(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_PWM0: Mode_Status = Port_SetModePWM0(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_PWM1: Mode_Status = Port_SetModePWM1(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_ADC:
            case PORT_PIN_MODE_ANALOG_COMPARATOR: Mode_Status = Port_SetModeANALOG(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_QEI:  Mode_Status = Port_SetModeQEI(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_ICU:
            case PORT_PIN_MODE_GPT:  Mode_Status = Port_SetModeGPT(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_USB:  Mode_Status = Port_SetModeUSB(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_NMI:  Mode_Status = Port_SetModeNMI(Pin, Port_Address_Ptr);
                                     break;
            case PORT_PIN_MODE_CORE: Mode_Status = Port_SetModeCORE(Pin, Port_Address_Ptr);
                                     break;
            }

#if (PORT_DEV_ERROR_DETECT == STD_ON)
            if(Mode_Status == E_NOT_OK)
            {
                Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
            }
            else
            {
                /* Do nothing */
            }
#endif
        }
        else
        {
            /* Do nothing */
        }

}
#endif




/* PORT_PIN_MODE_DIO */
STATIC Std_ReturnType Port_SetModeDIO(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
        Std_ReturnType status = E_NOT_OK;
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Clear pin's corresponding bit in Alternate function select register (AFSEL) to turn off alternate function feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Clear the PMCx bits for this pin in Port control register (PCTL) */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
        return status;
}

/* PORT_PIN_MODE_UART */
STATIC Std_ReturnType Port_SetModeUART(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if (((Port_PortPins[CurrentPin].port_id == PORT_PORTA) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN1))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTB) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN1))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTC) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN4) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN7)))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN4) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN7)))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTE) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5)))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN1)))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (1) for this pin in Port control register (PCTL) to enable UART */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_UART_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_JTAG */
STATIC Std_ReturnType Port_SetModeJTAG(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if( (Port_PortPins[CurrentPin].port_id == PORT_PORTC) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN3) ) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
            status = E_OK;      /* Mode is set */
        }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_SSI */
STATIC Std_ReturnType Port_SetModeSSI(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
         /* Check for pin mode compatibility in case of SSI3 mode on port D */
         if ((Port_PortPins[CurrentPin].pin_initial_mode == PORT_PIN_MODE_SSI3) && ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN3)))
         {
             /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
             /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
             /* Set the PMCx bits to (1) for this pin in Port control register (PCTL) to enable SSI3 */
             *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
             *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_SSI3_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
             /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
             status = E_OK;     /* Mode is set */
         }
         /* Check for pin mode compatibility in case SSI(0-2) */
    else if (((Port_PortPins[CurrentPin].port_id == PORT_PORTA) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN2) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN5)))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTB) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN4) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN7)))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN3))\
         ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN3)))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (2) for this pin in Port control register (PCTL) to enable SSI */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_SSI_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_I2C */
STATIC Std_ReturnType Port_SetModeI2C(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
       if (((Port_PortPins[CurrentPin].port_id == PORT_PORTA) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN6) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN7)))\
       ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTB) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN2) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN3)))\
       ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN1))\
       ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTE) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5))))
       {
           /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
           /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
           /* Set the PMCx bits to (3) for this pin in Port control register (PCTL) to enable I2C */
           *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
           *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_I2C_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
           /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
           status = E_OK;      /* Mode is set */
       }
       else
       {
           /* Do nothing */
       }
       return status;
}

/* PORT_PIN_MODE_CAN */
STATIC Std_ReturnType Port_SetModeCAN(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if (((Port_PortPins[CurrentPin].port_id == PORT_PORTA) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN1))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTB) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTE) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5))))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (8) for this pin in Port control register (PCTL) to enable CAN */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_CAN_MAIN_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else if((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN0) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN3)))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (3) for this pin in Port control register (PCTL) to enable CAN */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_CAN_ADD_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_PWM0 */
STATIC Std_ReturnType Port_SetModePWM0(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if (((Port_PortPins[CurrentPin].port_id == PORT_PORTB) &&  (Port_PortPins[CurrentPin].pin_id >= PORT_PIN4))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTC) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && ((Port_PortPins[CurrentPin].pin_id <= PORT_PIN2) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN6)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTE) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) &&  (Port_PortPins[CurrentPin].pin_id == PORT_PIN2)))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (4) for this pin in Port control register (PCTL) to enable PWM0 */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_PWM0_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_PWM1 */
STATIC Std_ReturnType Port_SetModePWM1(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if (((Port_PortPins[CurrentPin].port_id == PORT_PORTA) &&  (Port_PortPins[CurrentPin].pin_id >= PORT_PIN6))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) &&  (Port_PortPins[CurrentPin].pin_id <= PORT_PIN1))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTE) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN0) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN4))))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (5) for this pin in Port control register (PCTL) to enable PWM1 */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_PWM1_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_ADC and PORT_PIN_MODE_ANALOG_COMPARATOR */
STATIC Std_ReturnType Port_SetModeANALOG(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if (((Port_PortPins[CurrentPin].port_id == PORT_PORTB) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTC) &&  (Port_PortPins[CurrentPin].pin_id >= PORT_PIN4))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) &&  (Port_PortPins[CurrentPin].pin_id <= PORT_PIN3))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTE) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN0) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN5))))
    {
        /* Clear pin's Corresponding bit in Digital enable register (GPIODEN) to disable digital I/O feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Clear pin's corresponding bit in Alternate function select register (AFSEL) to turn off alternate function feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (0) for this pin in Port control register (PCTL) to enable ADC */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's corresponding bit in Analog mode select register (AMSEL) to turn on analog feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_QEI */
STATIC Std_ReturnType Port_SetModeQEI(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if (((Port_PortPins[CurrentPin].port_id == PORT_PORTC) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN4) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN6)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN3) || (Port_PortPins[CurrentPin].pin_id >= PORT_PIN6)))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && ((Port_PortPins[CurrentPin].pin_id <= PORT_PIN1) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN4))))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (6) for this pin in Port control register (PCTL) to enable QEI */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_QEI_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_GPT */
STATIC Std_ReturnType Port_SetModeGPT(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if ((Port_PortPins[CurrentPin].port_id == PORT_PORTB)\
    ||  (Port_PortPins[CurrentPin].port_id == PORT_PORTC)\
    ||  (Port_PortPins[CurrentPin].port_id == PORT_PORTD)\
    || ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN0) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN4))))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (7) for this pin in Port control register (PCTL) to enable GPT */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_GPT_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_USB */
STATIC Std_ReturnType Port_SetModeUSB(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
        /* Check for pin mode compatibility */
        if (((Port_PortPins[CurrentPin].port_id == PORT_PORTC) && (Port_PortPins[CurrentPin].pin_id >= PORT_PIN6))\
        ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN2) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN3)))\
        ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) &&  (Port_PortPins[CurrentPin].pin_id == PORT_PIN4)))
        {
            /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
            /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
            /* Set the PMCx bits to (8) for this pin in Port control register (PCTL) to enable USB */
            *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
            *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_USB_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
            /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
            status = E_OK;      /* Mode is set */
        }
        else if(((Port_PortPins[CurrentPin].port_id == PORT_PORTB) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN1))\
             || ((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && ((Port_PortPins[CurrentPin].pin_id == PORT_PIN4) || (Port_PortPins[CurrentPin].pin_id == PORT_PIN5))))
        {
            /* Clear pin's Corresponding bit in Digital enable register (GPIODEN) to disable digital I/O feature */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
            /* Clear pin's corresponding bit in Alternate function select register (AFSEL) to turn off alternate function feature */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
            /* Set the PMCx bits to (0) for this pin in Port control register (PCTL) to enable USB analog in */
            *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
            /* Set pin's corresponding bit in Analog mode select register (AMSEL) to turn on analog feature */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
            status = E_OK;      /* Mode is set */
        }
        else
        {
            /* Do nothing */
        }
        return status;
}

/* PORT_PIN_MODE_NMI */
STATIC Std_ReturnType Port_SetModeNMI(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if (((Port_PortPins[CurrentPin].port_id == PORT_PORTD) && (Port_PortPins[CurrentPin].pin_id == PORT_PIN7))\
    ||  ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && (Port_PortPins[CurrentPin].pin_id == PORT_PIN0)))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (8) for this pin in Port control register (PCTL) to enable NMI */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_NMI_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}

/* PORT_PIN_MODE_CORE */
STATIC Std_ReturnType Port_SetModeCORE(
uint8 CurrentPin,
volatile uint32* port_pointer
)
{
    Std_ReturnType status = E_NOT_OK;
    /* Check for pin mode compatibility */
    if ((Port_PortPins[CurrentPin].port_id == PORT_PORTF) && ((Port_PortPins[CurrentPin].pin_id >= PORT_PIN1) && (Port_PortPins[CurrentPin].pin_id <= PORT_PIN3)))
    {
        /* Clear pin's corresponding bit in Analog mode select register (AMSEL) to turn off analog feature */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set pin's corresponding bit in Alternate function select register (AFSEL) to turn on alternate function feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        /* Set the PMCx bits to (14, 0xE) for this pin in Port control register (PCTL) to enable Core (Trace data and clock) */
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) &= ~(PCTL_MASK << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        *(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_CTL_REG_OFFSET) |=  (PCTL_CORE_VALUE << ((Port_PortPins[CurrentPin].pin_id)*PCTL_4BIT_SHIFT));
        /* Set pin's Corresponding bit in Digital enable register (GPIODEN) to enable digital I/O feature */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_pointer + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[CurrentPin].pin_id);
        status = E_OK;      /* Mode is set */
    }
    else
    {
        /* Do nothing */
    }
    return status;
}


