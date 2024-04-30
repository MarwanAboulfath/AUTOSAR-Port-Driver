/***********************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 * Created on: Jan 4, 2024
 * Author: Marwan Aboulfath
 **********************************************************************/

#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/****************************************************
 *              PortGeneral Container
 ***************************************************/

/* Pre-compile option for Development Error Detection */
#define PORT_DEV_ERROR_DETECT                (STD_ON)
/* Pre-compile option for Set Pin Direction API */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)
/* Pre-compile option for Set Pin Mode API */
#define PORT_SET_PIN_MODE_API                (STD_ON)
/* Pre-compile option for Port (module) Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/****************************************************
 *              PortConfigSet Container
 ***************************************************/

/* Number of the configured Dio Channels */
#define PORT_CONFIGURED_PINS              (43U)

/* Port IDs */
#define PORT_PORTA                               (0u)        /* Port A ID */
#define PORT_PORTB                               (1u)        /* Port B ID */
#define PORT_PORTC                               (2u)        /* Port C ID */
#define PORT_PORTD                               (3u)        /* Port D ID */
#define PORT_PORTE                               (4u)        /* Port E ID */
#define PORT_PORTF                               (5u)        /* Port F ID */

/* Pin IDs*/
#define PORT_PIN0                                (0u)        /* Pin 0 ID */
#define PORT_PIN1                                (1u)        /* Pin 1 ID */
#define PORT_PIN2                                (2u)        /* Pin 2 ID */
#define PORT_PIN3                                (3u)        /* Pin 3 ID */
#define PORT_PIN4                                (4u)        /* Pin 4 ID */
#define PORT_PIN5                                (5u)        /* Pin 5 ID */
#define PORT_PIN6                                (6u)        /* Pin 6 ID */
#define PORT_PIN7                                (7u)        /* Pin 7 ID */



#endif /* PORT_CFG_H_ */
