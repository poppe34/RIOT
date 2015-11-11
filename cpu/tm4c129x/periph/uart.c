/*
 * Copyright (C) 2015 Rakendra Thapa <rakendrathapa@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_lm4f120
 * @{
 *
 * @file        uart.c
 * @brief       Implementation of the low-level UART driver for the LM4F120
 *
 * @author      Rakendra Thapa <rakendrathapa@gmail.com>
 */

#include <stdint.h>
#include "board.h"
#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph/uart.h"
#include "periph_conf.h"

/* guard the file in case no UART is defined */
#if UART_0_EN || UART_1_EN || UART_2_EN

/**
 * @brief Struct holding the configuration data for a UART device
 * @{
 */
typedef struct {
    uart_rx_cb_t rx_cb;         /**< receive callback */
    void *arg;                  /**< callback argument */
} uart_conf_t;
/**@}*/

/**
 * @brief UART device configurations
 */
static uart_conf_t config[UART_NUMOF];

/**
 * The list of UART peripherals.
 */
static const unsigned long g_ulUARTPeriph[3] =
{
    SYSCTL_PERIPH_UART0,
    SYSCTL_PERIPH_UART1,
    SYSCTL_PERIPH_UART2
};

/**
 * The list of all possible base address of the console UART
 */
static const unsigned long g_ulUARTBase[3] =
{
    UART0_BASE,
    UART1_BASE,
    UART2_BASE
};

/**
 * The list of possible interrupts for the console UART.
 */
static const unsigned long g_ulUARTInt[3] =
{
    INT_UART0,
    INT_UART1,
    INT_UART2
};

/* TODO: needs to have the other UARTS available */
void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    uint32_t dev = 0;

    switch (uart) {
#if UART_0_EN
        case UART_0:
            dev = UART0_BASE;
            break;
#endif
#if UART_1_EN
        case UART_1:
            dev = UART1_BASE;
            break;
#endif
#if UART_2_EN
        case UART_2:
            dev = UART2_BASE;
            break;
#endif
        default:
            return;
    }

    for (size_t i = 0; i < len; i++) {
        ROM_UARTCharPut(dev, *data++);
    }
}

void uart_poweron(uart_t uart)
{
    ROM_UARTEnable(UART0_BASE);
}

void uart_poweroff(uart_t uart)
{
    ROM_UARTDisable(UART0_BASE);
}

int uart_init_blocking(uart_t uart, uint32_t baudrate)
{
    switch(uart){
#if UART_0_EN
        case UART_0: /* TODO: make these configuarble */
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
            MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
            MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
            MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


//            MAP_UARTDisable(UART0_BASE);
            MAP_UARTConfigSetExpClk(UART0_BASE, F_CPU, baudrate,
                    (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                     UART_CONFIG_WLEN_8));

            ROM_IntMasterEnable(); /* TODO: Relocate this */
            MAP_UARTEnable(UART0_BASE);
            break;
#endif
        }
    return 0;
}

/**
 * Configuring the UART console
 */
int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    /* Check the arguments */
    ASSERT(uart == 0); //TODO: Make sure this doesn't need adjusting if we add a second or third uart
    /* Check to make sure the UART peripheral is present */
    if(!MAP_SysCtlPeripheralPresent(SYSCTL_PERIPH_UART0)){
        return -1;
    }

    int res = uart_init_blocking(uart, baudrate);
    if(res < 0){
        return res;
    }

    /* save callbacks */
    config[uart].rx_cb = rx_cb;
    config[uart].arg = arg;

/*  ulBase = g_ulUARTBase[uart]; */
    switch (uart){
#if UART_0_EN
        case UART_0:
            NVIC_SetPriority(UART_0_IRQ_CHAN, UART_IRQ_PRIO);

//           MAP_UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
//           MAP_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
//           MAP_UARTFIFOEnable(UART0_BASE);

            /* Enable the UART interrupt */
            NVIC_EnableIRQ(UART_0_IRQ_CHAN);
            /* Enable RX interrupt */
            UART0_IM_R = UART_IM_RXIM | UART_IM_RTIM;/*TODO: check for a new variable UART0_IM_R I dont' like where it is defined */
            break;
#endif
#if UART_1_EN
        case UART_1:
            /* Uart 1 has way less that uart0 double check this */
            NVIC_SetPriority(UART_1_IRQ_CHAN, UART_IRQ_PRIO);
            /* Enable the UART interrupt */
            NVIC_EnableIRQ(UART_1_IRQ_CHAN);
            break;
#endif
    }
    return 0;
}

/**
 * The UART interrupt handler.
 * TODO: This needs to be made Generic
 */
void isr_uart0(void)
{
    unsigned long ulStatus;

    ulStatus = MAP_UARTIntStatus(UART0_BASE, true);
    MAP_UARTIntClear(UART0_BASE, ulStatus);

    /* Are we interrupted due to a recieved character */
    if(ulStatus & (UART_INT_RX | UART_INT_RT))
    {
        while(MAP_UARTCharsAvail(UART0_BASE))
        {
            char cChar;
            long lChar;
            lChar = MAP_UARTCharGet(UART0_BASE);
            cChar = (unsigned char)(lChar & 0xFF);
            config[UART_0].rx_cb(config[UART_0].arg, cChar);
        }
    }
    if (sched_context_switch_request) {
        thread_yield();
    }
}
#endif /*UART_0_EN || UART_1_EN*/
/** @} */
