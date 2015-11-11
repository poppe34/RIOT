/*
 * Copyright (C) 2015 Matt Poppe <matt@poppe.me>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_tiva
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Matt Poppe <matt@poppe.me>
 *
 * @}
 */
#if (1) /*This is just to prevent compiling early on will remove */
#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph/gpio.h"
#include "periph_conf.h"

/**
 * @brief   Number of available external interrupt lines
 */
#define GPIO_ISR_CHAN_NUMOF             (16U)

/**
 * @brief   Datastructure to hold an interrupt context
 */
typedef struct {
    void (*cb)(void *arg);      /**< interrupt callback routine */
    void *arg;                  /**< optional argument */
} exti_ctx_t;

/**
 * @brief   Hold one callback function pointer for each interrupt line
 */
static exti_ctx_t exti_chan[GPIO_ISR_CHAN_NUMOF];

/**
 * @brief   Extract the port base address from the given pin identifier
 */
static inline uint32_t _port(gpio_t pin)
{
    return (uint32_t)(pin & ~(0x0ff));
}

/**
 * @brief   Extract the port number form the given identifier
 *
 * The port number is extracted by looking at bits 10, 11, 12, 13 of the base
 * register addresses.
 */
// static inline int _port_num(gpio_t pin)
// {
//     return ((pin >> 10) & 0x0f);
// }

/**
 * @brief   Extract the pin number from the last 4 bit of the pin identifier
 */
static inline uint32_t _pin_num(gpio_t pin)
{
    return (uint32_t)(pin & 0xff);
}

int gpio_init(gpio_t pin, gpio_dir_t dir, gpio_pp_t pullup)
{
    uint32_t port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    /* enable clock */

    /* configure pull register */

    /* set direction */

    return 0;
}

int gpio_init_int(gpio_t pin,
                   gpio_pp_t pullup, gpio_flank_t flank,
                   gpio_cb_t cb, void *arg)
{
    uint32_t pin_num = _pin_num(pin);
    uint32_t port_num = _port_num(pin);

    /* configure and save exti configuration struct */
    exti_chan[pin_num].cb = cb;
    exti_chan[pin_num].arg = arg;
    /* enable the SYSCFG clock */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    /* initialize pin as input */
    gpio_init(pin, GPIO_DIR_IN, pullup);
    /* enable global pin interrupt */
    if (pin_num < 5) {
        NVIC_EnableIRQ(EXTI0_IRQn + pin_num);
    }
    else if (pin_num < 10) {
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    else {
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
    /* configure the active edge(s) */
    switch (flank) {
        case GPIO_RISING:

            break;
        case GPIO_FALLING:

            break;
        case GPIO_BOTH:

            break;
    }
    /* enable specific pin as exti sources */
    SYSCFG->EXTICR[pin_num >> 2] &= ~(0xf << ((pin_num & 0x03) * 4));
    SYSCFG->EXTICR[pin_num >> 2] |= (port_num << ((pin_num & 0x03) * 4));
    /* clear any pending requests */
    EXTI->PR = (1 << pin_num);
    /* enable interrupt for EXTI line */
    EXTI->IMR |= (1 << pin_num);
    return 0;
}

void gpio_init_af(gpio_t pin, gpio_af_t af)
{
    uint32_t port = _port(pin);
    uint32_t pin_num = _pin_num(pin);


}

void gpio_irq_enable(gpio_t pin)
{
    EXTI->IMR |= (1 << _pin_num(pin));
}

void gpio_irq_disable(gpio_t pin)
{
    EXTI->IMR &= ~(1 << _pin_num(pin));
}

int gpio_read(gpio_t pin)
{
    uint32_t port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    return GPIOPinRead(port, pin_num);
}

void gpio_set(gpio_t pin)
{
    _port(pin)->BSRRL = (1 << _pin_num(pin));
}

void gpio_clear(gpio_t pin)
{
    _port(pin)->BSRRH = (1 << _pin_num(pin));
}

void gpio_toggle(gpio_t pin)
{
    if (gpio_read(pin)) {
        gpio_clear(pin);
    } else {
        gpio_set(pin);
    }
}

void gpio_write(gpio_t pin, int value)
{
    if (value) {
        gpio_set(pin);
    } else {
        gpio_clear(pin);
    }
}

void isr_exti_handler(void)
{
    for (int i = 0; i < GPIO_ISR_CHAN_NUMOF; i++) {
        if (EXTI->PR & (1 << i)) {
            EXTI->PR |= (1 << i);               /* clear by writing a 1 */
            exti_chan[i].cb(exti_chan[i].arg);
        }
    }
    if (sched_context_switch_request) {
        thread_yield();
    }
}
#endif
