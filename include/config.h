#pragma once

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>



//PIN and PORT for the TX and RX pin. Use for debug
#define pin_TX              GPIO2
#define port_TX                 GPIOA
#define pin_RX              GPIO3
#define port_RX                 GPIOA


