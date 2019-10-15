#include "spi.h"
#include "gpio.h"

#include "longfi.h"
#include "board.h"
#include "longfi.h"
#include "radio/sx1276/sx1276.h"

void radio_reset(void);

FlagStatus SpiGetFlag( uint16_t flag );

uint16_t DiscoverySpiInOut(LF_Spi_t *s, uint16_t outData);

void DiscoveryDelayMs(uint32_t ms);

void DiscoveryGpioInit(LF_Gpio_t *obj,
              PinNames pin,
              PinModes mode,
              PinConfigs config,
              PinTypes pin_type,
              uint32_t val);

void DiscoveryGpioWrite(LF_Gpio_t *obj, uint32_t val);

uint32_t DiscoveryGpioRead(LF_Gpio_t *obj);

void DiscoveryGpioSetInterrupt(LF_Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler);

void enable_tcxo(LongFi_t * handle);