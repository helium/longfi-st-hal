#include "lf_radio.h"

void radio_reset(void)
{
  //Radio Reset
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); 

  HAL_Delay(1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  HAL_Delay(6);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  HAL_Delay(1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
}

FlagStatus SpiGetFlag( uint16_t flag )
{
    FlagStatus bitstatus = RESET;

    // Check the status of the specified SPI flag
    if( ( hspi1.Instance->SR & flag ) != ( uint16_t )RESET )
    {
        // SPI_I2S_FLAG is set
        bitstatus = SET;
    }
    else
    {
        // SPI_I2S_FLAG is reset
        bitstatus = RESET;
    }
    // Return the SPI_I2S_FLAG status
    return  bitstatus;
}

uint16_t DiscoverySpiInOut(Spi_t *s, uint16_t outData){
    uint8_t rxData = 0;

    __HAL_SPI_ENABLE( &hspi1 );

    while( SpiGetFlag( SPI_FLAG_TXE ) == RESET );
    hspi1.Instance->DR = ( uint16_t ) ( outData & 0xFF );

    while( SpiGetFlag( SPI_FLAG_RXNE ) == RESET );
    rxData = ( uint16_t ) hspi1.Instance->DR;

    return( rxData );
}

void DiscoveryDelayMs(uint32_t ms){
    HAL_Delay(ms);
}

void DiscoveryGpioInit(Gpio_t *obj,
              PinNames pin,
              PinModes mode,
              PinConfigs config,
              PinTypes pin_type,
              uint32_t val){}



void DiscoveryGpioWrite(Gpio_t *obj, uint32_t val){
    if (val == 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    }
}

uint32_t DiscoveryGpioRead(Gpio_t *obj){
    return 0;
}

void DiscoveryGpioSetInterrupt(Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler){
}