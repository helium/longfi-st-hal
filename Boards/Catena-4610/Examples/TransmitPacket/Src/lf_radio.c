#include "lf_radio.h"

void BoardReset(bool enable)
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

uint8_t BoardSpiInOut(LF_Spi_t *s, uint8_t outData){
    uint8_t rxData = 0;

    __HAL_SPI_ENABLE( &hspi1 );

    while( SpiGetFlag( SPI_FLAG_TXE ) == RESET );
    hspi1.Instance->DR = ( uint8_t ) ( outData & 0xFF );

    while( SpiGetFlag( SPI_FLAG_RXNE ) == RESET );
    rxData = ( uint8_t ) hspi1.Instance->DR;

    return( rxData );
}

void BoardDelayMs(uint32_t ms){
    HAL_Delay(ms);
}

void BoardSpiNss(bool sel){
    if (!sel) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    }
}

uint32_t BoardGetRandomBits(uint8_t seed)
{
    return 0x1;
}

bool BoardBusyPinStatus(void)
{
    return true;
}

uint8_t BoardReducePower(uint8_t amount)
{
    return 0;
}

uint8_t BoardSetBoardTcxo(bool enable)
{
    if(enable)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }
    return 6;
}

void BoardSetAntennaPins(AntPinsMode_t mode, uint8_t power)
{
}