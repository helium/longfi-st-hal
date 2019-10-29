#include "spi.h"
#include "gpio.h"
#include "rng.h"

#include "longfi.h"
#include "board.h"
#include "radio/sx1276/sx1276.h"

void LongFiInit(LongFi_t *handle);

uint8_t BoardSpiInOut(LF_Spi_t *s, uint8_t outData);

void BoardSpiNss(bool sel);

void BoardReset(bool enable);

void BoardDelayMs(uint32_t ms);

uint32_t BoardGetRandomBits(uint8_t);

bool BoardBusyPinStatus(void);

uint8_t BoardReducePower(uint8_t);

uint8_t BoardSetBoardTcxo(bool enable);

void BoardSetAntennaPins(AntPinsMode_t mode, uint8_t power);