/*
 * Driver for decaWave DW1000 802.15.4 UWB radio chip.
 */

#ifndef __LIBDW1000_SPI_H__
#define __LIBDW1000_SPI_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "dw1000.h"
#include "libdw1000Types.h"

/**
 * Read from the dw1000 SPI interface
 */
void dwSpiRead(dwDevice_t *dev, uint8_t regid, uint32_t address, void* data, size_t length);
uint16_t dwSpiRead16(dwDevice_t *dev, uint8_t regid, uint32_t address);
uint32_t dwSpiRead32(dwDevice_t *dev, uint8_t regid, uint32_t address);

/**
 * Write to the dw1000 SPI interface
 */
void dwSpiWrite(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                 const void* data, size_t length);

void dwSpiWrite8(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint8_t data);

void dwSpiWrite32(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                  uint32_t data);

#endif //__LIBDW1000_SPI_H__
