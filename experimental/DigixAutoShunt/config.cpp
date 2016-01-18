#include "config.h"


//---------------------------- Sd2Card.cpp

/* Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino Sd2Card Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
//--RM--#define USE_SPI_LIB
#include <Arduino.h>
//--RM--#include "Sd2Card.h"
//------------------------------------------------------------------------------
#ifndef SOFTWARE_SPI
#ifdef USE_SPI_LIB
#include <SPI.h>
//--RM--static SPISettings settings;
#endif
// functions for hardware SPI
/** Send a byte to the card */
static void spiSend(uint8_t b) {
#ifndef USE_SPI_LIB
  SPDR = b;
  while (!(SPSR & (1 << SPIF)))
    ;
#else
  SPI.transfer(b);
#endif
}
/** Receive a byte from the card */
static  uint8_t spiRec(void) {
#ifndef USE_SPI_LIB
  spiSend(0XFF);
  return SPDR;
#else
  return SPI.transfer(0xFF);
#endif
}
#else  // SOFTWARE_SPI
//------------------------------------------------------------------------------
/** nop to tune soft SPI timing */
#define nop asm volatile ("nop\n\t")
//------------------------------------------------------------------------------
/** Soft SPI receive */
uint8_t spiRec(void) {
  uint8_t data = 0;
  // no interrupts during byte receive - about 8 us
  cli();
  // output pin high - like sending 0XFF
  fastDigitalWrite(SPI_MOSI_PIN, HIGH);

  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, HIGH);

    // adjust so SCK is nice
    nop;
    nop;

    data <<= 1;

    if (fastDigitalRead(SPI_MISO_PIN)) data |= 1;

    fastDigitalWrite(SPI_SCK_PIN, LOW);
  }
  // enable interrupts
  sei();
  return data;
}
//------------------------------------------------------------------------------
/** Soft SPI send */
void spiSend(uint8_t data) {
  // no interrupts during byte send - about 8 us
  cli();
  for (uint8_t i = 0; i < 8; i++) {
    fastDigitalWrite(SPI_SCK_PIN, LOW);

    fastDigitalWrite(SPI_MOSI_PIN, data & 0X80);

    data <<= 1;

    fastDigitalWrite(SPI_SCK_PIN, HIGH);
  }
  // hold SCK high for a few ns
  nop;
  nop;
  nop;
  nop;

  fastDigitalWrite(SPI_SCK_PIN, LOW);
  // enable interrupts
  sei();
}
#endif  // SOFTWARE_SPI
//------------------------------------------------------------------------------
// send command and return error code.  Return zero for OK
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
  // end read if in partialBlockRead mode
  readEnd();

  // select card
  chipSelectLow();

  // wait up to 300 ms if busy
  waitNotBusy(300);

  // send command
  spiSend(cmd | 0x40);

  // send argument
  for (int8_t s = 24; s >= 0; s -= 8) spiSend(arg >> s);

  // send CRC
  uint8_t crc = 0XFF;
  if (cmd == CMD0) crc = 0X95;  // correct crc for CMD0 with arg 0
  if (cmd == CMD8) crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
  spiSend(crc);

  // wait for response
  for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++)
    ;
  return status_;
}
//------------------------------------------------------------------------------
/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t Sd2Card::cardSize(void) {
  csd_t csd;
  if (!readCSD(&csd)) return 0;
  if (csd.v1.csd_ver == 0) {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10)
                      | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
    uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
                          | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  } else if (csd.v2.csd_ver == 1) {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
                      | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  } else {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}
//------------------------------------------------------------------------------
static uint8_t chip_select_asserted = 0;

void Sd2Card::chipSelectHigh(void) {
  digitalWrite(chipSelectPin_, HIGH);
#ifdef USE_SPI_LIB
  if (chip_select_asserted) {
    chip_select_asserted = 0;
//--RM--    SPI.endTransaction();
  }
#endif
}
//------------------------------------------------------------------------------
void Sd2Card::chipSelectLow(void) {
#ifdef USE_SPI_LIB
  if (!chip_select_asserted) {
    chip_select_asserted = 1;
//--RM--    SPI.beginTransaction(settings);
  }
#endif
  digitalWrite(chipSelectPin_, LOW);
}
//------------------------------------------------------------------------------
/** Erase a range of blocks.
 *
 * \param[in] firstBlock The address of the first block in the range.
 * \param[in] lastBlock The address of the last block in the range.
 *
 * \note This function requests the SD card to do a flash erase for a
 * range of blocks.  The data on the card after an erase operation is
 * either 0 or 1, depends on the card vendor.  The card must support
 * single block erase.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
  if (!eraseSingleBlockEnable()) {
    error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
    goto fail;
  }
  if (type_ != SD_CARD_TYPE_SDHC) {
    firstBlock <<= 9;
    lastBlock <<= 9;
  }
  if (cardCommand(CMD32, firstBlock)
    || cardCommand(CMD33, lastBlock)
    || cardCommand(CMD38, 0)) {
      error(SD_CARD_ERROR_ERASE);
      goto fail;
  }
  if (!waitNotBusy(SD_ERASE_TIMEOUT)) {
    error(SD_CARD_ERROR_ERASE_TIMEOUT);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Determine if card supports single block erase.
 *
 * \return The value one, true, is returned if single block erase is supported.
 * The value zero, false, is returned if single block erase is not supported.
 */
uint8_t Sd2Card::eraseSingleBlockEnable(void) {
  csd_t csd;
  return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}
//------------------------------------------------------------------------------
/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  The reason for failure
 * can be determined by calling errorCode() and errorData().
 */
uint8_t Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {
  errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;
  chipSelectPin_ = chipSelectPin;
  // 16-bit init start time allows over a minute
  uint16_t t0 = (uint16_t)millis();
  uint32_t arg;

  // set pin modes
  pinMode(chipSelectPin_, OUTPUT);
  digitalWrite(chipSelectPin_, HIGH);
#ifndef USE_SPI_LIB
  pinMode(SPI_MISO_PIN, INPUT);
  pinMode(SPI_MOSI_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, OUTPUT);
#endif

#ifndef SOFTWARE_SPI
#ifndef USE_SPI_LIB
  // SS must be in output mode even it is not chip select
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH); // disable any SPI device using hardware SS pin
  // Enable SPI, Master, clock rate f_osc/128
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
  // clear double speed
  SPSR &= ~(1 << SPI2X);
#else // USE_SPI_LIB
  SPI.begin();
//--RM--  settings = SPISettings(250000, MSBFIRST, SPI_MODE0);
#endif // USE_SPI_LIB
#endif // SOFTWARE_SPI

  // must supply min of 74 clock cycles with CS high.
#ifdef USE_SPI_LIB
//--RM--  SPI.beginTransaction(settings);
#endif
  for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);
#ifdef USE_SPI_LIB
//--RM--  SPI.endTransaction();
#endif

  chipSelectLow();

  // command to go idle in SPI mode
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
    if (((uint16_t)(millis() - t0)) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_CMD0);
      goto fail;
    }
  }
  // check SD version
  if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
    type(SD_CARD_TYPE_SD1);
  } else {
    // only need last byte of r7 response
    for (uint8_t i = 0; i < 4; i++) status_ = spiRec();
    if (status_ != 0XAA) {
      error(SD_CARD_ERROR_CMD8);
      goto fail;
    }
    type(SD_CARD_TYPE_SD2);
  }
  // initialize card and send host supports SDHC if SD2
  arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;

  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    // check for timeout
    if (((uint16_t)(millis() - t0)) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_ACMD41);
      goto fail;
    }
  }
  // if SD2 read OCR register to check for SDHC card
  if (type() == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
      error(SD_CARD_ERROR_CMD58);
      goto fail;
    }
    if ((spiRec() & 0XC0) == 0XC0) type(SD_CARD_TYPE_SDHC);
    // discard rest of ocr - contains allowed voltage range
    for (uint8_t i = 0; i < 3; i++) spiRec();
  }
  chipSelectHigh();

#ifndef SOFTWARE_SPI
  return setSckRate(sckRateID);
#else  // SOFTWARE_SPI
  return true;
#endif  // SOFTWARE_SPI

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Enable or disable partial block reads.
 *
 * Enabling partial block reads improves performance by allowing a block
 * to be read over the SPI bus as several sub-blocks.  Errors may occur
 * if the time between reads is too long since the SD card may timeout.
 * The SPI SS line will be held low until the entire block is read or
 * readEnd() is called.
 *
 * Use this for applications like the Adafruit Wave Shield.
 *
 * \param[in] value The value TRUE (non-zero) or FALSE (zero).)
 */
void Sd2Card::partialBlockRead(uint8_t value) {
  readEnd();
  partialBlockRead_ = value;
}
//------------------------------------------------------------------------------
/**
 * Read a 512 byte block from an SD card device.
 *
 * \param[in] block Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.

 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
  return readData(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
 * Read part of a 512 byte block from an SD card.
 *
 * \param[in] block Logical block to be read.
 * \param[in] offset Number of bytes to skip at start of block
 * \param[out] dst Pointer to the location that will receive the data.
 * \param[in] count Number of bytes to read
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readData(uint32_t block,
        uint16_t offset, uint16_t count, uint8_t* dst) {
  uint16_t n;
  if (count == 0) return true;
  if ((count + offset) > 512) {
    goto fail;
  }
  if (!inBlock_ || block != block_ || offset < offset_) {
    block_ = block;
    // use address if not SDHC card
    if (type()!= SD_CARD_TYPE_SDHC) block <<= 9;
    if (cardCommand(CMD17, block)) {
      error(SD_CARD_ERROR_CMD17);
      goto fail;
    }
    if (!waitStartBlock()) {
      goto fail;
    }
    offset_ = 0;
    inBlock_ = 1;
  }

#ifdef OPTIMIZE_HARDWARE_SPI
  // start first spi transfer
  SPDR = 0XFF;

  // skip data before offset
  for (;offset_ < offset; offset_++) {
    while (!(SPSR & (1 << SPIF)))
      ;
    SPDR = 0XFF;
  }
  // transfer data
  n = count - 1;
  for (uint16_t i = 0; i < n; i++) {
    while (!(SPSR & (1 << SPIF)))
      ;
    dst[i] = SPDR;
    SPDR = 0XFF;
  }
  // wait for last byte
  while (!(SPSR & (1 << SPIF)))
    ;
  dst[n] = SPDR;

#else  // OPTIMIZE_HARDWARE_SPI

  // skip data before offset
  for (;offset_ < offset; offset_++) {
    spiRec();
  }
  // transfer data
  for (uint16_t i = 0; i < count; i++) {
    dst[i] = spiRec();
  }
#endif  // OPTIMIZE_HARDWARE_SPI

  offset_ += count;
  if (!partialBlockRead_ || offset_ >= 512) {
    // read rest of data, checksum and set chip select high
    readEnd();
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void Sd2Card::readEnd(void) {
  if (inBlock_) {
      // skip data and crc
#ifdef OPTIMIZE_HARDWARE_SPI
    // optimize skip for hardware
    SPDR = 0XFF;
    while (offset_++ < 513) {
      while (!(SPSR & (1 << SPIF)))
        ;
      SPDR = 0XFF;
    }
    // wait for last crc byte
    while (!(SPSR & (1 << SPIF)))
      ;
#else  // OPTIMIZE_HARDWARE_SPI
    while (offset_++ < 514) spiRec();
#endif  // OPTIMIZE_HARDWARE_SPI
    chipSelectHigh();
    inBlock_ = 0;
  }
}
//------------------------------------------------------------------------------
/** read CID or CSR register */
uint8_t Sd2Card::readRegister(uint8_t cmd, void* buf) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  if (cardCommand(cmd, 0)) {
    error(SD_CARD_ERROR_READ_REG);
    goto fail;
  }
  if (!waitStartBlock()) goto fail;
  // transfer data
  for (uint16_t i = 0; i < 16; i++) dst[i] = spiRec();
  spiRec();  // get first crc byte
  spiRec();  // get second crc byte
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Set the SPI clock rate.
 *
 * \param[in] sckRateID A value in the range [0, 6].
 *
 * The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
 * SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
 * for \a scsRateID = 6.
 *
 * \return The value one, true, is returned for success and the value zero,
 * false, is returned for an invalid value of \a sckRateID.
 */
uint8_t Sd2Card::setSckRate(uint8_t sckRateID) {
  if (sckRateID > 6) {
    error(SD_CARD_ERROR_SCK_RATE);
    return false;
  }
#ifndef USE_SPI_LIB
  // see avr processor datasheet for SPI register bit definitions
  if ((sckRateID & 1) || sckRateID == 6) {
    SPSR &= ~(1 << SPI2X);
  } else {
    SPSR |= (1 << SPI2X);
  }
  SPCR &= ~((1 <<SPR1) | (1 << SPR0));
  SPCR |= (sckRateID & 4 ? (1 << SPR1) : 0)
    | (sckRateID & 2 ? (1 << SPR0) : 0);
#else // USE_SPI_LIB
/*--RM--
  switch (sckRateID) {
    case 0:  settings = SPISettings(25000000, MSBFIRST, SPI_MODE0); break;
    case 1:  settings = SPISettings(4000000, MSBFIRST, SPI_MODE0); break;
    case 2:  settings = SPISettings(2000000, MSBFIRST, SPI_MODE0); break;
    case 3:  settings = SPISettings(1000000, MSBFIRST, SPI_MODE0); break;
    case 4:  settings = SPISettings(500000, MSBFIRST, SPI_MODE0); break;
    case 5:  settings = SPISettings(250000, MSBFIRST, SPI_MODE0); break;
    default: settings = SPISettings(125000, MSBFIRST, SPI_MODE0);
  }
*/
#endif // USE_SPI_LIB
  return true;
}
//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t Sd2Card::waitNotBusy(uint16_t timeoutMillis) {
  uint16_t t0 = millis();
  do {
    if (spiRec() == 0XFF) return true;
  }
  while (((uint16_t)millis() - t0) < timeoutMillis);
  return false;
}
//------------------------------------------------------------------------------
/** Wait for start block token */
uint8_t Sd2Card::waitStartBlock(void) {
  uint16_t t0 = millis();
  while ((status_ = spiRec()) == 0XFF) {
    if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK) {
    error(SD_CARD_ERROR_READ);
    goto fail;
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
#if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
#endif  // SD_PROTECT_BLOCK_ZERO

  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD24, blockNumber)) {
    error(SD_CARD_ERROR_CMD24);
    goto fail;
  }
  if (!writeData(DATA_START_BLOCK, src)) goto fail;

  // wait for flash programming to complete
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    error(SD_CARD_ERROR_WRITE_TIMEOUT);
    goto fail;
  }
  // response is r2 so get and check two bytes for nonzero
  if (cardCommand(CMD13, 0) || spiRec()) {
    error(SD_CARD_ERROR_WRITE_PROGRAMMING);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** Write one data block in a multiple block write sequence */
uint8_t Sd2Card::writeData(const uint8_t* src) {
  // wait for previous write to finish
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    error(SD_CARD_ERROR_WRITE_MULTIPLE);
    chipSelectHigh();
    return false;
  }
  return writeData(WRITE_MULTIPLE_TOKEN, src);
}
//------------------------------------------------------------------------------
// send one block of data for write block or write multiple blocks
uint8_t Sd2Card::writeData(uint8_t token, const uint8_t* src) {
#ifdef OPTIMIZE_HARDWARE_SPI

  // send data - optimized loop
  SPDR = token;

  // send two byte per iteration
  for (uint16_t i = 0; i < 512; i += 2) {
    while (!(SPSR & (1 << SPIF)))
      ;
    SPDR = src[i];
    while (!(SPSR & (1 << SPIF)))
      ;
    SPDR = src[i+1];
  }

  // wait for last data byte
  while (!(SPSR & (1 << SPIF)))
    ;

#else  // OPTIMIZE_HARDWARE_SPI
  spiSend(token);
  for (uint16_t i = 0; i < 512; i++) {
    spiSend(src[i]);
  }
#endif  // OPTIMIZE_HARDWARE_SPI
  spiSend(0xff);  // dummy crc
  spiSend(0xff);  // dummy crc

  status_ = spiRec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    error(SD_CARD_ERROR_WRITE);
    chipSelectHigh();
    return false;
  }
  return true;
}
//------------------------------------------------------------------------------
/** Start a write multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 * \param[in] eraseCount The number of blocks to be pre-erased.
 *
 * \note This function is used with writeData() and writeStop()
 * for optimized multiple block writes.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount) {
#if SD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
#endif  // SD_PROTECT_BLOCK_ZERO
  // send pre-erase count
  if (cardAcmd(ACMD23, eraseCount)) {
    error(SD_CARD_ERROR_ACMD23);
    goto fail;
  }
  // use address if not SDHC card
  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD25, blockNumber)) {
    error(SD_CARD_ERROR_CMD25);
    goto fail;
  }
  return true;

 fail:
  chipSelectHigh();
  return false;
}
//------------------------------------------------------------------------------
/** End a write multiple blocks sequence.
 *
* \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeStop(void) {
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
  spiSend(STOP_TRAN_TOKEN);
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) goto fail;
  chipSelectHigh();
  return true;

 fail:
  error(SD_CARD_ERROR_STOP_TRAN);
  chipSelectHigh();
  return false;
}


//---------------------------- SdFile.cpp

/* Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino SdFat Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SdFat Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
//--RM--#include "SdFat.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#include <Arduino.h>
//------------------------------------------------------------------------------
// callback function for date/time
void (*SdFile::dateTime_)(uint16_t* date, uint16_t* time) = NULL;

#if ALLOW_DEPRECATED_FUNCTIONS
// suppress cpplint warnings with NOLINT comment
void (*SdFile::oldDateTime_)(uint16_t& date, uint16_t& time) = NULL;  // NOLINT
#endif  // ALLOW_DEPRECATED_FUNCTIONS
//------------------------------------------------------------------------------
// add a cluster to a file
uint8_t SdFile::addCluster() {
  if (!vol_->allocContiguous(1, &curCluster_)) return false;

  // if first cluster of file link to directory entry
  if (firstCluster_ == 0) {
    firstCluster_ = curCluster_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  return true;
}
//------------------------------------------------------------------------------
// Add a cluster to a directory file and zero the cluster.
// return with first block of cluster in the cache
uint8_t SdFile::addDirCluster(void) {
  if (!addCluster()) return false;

  // zero data in cluster insure first cluster is in cache
  uint32_t block = vol_->clusterStartBlock(curCluster_);
  for (uint8_t i = vol_->blocksPerCluster_; i != 0; i--) {
    if (!SdVolume::cacheZeroBlock(block + i - 1)) return false;
  }
  // Increase directory file size by cluster size
  fileSize_ += 512UL << vol_->clusterSizeShift_;
  return true;
}
//------------------------------------------------------------------------------
// cache a file's directory entry
// return pointer to cached entry or null for failure
dir_t* SdFile::cacheDirEntry(uint8_t action) {
  if (!SdVolume::cacheRawBlock(dirBlock_, action)) return NULL;
  return SdVolume::cacheBuffer_.dir + dirIndex_;
}
//------------------------------------------------------------------------------
/**
 *  Close a file and force cached data and directory information
 *  to be written to the storage device.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include no file is open or an I/O error.
 */
uint8_t SdFile::close(void) {
  if (!sync())return false;
  type_ = FAT_FILE_TYPE_CLOSED;
  return true;
}
//------------------------------------------------------------------------------
/**
 * Check for contiguous file and return its raw block range.
 *
 * \param[out] bgnBlock the first block address for the file.
 * \param[out] endBlock the last  block address for the file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include file is not contiguous, file has zero length
 * or an I/O error occurred.
 */
uint8_t SdFile::contiguousRange(uint32_t* bgnBlock, uint32_t* endBlock) {
  // error if no blocks
  if (firstCluster_ == 0) return false;

  for (uint32_t c = firstCluster_; ; c++) {
    uint32_t next;
    if (!vol_->fatGet(c, &next)) return false;

    // check for contiguous
    if (next != (c + 1)) {
      // error if not end of chain
      if (!vol_->isEOC(next)) return false;
      *bgnBlock = vol_->clusterStartBlock(firstCluster_);
      *endBlock = vol_->clusterStartBlock(c)
                  + vol_->blocksPerCluster_ - 1;
      return true;
    }
  }
}
//------------------------------------------------------------------------------
/**
 * Create and open a new contiguous file of a specified size.
 *
 * \note This function only supports short DOS 8.3 names.
 * See open() for more information.
 *
 * \param[in] dirFile The directory where the file will be created.
 * \param[in] fileName A valid DOS 8.3 file name.
 * \param[in] size The desired file size.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include \a fileName contains
 * an invalid DOS 8.3 file name, the FAT volume has not been initialized,
 * a file is already open, the file already exists, the root
 * directory is full or an I/O error.
 *
 */
uint8_t SdFile::createContiguous(SdFile* dirFile,
        const char* fileName, uint32_t size) {
  // don't allow zero length file
  if (size == 0) return false;
  if (!open(dirFile, fileName, O_CREAT | O_EXCL | O_RDWR)) return false;

  // calculate number of clusters needed
  uint32_t count = ((size - 1) >> (vol_->clusterSizeShift_ + 9)) + 1;

  // allocate clusters
  if (!vol_->allocContiguous(count, &firstCluster_)) {
    remove();
    return false;
  }
  fileSize_ = size;

  // insure sync() will update dir entry
  flags_ |= F_FILE_DIR_DIRTY;
  return sync();
}
//------------------------------------------------------------------------------
/**
 * Return a files directory entry
 *
 * \param[out] dir Location for return of the files directory entry.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t SdFile::dirEntry(dir_t* dir) {
  // make sure fields on SD are correct
  if (!sync()) return false;

  // read entry
  dir_t* p = cacheDirEntry(SdVolume::CACHE_FOR_READ);
  if (!p) return false;

  // copy to caller's struct
  memcpy(dir, p, sizeof(dir_t));
  return true;
}
//------------------------------------------------------------------------------
/**
 * Format the name field of \a dir into the 13 byte array
 * \a name in standard 8.3 short name format.
 *
 * \param[in] dir The directory structure containing the name.
 * \param[out] name A 13 byte char array for the formatted name.
 */
void SdFile::dirName(const dir_t& dir, char* name) {
  uint8_t j = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ')continue;
    if (i == 8) name[j++] = '.';
    name[j++] = dir.name[i];
  }
  name[j] = 0;
}
//------------------------------------------------------------------------------
/** List directory contents to Serial.
 *
 * \param[in] flags The inclusive OR of
 *
 * LS_DATE - %Print file modification date
 *
 * LS_SIZE - %Print file size.
 *
 * LS_R - Recursive list of subdirectories.
 *
 * \param[in] indent Amount of space before file name. Used for recursive
 * list to indicate subdirectory level.
 */
void SdFile::ls(uint8_t flags, uint8_t indent) {
  dir_t* p;

  rewind();
  while ((p = readDirCache())) {
    // done if past last used entry
    if (p->name[0] == DIR_NAME_FREE) break;

    // skip deleted entry and entries for . and  ..
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;

    // only list subdirectories and files
    if (!DIR_IS_FILE_OR_SUBDIR(p)) continue;

    // print any indent spaces
    for (int8_t i = 0; i < indent; i++) Serial.print(' ');

    // print file name with possible blank fill
    printDirName(*p, flags & (LS_DATE | LS_SIZE) ? 14 : 0);

    // print modify date/time if requested
    if (flags & LS_DATE) {
       printFatDate(p->lastWriteDate);
       Serial.print(' ');
       printFatTime(p->lastWriteTime);
    }
    // print size if requested
    if (!DIR_IS_SUBDIR(p) && (flags & LS_SIZE)) {
      Serial.print(' ');
      Serial.print(p->fileSize);
    }
    Serial.println();

    // list subdirectory content if requested
    if ((flags & LS_R) && DIR_IS_SUBDIR(p)) {
      uint16_t index = curPosition()/32 - 1;
      SdFile s;
      if (s.open(this, index, O_READ)) s.ls(flags, indent + 2);
      seekSet(32 * (index + 1));
    }
  }
}
//------------------------------------------------------------------------------
// format directory name field from a 8.3 name string
uint8_t SdFile::make83Name(const char* str, uint8_t* name) {
  uint8_t c;
  uint8_t n = 7;  // max index for part before dot
  uint8_t i = 0;
  // blank fill name and extension
  while (i < 11) name[i++] = ' ';
  i = 0;
  while ((c = *str++) != '\0') {
    if (c == '.') {
      if (n == 10) return false;  // only one dot allowed
      n = 10;  // max index for full 8.3 name
      i = 8;   // place for extension
    } else {
      // illegal FAT characters
      uint8_t b;
#if defined(__AVR__)
      PGM_P p = PSTR("|<>^+=?/[];,*\"\\");
      while ((b = pgm_read_byte(p++))) if (b == c) return false;
#elif defined(__arm__)
      const uint8_t valid[] = "|<>^+=?/[];,*\"\\";
      const uint8_t *p = valid;
      while ((b = *p++)) if (b == c) return false;
#endif
      // check size and only allow ASCII printable characters
      if (i > n || c < 0X21 || c > 0X7E)return false;
      // only upper case allowed in 8.3 names - convert lower to upper
      name[i++] = c < 'a' || c > 'z' ?  c : c + ('A' - 'a');
    }
  }
  // must have a file name, extension is optional
  return name[0] != ' ';
}
//------------------------------------------------------------------------------
/** Make a new directory.
 *
 * \param[in] dir An open SdFat instance for the directory that will containing
 * the new directory.
 *
 * \param[in] dirName A valid 8.3 DOS name for the new directory.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include this SdFile is already open, \a dir is not a
 * directory, \a dirName is invalid or already exists in \a dir.
 */
uint8_t SdFile::makeDir(SdFile* dir, const char* dirName) {
  dir_t d;

  // create a normal file
  if (!open(dir, dirName, O_CREAT | O_EXCL | O_RDWR)) return false;

  // convert SdFile to directory
  flags_ = O_READ;
  type_ = FAT_FILE_TYPE_SUBDIR;

  // allocate and zero first cluster
  if (!addDirCluster())return false;

  // force entry to SD
  if (!sync()) return false;

  // cache entry - should already be in cache due to sync() call
  dir_t* p = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!p) return false;

  // change directory entry  attribute
  p->attributes = DIR_ATT_DIRECTORY;

  // make entry for '.'
  memcpy(&d, p, sizeof(d));
  for (uint8_t i = 1; i < 11; i++) d.name[i] = ' ';
  d.name[0] = '.';

  // cache block for '.'  and '..'
  uint32_t block = vol_->clusterStartBlock(firstCluster_);
  if (!SdVolume::cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE)) return false;

  // copy '.' to block
  memcpy(&SdVolume::cacheBuffer_.dir[0], &d, sizeof(d));

  // make entry for '..'
  d.name[1] = '.';
  if (dir->isRoot()) {
    d.firstClusterLow = 0;
    d.firstClusterHigh = 0;
  } else {
    d.firstClusterLow = dir->firstCluster_ & 0XFFFF;
    d.firstClusterHigh = dir->firstCluster_ >> 16;
  }
  // copy '..' to block
  memcpy(&SdVolume::cacheBuffer_.dir[1], &d, sizeof(d));

  // set position after '..'
  curPosition_ = 2 * sizeof(d);

  // write first block
  return SdVolume::cacheFlush();
}
//------------------------------------------------------------------------------
/**
 * Open a file or directory by name.
 *
 * \param[in] dirFile An open SdFat instance for the directory containing the
 * file to be opened.
 *
 * \param[in] fileName A valid 8.3 DOS name for a file to be opened.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of flags from the following list
 *
 * O_READ - Open for reading.
 *
 * O_RDONLY - Same as O_READ.
 *
 * O_WRITE - Open for writing.
 *
 * O_WRONLY - Same as O_WRITE.
 *
 * O_RDWR - Open for reading and writing.
 *
 * O_APPEND - If set, the file offset shall be set to the end of the
 * file prior to each write.
 *
 * O_CREAT - If the file exists, this flag has no effect except as noted
 * under O_EXCL below. Otherwise, the file shall be created
 *
 * O_EXCL - If O_CREAT and O_EXCL are set, open() shall fail if the file exists.
 *
 * O_SYNC - Call sync() after each write.  This flag should not be used with
 * write(uint8_t), write_P(PGM_P), writeln_P(PGM_P), or the Arduino Print class.
 * These functions do character at a time writes so sync() will be called
 * after each byte.
 *
 * O_TRUNC - If the file exists and is a regular file, and the file is
 * successfully opened and is not read only, its length shall be truncated to 0.
 *
 * \note Directory files must be opened read only.  Write and truncation is
 * not allowed for directory files.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include this SdFile is already open, \a difFile is not
 * a directory, \a fileName is invalid, the file does not exist
 * or can't be opened in the access mode specified by oflag.
 */
uint8_t SdFile::open(SdFile* dirFile, const char* fileName, uint8_t oflag) {
  uint8_t dname[11];
  dir_t* p;

  // error if already open
  if (isOpen())return false;

  if (!make83Name(fileName, dname)) return false;
  vol_ = dirFile->vol_;
  dirFile->rewind();

  // bool for empty entry found
  uint8_t emptyFound = false;

  // search for file
  while (dirFile->curPosition_ < dirFile->fileSize_) {
    uint8_t index = 0XF & (dirFile->curPosition_ >> 5);
    p = dirFile->readDirCache();
    if (p == NULL) return false;

    if (p->name[0] == DIR_NAME_FREE || p->name[0] == DIR_NAME_DELETED) {
      // remember first empty slot
      if (!emptyFound) {
        emptyFound = true;
        dirIndex_ = index;
        dirBlock_ = SdVolume::cacheBlockNumber_;
      }
      // done if no entries follow
      if (p->name[0] == DIR_NAME_FREE) break;
    } else if (!memcmp(dname, p->name, 11)) {
      // don't open existing file if O_CREAT and O_EXCL
      if ((oflag & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL)) return false;

      // open found file
      return openCachedEntry(0XF & index, oflag);
    }
  }
  // only create file if O_CREAT and O_WRITE
  if ((oflag & (O_CREAT | O_WRITE)) != (O_CREAT | O_WRITE)) return false;

  // cache found slot or add cluster if end of file
  if (emptyFound) {
    p = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    if (!p) return false;
  } else {
    if (dirFile->type_ == FAT_FILE_TYPE_ROOT16) return false;

    // add and zero cluster for dirFile - first cluster is in cache for write
    if (!dirFile->addDirCluster()) return false;

    // use first entry in cluster
    dirIndex_ = 0;
    p = SdVolume::cacheBuffer_.dir;
  }
  // initialize as empty file
  memset(p, 0, sizeof(dir_t));
  memcpy(p->name, dname, 11);

  // set timestamps
  if (dateTime_) {
    // call user function
    dateTime_(&p->creationDate, &p->creationTime);
  } else {
    // use default date/time
    p->creationDate = FAT_DEFAULT_DATE;
    p->creationTime = FAT_DEFAULT_TIME;
  }
  p->lastAccessDate = p->creationDate;
  p->lastWriteDate = p->creationDate;
  p->lastWriteTime = p->creationTime;

  // force write of entry to SD
  if (!SdVolume::cacheFlush()) return false;

  // open entry in cache
  return openCachedEntry(dirIndex_, oflag);
}
//------------------------------------------------------------------------------
/**
 * Open a file by index.
 *
 * \param[in] dirFile An open SdFat instance for the directory.
 *
 * \param[in] index The \a index of the directory entry for the file to be
 * opened.  The value for \a index is (directory file position)/32.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of flags O_READ, O_WRITE, O_TRUNC, and O_SYNC.
 *
 * See open() by fileName for definition of flags and return values.
 *
 */
uint8_t SdFile::open(SdFile* dirFile, uint16_t index, uint8_t oflag) {
  // error if already open
  if (isOpen())return false;

  // don't open existing file if O_CREAT and O_EXCL - user call error
  if ((oflag & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL)) return false;

  vol_ = dirFile->vol_;

  // seek to location of entry
  if (!dirFile->seekSet(32 * index)) return false;

  // read entry into cache
  dir_t* p = dirFile->readDirCache();
  if (p == NULL) return false;

  // error if empty slot or '.' or '..'
  if (p->name[0] == DIR_NAME_FREE ||
      p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') {
    return false;
  }
  // open cached entry
  return openCachedEntry(index & 0XF, oflag);
}
//------------------------------------------------------------------------------
// open a cached directory entry. Assumes vol_ is initializes
uint8_t SdFile::openCachedEntry(uint8_t dirIndex, uint8_t oflag) {
  // location of entry in cache
  dir_t* p = SdVolume::cacheBuffer_.dir + dirIndex;

  // write or truncate is an error for a directory or read-only file
  if (p->attributes & (DIR_ATT_READ_ONLY | DIR_ATT_DIRECTORY)) {
    if (oflag & (O_WRITE | O_TRUNC)) return false;
  }
  // remember location of directory entry on SD
  dirIndex_ = dirIndex;
  dirBlock_ = SdVolume::cacheBlockNumber_;

  // copy first cluster number for directory fields
  firstCluster_ = (uint32_t)p->firstClusterHigh << 16;
  firstCluster_ |= p->firstClusterLow;

  // make sure it is a normal file or subdirectory
  if (DIR_IS_FILE(p)) {
    fileSize_ = p->fileSize;
    type_ = FAT_FILE_TYPE_NORMAL;
  } else if (DIR_IS_SUBDIR(p)) {
    if (!vol_->chainSize(firstCluster_, &fileSize_)) return false;
    type_ = FAT_FILE_TYPE_SUBDIR;
  } else {
    return false;
  }
  // save open flags for read/write
  flags_ = oflag & (O_ACCMODE | O_SYNC | O_APPEND);

  // set to start of file
  curCluster_ = 0;
  curPosition_ = 0;

  // truncate file to zero length if requested
  if (oflag & O_TRUNC) return truncate(0);
  return true;
}
//------------------------------------------------------------------------------
/**
 * Open a volume's root directory.
 *
 * \param[in] vol The FAT volume containing the root directory to be opened.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the FAT volume has not been initialized
 * or it a FAT12 volume.
 */
uint8_t SdFile::openRoot(SdVolume* vol) {
  // error if file is already open
  if (isOpen()) return false;

  if (vol->fatType() == 16) {
    type_ = FAT_FILE_TYPE_ROOT16;
    firstCluster_ = 0;
    fileSize_ = 32 * vol->rootDirEntryCount();
  } else if (vol->fatType() == 32) {
    type_ = FAT_FILE_TYPE_ROOT32;
    firstCluster_ = vol->rootDirStart();
    if (!vol->chainSize(firstCluster_, &fileSize_)) return false;
  } else {
    // volume is not initialized or FAT12
    return false;
  }
  vol_ = vol;
  // read only
  flags_ = O_READ;

  // set to start of file
  curCluster_ = 0;
  curPosition_ = 0;

  // root has no directory entry
  dirBlock_ = 0;
  dirIndex_ = 0;
  return true;
}
//------------------------------------------------------------------------------
/** %Print the name field of a directory entry in 8.3 format to Serial.
 *
 * \param[in] dir The directory structure containing the name.
 * \param[in] width Blank fill name if length is less than \a width.
 */
void SdFile::printDirName(const dir_t& dir, uint8_t width) {
  uint8_t w = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ')continue;
    if (i == 8) {
      Serial.print('.');
      w++;
    }
    Serial.write(dir.name[i]);
    w++;
  }
  if (DIR_IS_SUBDIR(&dir)) {
    Serial.print('/');
    w++;
  }
  while (w < width) {
    Serial.print(' ');
    w++;
  }
}
//------------------------------------------------------------------------------
/** %Print a directory date field to Serial.
 *
 *  Format is yyyy-mm-dd.
 *
 * \param[in] fatDate The date field from a directory entry.
 */
void SdFile::printFatDate(uint16_t fatDate) {
  Serial.print(FAT_YEAR(fatDate));
  Serial.print('-');
  printTwoDigits(FAT_MONTH(fatDate));
  Serial.print('-');
  printTwoDigits(FAT_DAY(fatDate));
}
//------------------------------------------------------------------------------
/** %Print a directory time field to Serial.
 *
 * Format is hh:mm:ss.
 *
 * \param[in] fatTime The time field from a directory entry.
 */
void SdFile::printFatTime(uint16_t fatTime) {
  printTwoDigits(FAT_HOUR(fatTime));
  Serial.print(':');
  printTwoDigits(FAT_MINUTE(fatTime));
  Serial.print(':');
  printTwoDigits(FAT_SECOND(fatTime));
}
//------------------------------------------------------------------------------
/** %Print a value as two digits to Serial.
 *
 * \param[in] v Value to be printed, 0 <= \a v <= 99
 */
void SdFile::printTwoDigits(uint8_t v) {
  char str[3];
  str[0] = '0' + v/10;
  str[1] = '0' + v % 10;
  str[2] = 0;
  Serial.print(str);
}
//------------------------------------------------------------------------------
/**
 * Read data from a file starting at the current position.
 *
 * \param[out] buf Pointer to the location that will receive the data.
 *
 * \param[in] nbyte Maximum number of bytes to read.
 *
 * \return For success read() returns the number of bytes read.
 * A value less than \a nbyte, including zero, will be returned
 * if end of file is reached.
 * If an error occurs, read() returns -1.  Possible errors include
 * read() called before a file has been opened, corrupt file system
 * or an I/O error occurred.
 */
int16_t SdFile::read(void* buf, uint16_t nbyte) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);

  // error if not open or write only
  if (!isOpen() || !(flags_ & O_READ)) return -1;

  // max bytes left in file
  if (nbyte > (fileSize_ - curPosition_)) nbyte = fileSize_ - curPosition_;

  // amount left to read
  uint16_t toRead = nbyte;
  while (toRead > 0) {
    uint32_t block;  // raw device block number
    uint16_t offset = curPosition_ & 0X1FF;  // offset in block
    if (type_ == FAT_FILE_TYPE_ROOT16) {
      block = vol_->rootDirStart() + (curPosition_ >> 9);
    } else {
      uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
      if (offset == 0 && blockOfCluster == 0) {
        // start of new cluster
        if (curPosition_ == 0) {
          // use first cluster in file
          curCluster_ = firstCluster_;
        } else {
          // get next cluster from FAT
          if (!vol_->fatGet(curCluster_, &curCluster_)) return -1;
        }
      }
      block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    }
    uint16_t n = toRead;

    // amount to be read from current block
    if (n > (512 - offset)) n = 512 - offset;

    // no buffering needed if n == 512 or user requests no buffering
    if ((unbufferedRead() || n == 512) &&
      block != SdVolume::cacheBlockNumber_) {
      if (!vol_->readData(block, offset, n, dst)) return -1;
      dst += n;
    } else {
      // read block to cache and copy data to caller
      if (!SdVolume::cacheRawBlock(block, SdVolume::CACHE_FOR_READ)) return -1;
      uint8_t* src = SdVolume::cacheBuffer_.data + offset;
      uint8_t* end = src + n;
      while (src != end) *dst++ = *src++;
    }
    curPosition_ += n;
    toRead -= n;
  }
  return nbyte;
}
//------------------------------------------------------------------------------
/**
 * Read the next directory entry from a directory file.
 *
 * \param[out] dir The dir_t struct that will receive the data.
 *
 * \return For success readDir() returns the number of bytes read.
 * A value of zero will be returned if end of file is reached.
 * If an error occurs, readDir() returns -1.  Possible errors include
 * readDir() called before a directory has been opened, this is not
 * a directory file or an I/O error occurred.
 */
int8_t SdFile::readDir(dir_t* dir) {
  int8_t n;
  // if not a directory file or miss-positioned return an error
  if (!isDir() || (0X1F & curPosition_)) return -1;

  while ((n = read(dir, sizeof(dir_t))) == sizeof(dir_t)) {
    // last entry if DIR_NAME_FREE
    if (dir->name[0] == DIR_NAME_FREE) break;
    // skip empty entries and entry for .  and ..
    if (dir->name[0] == DIR_NAME_DELETED || dir->name[0] == '.') continue;
    // return if normal file or subdirectory
    if (DIR_IS_FILE_OR_SUBDIR(dir)) return n;
  }
  // error, end of file, or past last entry
  return n < 0 ? -1 : 0;
}
//------------------------------------------------------------------------------
// Read next directory entry into the cache
// Assumes file is correctly positioned
dir_t* SdFile::readDirCache(void) {
  // error if not directory
  if (!isDir()) return NULL;

  // index of entry in cache
  uint8_t i = (curPosition_ >> 5) & 0XF;

  // use read to locate and cache block
  if (read() < 0) return NULL;

  // advance to next entry
  curPosition_ += 31;

  // return pointer to entry
  return (SdVolume::cacheBuffer_.dir + i);
}
//------------------------------------------------------------------------------
/**
 * Remove a file.
 *
 * The directory entry and all data for the file are deleted.
 *
 * \note This function should not be used to delete the 8.3 version of a
 * file that has a long name. For example if a file has the long name
 * "New Text Document.txt" you should not delete the 8.3 name "NEWTEX~1.TXT".
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the file read-only, is a directory,
 * or an I/O error occurred.
 */
uint8_t SdFile::remove(void) {
  // free any clusters - will fail if read-only or directory
  if (!truncate(0)) return false;

  // cache directory entry
  dir_t* d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  // mark entry deleted
  d->name[0] = DIR_NAME_DELETED;

  // set this SdFile closed
  type_ = FAT_FILE_TYPE_CLOSED;

  // write entry to SD
  return SdVolume::cacheFlush();
}
//------------------------------------------------------------------------------
/**
 * Remove a file.
 *
 * The directory entry and all data for the file are deleted.
 *
 * \param[in] dirFile The directory that contains the file.
 * \param[in] fileName The name of the file to be removed.
 *
 * \note This function should not be used to delete the 8.3 version of a
 * file that has a long name. For example if a file has the long name
 * "New Text Document.txt" you should not delete the 8.3 name "NEWTEX~1.TXT".
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the file is a directory, is read only,
 * \a dirFile is not a directory, \a fileName is not found
 * or an I/O error occurred.
 */
uint8_t SdFile::remove(SdFile* dirFile, const char* fileName) {
  SdFile file;
  if (!file.open(dirFile, fileName, O_WRITE)) return false;
  return file.remove();
}
//------------------------------------------------------------------------------
/** Remove a directory file.
 *
 * The directory file will be removed only if it is empty and is not the
 * root directory.  rmDir() follows DOS and Windows and ignores the
 * read-only attribute for the directory.
 *
 * \note This function should not be used to delete the 8.3 version of a
 * directory that has a long name. For example if a directory has the
 * long name "New folder" you should not delete the 8.3 name "NEWFOL~1".
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the file is not a directory, is the root
 * directory, is not empty, or an I/O error occurred.
 */
uint8_t SdFile::rmDir(void) {
  // must be open subdirectory
  if (!isSubDir()) return false;

  rewind();

  // make sure directory is empty
  while (curPosition_ < fileSize_) {
    dir_t* p = readDirCache();
    if (p == NULL) return false;
    // done if past last used entry
    if (p->name[0] == DIR_NAME_FREE) break;
    // skip empty slot or '.' or '..'
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;
    // error not empty
    if (DIR_IS_FILE_OR_SUBDIR(p)) return false;
  }
  // convert empty directory to normal file for remove
  type_ = FAT_FILE_TYPE_NORMAL;
  flags_ |= O_WRITE;
  return remove();
}
//------------------------------------------------------------------------------
/** Recursively delete a directory and all contained files.
 *
 * This is like the Unix/Linux 'rm -rf *' if called with the root directory
 * hence the name.
 *
 * Warning - This will remove all contents of the directory including
 * subdirectories.  The directory will then be removed if it is not root.
 * The read-only attribute for files will be ignored.
 *
 * \note This function should not be used to delete the 8.3 version of
 * a directory that has a long name.  See remove() and rmDir().
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t SdFile::rmRfStar(void) {
  rewind();
  while (curPosition_ < fileSize_) {
    SdFile f;

    // remember position
    uint16_t index = curPosition_/32;

    dir_t* p = readDirCache();
    if (!p) return false;

    // done if past last entry
    if (p->name[0] == DIR_NAME_FREE) break;

    // skip empty slot or '.' or '..'
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;

    // skip if part of long file name or volume label in root
    if (!DIR_IS_FILE_OR_SUBDIR(p)) continue;

    if (!f.open(this, index, O_READ)) return false;
    if (f.isSubDir()) {
      // recursively delete
      if (!f.rmRfStar()) return false;
    } else {
      // ignore read-only
      f.flags_ |= O_WRITE;
      if (!f.remove()) return false;
    }
    // position to next entry if required
    if (curPosition_ != (32*(index + 1))) {
      if (!seekSet(32*(index + 1))) return false;
    }
  }
  // don't try to delete root
  if (isRoot()) return true;
  return rmDir();
}
//------------------------------------------------------------------------------
/**
 * Sets a file's position.
 *
 * \param[in] pos The new position in bytes from the beginning of the file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t SdFile::seekSet(uint32_t pos) {
  // error if file not open or seek past end of file
  if (!isOpen() || pos > fileSize_) return false;

  if (type_ == FAT_FILE_TYPE_ROOT16) {
    curPosition_ = pos;
    return true;
  }
  if (pos == 0) {
    // set position to start of file
    curCluster_ = 0;
    curPosition_ = 0;
    return true;
  }
  // calculate cluster index for cur and new position
  uint32_t nCur = (curPosition_ - 1) >> (vol_->clusterSizeShift_ + 9);
  uint32_t nNew = (pos - 1) >> (vol_->clusterSizeShift_ + 9);

  if (nNew < nCur || curPosition_ == 0) {
    // must follow chain from first cluster
    curCluster_ = firstCluster_;
  } else {
    // advance from curPosition
    nNew -= nCur;
  }
  while (nNew--) {
    if (!vol_->fatGet(curCluster_, &curCluster_)) return false;
  }
  curPosition_ = pos;
  return true;
}
//------------------------------------------------------------------------------
/**
 * The sync() call causes all modified data and directory fields
 * to be written to the storage device.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include a call to sync() before a file has been
 * opened or an I/O error.
 */
uint8_t SdFile::sync(void) {
  // only allow open files and directories
  if (!isOpen()) return false;

  if (flags_ & F_FILE_DIR_DIRTY) {
    dir_t* d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    if (!d) return false;

    // do not set filesize for dir files
    if (!isDir()) d->fileSize = fileSize_;

    // update first cluster fields
    d->firstClusterLow = firstCluster_ & 0XFFFF;
    d->firstClusterHigh = firstCluster_ >> 16;

    // set modify time if user supplied a callback date/time function
    if (dateTime_) {
      dateTime_(&d->lastWriteDate, &d->lastWriteTime);
      d->lastAccessDate = d->lastWriteDate;
    }
    // clear directory dirty
    flags_ &= ~F_FILE_DIR_DIRTY;
  }
  return SdVolume::cacheFlush();
}
//------------------------------------------------------------------------------
/**
 * Set a file's timestamps in its directory entry.
 *
 * \param[in] flags Values for \a flags are constructed by a bitwise-inclusive
 * OR of flags from the following list
 *
 * T_ACCESS - Set the file's last access date.
 *
 * T_CREATE - Set the file's creation date and time.
 *
 * T_WRITE - Set the file's last write/modification date and time.
 *
 * \param[in] year Valid range 1980 - 2107 inclusive.
 *
 * \param[in] month Valid range 1 - 12 inclusive.
 *
 * \param[in] day Valid range 1 - 31 inclusive.
 *
 * \param[in] hour Valid range 0 - 23 inclusive.
 *
 * \param[in] minute Valid range 0 - 59 inclusive.
 *
 * \param[in] second Valid range 0 - 59 inclusive
 *
 * \note It is possible to set an invalid date since there is no check for
 * the number of days in a month.
 *
 * \note
 * Modify and access timestamps may be overwritten if a date time callback
 * function has been set by dateTimeCallback().
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t SdFile::timestamp(uint8_t flags, uint16_t year, uint8_t month,
         uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  if (!isOpen()
    || year < 1980
    || year > 2107
    || month < 1
    || month > 12
    || day < 1
    || day > 31
    || hour > 23
    || minute > 59
    || second > 59) {
      return false;
  }
  dir_t* d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  uint16_t dirDate = FAT_DATE(year, month, day);
  uint16_t dirTime = FAT_TIME(hour, minute, second);
  if (flags & T_ACCESS) {
    d->lastAccessDate = dirDate;
  }
  if (flags & T_CREATE) {
    d->creationDate = dirDate;
    d->creationTime = dirTime;
    // seems to be units of 1/100 second not 1/10 as Microsoft states
    d->creationTimeTenths = second & 1 ? 100 : 0;
  }
  if (flags & T_WRITE) {
    d->lastWriteDate = dirDate;
    d->lastWriteTime = dirTime;
  }
  SdVolume::cacheSetDirty();
  return sync();
}
//------------------------------------------------------------------------------
/**
 * Truncate a file to a specified length.  The current file position
 * will be maintained if it is less than or equal to \a length otherwise
 * it will be set to end of file.
 *
 * \param[in] length The desired length for the file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include file is read only, file is a directory,
 * \a length is greater than the current file size or an I/O error occurs.
 */
uint8_t SdFile::truncate(uint32_t length) {
// error if not a normal file or read-only
  if (!isFile() || !(flags_ & O_WRITE)) return false;

  // error if length is greater than current size
  if (length > fileSize_) return false;

  // fileSize and length are zero - nothing to do
  if (fileSize_ == 0) return true;

  // remember position for seek after truncation
  uint32_t newPos = curPosition_ > length ? length : curPosition_;

  // position to last cluster in truncated file
  if (!seekSet(length)) return false;

  if (length == 0) {
    // free all clusters
    if (!vol_->freeChain(firstCluster_)) return false;
    firstCluster_ = 0;
  } else {
    uint32_t toFree;
    if (!vol_->fatGet(curCluster_, &toFree)) return false;

    if (!vol_->isEOC(toFree)) {
      // free extra clusters
      if (!vol_->freeChain(toFree)) return false;

      // current cluster is end of chain
      if (!vol_->fatPutEOC(curCluster_)) return false;
    }
  }
  fileSize_ = length;

  // need to update directory entry
  flags_ |= F_FILE_DIR_DIRTY;

  if (!sync()) return false;

  // set file to correct position
  return seekSet(newPos);
}
//------------------------------------------------------------------------------
/**
 * Write data to an open file.
 *
 * \note Data is moved to the cache but may not be written to the
 * storage device until sync() is called.
 *
 * \param[in] buf Pointer to the location of the data to be written.
 *
 * \param[in] nbyte Number of bytes to write.
 *
 * \return For success write() returns the number of bytes written, always
 * \a nbyte.  If an error occurs, write() returns -1.  Possible errors
 * include write() is called before a file has been opened, write is called
 * for a read-only file, device is full, a corrupt file system or an I/O error.
 *
 */
size_t SdFile::write(const void* buf, uint16_t nbyte) {
  // convert void* to uint8_t*  -  must be before goto statements
  const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);

  // number of bytes left to write  -  must be before goto statements
  uint16_t nToWrite = nbyte;

  // error if not a normal file or is read-only
  if (!isFile() || !(flags_ & O_WRITE)) goto writeErrorReturn;

  // seek to end of file if append flag
  if ((flags_ & O_APPEND) && curPosition_ != fileSize_) {
    if (!seekEnd()) goto writeErrorReturn;
  }

  while (nToWrite > 0) {
    uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
    uint16_t blockOffset = curPosition_ & 0X1FF;
    if (blockOfCluster == 0 && blockOffset == 0) {
      // start of new cluster
      if (curCluster_ == 0) {
        if (firstCluster_ == 0) {
          // allocate first cluster of file
          if (!addCluster()) goto writeErrorReturn;
        } else {
          curCluster_ = firstCluster_;
        }
      } else {
        uint32_t next;
        if (!vol_->fatGet(curCluster_, &next)) return false;
        if (vol_->isEOC(next)) {
          // add cluster if at end of chain
          if (!addCluster()) goto writeErrorReturn;
        } else {
          curCluster_ = next;
        }
      }
    }
    // max space in block
    uint16_t n = 512 - blockOffset;

    // lesser of space and amount to write
    if (n > nToWrite) n = nToWrite;

    // block for data write
    uint32_t block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    if (n == 512) {
      // full block - don't need to use cache
      // invalidate cache if block is in cache
      if (SdVolume::cacheBlockNumber_ == block) {
        SdVolume::cacheBlockNumber_ = 0XFFFFFFFF;
      }
      if (!vol_->writeBlock(block, src)) goto writeErrorReturn;
      src += 512;
    } else {
      if (blockOffset == 0 && curPosition_ >= fileSize_) {
        // start of new block don't need to read into cache
        if (!SdVolume::cacheFlush()) goto writeErrorReturn;
        SdVolume::cacheBlockNumber_ = block;
        SdVolume::cacheSetDirty();
      } else {
        // rewrite part of block
        if (!SdVolume::cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE)) {
          goto writeErrorReturn;
        }
      }
      uint8_t* dst = SdVolume::cacheBuffer_.data + blockOffset;
      uint8_t* end = dst + n;
      while (dst != end) *dst++ = *src++;
    }
    nToWrite -= n;
    curPosition_ += n;
  }
  if (curPosition_ > fileSize_) {
    // update fileSize and insure sync will update dir entry
    fileSize_ = curPosition_;
    flags_ |= F_FILE_DIR_DIRTY;
  } else if (dateTime_ && nbyte) {
    // insure sync will update modified date and time
    flags_ |= F_FILE_DIR_DIRTY;
  }

  if (flags_ & O_SYNC) {
    if (!sync()) goto writeErrorReturn;
  }
  return nbyte;

 writeErrorReturn:
  // return for write error
  //writeError = true;
  setWriteError();
  return 0;
}
//------------------------------------------------------------------------------
/**
 * Write a byte to a file. Required by the Arduino Print class.
 *
 * Use SdFile::writeError to check for errors.
 */
size_t SdFile::write(uint8_t b) {
  return write(&b, 1);
}
//------------------------------------------------------------------------------
/**
 * Write a string to a file. Used by the Arduino Print class.
 *
 * Use SdFile::writeError to check for errors.
 */
size_t SdFile::write(const char* str) {
  return write(str, strlen(str));
}
#ifdef __AVR__
//------------------------------------------------------------------------------
/**
 * Write a PROGMEM string to a file.
 *
 * Use SdFile::writeError to check for errors.
 */
void SdFile::write_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) write(c);
}
//------------------------------------------------------------------------------
/**
 * Write a PROGMEM string followed by CR/LF to a file.
 *
 * Use SdFile::writeError to check for errors.
 */
void SdFile::writeln_P(PGM_P str) {
  write_P(str);
  println();
}
#endif

//---------------------------- SdVolume.cpp

/* Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino SdFat Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SdFat Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
//--RM--#include "SdFat.h"
//------------------------------------------------------------------------------
// raw block cache
// init cacheBlockNumber_to invalid SD block number
uint32_t SdVolume::cacheBlockNumber_ = 0XFFFFFFFF;
cache_t  SdVolume::cacheBuffer_;     // 512 byte cache for Sd2Card
Sd2Card* SdVolume::sdCard_;          // pointer to SD card object
uint8_t  SdVolume::cacheDirty_ = 0;  // cacheFlush() will write block if true
uint32_t SdVolume::cacheMirrorBlock_ = 0;  // mirror  block for second FAT
//------------------------------------------------------------------------------
// find a contiguous group of clusters
uint8_t SdVolume::allocContiguous(uint32_t count, uint32_t* curCluster) {
  // start of group
  uint32_t bgnCluster;

  // flag to save place to start next search
  uint8_t setStart;

  // set search start cluster
  if (*curCluster) {
    // try to make file contiguous
    bgnCluster = *curCluster + 1;

    // don't save new start location
    setStart = false;
  } else {
    // start at likely place for free cluster
    bgnCluster = allocSearchStart_;

    // save next search start if one cluster
    setStart = 1 == count;
  }
  // end of group
  uint32_t endCluster = bgnCluster;

  // last cluster of FAT
  uint32_t fatEnd = clusterCount_ + 1;

  // search the FAT for free clusters
  for (uint32_t n = 0;; n++, endCluster++) {
    // can't find space checked all clusters
    if (n >= clusterCount_) return false;

    // past end - start from beginning of FAT
    if (endCluster > fatEnd) {
      bgnCluster = endCluster = 2;
    }
    uint32_t f;
    if (!fatGet(endCluster, &f)) return false;

    if (f != 0) {
      // cluster in use try next cluster as bgnCluster
      bgnCluster = endCluster + 1;
    } else if ((endCluster - bgnCluster + 1) == count) {
      // done - found space
      break;
    }
  }
  // mark end of chain
  if (!fatPutEOC(endCluster)) return false;

  // link clusters
  while (endCluster > bgnCluster) {
    if (!fatPut(endCluster - 1, endCluster)) return false;
    endCluster--;
  }
  if (*curCluster != 0) {
    // connect chains
    if (!fatPut(*curCluster, bgnCluster)) return false;
  }
  // return first cluster number to caller
  *curCluster = bgnCluster;

  // remember possible next free cluster
  if (setStart) allocSearchStart_ = bgnCluster + 1;

  return true;
}
//------------------------------------------------------------------------------
uint8_t SdVolume::cacheFlush(void) {
  if (cacheDirty_) {
    if (!sdCard_->writeBlock(cacheBlockNumber_, cacheBuffer_.data)) {
      return false;
    }
    // mirror FAT tables
    if (cacheMirrorBlock_) {
      if (!sdCard_->writeBlock(cacheMirrorBlock_, cacheBuffer_.data)) {
        return false;
      }
      cacheMirrorBlock_ = 0;
    }
    cacheDirty_ = 0;
  }
  return true;
}
//------------------------------------------------------------------------------
uint8_t SdVolume::cacheRawBlock(uint32_t blockNumber, uint8_t action) {
  if (cacheBlockNumber_ != blockNumber) {
    if (!cacheFlush()) return false;
    if (!sdCard_->readBlock(blockNumber, cacheBuffer_.data)) return false;
    cacheBlockNumber_ = blockNumber;
  }
  cacheDirty_ |= action;
  return true;
}
//------------------------------------------------------------------------------
// cache a zero block for blockNumber
uint8_t SdVolume::cacheZeroBlock(uint32_t blockNumber) {
  if (!cacheFlush()) return false;

  // loop take less flash than memset(cacheBuffer_.data, 0, 512);
  for (uint16_t i = 0; i < 512; i++) {
    cacheBuffer_.data[i] = 0;
  }
  cacheBlockNumber_ = blockNumber;
  cacheSetDirty();
  return true;
}
//------------------------------------------------------------------------------
// return the size in bytes of a cluster chain
uint8_t SdVolume::chainSize(uint32_t cluster, uint32_t* size) const {
  uint32_t s = 0;
  do {
    if (!fatGet(cluster, &cluster)) return false;
    s += 512UL << clusterSizeShift_;
  } while (!isEOC(cluster));
  *size = s;
  return true;
}
//------------------------------------------------------------------------------
// Fetch a FAT entry
uint8_t SdVolume::fatGet(uint32_t cluster, uint32_t* value) const {
  if (cluster > (clusterCount_ + 1)) return false;
  uint32_t lba = fatStartBlock_;
  lba += fatType_ == 16 ? cluster >> 8 : cluster >> 7;
  if (lba != cacheBlockNumber_) {
    if (!cacheRawBlock(lba, CACHE_FOR_READ)) return false;
  }
  if (fatType_ == 16) {
    *value = cacheBuffer_.fat16[cluster & 0XFF];
  } else {
    *value = cacheBuffer_.fat32[cluster & 0X7F] & FAT32MASK;
  }
  return true;
}
//------------------------------------------------------------------------------
// Store a FAT entry
uint8_t SdVolume::fatPut(uint32_t cluster, uint32_t value) {
  // error if reserved cluster
  if (cluster < 2) return false;

  // error if not in FAT
  if (cluster > (clusterCount_ + 1)) return false;

  // calculate block address for entry
  uint32_t lba = fatStartBlock_;
  lba += fatType_ == 16 ? cluster >> 8 : cluster >> 7;

  if (lba != cacheBlockNumber_) {
    if (!cacheRawBlock(lba, CACHE_FOR_READ)) return false;
  }
  // store entry
  if (fatType_ == 16) {
    cacheBuffer_.fat16[cluster & 0XFF] = value;
  } else {
    cacheBuffer_.fat32[cluster & 0X7F] = value;
  }
  cacheSetDirty();

  // mirror second FAT
  if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
  return true;
}
//------------------------------------------------------------------------------
// free a cluster chain
uint8_t SdVolume::freeChain(uint32_t cluster) {
  // clear free cluster location
  allocSearchStart_ = 2;

  do {
    uint32_t next;
    if (!fatGet(cluster, &next)) return false;

    // free cluster
    if (!fatPut(cluster, 0)) return false;

    cluster = next;
  } while (!isEOC(cluster));

  return true;
}
//------------------------------------------------------------------------------
/**
 * Initialize a FAT volume.
 *
 * \param[in] dev The SD card where the volume is located.
 *
 * \param[in] part The partition to be used.  Legal values for \a part are
 * 1-4 to use the corresponding partition on a device formatted with
 * a MBR, Master Boot Record, or zero if the device is formatted as
 * a super floppy with the FAT boot sector in block zero.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  Reasons for
 * failure include not finding a valid partition, not finding a valid
 * FAT file system in the specified partition or an I/O error.
 */
uint8_t SdVolume::init(Sd2Card* dev, uint8_t part) {
  uint32_t volumeStartBlock = 0;
  sdCard_ = dev;
  // if part == 0 assume super floppy with FAT boot sector in block zero
  // if part > 0 assume mbr volume with partition table
  if (part) {
    if (part > 4)return false;
    if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ)) return false;
    part_t* p = &cacheBuffer_.mbr.part[part-1];
    if ((p->boot & 0X7F) !=0  ||
      p->totalSectors < 100 ||
      p->firstSector == 0) {
      // not a valid partition
      return false;
    }
    volumeStartBlock = p->firstSector;
  }
  if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ)) return false;
  bpb_t* bpb = &cacheBuffer_.fbs.bpb;
  if (bpb->bytesPerSector != 512 ||
    bpb->fatCount == 0 ||
    bpb->reservedSectorCount == 0 ||
    bpb->sectorsPerCluster == 0) {
       // not valid FAT volume
      return false;
  }
  fatCount_ = bpb->fatCount;
  blocksPerCluster_ = bpb->sectorsPerCluster;

  // determine shift that is same as multiply by blocksPerCluster_
  clusterSizeShift_ = 0;
  while (blocksPerCluster_ != (1 << clusterSizeShift_)) {
    // error if not power of 2
    if (clusterSizeShift_++ > 7) return false;
  }
  blocksPerFat_ = bpb->sectorsPerFat16 ?
                    bpb->sectorsPerFat16 : bpb->sectorsPerFat32;

  fatStartBlock_ = volumeStartBlock + bpb->reservedSectorCount;

  // count for FAT16 zero for FAT32
  rootDirEntryCount_ = bpb->rootDirEntryCount;

  // directory start for FAT16 dataStart for FAT32
  rootDirStart_ = fatStartBlock_ + bpb->fatCount * blocksPerFat_;

  // data start for FAT16 and FAT32
  dataStartBlock_ = rootDirStart_ + ((32 * bpb->rootDirEntryCount + 511)/512);

  // total blocks for FAT16 or FAT32
  uint32_t totalBlocks = bpb->totalSectors16 ?
                           bpb->totalSectors16 : bpb->totalSectors32;
  // total data blocks
  clusterCount_ = totalBlocks - (dataStartBlock_ - volumeStartBlock);

  // divide by cluster size to get cluster count
  clusterCount_ >>= clusterSizeShift_;

  // FAT type is determined by cluster count
  if (clusterCount_ < 4085) {
    fatType_ = 12;
  } else if (clusterCount_ < 65525) {
    fatType_ = 16;
  } else {
    rootDirStart_ = bpb->fat32RootCluster;
    fatType_ = 32;
  }
  return true;
}


//---------------------------- File.cpp

/*

 SD - a slightly more friendly wrapper for sdfatlib

 This library aims to expose a subset of SD card functionality
 in the form of a higher level "wrapper" object.

 License: GNU General Public License V3
          (Because sdfatlib is licensed with this.)

 (C) Copyright 2010 SparkFun Electronics

 */

//--RM--#include <SD.h>

/* for debugging file open/close leaks
   uint8_t nfilecount=0;
*/

File::File(SdFile f, const char *n) {
  // oh man you are kidding me, new() doesnt exist? Ok we do it by hand!
  _file = (SdFile *)malloc(sizeof(SdFile)); 
  if (_file) {
    memcpy(_file, &f, sizeof(SdFile));
    
    strncpy(_name, n, 12);
    _name[12] = 0;
    
    /* for debugging file open/close leaks
       nfilecount++;
       Serial.print("Created \"");
       Serial.print(n);
       Serial.print("\": ");
       Serial.println(nfilecount, DEC);
    */
  }
}

File::File(void) {
  _file = 0;
  _name[0] = 0;
  //Serial.print("Created empty file object");
}

// returns a pointer to the file name
char *File::name(void) {
  return _name;
}

// a directory is a special type of file
boolean File::isDirectory(void) {
  return (_file && _file->isDir());
}


size_t File::write(uint8_t val) {
  return write(&val, 1);
}

size_t File::write(const uint8_t *buf, size_t size) {
  size_t t;
  if (!_file) {
    setWriteError();
    return 0;
  }
  _file->clearWriteError();
  t = _file->write(buf, size);
  if (_file->getWriteError()) {
    setWriteError();
    return 0;
  }
  return t;
}

int File::peek() {
  if (! _file) 
    return 0;

  int c = _file->read();
  if (c != -1) _file->seekCur(-1);
  return c;
}

int File::read() {
  if (_file) 
    return _file->read();
  return -1;
}

// buffered read for more efficient, high speed reading
int File::read(void *buf, uint16_t nbyte) {
  if (_file) 
    return _file->read(buf, nbyte);
  return 0;
}

int File::available() {
  if (! _file) return 0;

  uint32_t n = size() - position();

  return n > 0X7FFF ? 0X7FFF : n;
}

void File::flush() {
  if (_file)
    _file->sync();
}

boolean File::seek(uint32_t pos) {
  if (! _file) return false;

  return _file->seekSet(pos);
}

uint32_t File::position() {
  if (! _file) return -1;
  return _file->curPosition();
}

uint32_t File::size() {
  if (! _file) return 0;
  return _file->fileSize();
}

void File::close() {
  if (_file) {
    _file->close();
    free(_file); 
    _file = 0;

    /* for debugging file open/close leaks
    nfilecount--;
    Serial.print("Deleted ");
    Serial.println(nfilecount, DEC);
    */
  }
}

File::operator bool() {
  if (_file) 
    return  _file->isOpen();
  return false;
}


//---------------------------- SD.cpp

/*

 SD - a slightly more friendly wrapper for sdfatlib

 This library aims to expose a subset of SD card functionality
 in the form of a higher level "wrapper" object.

 License: GNU General Public License V3
          (Because sdfatlib is licensed with this.)

 (C) Copyright 2010 SparkFun Electronics


 This library provides four key benefits:

   * Including `SD.h` automatically creates a global
     `SD` object which can be interacted with in a similar
     manner to other standard global objects like `Serial` and `Ethernet`.

   * Boilerplate initialisation code is contained in one method named 
     `begin` and no further objects need to be created in order to access
     the SD card.

   * Calls to `open` can supply a full path name including parent 
     directories which simplifies interacting with files in subdirectories.

   * Utility methods are provided to determine whether a file exists
     and to create a directory heirarchy.


  Note however that not all functionality provided by the underlying
  sdfatlib library is exposed.

 */

/*

  Implementation Notes

  In order to handle multi-directory path traversal, functionality that 
  requires this ability is implemented as callback functions.

  Individual methods call the `walkPath` function which performs the actual
  directory traversal (swapping between two different directory/file handles
  along the way) and at each level calls the supplied callback function.

  Some types of functionality will take an action at each level (e.g. exists
  or make directory) which others will only take an action at the bottom
  level (e.g. open).

 */

//--RM--#include "SD.h"

// Used by `getNextPathComponent`
#define MAX_COMPONENT_LEN 12 // What is max length?
#define PATH_COMPONENT_BUFFER_LEN MAX_COMPONENT_LEN+1

bool getNextPathComponent(char *path, unsigned int *p_offset,
        char *buffer) {
  /*

    Parse individual path components from a path.

      e.g. after repeated calls '/foo/bar/baz' will be split
           into 'foo', 'bar', 'baz'.

    This is similar to `strtok()` but copies the component into the
    supplied buffer rather than modifying the original string.


    `buffer` needs to be PATH_COMPONENT_BUFFER_LEN in size.

    `p_offset` needs to point to an integer of the offset at
    which the previous path component finished.

    Returns `true` if more components remain.

    Returns `false` if this is the last component.
      (This means path ended with 'foo' or 'foo/'.)

   */

  // TODO: Have buffer local to this function, so we know it's the
  //       correct length?

  int bufferOffset = 0;

  int offset = *p_offset;

  // Skip root or other separator
  if (path[offset] == '/') {
    offset++;
  }
  
  // Copy the next next path segment
  while (bufferOffset < MAX_COMPONENT_LEN
   && (path[offset] != '/')
   && (path[offset] != '\0')) {
    buffer[bufferOffset++] = path[offset++];
  }

  buffer[bufferOffset] = '\0';

  // Skip trailing separator so we can determine if this
  // is the last component in the path or not.
  if (path[offset] == '/') {
    offset++;
  }

  *p_offset = offset;

  return (path[offset] != '\0');
}



boolean walkPath(char *filepath, SdFile& parentDir,
     boolean (*callback)(SdFile& parentDir,
             char *filePathComponent,
             boolean isLastComponent,
             void *object),
     void *object = NULL) {
  /*
     
     When given a file path (and parent directory--normally root),
     this function traverses the directories in the path and at each
     level calls the supplied callback function while also providing
     the supplied object for context if required.

       e.g. given the path '/foo/bar/baz'
            the callback would be called at the equivalent of
      '/foo', '/foo/bar' and '/foo/bar/baz'.

     The implementation swaps between two different directory/file
     handles as it traverses the directories and does not use recursion
     in an attempt to use memory efficiently.

     If a callback wishes to stop the directory traversal it should
     return false--in this case the function will stop the traversal,
     tidy up and return false.

     If a directory path doesn't exist at some point this function will
     also return false and not subsequently call the callback.

     If a directory path specified is complete, valid and the callback
     did not indicate the traversal should be interrupted then this
     function will return true.

   */


  SdFile subfile1;
  SdFile subfile2;

  char buffer[PATH_COMPONENT_BUFFER_LEN]; 

  unsigned int offset = 0;

  SdFile *p_parent;
  SdFile *p_child;

  SdFile *p_tmp_sdfile;  
  
  p_child = &subfile1;
  
  p_parent = &parentDir;

  while (true) {

    boolean moreComponents = getNextPathComponent(filepath, &offset, buffer);

    boolean shouldContinue = callback((*p_parent), buffer, !moreComponents, object);

    if (!shouldContinue) {
      // TODO: Don't repeat this code?
      // If it's one we've created then we
      // don't need the parent handle anymore.
      if (p_parent != &parentDir) {
        (*p_parent).close();
      }
      return false;
    }
    
    if (!moreComponents) {
      break;
    }
    
    boolean exists = (*p_child).open(*p_parent, buffer, O_RDONLY);

    // If it's one we've created then we
    // don't need the parent handle anymore.
    if (p_parent != &parentDir) {
      (*p_parent).close();
    }
    
    // Handle case when it doesn't exist and we can't continue...
    if (exists) {
      // We alternate between two file handles as we go down
      // the path.
      if (p_parent == &parentDir) {
        p_parent = &subfile2;
      }

      p_tmp_sdfile = p_parent;
      p_parent = p_child;
      p_child = p_tmp_sdfile;
    } else {
      return false;
    }
  }
  
  if (p_parent != &parentDir) {
    (*p_parent).close(); // TODO: Return/ handle different?
  }

  return true;
}



/*

   The callbacks used to implement various functionality follow.

   Each callback is supplied with a parent directory handle,
   character string with the name of the current file path component,
   a flag indicating if this component is the last in the path and
   a pointer to an arbitrary object used for context.

 */

boolean callback_pathExists(SdFile& parentDir, char *filePathComponent, 
          boolean isLastComponent, void *object) {
  /*

    Callback used to determine if a file/directory exists in parent
    directory.

    Returns true if file path exists.

  */
  SdFile child;

  boolean exists = child.open(parentDir, filePathComponent, O_RDONLY);
  
  if (exists) {
     child.close(); 
  }
  
  return exists;
}



boolean callback_makeDirPath(SdFile& parentDir, char *filePathComponent, 
           boolean isLastComponent, void *object) {
  /*

    Callback used to create a directory in the parent directory if
    it does not already exist.

    Returns true if a directory was created or it already existed.

  */
  boolean result = false;
  SdFile child;
  
  result = callback_pathExists(parentDir, filePathComponent, isLastComponent, object);
  if (!result) {
    result = child.makeDir(parentDir, filePathComponent);
  } 
  
  return result;
}


  /*

boolean callback_openPath(SdFile& parentDir, char *filePathComponent, 
        boolean isLastComponent, void *object) {

    Callback used to open a file specified by a filepath that may
    specify one or more directories above it.

    Expects the context object to be an instance of `SDClass` and
    will use the `file` property of the instance to open the requested
    file/directory with the associated file open mode property.

    Always returns true if the directory traversal hasn't reached the
    bottom of the directory heirarchy.

    Returns false once the file has been opened--to prevent the traversal
    from descending further. (This may be unnecessary.)

  if (isLastComponent) {
    SDClass *p_SD = static_cast<SDClass*>(object);
    p_SD->file.open(parentDir, filePathComponent, p_SD->fileOpenMode);
    if (p_SD->fileOpenMode == FILE_WRITE) {
      p_SD->file.seekSet(p_SD->file.fileSize());
    }
    // TODO: Return file open result?
    return false;
  }
  return true;
}
  */



boolean callback_remove(SdFile& parentDir, char *filePathComponent, 
      boolean isLastComponent, void *object) {
  if (isLastComponent) {
    return SdFile::remove(parentDir, filePathComponent);
  }
  return true;
}

boolean callback_rmdir(SdFile& parentDir, char *filePathComponent, 
      boolean isLastComponent, void *object) {
  if (isLastComponent) {
    SdFile f;
    if (!f.open(parentDir, filePathComponent, O_READ)) return false;
    return f.rmDir();
  }
  return true;
}



/* Implementation of class used to create `SDCard` object. */



boolean SDClass::begin(uint8_t csPin) {
  /*

    Performs the initialisation required by the sdfatlib library.

    Return true if initialization succeeds, false otherwise.

   */
  return card.init(SPI_HALF_SPEED, csPin) &&
         volume.init(card) &&
         root.openRoot(volume);
}



// this little helper is used to traverse paths
SdFile SDClass::getParentDir(const char *filepath, int *index) {
  // get parent directory
  SdFile d1 = root; // start with the mostparent, root!
  SdFile d2;

  // we'll use the pointers to swap between the two objects
  SdFile *parent = &d1;
  SdFile *subdir = &d2;
  
  const char *origpath = filepath;

  while (strchr(filepath, '/')) {

    // get rid of leading /'s
    if (filepath[0] == '/') {
      filepath++;
      continue;
    }
    
    if (! strchr(filepath, '/')) {
      // it was in the root directory, so leave now
      break;
    }

    // extract just the name of the next subdirectory
    uint8_t idx = strchr(filepath, '/') - filepath;
    if (idx > 12)
      idx = 12;    // dont let them specify long names
    char subdirname[13];
    strncpy(subdirname, filepath, idx);
    subdirname[idx] = 0;

    // close the subdir (we reuse them) if open
    subdir->close();
    if (! subdir->open(parent, subdirname, O_READ)) {
      // failed to open one of the subdirectories
      return SdFile();
    }
    // move forward to the next subdirectory
    filepath += idx;

    // we reuse the objects, close it.
    parent->close();

    // swap the pointers
    SdFile *t = parent;
    parent = subdir;
    subdir = t;
  }

  *index = (int)(filepath - origpath);
  // parent is now the parent diretory of the file!
  return *parent;
}


File SDClass::open(const char *filepath, uint8_t mode) {
  /*

     Open the supplied file path for reading or writing.

     The file content can be accessed via the `file` property of
     the `SDClass` object--this property is currently
     a standard `SdFile` object from `sdfatlib`.

     Defaults to read only.

     If `write` is true, default action (when `append` is true) is to
     append data to the end of the file.

     If `append` is false then the file will be truncated first.

     If the file does not exist and it is opened for writing the file
     will be created.

     An attempt to open a file for reading that does not exist is an
     error.

   */

  int pathidx;

  // do the interative search
  SdFile parentdir = getParentDir(filepath, &pathidx);
  // no more subdirs!

  filepath += pathidx;

  if (! filepath[0]) {
    // it was the directory itself!
    return File(parentdir, "/");
  }

  // Open the file itself
  SdFile file;

  // failed to open a subdir!
  if (!parentdir.isOpen())
    return File();

  // there is a special case for the Root directory since its a static dir
  if (parentdir.isRoot()) {
    if ( ! file.open(SD.root, filepath, mode)) {
      // failed to open the file :(
      return File();
    }
    // dont close the root!
  } else {
    if ( ! file.open(parentdir, filepath, mode)) {
      return File();
    }
    // close the parent
    parentdir.close();
  }

  if (mode & (O_APPEND | O_WRITE)) 
    file.seekSet(file.fileSize());
  return File(file, filepath);
}


/*
File SDClass::open(char *filepath, uint8_t mode) {
  //

     Open the supplied file path for reading or writing.

     The file content can be accessed via the `file` property of
     the `SDClass` object--this property is currently
     a standard `SdFile` object from `sdfatlib`.

     Defaults to read only.

     If `write` is true, default action (when `append` is true) is to
     append data to the end of the file.

     If `append` is false then the file will be truncated first.

     If the file does not exist and it is opened for writing the file
     will be created.

     An attempt to open a file for reading that does not exist is an
     error.

   //

  // TODO: Allow for read&write? (Possibly not, as it requires seek.)

  fileOpenMode = mode;
  walkPath(filepath, root, callback_openPath, this);

  return File();

}
*/


//boolean SDClass::close() {
//  /*
//
//    Closes the file opened by the `open` method.
//
//   */
//  file.close();
//}


boolean SDClass::exists(char *filepath) {
  /*

     Returns true if the supplied file path exists.

   */
  return walkPath(filepath, root, callback_pathExists);
}


//boolean SDClass::exists(char *filepath, SdFile& parentDir) {
//  /*
//
//     Returns true if the supplied file path rooted at `parentDir`
//     exists.
//
//   */
//  return walkPath(filepath, parentDir, callback_pathExists);
//}


boolean SDClass::mkdir(char *filepath) {
  /*
  
    Makes a single directory or a heirarchy of directories.

    A rough equivalent to `mkdir -p`.
  
   */
  return walkPath(filepath, root, callback_makeDirPath);
}

boolean SDClass::rmdir(char *filepath) {
  /*
  
    Remove a single directory or a heirarchy of directories.

    A rough equivalent to `rm -rf`.
  
   */
  return walkPath(filepath, root, callback_rmdir);
}

boolean SDClass::remove(char *filepath) {
  return walkPath(filepath, root, callback_remove);
}


// allows you to recurse into a directory
File File::openNextFile(uint8_t mode) {
  dir_t p;

  //Serial.print("\t\treading dir...");
  while (_file->readDir(&p) > 0) {

    // done if past last used entry
    if (p.name[0] == DIR_NAME_FREE) {
      //Serial.println("end");
      return File();
    }

    // skip deleted entry and entries for . and  ..
    if (p.name[0] == DIR_NAME_DELETED || p.name[0] == '.') {
      //Serial.println("dots");
      continue;
    }

    // only list subdirectories and files
    if (!DIR_IS_FILE_OR_SUBDIR(&p)) {
      //Serial.println("notafile");
      continue;
    }

    // print file name with possible blank fill
    SdFile f;
    char name[13];
    _file->dirName(p, name);
    //Serial.print("try to open file ");
    //Serial.println(name);

    if (f.open(_file, name, mode)) {
      //Serial.println("OK!");
      return File(f, name);    
    } else {
      //Serial.println("ugh");
      return File();
    }
  }

  //Serial.println("nothing");
  return File();
}

void File::rewindDirectory(void) {  
  if (isDirectory())
    _file->rewind();
}

SDClass SD;


//----------------------------

// Pins used by the SPI and ChipSelect. Can't be used for I/O obviously unless SD is released.
#define PIN_SPI_MOSI 11
#define PIN_SPI_MISO 12
#define PIN_SPI_CLK 13
#define PIN_SPI_CS 4    // chipSelect

String _sd_content;

void setup_config() {
  // no-op 
}

void read_config() {
  if (!SD.begin(PIN_SPI_CS)) {
    Serial.println("SDCard failed. Not present?");
    return;
  }

  File f = SD.open("config.txt");
  if (!f || !f.available()) {
    Serial.println("SD config.txt missing or empty.");
    return;
  }

  Serial.println("SD reading config.txt.");
  _sd_content = f.readString();
  f.close();
}

String get_config_string(const char *key) {
  String k('\n');
  k += key;
  k += '=';
  int pos = _sd_content.indexOf(k);
  if (pos >= 0) {
    int end = _sd_content.indexOf('\n', pos + k.length());
    if (end > pos) {
      return _sd_content.substring(pos + k.length(), end);
    }
  }
  return "";
}

int get_config_int(const char *key) {
  String value = get_config_string(key);
  return value.toInt();
}

