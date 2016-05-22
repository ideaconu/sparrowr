/**
 * \file
 *
 * \brief SAM EEPROM Emulator
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifdef __cplusplus

#ifndef EEPROM_H_INCLUDED
#define EEPROM_H_INCLUDED

#include <Arduino.h>
#include <nvm.h>

#  define EEPROM_MAX_PAGES            (64 * NVMCTRL_ROW_PAGES)
#  define EEPROM_MASTER_PAGE_NUMBER   (_eeprom_instance.physical_pages - 1)
#  define EEPROM_INVALID_PAGE_NUMBER  0xFF
#  define EEPROM_INVALID_ROW_NUMBER   (EEPROM_INVALID_PAGE_NUMBER / NVMCTRL_ROW_PAGES)
#  define EEPROM_HEADER_SIZE          4

/** \name EEPROM Emulator Information
 * @{
 */

/** Emulator scheme ID, identifying the scheme used to emulated EEPROM storage. */
#define EEPROM_EMULATOR_ID          1

/** Emulator major version number, identifying the emulator major version. */
#define EEPROM_MAJOR_VERSION        1

/** Emulator minor version number, identifying the emulator minor version. */
#define EEPROM_MINOR_VERSION        0

/** Emulator revision version number, identifying the emulator revision. */
#define EEPROM_REVISION             0

/** Size of the user data portion of each logical EEPROM page, in bytes. */
#define EEPROM_PAGE_SIZE            (NVMCTRL_PAGE_SIZE - EEPROM_HEADER_SIZE)

#define EEPROM_SIZE                 (128 * EEPROM_PAGE_SIZE)


/**
 * \brief EEPROM memory parameter structure.
 *
 * Structure containing the memory layout parameters of the EEPROM emulator module.
 */
struct eeprom_emulator_parameters {
	/** Number of bytes per emulated EEPROM page. */
	uint8_t  page_size;
	/** Number of emulated pages of EEPROM. */
	uint16_t eeprom_number_of_pages;
};

/** @} */

/** \name Configuration and Initialization
 * @{
 */

enum status_code eeprom_emulator_init(void);

void eeprom_emulator_erase_memory(void);

enum status_code eeprom_emulator_get_parameters(
		struct eeprom_emulator_parameters *const parameters);

/** @} */


/** \name Logical EEPROM Page Reading/Writing
 * @{
 */

enum status_code eeprom_emulator_commit_page_buffer(void);

enum status_code eeprom_emulator_write_page(
		const uint8_t logical_page,
		const uint8_t *const data);

enum status_code eeprom_emulator_read_page(
		const uint8_t logical_page,
		uint8_t *const data);

/** @} */

/** \name Buffer EEPROM Reading/Writing
 * @{
 */

enum status_code eeprom_emulator_write_buffer(
		const uint16_t offset,
		const uint8_t *const data,
		const uint16_t length);

enum status_code eeprom_emulator_read_buffer(
		const uint16_t offset,
		uint8_t *const data,
		const uint16_t length);

/** @} */


/** @} */
class EEPROMClass {
    public:

    void init();

    uint16_t write(uint16_t addr, uint8_t *bytes, uint16_t length);
    uint16_t read(uint16_t addr, uint8_t *bytes, uint16_t length);

    private:

};

extern EEPROMClass EEPROM;

#endif /* EEPROM_H_INCLUDED */

#endif /* _cplusplus */
