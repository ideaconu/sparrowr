/**
 * \file
 *
 * \brief SAM Non-Volatile Memory driver
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
#ifndef NVM_H_INCLUDED
#define NVM_H_INCLUDED

#include <Arduino.h>

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/** Mask to retrieve the error category of a status code. */
#define STATUS_CATEGORY_MASK  0xF0

/** Mask to retrieve the error code within the category of a status code. */
#define STATUS_ERROR_MASK     0x0F

/** Status code error categories. */
enum status_categories {
	STATUS_CATEGORY_OK                = 0x00,
	STATUS_CATEGORY_COMMON            = 0x10,
	STATUS_CATEGORY_ANALOG            = 0x30,
	STATUS_CATEGORY_COM               = 0x40,
	STATUS_CATEGORY_IO                = 0x50,
};

/**
 * Status code that may be returned by shell commands and protocol
 * implementations.
 *
 * \note Any change to these status codes and the corresponding
 * message strings is strictly forbidden. New codes can be added,
 * however, but make sure that any message string tables are updated
 * at the same time.
 */
enum status_code {
	STATUS_OK                         = STATUS_CATEGORY_OK     | 0x00,
	STATUS_VALID_DATA                 = STATUS_CATEGORY_OK     | 0x01,
	STATUS_NO_CHANGE                  = STATUS_CATEGORY_OK     | 0x02,
	STATUS_ABORTED                    = STATUS_CATEGORY_OK     | 0x04,
	STATUS_BUSY                       = STATUS_CATEGORY_OK     | 0x05,
	STATUS_SUSPEND                    = STATUS_CATEGORY_OK     | 0x06,

	STATUS_ERR_IO                     = STATUS_CATEGORY_COMMON | 0x00,
	STATUS_ERR_REQ_FLUSHED            = STATUS_CATEGORY_COMMON | 0x01,
	STATUS_ERR_TIMEOUT                = STATUS_CATEGORY_COMMON | 0x02,
	STATUS_ERR_BAD_DATA               = STATUS_CATEGORY_COMMON | 0x03,
	STATUS_ERR_NOT_FOUND              = STATUS_CATEGORY_COMMON | 0x04,
	STATUS_ERR_UNSUPPORTED_DEV        = STATUS_CATEGORY_COMMON | 0x05,
	STATUS_ERR_NO_MEMORY              = STATUS_CATEGORY_COMMON | 0x06,
	STATUS_ERR_INVALID_ARG            = STATUS_CATEGORY_COMMON | 0x07,
	STATUS_ERR_BAD_ADDRESS            = STATUS_CATEGORY_COMMON | 0x08,
	STATUS_ERR_BAD_FORMAT             = STATUS_CATEGORY_COMMON | 0x0A,
	STATUS_ERR_BAD_FRQ                = STATUS_CATEGORY_COMMON | 0x0B,
	STATUS_ERR_DENIED                 = STATUS_CATEGORY_COMMON | 0x0c,
	STATUS_ERR_ALREADY_INITIALIZED    = STATUS_CATEGORY_COMMON | 0x0d,
	STATUS_ERR_OVERFLOW               = STATUS_CATEGORY_COMMON | 0x0e,
	STATUS_ERR_NOT_INITIALIZED        = STATUS_CATEGORY_COMMON | 0x0f,

	STATUS_ERR_SAMPLERATE_UNAVAILABLE = STATUS_CATEGORY_ANALOG | 0x00,
	STATUS_ERR_RESOLUTION_UNAVAILABLE = STATUS_CATEGORY_ANALOG | 0x01,

	STATUS_ERR_BAUDRATE_UNAVAILABLE   = STATUS_CATEGORY_COM    | 0x00,
	STATUS_ERR_PACKET_COLLISION       = STATUS_CATEGORY_COM    | 0x01,
	STATUS_ERR_PROTOCOL               = STATUS_CATEGORY_COM    | 0x02,

	STATUS_ERR_PIN_MUX_INVALID        = STATUS_CATEGORY_IO     | 0x00,
};


#if !defined(__DOXYGEN__)
/**
 * \brief Mask for the error flags in the status register.
 */
#  define NVM_ERRORS_MASK (NVMCTRL_STATUS_PROGE | \
                           NVMCTRL_STATUS_LOCKE | \
                           NVMCTRL_STATUS_NVME)
#endif

/**
 * \brief NVM error flags.
 *
 * Possible NVM controller error codes, which can be returned by the NVM
 * controller after a command is issued.
 */
enum nvm_error {
	/** No errors */
	NVM_ERROR_NONE = 0,
	/** Lock error, a locked region was attempted accessed */
	NVM_ERROR_LOCK = NVMCTRL_STATUS_NVME | NVMCTRL_STATUS_LOCKE,
	/** Program error, invalid command was executed */
	NVM_ERROR_PROG = NVMCTRL_STATUS_NVME | NVMCTRL_STATUS_PROGE,
};

/**
 * \brief NVM controller commands.
 */
enum nvm_command {
	/** Erases the addressed memory row */
	NVM_COMMAND_ERASE_ROW                  = NVMCTRL_CTRLA_CMD_ER,

	/** Write the contents of the page buffer to the addressed memory page */
	NVM_COMMAND_WRITE_PAGE                 = NVMCTRL_CTRLA_CMD_WP,

	/** Erases the addressed auxiliary memory row.
	 *
	 *  \note This command can only be given when the security bit is not set.
	 */
	NVM_COMMAND_ERASE_AUX_ROW              = NVMCTRL_CTRLA_CMD_EAR,

	/** Write the contents of the page buffer to the addressed auxiliary memory
	 *  row.
	 *
	 *  \note This command can only be given when the security bit is not set.
	 */
	NVM_COMMAND_WRITE_AUX_ROW              = NVMCTRL_CTRLA_CMD_WAP,

	/** Locks the addressed memory region, preventing further modifications
	 *  until the region is unlocked or the device is erased
	 */
	NVM_COMMAND_LOCK_REGION                = NVMCTRL_CTRLA_CMD_LR,

	/** Unlocks the addressed memory region, allowing the region contents to be
	 *  modified
	 */
	NVM_COMMAND_UNLOCK_REGION              = NVMCTRL_CTRLA_CMD_UR,

	/** Clears the page buffer of the NVM controller, resetting the contents to
	 *  all zero values
	 */
	NVM_COMMAND_PAGE_BUFFER_CLEAR          = NVMCTRL_CTRLA_CMD_PBC,

	/** Sets the device security bit, disallowing the changing of lock bits and
	 *  auxiliary row data until a chip erase has been performed
	 */
	NVM_COMMAND_SET_SECURITY_BIT           = NVMCTRL_CTRLA_CMD_SSB,

	/** Enter power reduction mode in the NVM controller to reduce the power
	 *  consumption of the system
	 */
	NVM_COMMAND_ENTER_LOW_POWER_MODE       = NVMCTRL_CTRLA_CMD_SPRM,

	/** Exit power reduction mode in the NVM controller to allow other NVM
	 *  commands to be issued
	 */
	NVM_COMMAND_EXIT_LOW_POWER_MODE        = NVMCTRL_CTRLA_CMD_CPRM,
};

/**
 * \brief NVM controller power reduction mode configurations.
 *
 * Power reduction modes of the NVM controller, to conserve power while the
 * device is in sleep.
 */
enum nvm_sleep_power_mode {
	/** NVM controller exits low-power mode on first access after sleep */
	NVM_SLEEP_POWER_MODE_WAKEONACCESS  = NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val,
	/** NVM controller exits low-power mode when the device exits sleep mode */
	NVM_SLEEP_POWER_MODE_WAKEUPINSTANT = NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val,
	/** Power reduction mode in the NVM controller disabled */
	NVM_SLEEP_POWER_MODE_ALWAYS_AWAKE  = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val,
};

/**
 * \brief NVM controller cache readmode configuration.
 *
 * Control how the NVM cache prefetch data from flash.
 *
 */
enum nvm_cache_readmode {
	/** The NVM Controller (cache system) does not insert wait states on
	 *  a cache miss. Gives the best system performance.
	 */
	NVM_CACHE_READMODE_NO_MISS_PENALTY,
	/** Reduces power consumption of the cache system, but inserts a
	 *  wait state each time there is a cache miss
	 */
	NVM_CACHE_READMODE_LOW_POWER,
	/** The cache system ensures that a cache hit or miss takes the same
	 *  amount of time, determined by the number of programmed flash
	 *  wait states
	 */
	NVM_CACHE_READMODE_DETERMINISTIC,
};

/**
 * \brief NVM controller configuration structure.
 *
 * Configuration structure for the NVM controller within the device.
 */
struct nvm_config {
	/** Power reduction mode during device sleep */
	enum nvm_sleep_power_mode sleep_power_mode;
	/** Manual write mode; if enabled, pages loaded into the NVM buffer will
	 *  not be written until a separate write command is issued. If disabled,
	 *  writing to the last byte in the NVM page buffer will trigger an automatic
	 *  write.
	 *
	 *  \note If a partial page is to be written, a manual write command must be
	 *        executed in either mode.
	 */
	bool manual_page_write;
	/** Number of wait states to insert when reading from flash, to prevent
	 *  invalid data from being read at high clock frequencies
	 */
	uint8_t wait_states;

	/**
	 * Setting this to true will disable the pre-fetch cache in front of the
	 * NVM controller
	 */
	bool disable_cache;
	/**
	 * Select the mode for  how the cache will pre-fetch data from the flash
	 */
	enum nvm_cache_readmode cache_readmode;
};

/**
 * \brief NVM memory parameter structure.
 *
 * Structure containing the memory layout parameters of the NVM module.
 */
struct nvm_parameters {
	/** Number of bytes per page */
	uint8_t  page_size;
	/** Number of pages in the main array */
	uint16_t nvm_number_of_pages;
	/** Size of the emulated EEPROM memory section configured in the NVM
	 *  auxiliary memory space */
	uint32_t eeprom_number_of_pages;
	/** Size of the Bootloader memory section configured in the NVM auxiliary
	 *  memory space */
	uint32_t bootloader_number_of_pages;
};

/**
 * \brief Bootloader size.
 *
 * Available bootloader protection sizes in kilobytes.
 *
 */
enum nvm_bootloader_size {
	/** Boot Loader Size is 32768 bytes */
	NVM_BOOTLOADER_SIZE_128,
	/** Boot Loader Size is 16384 bytes */
	NVM_BOOTLOADER_SIZE_64,
	/** Boot Loader Size is 8192 bytes */
	NVM_BOOTLOADER_SIZE_32,
	/** Boot Loader Size is 4096 bytes */
	NVM_BOOTLOADER_SIZE_16,
	/** Boot Loader Size is 2048 bytes */
	NVM_BOOTLOADER_SIZE_8,
	/** Boot Loader Size is 1024 bytes */
	NVM_BOOTLOADER_SIZE_4,
	/** Boot Loader Size is 512 bytes */
	NVM_BOOTLOADER_SIZE_2,
	/** Boot Loader Size is 0 bytes */
	NVM_BOOTLOADER_SIZE_0,
};

/**
 * \brief EEPROM emulator size.
 *
 * Available space in flash dedicated for EEPROM emulator in bytes.
 *
 */
enum nvm_eeprom_emulator_size {
	/** EEPROM Size for EEPROM emulation is 16384 bytes */
	NVM_EEPROM_EMULATOR_SIZE_16384,
	/** EEPROM Size for EEPROM emulation is 8192 bytes */
	NVM_EEPROM_EMULATOR_SIZE_8192,
	/** EEPROM Size for EEPROM emulation is 4096 bytes */
	NVM_EEPROM_EMULATOR_SIZE_4096,
	/** EEPROM Size for EEPROM emulation is 2048 bytes */
	NVM_EEPROM_EMULATOR_SIZE_2048,
	/** EEPROM Size for EEPROM emulation is 1024 bytes */
	NVM_EEPROM_EMULATOR_SIZE_1024,
	/** EEPROM Size for EEPROM emulation is 512 bytes */
	NVM_EEPROM_EMULATOR_SIZE_512,
	/** EEPROM Size for EEPROM emulation is 256 bytes */
	NVM_EEPROM_EMULATOR_SIZE_256,
	/** EEPROM Size for EEPROM emulation is 0 bytes */
	NVM_EEPROM_EMULATOR_SIZE_0,
};

/**
 * \brief BOD33 Action.
 *
 * What action should be triggered when BOD33 is detected.
 *
 */
enum nvm_bod33_action {
	/** No action */
	NVM_BOD33_ACTION_NONE,
	/** The BOD33 generates a reset */
	NVM_BOD33_ACTION_RESET,
	/** The BOD33 generates an interrupt */
	NVM_BOD33_ACTION_INTERRUPT,
};

/**
 * \brief WDT Window time-out period.
 *
 * Window mode time-out period in clock cycles.
 *
 */
enum nvm_wdt_window_timeout {
	/** 8 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_8,
	/** 16 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_16,
	/** 32 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_32,
	/** 64 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_64,
	/** 128 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_128,
	/** 256 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_256,
	/** 512 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_512,
	/** 1024 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_1024,
	/** 2048 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_2048,
	/** 4096 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_4096,
	/** 8192 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_8192,
	/** 16384 clock cycles */
	NVM_WDT_WINDOW_TIMEOUT_PERIOD_16384,
};

/**
 * \brief WDT Early warning offset.
 *
 * This setting determine how many GCLK_WDT cycles before a watchdog time-out period
 * an early warning interrupt should be triggered.
 *
 */
enum nvm_wdt_early_warning_offset {
	/** 8 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_8,
	/** 16 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_16,
	/** 32 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_32,
	/** 64 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_64,
	/** 128 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_128,
	/** 256 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_256,
	/** 512 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_512,
	/** 1024 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_1024,
	/** 2048 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_2048,
	/** 4096 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_4096,
	/** 8192 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_8192,
	/** 16384 clock cycles */
	NVM_WDT_EARLY_WARNING_OFFSET_16384,
};

/**
 * \brief NVM user row fuse setting structure.
 *
 * This structure contain the layout of the first 64 bits of the user row
 * which contain the fuse settings.
 */
struct nvm_fusebits {
	/** Bootloader size */
	enum nvm_bootloader_size          bootloader_size;
	/** EEPROM emulation area size */
	enum nvm_eeprom_emulator_size     eeprom_size;
	/** BOD33 Threshold level at power on */
	uint8_t                           bod33_level;
	/** BOD33 Enable at power on */
	bool                              bod33_enable;
	/** BOD33 Action at power on */
	enum nvm_bod33_action             bod33_action;
	/* BOD33 Hysteresis at power on */
	bool                              bod33_hysteresis;
	/** WDT Enable at power on */
	bool                              wdt_enable;
	/** WDT Always-on at power on */
	bool                              wdt_always_on;
	/** WDT Period at power on */
	uint8_t                           wdt_timeout_period;
	/** WDT Window mode time-out at power on */
	enum nvm_wdt_window_timeout       wdt_window_timeout;
	/** WDT Early warning interrupt time offset at power on */
	enum nvm_wdt_early_warning_offset wdt_early_warning_offset;
	/** WDT Window mode enabled at power on */
	bool                              wdt_window_mode_enable_at_poweron;
	/** NVM Lock bits */
	uint16_t                          lockbits;
};

/**
 * \name Configuration and Initialization
 * @{
 */

/**
 * \brief Initializes an NVM controller configuration structure to defaults.
 *
 * Initializes a given NVM controller configuration structure to a set of
 * known default values. This function should be called on all new
 * instances of these configuration structures before being modified by the
 * user application.
 *
 * The default configuration is as follows:
 *  \li Power reduction mode enabled after sleep mode until first NVM access
 *  \li Automatic page write mode disabled
 *  \li Number of FLASH wait states left unchanged
 *
 * \param[out] config  Configuration structure to initialize to default values
 *
 */
static inline void nvm_get_config_defaults(
		struct nvm_config *const config)
{
	/* Sanity check the parameters */
	/* Write the default configuration for the NVM configuration */
	config->sleep_power_mode  = NVM_SLEEP_POWER_MODE_WAKEONACCESS;
	config->manual_page_write = true;
	config->wait_states       = NVMCTRL->CTRLB.bit.RWS;
	config->disable_cache     = false;
	config->cache_readmode    = NVM_CACHE_READMODE_NO_MISS_PENALTY;
}

enum status_code nvm_set_config(
		const struct nvm_config *const config);

/**
 * \brief Checks if the NVM controller is ready to accept a new command.
 *
 * Checks the NVM controller to determine if it is currently busy execution an
 * operation, or ready for a new command.
 *
 * \return Busy state of the NVM controller.
 *
 * \retval true   If the hardware module is ready for a new command
 * \retval false  If the hardware module is busy executing a command
 *
 */
static inline bool nvm_is_ready(void)
{
	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	return nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY;
}

/** @} */

/**
 * \name NVM Access Management
 * @{
 */

void nvm_get_parameters(
		struct nvm_parameters *const parameters);

enum status_code nvm_write_buffer(
		const uint32_t destination_address,
		const uint8_t *buffer,
		uint16_t length);

enum status_code nvm_read_buffer(
		const uint32_t source_address,
		uint8_t *const buffer,
		uint16_t length);

enum status_code nvm_update_buffer(
		const uint32_t destination_address,
		uint8_t *const buffer,
		uint16_t offset,
		uint16_t length);

enum status_code nvm_erase_row(
		const uint32_t row_address);

enum status_code nvm_execute_command(
		const enum nvm_command command,
		const uint32_t address,
		const uint32_t parameter);

enum status_code nvm_get_fuses(struct nvm_fusebits *fusebits);
enum status_code nvm_set_fuses(struct nvm_fusebits *fb);

bool nvm_is_page_locked(uint16_t page_number);

/**
 * \brief Retrieves the error code of the last issued NVM operation.
 *
 * Retrieves the error code from the last executed NVM operation. Once
 * retrieved, any error state flags in the controller are cleared.
 *
 * \note The \ref nvm_is_ready() function is an exception. Thus, errors
 *       retrieved after running this function should be valid for the function
 *       executed before \ref nvm_is_ready().
 *
 * \return Error caused by the last NVM operation.
 *
 * \retval NVM_ERROR_NONE  No error occurred in the last NVM operation
 *
 * \retval NVM_ERROR_LOCK  The last NVM operation attempted to access a locked
 *                         region
 * \retval NVM_ERROR_PROG  An invalid NVM command was issued
 */
static inline enum nvm_error nvm_get_error(void)
{
	enum nvm_error ret_val;

	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	/* Mask out non-error bits */
	ret_val = (enum nvm_error)(nvm_module->STATUS.reg & NVM_ERRORS_MASK);

	/* Clear error flags */
	nvm_module->STATUS.reg &= ~NVMCTRL_STATUS_MASK;

	/* Return error code from the NVM controller */
	return ret_val;
}

/** @} */

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* NVM_H_INCLUDED */
