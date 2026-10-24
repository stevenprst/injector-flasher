#include "mainloop.hpp"

#include <cstring>

#include "usbd_cdc_if.h"

namespace injector
{

namespace
{

// The one injector of this image.
InjectorFlasher flasher;

// The stored image as words, read straight from flash.
const uint32_t *imageWords()
{
	return reinterpret_cast<const uint32_t *>(image_address);
}

void clearFlashErrorFlags()
{
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
			       | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

// The data cache can hold flash lines read before a program or erase, so reset it before reading the result back.
void flushFlashCaches()
{
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_DATA_CACHE_ENABLE();
}

// True when every word in the range reads as erased flash.
bool isBlank(uint32_t address, uint32_t word_count)
{
	const volatile uint32_t *flash_words = reinterpret_cast<const volatile uint32_t *>(address);

	for (uint32_t word_number = 0; word_number < word_count; ++word_number) {
		if (flash_words[word_number] != 0xFFFFFFFFu) {
			return false;
		}
	}

	return true;
}

// Erase the sector that starts at the address. False when the address is not a sector start or the sector is not blank.
// The CPU waits while the flash erases (up to 4 s for 128 KB), and so does the USB link.
bool eraseSectorStartingAt(uint32_t address)
{
	const flash_layout::FlashSector *sector = flash_layout::sectorStartingAt(address);

	if (sector == nullptr) {
		return false;
	}

	FLASH_EraseInitTypeDef erase_config{};
	erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase_config.Sector = sector->number;
	erase_config.NbSectors = 1;
	erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t failed_sector_number = 0;

	HAL_FLASH_Unlock();
	clearFlashErrorFlags();
	const HAL_StatusTypeDef erase_status = HAL_FLASHEx_Erase(&erase_config, &failed_sector_number);
	HAL_FLASH_Lock();

	if (erase_status != HAL_OK) {
		return false;
	}

	flushFlashCaches();

	return isBlank(sector->start_address, sector->size_bytes / 4);
}

// The hardware CRC of the words, fed one 32-bit word at a time.
uint32_t calculateWordsCrc(const uint32_t *words, uint32_t word_count)
{
	__HAL_CRC_DR_RESET(&hcrc);

	return HAL_CRC_Calculate(&hcrc, const_cast<uint32_t *>(words), word_count);
}

} // namespace

void InjectorFlasher::init()
{
	_loadHeader();
}

void InjectorFlasher::run()
{
}

// ---- the stored image ----

void InjectorFlasher::_loadHeader()
{
	const ImageHeader *stored = reinterpret_cast<const ImageHeader *>(header_sector_address);

	const bool magic_present = stored->magic == header_magic;
	const bool page_count_valid = stored->page_count >= 1 && stored->page_count <= max_page_count;

	if (!magic_present || !page_count_valid) {
		_has_image = false;
		return;
	}

	// The image counts only when it still matches the CRC that was checked when it was received.
	const uint32_t image_crc = calculateWordsCrc(imageWords(), stored->page_count * page_size_words);
	const bool image_intact = image_crc == stored->image_crc;

	if (!image_intact) {
		_has_image = false;
		return;
	}

	_header = *stored;
	_has_image = true;
}

bool InjectorFlasher::_deleteStoredImage()
{
	_has_image = false;
	_receiving = false;

	return eraseSectorStartingAt(header_sector_address);
}

} // namespace injector

void mainLoopInit(void)
{
	injector::flasher.init();
}

void mainLoopRun(void)
{
	injector::flasher.run();
}

// The USB receive callback of the CDC interface calls this for every packet.
void injectorUsbReceive(const uint8_t *data, uint32_t length)
{
	(void)data;
	(void)length;
}
