#ifndef MAINLOOP_HPP
#define MAINLOOP_HPP

#include <atomic>
#include <cstdint>

#include "main.h"

// The hooks that main.c and usbd_cdc_if.c call are declared in main.h, inside USER CODE EFP:
//   void mainLoopInit(void);
//   void mainLoopRun(void);
//   void injectorUsbReceive(const uint8_t *data, uint32_t length);

// Handles that CubeMX creates in main.c.
extern "C" {
extern CAN_HandleTypeDef hcan1;
extern CRC_HandleTypeDef hcrc;
}

namespace injector
{

// ---- flash layout ----

// Sectors 0 to 3 hold this firmware (the linker script limits it to 64 KB), sector 4 the header of the stored image,
// sectors 5 to 11 the image. The header is written last, so a transfer that stops halfway leaves no valid image.
static constexpr uint32_t header_sector_address = 0x08010000;
static constexpr uint32_t image_address = 0x08020000;
static constexpr uint32_t image_end_address = 0x08100000;

static constexpr uint32_t page_size_words = 256;
static constexpr uint32_t page_size_bytes = page_size_words * 4;

// The most pages the image area holds, 896 pages of 1 KB.
static constexpr uint32_t max_page_count = (image_end_address - image_address) / page_size_bytes;
static_assert(max_page_count == 896, "the image area is 896 KB");

// What is written into the header sector when an image is complete and its CRC has been checked.
static constexpr uint32_t header_magic = 0x494E4A31;	// "INJ1"

struct ImageHeader {
	uint32_t magic;
	uint32_t image_size_bytes;	// the size of the file, not padded
	uint32_t image_crc;		// hardware CRC over page_count pages, the file padded with zeros
	uint32_t page_count;
	uint32_t target_node_id;	// the bootloader that the image is meant for
};

static_assert(sizeof(ImageHeader) == 20, "the header is five flash words");

// Only the sectors this firmware erases: the header sector and the image sectors.
namespace flash_layout
{

struct FlashSector {
	uint32_t start_address;
	uint32_t size_bytes;
	uint32_t number;
};

inline constexpr FlashSector erasable_sectors[] = {
	{0x08010000, 64 * 1024, FLASH_SECTOR_4},
	{0x08020000, 128 * 1024, FLASH_SECTOR_5},
	{0x08040000, 128 * 1024, FLASH_SECTOR_6},
	{0x08060000, 128 * 1024, FLASH_SECTOR_7},
	{0x08080000, 128 * 1024, FLASH_SECTOR_8},
	{0x080A0000, 128 * 1024, FLASH_SECTOR_9},
	{0x080C0000, 128 * 1024, FLASH_SECTOR_10},
	{0x080E0000, 128 * 1024, FLASH_SECTOR_11},
};

inline constexpr uint32_t erasable_sector_count = sizeof(erasable_sectors) / sizeof(erasable_sectors[0]);

// The sector that starts exactly at the address, or nullptr when the address is not a sector start.
constexpr const FlashSector *sectorStartingAt(uint32_t address)
{
	for (const FlashSector &sector : erasable_sectors) {
		if (sector.start_address == address) {
			return &sector;
		}
	}

	return nullptr;
}

constexpr bool isSectorStart(uint32_t address)
{
	return sectorStartingAt(address) != nullptr;
}

// The number of the sector that starts at the address, 0xFFFFFFFF when none does.
constexpr uint32_t sectorNumberStartingAt(uint32_t address)
{
	for (const FlashSector &sector : erasable_sectors) {
		if (sector.start_address == address) {
			return sector.number;
		}
	}

	return 0xFFFFFFFFu;
}

// These checks run at build time, so a wrong table or a wrong layout constant stops the build.
static_assert(erasable_sector_count == 8, "sectors 4 to 11");
static_assert(erasable_sectors[erasable_sector_count - 1].start_address
	      + erasable_sectors[erasable_sector_count - 1].size_bytes == image_end_address,
	      "the last sector ends where the image area ends");
static_assert(sectorNumberStartingAt(header_sector_address) == FLASH_SECTOR_4, "the header is in sector 4");
static_assert(sectorNumberStartingAt(image_address) == FLASH_SECTOR_5, "page 0 opens sector 5");
static_assert(!isSectorStart(image_address + 1 * page_size_bytes), "page 1 is inside sector 5");
static_assert(!isSectorStart(image_address + 127 * page_size_bytes), "page 127 is the last page of sector 5");
static_assert(sectorNumberStartingAt(image_address + 128 * page_size_bytes) == FLASH_SECTOR_6, "page 128 opens sector 6");
static_assert(sectorNumberStartingAt(image_address + 768 * page_size_bytes) == FLASH_SECTOR_11, "page 768 opens sector 11");

} // namespace flash_layout

class InjectorFlasher
{
public:
	// Look for a stored image.
	void init();

	// One pass of the main loop.
	void run();

private:
	// The stored image
	void _loadHeader();
	bool _deleteStoredImage();

	// The stored image: valid when a header with the right CRC is in flash
	bool _has_image{false};
	ImageHeader _header{};

	// True between BeginImage and EndImage, while the image in the flash is not yet complete
	bool _receiving{false};

};

} // namespace injector

#endif // MAINLOOP_HPP
