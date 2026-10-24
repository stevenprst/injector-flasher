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

// ---- USB link to the PC (a virtual COM port) ----

// A request is: magic, command, length (u16), payload. A response is: magic, command, code, length, payload. Both fit
// in one 64 byte USB packet. The PC sends one request and waits for its response.
static constexpr uint8_t link_version = 1;
static constexpr uint8_t request_magic = 0xA5;
static constexpr uint8_t response_magic = 0x5A;
static constexpr uint32_t request_header_bytes = 4;
static constexpr uint32_t response_header_bytes = 4;
static constexpr uint32_t max_payload_bytes = 60;
static constexpr uint32_t chunk_offset_bytes = 4;
static constexpr uint32_t max_chunk_data_bytes = max_payload_bytes - chunk_offset_bytes;	// 56, a multiple of 4
static_assert(max_chunk_data_bytes % 4 == 0, "a chunk carries whole flash words");
static_assert(request_header_bytes + max_payload_bytes <= 64, "a request fits in one USB packet");
static_assert(response_header_bytes + max_payload_bytes <= 64, "a response fits in one USB packet");

// A request that stops half way is dropped after this long, so the next request starts clean.
static constexpr uint32_t request_timeout_ms = 200;

// Bytes from the USB interrupt to the main loop. A power of two.
static constexpr uint32_t usb_ring_size = 512;
static_assert((usb_ring_size & (usb_ring_size - 1)) == 0, "the ring index is taken with a modulo of a power of two");

enum class LinkCommand : uint8_t {
	GetStatus = 1,		// what the injector holds and how an injection is going
	BeginImage = 2,		// start a new image: invalidates the stored one
	WriteChunk = 3,		// the next bytes of the image
	EndImage = 4,		// check the image CRC and store the header
};

enum class LinkCode : uint8_t {
	Ok = 0,
	BadCommand = 1,
	BadLength = 2,		// the payload has the wrong size
	BadOffset = 3,		// a chunk that is not the next one
	FlashFailed = 4,	// an erase or a program failed
	CrcMismatch = 5,	// the received image does not match the CRC of BeginImage
	Busy = 6,		// an injection is running
	NoImage = 7,
	BadState = 8,		// WriteChunk or EndImage without BeginImage, or the image is incomplete
	BadImage = 9,		// the sizes of BeginImage do not agree or are too big
};

struct __attribute__((packed)) BeginImagePayload {
	uint32_t image_size_bytes;
	uint32_t image_crc;
	uint16_t page_count;
	uint8_t target_node_id;
};

static_assert(sizeof(BeginImagePayload) == 11, "BeginImage carries 11 bytes");

enum class InjectState : uint8_t {
	Idle = 0,
	Connecting = 1,		// trigger frame, then pings until the bootloader answers
	SendingWords = 2,	// the 256 words of a page, one command each
	WritingPage = 3,	// WritePage and its answer
	WritingCrc = 4,		// WriteCrc and its answer
	StartingApp = 5,
	Done = 6,
	Failed = 7,
};

struct __attribute__((packed)) StatusPayload {
	uint8_t link_version;
	uint8_t receiving;		// 1 between BeginImage and EndImage
	uint8_t has_image;
	uint8_t target_node_id;
	uint16_t page_count;
	uint16_t pages_done;		// pages the bootloader has acknowledged in this injection
	uint8_t inject_state;
	uint8_t inject_error;
	uint8_t target_error_code;	// the code of the bootloader when inject_error is TargetRefused
	uint32_t image_size_bytes;
	uint32_t image_crc;
};

static_assert(sizeof(StatusPayload) == 19, "GetStatus carries 19 bytes");

class InjectorFlasher
{
public:
	// Look for a stored image.
	void init();

	// One pass of the main loop: the PC link.
	void run();

	// Called from the USB interrupt for every packet from the PC.
	void onUsbReceive(const uint8_t *data, uint32_t length);

private:
	enum class ParserState : uint8_t {
		WaitingForMagic,
		ReadingHeader,
		ReadingPayload,
	};

	// PC link
	void _serviceUsb();
	void _parseByte(uint8_t byte);
	void _handleRequest();
	void _handleGetStatus();
	void _handleBeginImage();
	void _handleWriteChunk();
	void _handleEndImage();
	void _reply(LinkCode code, const void *payload, uint32_t payload_bytes);
	void _sendPendingReply();

	// The stored image
	void _loadHeader();
	bool _deleteStoredImage();
	bool _appendWord(uint32_t word);
	bool _isInjecting() const;

	// Bytes from the USB interrupt. The interrupt only writes _rx_head, the main loop only _rx_tail.
	std::atomic<uint32_t> _rx_head{0};
	std::atomic<uint32_t> _rx_tail{0};
	uint8_t _rx_ring[usb_ring_size]{};

	// The request being read
	ParserState _parser_state{ParserState::WaitingForMagic};
	uint8_t _header_bytes[3]{};	// command, length low, length high
	uint32_t _header_received{0};
	LinkCommand _request_command{LinkCommand::GetStatus};
	uint32_t _request_length{0};
	uint8_t _request_payload[max_payload_bytes]{};
	uint32_t _payload_received{0};
	uint32_t _last_byte_ms{0};

	// The response waiting to leave. The next request is not read before it has gone.
	uint8_t _reply_buffer[response_header_bytes + max_payload_bytes]{};
	uint32_t _reply_length{0};
	bool _reply_pending{false};

	// The stored image: valid when a header with the right CRC is in flash
	bool _has_image{false};
	ImageHeader _header{};

	// True between BeginImage and EndImage, while the image in the flash is not yet complete
	bool _receiving{false};

	// The image being received
	BeginImagePayload _incoming{};
	uint32_t _expected_offset{0};
	uint32_t _page_buffer_words[page_size_words]{};
	uint32_t _words_in_page{0};
	uint32_t _pages_written{0};

	// The injection
	InjectState _inject_state{InjectState::Idle};

};

} // namespace injector

#endif // MAINLOOP_HPP
