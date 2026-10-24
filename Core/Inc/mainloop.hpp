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

// ---- front panel ----

// The pins come from the CubeMX labels LED_STATUS, BUTTON_INJECT and BUTTON_DELETE, so moving a pin needs no change here.
static constexpr GPIO_PinState led_on_level = GPIO_PIN_SET;
static constexpr GPIO_PinState led_off_level = GPIO_PIN_RESET;
static_assert(led_on_level != led_off_level, "the LED has two levels");
static constexpr GPIO_PinState button_pressed_level = GPIO_PIN_RESET;	// the buttons pull the pin to ground

static constexpr uint32_t button_debounce_ms = 50;	// a press counts after it has read pressed for this long
static constexpr uint32_t delete_hold_ms = 1000;	// the delete button has to be held, so a brush does not erase
static constexpr uint32_t led_blink_half_period_ms = 100;	// the LED blinks 5 times a second while injecting

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
	DeleteImage = 5,	// invalidate the stored image
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

// ---- CAN side: this firmware is the host of the CAN bootloader (protocol version 2) ----

static constexpr uint16_t boot_command_can_id = 0x6A5;	// injector to bootloader
static constexpr uint16_t boot_response_can_id = 0x6A6;	// bootloader to injector
static constexpr uint16_t app_trigger_can_id = 0x6A4;	// injector to the running app: reset into the bootloader
static constexpr uint16_t app_trigger_magic = 0xB007;

enum class BootCommand : uint8_t {
	WriteBuffer = 1,
	WritePage = 2,
	WriteCrc = 3,
	Ping = 4,
	StartApp = 6,
};

// The 8 bytes sent to the bootloader and the 8 bytes it answers with, little-endian.
struct __attribute__((packed)) BootCommandFrame {
	uint8_t target_node_id;
	uint8_t command;
	uint16_t index;
	uint32_t value;
};

struct __attribute__((packed)) BootResponseFrame {
	uint8_t node_id;
	uint8_t command;
	uint8_t code;
	uint16_t index;
	uint8_t payload[3];
};

static_assert(sizeof(BootCommandFrame) == 8, "a command is exactly one 8-byte CAN frame");
static_assert(sizeof(BootResponseFrame) == 8, "a reply is exactly one 8-byte CAN frame");

static constexpr uint32_t reply_timeout_ms = 1000;
static constexpr uint32_t write_page_timeout_ms = 20000;	// a page that opens a 128 KB sector waits for the erase
static constexpr uint32_t write_crc_timeout_ms = 5000;		// the bootloader reads the whole image
static constexpr uint32_t start_app_timeout_ms = 5000;
static constexpr uint32_t ping_timeout_ms = 100;
static constexpr uint32_t ping_tries = 10;
static constexpr uint32_t command_retries = 3;			// extra tries when a reply does not come
static constexpr uint32_t tx_mailbox_timeout_ms = 2000;		// the bus is dead when no mailbox frees up for this long

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

enum class InjectError : uint8_t {
	None = 0,
	CannotConnect = 1,	// no ping was answered
	NoReply = 2,		// a command went unanswered after its retries
	TargetRefused = 3,	// the bootloader answered with an error code, see target_error_code
	BusDead = 4,		// no transmit mailbox freed up
	CanNotStarted = 5,
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

// A button and what the main loop knows of it.
struct Button {
	bool pressed{false};		// the pin reads pressed
	uint32_t pressed_since_ms{0};	// since when it has read pressed
	bool action_done{false};	// the action of this press has run, so wait for the release
};

class InjectorFlasher
{
public:
	// Look for a stored image, set up the CAN filter and start CAN.
	void init();

	// One pass of the main loop: the PC link, the injection, the front panel.
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
	void _handleDeleteImage();
	void _reply(LinkCode code, const void *payload, uint32_t payload_bytes);
	void _sendPendingReply();

	// The stored image
	void _loadHeader();
	bool _deleteStoredImage();
	bool _appendWord(uint32_t word);
	bool _isInjecting() const;

	// The injection into the target
	void _startInjection();
	void _serviceInjection();
	void _readCanReplies();
	bool _mailboxAvailable();
	bool _sendBootCommand(BootCommand command, uint16_t index, uint32_t value);
	bool _sendTrigger();
	void _failInjection(InjectError error, uint8_t target_error_code);
	enum class Progress : uint8_t { InProgress, Succeeded, Failed };
	Progress _runCommand(BootCommand command, uint16_t index, uint32_t value, uint32_t timeout_ms);
	void _stepConnecting();

	// The front panel
	void _servicePanel();
	void _updateButton(Button &button, GPIO_TypeDef *port, uint16_t pin, uint32_t now_ms);
	void _updateLed(uint32_t now_ms);

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
	InjectError _inject_error{InjectError::None};
	uint8_t _target_error_code{0};
	uint32_t _inject_page{0};
	uint32_t _inject_word{0};
	uint32_t _page_crc{0};
	bool _can_ready{false};
	bool _trigger_sent{false};
	uint32_t _ping_count{0};
	bool _command_in_flight{false};
	uint32_t _retries{0};
	uint32_t _command_sent_ms{0};
	uint32_t _send_attempt_started_ms{0};
	bool _send_attempt_open{false};
	BootCommand _awaiting_command{BootCommand::Ping};
	uint16_t _awaiting_index{0};
	bool _reply_valid{false};
	BootResponseFrame _boot_reply{};

	// The front panel
	Button _inject_button{};
	Button _delete_button{};
};

} // namespace injector

#endif // MAINLOOP_HPP
