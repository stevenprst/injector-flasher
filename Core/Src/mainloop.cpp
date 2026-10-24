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
	_serviceUsb();
}

void InjectorFlasher::onUsbReceive(const uint8_t *data, uint32_t length)
{
	for (uint32_t byte_number = 0; byte_number < length; ++byte_number) {
		const uint32_t head = _rx_head.load(std::memory_order_relaxed);
		const uint32_t tail = _rx_tail.load(std::memory_order_acquire);
		const bool ring_full = (head - tail) >= usb_ring_size;

		// The PC sends one request and waits, so a full ring means a broken sender and the newest bytes are dropped.
		if (ring_full) {
			return;
		}

		_rx_ring[head % usb_ring_size] = data[byte_number];
		_rx_head.store(head + 1, std::memory_order_release);
	}
}

// ---- PC link ----

void InjectorFlasher::_serviceUsb()
{
	_sendPendingReply();

	// A request is not read while its predecessor's response is still waiting to leave.
	if (_reply_pending) {
		return;
	}

	const uint32_t now_ms = HAL_GetTick();
	const bool request_started = _parser_state != ParserState::WaitingForMagic;
	const bool request_stalled = (now_ms - _last_byte_ms) > request_timeout_ms;

	if (request_started && request_stalled) {
		_parser_state = ParserState::WaitingForMagic;
	}

	while (!_reply_pending) {
		const uint32_t tail = _rx_tail.load(std::memory_order_relaxed);
		const uint32_t head = _rx_head.load(std::memory_order_acquire);
		const bool byte_waiting = head != tail;

		if (!byte_waiting) {
			return;
		}

		const uint8_t byte = _rx_ring[tail % usb_ring_size];
		_rx_tail.store(tail + 1, std::memory_order_release);
		_last_byte_ms = HAL_GetTick();
		_parseByte(byte);
	}
}

void InjectorFlasher::_parseByte(uint8_t byte)
{
	switch (_parser_state) {
	case ParserState::WaitingForMagic:
		if (byte == request_magic) {
			_header_received = 0;
			_parser_state = ParserState::ReadingHeader;
		}

		break;

	case ParserState::ReadingHeader: {
			_header_bytes[_header_received] = byte;
			_header_received++;

			const bool header_complete = _header_received == sizeof(_header_bytes);

			if (!header_complete) {
				break;
			}

			_request_command = static_cast<LinkCommand>(_header_bytes[0]);
			_request_length = _header_bytes[1] | (_header_bytes[2] << 8);
			_payload_received = 0;

			const bool payload_too_long = _request_length > max_payload_bytes;

			if (payload_too_long) {
				_parser_state = ParserState::WaitingForMagic;
				_reply(LinkCode::BadLength, nullptr, 0);
				break;
			}

			const bool has_payload = _request_length != 0;

			if (has_payload) {
				_parser_state = ParserState::ReadingPayload;
				break;
			}

			_parser_state = ParserState::WaitingForMagic;
			_handleRequest();
			break;
		}

	case ParserState::ReadingPayload:
		_request_payload[_payload_received] = byte;
		_payload_received++;

		if (_payload_received == _request_length) {
			_parser_state = ParserState::WaitingForMagic;
			_handleRequest();
		}

		break;
	}
}

void InjectorFlasher::_handleRequest()
{
	switch (_request_command) {
	case LinkCommand::GetStatus:
		_handleGetStatus();
		break;

	default:
		_reply(LinkCode::BadCommand, nullptr, 0);
		break;
	}
}

void InjectorFlasher::_handleGetStatus()
{
	StatusPayload status{};
	status.link_version = link_version;
	status.receiving = _receiving ? 1 : 0;
	status.has_image = _has_image ? 1 : 0;
	status.target_node_id = static_cast<uint8_t>(_has_image ? _header.target_node_id : 0);
	status.page_count = static_cast<uint16_t>(_has_image ? _header.page_count : 0);
	status.image_size_bytes = _has_image ? _header.image_size_bytes : 0;
	status.image_crc = _has_image ? _header.image_crc : 0;

	_reply(LinkCode::Ok, &status, sizeof(status));
}

void InjectorFlasher::_reply(LinkCode code, const void *payload, uint32_t payload_bytes)
{
	_reply_buffer[0] = response_magic;
	_reply_buffer[1] = static_cast<uint8_t>(_request_command);
	_reply_buffer[2] = static_cast<uint8_t>(code);
	_reply_buffer[3] = static_cast<uint8_t>(payload_bytes);

	if (payload_bytes != 0) {
		memcpy(&_reply_buffer[response_header_bytes], payload, payload_bytes);
	}

	_reply_length = response_header_bytes + payload_bytes;
	_reply_pending = true;
}

void InjectorFlasher::_sendPendingReply()
{
	if (!_reply_pending) {
		return;
	}

	// The USB stack is busy while an earlier packet is still leaving, so try again on the next pass.
	const bool transmitted = CDC_Transmit_FS(_reply_buffer, static_cast<uint16_t>(_reply_length)) == USBD_OK;

	if (transmitted) {
		_reply_pending = false;
	}
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
	injector::flasher.onUsbReceive(data, length);
}
