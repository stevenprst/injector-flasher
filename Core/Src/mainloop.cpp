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

// Run the erase of one sector. The CPU waits while the flash erases (up to 4 s for 128 KB), and so does the USB link.
bool runSectorErase(const flash_layout::FlashSector &sector)
{
	FLASH_EraseInitTypeDef erase_config{};
	erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase_config.Sector = sector.number;
	erase_config.NbSectors = 1;
	erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t failed_sector_number = 0;

	HAL_FLASH_Unlock();
	clearFlashErrorFlags();
	const HAL_StatusTypeDef erase_status = HAL_FLASHEx_Erase(&erase_config, &failed_sector_number);
	HAL_FLASH_Lock();

	return erase_status == HAL_OK;
}

// Erase the sector that starts at the address. False when the address is not a sector start or the sector is not blank.
bool eraseSectorStartingAt(uint32_t address)
{
	const flash_layout::FlashSector *sector = flash_layout::sectorStartingAt(address);

	if (sector == nullptr) {
		return false;
	}

	if (!runSectorErase(*sector)) {
		return false;
	}

	flushFlashCaches();

	return isBlank(sector->start_address, sector->size_bytes / 4);
}

// True when the flash at the address already holds exactly these words.
bool flashHoldsWords(uint32_t address, const uint32_t *words, uint32_t word_count)
{
	const uint32_t *flash_words = reinterpret_cast<const uint32_t *>(address);

	return memcmp(flash_words, words, word_count * sizeof(uint32_t)) == 0;
}

// Program the words one at a time, with the flash unlocked only for the duration.
bool programEachWord(uint32_t address, const uint32_t *words, uint32_t word_count)
{
	HAL_FLASH_Unlock();
	clearFlashErrorFlags();

	for (uint32_t word_number = 0; word_number < word_count; ++word_number) {
		const uint32_t word_address = address + word_number * sizeof(uint32_t);
		const HAL_StatusTypeDef program_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, word_address,
							 words[word_number]);
		const bool word_programmed = program_status == HAL_OK;

		if (!word_programmed) {
			HAL_FLASH_Lock();
			return false;
		}
	}

	HAL_FLASH_Lock();

	return true;
}

// Program words into erased flash and read them back. Words already there count as programmed.
bool programWords(uint32_t address, const uint32_t *words, uint32_t word_count)
{
	if (flashHoldsWords(address, words, word_count)) {
		return true;
	}

	if (!programEachWord(address, words, word_count)) {
		return false;
	}

	flushFlashCaches();

	return flashHoldsWords(address, words, word_count);
}

// The hardware CRC of the words, fed one 32-bit word at a time.
uint32_t calculateWordsCrc(const uint32_t *words, uint32_t word_count)
{
	__HAL_CRC_DR_RESET(&hcrc);

	return HAL_CRC_Calculate(&hcrc, const_cast<uint32_t *>(words), word_count);
}

bool pageCountIsValid(uint32_t page_count)
{
	return page_count >= 1 && page_count <= max_page_count;
}

// The page count has to be valid, and the size in bytes has to fill the last page at least by one byte and not more
// than the pages hold.
bool imageDescriptionIsValid(const BeginImagePayload &image)
{
	if (!pageCountIsValid(image.page_count)) {
		return false;
	}

	const uint32_t bytes_before_last_page = (image.page_count - 1u) * page_size_bytes;
	const uint32_t bytes_in_all_pages = image.page_count * page_size_bytes;

	return image.image_size_bytes > bytes_before_last_page && image.image_size_bytes <= bytes_in_all_pages;
}

bool isPressed(GPIO_TypeDef *port, uint16_t pin)
{
	return HAL_GPIO_ReadPin(port, pin) == button_pressed_level;
}

uint32_t readU32(const uint8_t *bytes)
{
	uint32_t value;
	memcpy(&value, bytes, sizeof(value));

	return value;
}

// Pass only the answers of the bootloaders: standard ID boot_response_can_id, data frame. The mask bits are the ID,
// IDE and RTR bits. All filter banks belong to CAN1 because CAN2 is not used.
bool configureCanFilter()
{
	CAN_FilterTypeDef filter{};
	filter.FilterIdHigh = boot_response_can_id << 5;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x7FF << 5;
	filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.SlaveStartFilterBank = 14;

	return HAL_CAN_ConfigFilter(&hcan1, &filter) == HAL_OK;
}

// The replies are read by polling in the main loop, so no CAN interrupt is enabled.
bool startCan()
{
	if (!configureCanFilter()) {
		return false;
	}

	return HAL_CAN_Start(&hcan1) == HAL_OK;
}

bool isBootResponse(const CAN_RxHeaderTypeDef &header)
{
	return header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA
	       && header.StdId == boot_response_can_id && header.DLC == sizeof(BootResponseFrame);
}

// Put one standard data frame in a transmit mailbox. The caller has checked that a mailbox is free.
bool queueCanFrame(uint16_t can_id, const uint8_t *data, uint32_t length)
{
	CAN_TxHeaderTypeDef header{};
	header.StdId = can_id;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = length;

	uint32_t mailbox = 0;

	return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK;
}

} // namespace

void InjectorFlasher::init()
{
	_can_ready = startCan();

	_loadHeader();
	_readButtonsAtStart();
}

// A button that is held while the board starts counts only after it has been released.
void InjectorFlasher::_readButtonsAtStart()
{
	_inject_button.pressed = isPressed(BUTTON_INJECT_GPIO_Port, BUTTON_INJECT_Pin);
	_inject_button.action_done = _inject_button.pressed;
	_delete_button.pressed = isPressed(BUTTON_DELETE_GPIO_Port, BUTTON_DELETE_Pin);
	_delete_button.action_done = _delete_button.pressed;
}

void InjectorFlasher::run()
{
	_serviceUsb();
	_serviceInjection();
	_servicePanel();
}

void InjectorFlasher::onUsbReceive(const uint8_t *data, uint32_t length)
{
	for (uint32_t byte_number = 0; byte_number < length; ++byte_number) {
		if (!_queueUsbByte(data[byte_number])) {
			return;
		}
	}
}

// The PC sends one request and waits, so a full ring means a broken sender and the newest bytes are dropped.
bool InjectorFlasher::_queueUsbByte(uint8_t byte)
{
	const uint32_t head = _rx_head.load(std::memory_order_relaxed);
	const uint32_t tail = _rx_tail.load(std::memory_order_acquire);
	const bool ring_full = (head - tail) >= usb_ring_size;

	if (ring_full) {
		return false;
	}

	_rx_ring[head % usb_ring_size] = byte;
	_rx_head.store(head + 1, std::memory_order_release);

	return true;
}

// ---- PC link ----

void InjectorFlasher::_serviceUsb()
{
	_sendPendingReply();

	// A request is not read while its predecessor's response is still waiting to leave.
	if (_reply_pending) {
		return;
	}

	_dropStalledRequest();
	_parsePendingBytes();
}

void InjectorFlasher::_dropStalledRequest()
{
	const bool request_started = _parser_state != ParserState::WaitingForMagic;
	const bool request_stalled = (HAL_GetTick() - _last_byte_ms) > request_timeout_ms;

	if (!request_started || !request_stalled) {
		return;
	}

	_parser_state = ParserState::WaitingForMagic;
}

void InjectorFlasher::_parsePendingBytes()
{
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
		_parseMagicByte(byte);
		break;

	case ParserState::ReadingHeader:
		_parseHeaderByte(byte);
		break;

	case ParserState::ReadingPayload:
		_parsePayloadByte(byte);
		break;
	}
}

void InjectorFlasher::_parseMagicByte(uint8_t byte)
{
	if (byte != request_magic) {
		return;
	}

	_header_received = 0;
	_parser_state = ParserState::ReadingHeader;
}

void InjectorFlasher::_parseHeaderByte(uint8_t byte)
{
	_header_bytes[_header_received] = byte;
	_header_received++;

	const bool header_complete = _header_received == sizeof(_header_bytes);

	if (!header_complete) {
		return;
	}

	_finishHeader();
}

void InjectorFlasher::_finishHeader()
{
	_request_command = static_cast<LinkCommand>(_header_bytes[0]);
	_request_length = _header_bytes[1] | (_header_bytes[2] << 8);
	_payload_received = 0;

	const bool payload_too_long = _request_length > max_payload_bytes;

	if (payload_too_long) {
		_parser_state = ParserState::WaitingForMagic;
		return _reply(LinkCode::BadLength, nullptr, 0);
	}

	const bool has_payload = _request_length != 0;

	if (has_payload) {
		_parser_state = ParserState::ReadingPayload;
		return;
	}

	_parser_state = ParserState::WaitingForMagic;
	_handleRequest();
}

void InjectorFlasher::_parsePayloadByte(uint8_t byte)
{
	_request_payload[_payload_received] = byte;
	_payload_received++;

	const bool payload_complete = _payload_received == _request_length;

	if (!payload_complete) {
		return;
	}

	_parser_state = ParserState::WaitingForMagic;
	_handleRequest();
}

void InjectorFlasher::_handleRequest()
{
	switch (_request_command) {
	case LinkCommand::GetStatus:
		_handleGetStatus();
		break;

	case LinkCommand::BeginImage:
		_handleBeginImage();
		break;

	case LinkCommand::WriteChunk:
		_handleWriteChunk();
		break;

	case LinkCommand::EndImage:
		_handleEndImage();
		break;

	case LinkCommand::DeleteImage:
		_handleDeleteImage();
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
	status.pages_done = static_cast<uint16_t>(_inject_page);
	status.inject_state = static_cast<uint8_t>(_inject_state);
	status.inject_error = static_cast<uint8_t>(_inject_error);
	status.target_error_code = _target_error_code;
	status.image_size_bytes = _has_image ? _header.image_size_bytes : 0;
	status.image_crc = _has_image ? _header.image_crc : 0;

	_reply(LinkCode::Ok, &status, sizeof(status));
}

void InjectorFlasher::_handleBeginImage()
{
	if (_isInjecting()) {
		return _reply(LinkCode::Busy, nullptr, 0);
	}

	if (_request_length != sizeof(BeginImagePayload)) {
		return _reply(LinkCode::BadLength, nullptr, 0);
	}

	BeginImagePayload incoming;
	memcpy(&incoming, _request_payload, sizeof(incoming));

	if (!imageDescriptionIsValid(incoming)) {
		return _reply(LinkCode::BadImage, nullptr, 0);
	}

	// The stored image is given up here: without a header there is no image, whatever happens next.
	_forgetStoredImage();

	if (!eraseSectorStartingAt(header_sector_address)) {
		_receiving = false;
		return _reply(LinkCode::FlashFailed, nullptr, 0);
	}

	_startReceiving(incoming);
	_reply(LinkCode::Ok, nullptr, 0);
}

void InjectorFlasher::_handleWriteChunk()
{
	if (!_receiving) {
		return _reply(LinkCode::BadState, nullptr, 0);
	}

	if (!_chunkCarriesWords()) {
		return _reply(LinkCode::BadLength, nullptr, 0);
	}

	const uint32_t offset = readU32(_request_payload);
	const uint32_t data_bytes = _request_length - chunk_offset_bytes;

	if (offset != _expected_offset) {
		return _reply(LinkCode::BadOffset, nullptr, 0);
	}

	if (!_chunkFitsInImage(offset, data_bytes)) {
		return _reply(LinkCode::BadLength, nullptr, 0);
	}

	if (!_appendChunkWords(data_bytes)) {
		_receiving = false;
		return _reply(LinkCode::FlashFailed, nullptr, 0);
	}

	_expected_offset += data_bytes;
	_reply(LinkCode::Ok, &_expected_offset, sizeof(_expected_offset));
}

void InjectorFlasher::_handleEndImage()
{
	if (!_receiving) {
		return _reply(LinkCode::BadState, nullptr, 0);
	}

	if (!_incomingImageIsComplete()) {
		return _reply(LinkCode::BadState, nullptr, 0);
	}

	_receiving = false;

	if (!_incomingImageCrcMatches()) {
		return _reply(LinkCode::CrcMismatch, nullptr, 0);
	}

	if (!_storeHeader()) {
		return _reply(LinkCode::FlashFailed, nullptr, 0);
	}

	_has_image = true;
	_reply(LinkCode::Ok, nullptr, 0);
}

void InjectorFlasher::_handleDeleteImage()
{
	if (_isInjecting()) {
		return _reply(LinkCode::Busy, nullptr, 0);
	}

	if (!_deleteStoredImage()) {
		return _reply(LinkCode::FlashFailed, nullptr, 0);
	}

	_reply(LinkCode::Ok, nullptr, 0);
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

	if (!magic_present || !pageCountIsValid(stored->page_count)) {
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

// Without a stored image the injection state is blank too.
void InjectorFlasher::_forgetStoredImage()
{
	_has_image = false;
	_inject_state = InjectState::Idle;
	_inject_error = InjectError::None;
	_target_error_code = 0;
	_inject_page = 0;
}

bool InjectorFlasher::_deleteStoredImage()
{
	_forgetStoredImage();
	_receiving = false;

	return eraseSectorStartingAt(header_sector_address);
}

void InjectorFlasher::_startReceiving(const BeginImagePayload &incoming)
{
	_incoming = incoming;
	_receiving = true;
	_expected_offset = 0;
	_words_in_page = 0;
	_pages_written = 0;
}

// A chunk is its offset and then whole flash words, at least one.
bool InjectorFlasher::_chunkCarriesWords() const
{
	return _request_length > chunk_offset_bytes && (_request_length - chunk_offset_bytes) % sizeof(uint32_t) == 0;
}

bool InjectorFlasher::_chunkFitsInImage(uint32_t offset, uint32_t data_bytes) const
{
	const uint32_t image_bytes = _incoming.page_count * page_size_bytes;

	return offset + data_bytes <= image_bytes;
}

bool InjectorFlasher::_appendChunkWords(uint32_t data_bytes)
{
	for (uint32_t byte_number = 0; byte_number < data_bytes; byte_number += sizeof(uint32_t)) {
		const uint32_t word = readU32(&_request_payload[chunk_offset_bytes + byte_number]);

		if (!_appendWord(word)) {
			return false;
		}
	}

	return true;
}

bool InjectorFlasher::_appendWord(uint32_t word)
{
	_page_buffer_words[_words_in_page] = word;
	_words_in_page++;

	const bool page_full = _words_in_page == page_size_words;

	if (!page_full) {
		return true;
	}

	return _commitPage();
}

// Pages arrive in order, so a sector is erased when the page that opens it arrives.
bool InjectorFlasher::_commitPage()
{
	const uint32_t page_address = image_address + _pages_written * page_size_bytes;
	const bool starts_sector = flash_layout::isSectorStart(page_address);

	if (starts_sector && !eraseSectorStartingAt(page_address)) {
		return false;
	}

	if (!programWords(page_address, _page_buffer_words, page_size_words)) {
		return false;
	}

	_pages_written++;
	_words_in_page = 0;

	return true;
}

bool InjectorFlasher::_incomingImageIsComplete() const
{
	const uint32_t image_bytes = _incoming.page_count * page_size_bytes;

	return _expected_offset == image_bytes;
}

bool InjectorFlasher::_incomingImageCrcMatches() const
{
	const uint32_t stored_crc = calculateWordsCrc(imageWords(), _incoming.page_count * page_size_words);

	return stored_crc == _incoming.image_crc;
}

// The header is written last, so a transfer that stops halfway leaves no valid image.
bool InjectorFlasher::_storeHeader()
{
	ImageHeader header{};
	header.magic = header_magic;
	header.image_size_bytes = _incoming.image_size_bytes;
	header.image_crc = _incoming.image_crc;
	header.page_count = _incoming.page_count;
	header.target_node_id = _incoming.target_node_id;

	const bool header_stored = programWords(header_sector_address, reinterpret_cast<const uint32_t *>(&header),
					 sizeof(header) / sizeof(uint32_t));

	if (!header_stored) {
		return false;
	}

	_header = header;

	return true;
}

bool InjectorFlasher::_isInjecting() const
{
	switch (_inject_state) {
	case InjectState::Connecting:
	case InjectState::SendingWords:
	case InjectState::WritingPage:
	case InjectState::WritingCrc:
	case InjectState::StartingApp:
		return true;

	case InjectState::Idle:
	case InjectState::Done:
	case InjectState::Failed:
		return false;
	}

	return false;
}

// ---- the injection into the target ----

void InjectorFlasher::_startInjection()
{
	const bool can_start = _has_image && !_receiving && !_isInjecting();

	if (!can_start) {
		return;
	}

	_resetInjectionProgress();

	if (!_can_ready) {
		return _failInjection(InjectError::CanNotStarted, 0);
	}

	_inject_state = InjectState::Connecting;
}

void InjectorFlasher::_resetInjectionProgress()
{
	_inject_error = InjectError::None;
	_target_error_code = 0;
	_inject_page = 0;
	_inject_word = 0;
	_trigger_sent = false;
	_ping_count = 0;
	_command_in_flight = false;
	_send_attempt_open = false;
}

void InjectorFlasher::_serviceInjection()
{
	// Drain the CAN FIFO on every pass, so a late reply never waits in it.
	_readCanReplies();

	switch (_inject_state) {
	case InjectState::Connecting:
		_stepConnecting();
		break;

	case InjectState::SendingWords:
		_stepSendingWords();
		break;

	case InjectState::WritingPage:
		_stepWritingPage();
		break;

	case InjectState::WritingCrc:
		_stepWritingCrc();
		break;

	case InjectState::StartingApp:
		_stepStartingApp();
		break;

	case InjectState::Idle:
	case InjectState::Done:
	case InjectState::Failed:
		break;
	}
}

void InjectorFlasher::_stepSendingWords()
{
	const uint32_t word = imageWords()[_inject_page * page_size_words + _inject_word];
	const Progress progress = _runCommand(BootCommand::WriteBuffer, static_cast<uint16_t>(_inject_word), word,
					      reply_timeout_ms);

	if (progress != Progress::Succeeded) {
		return;
	}

	_inject_word++;

	const bool page_complete = _inject_word == page_size_words;

	if (!page_complete) {
		return;
	}

	_page_crc = calculateWordsCrc(&imageWords()[_inject_page * page_size_words], page_size_words);
	_inject_word = 0;
	_inject_state = InjectState::WritingPage;
}

void InjectorFlasher::_stepWritingPage()
{
	const Progress progress = _runCommand(BootCommand::WritePage, static_cast<uint16_t>(_inject_page), _page_crc,
					      write_page_timeout_ms);

	if (progress != Progress::Succeeded) {
		return;
	}

	_inject_page++;

	const bool last_page_done = _inject_page == _header.page_count;

	if (last_page_done) {
		_inject_state = InjectState::WritingCrc;
		return;
	}

	_inject_state = InjectState::SendingWords;
}

void InjectorFlasher::_stepWritingCrc()
{
	const Progress progress = _runCommand(BootCommand::WriteCrc, static_cast<uint16_t>(_header.page_count),
					      _header.image_crc, write_crc_timeout_ms);

	if (progress != Progress::Succeeded) {
		return;
	}

	_inject_state = InjectState::StartingApp;
}

void InjectorFlasher::_stepStartingApp()
{
	const Progress progress = _runCommand(BootCommand::StartApp, 0, 0, start_app_timeout_ms);

	if (progress != Progress::Succeeded) {
		return;
	}

	_inject_state = InjectState::Done;
}

void InjectorFlasher::_readCanReplies()
{
	while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
		CAN_RxHeaderTypeDef header;
		uint8_t data[8];

		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data) != HAL_OK) {
			return;
		}

		if (!isBootResponse(header)) {
			continue;
		}

		BootResponseFrame response;
		memcpy(&response, data, sizeof(response));

		_acceptReply(response);
	}
}

// Keep the reply only when it answers the command that is waiting, from the board that was asked.
void InjectorFlasher::_acceptReply(const BootResponseFrame &response)
{
	const bool from_target = response.node_id == _header.target_node_id;
	const bool answers_command = response.command == static_cast<uint8_t>(_awaiting_command)
				     && response.index == _awaiting_index;

	if (!_command_in_flight || !from_target || !answers_command) {
		return;
	}

	_boot_reply = response;
	_reply_valid = true;
}

bool InjectorFlasher::_mailboxAvailable()
{
	const bool mailbox_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0;
	const uint32_t now_ms = HAL_GetTick();

	if (mailbox_free) {
		_send_attempt_open = false;
		return true;
	}

	if (!_send_attempt_open) {
		_send_attempt_open = true;
		_send_attempt_started_ms = now_ms;
		return false;
	}

	const bool waited_too_long = (now_ms - _send_attempt_started_ms) > tx_mailbox_timeout_ms;

	if (waited_too_long) {
		_failInjection(InjectError::BusDead, 0);
	}

	return false;
}

bool InjectorFlasher::_sendBootCommand(BootCommand command, uint16_t index, uint32_t value)
{
	if (!_mailboxAvailable()) {
		return false;
	}

	BootCommandFrame frame{};
	frame.target_node_id = static_cast<uint8_t>(_header.target_node_id);
	frame.command = static_cast<uint8_t>(command);
	frame.index = index;
	frame.value = value;

	if (!queueCanFrame(boot_command_can_id, reinterpret_cast<const uint8_t *>(&frame), sizeof(frame))) {
		return false;
	}

	_awaiting_command = command;
	_awaiting_index = index;
	_reply_valid = false;
	_command_sent_ms = HAL_GetTick();

	return true;
}

bool InjectorFlasher::_sendTrigger()
{
	if (!_mailboxAvailable()) {
		return false;
	}

	// The trigger is: node id, then the magic as two little-endian bytes. A bootloader ignores it.
	const uint8_t data[3] = {static_cast<uint8_t>(_header.target_node_id),
				 static_cast<uint8_t>(app_trigger_magic & 0xFF),
				 static_cast<uint8_t>(app_trigger_magic >> 8)};

	return queueCanFrame(app_trigger_can_id, data, sizeof(data));
}

void InjectorFlasher::_failInjection(InjectError error, uint8_t target_error_code)
{
	_command_in_flight = false;
	_inject_error = error;
	_target_error_code = target_error_code;
	_inject_state = InjectState::Failed;
}

InjectorFlasher::Progress InjectorFlasher::_runCommand(BootCommand command, uint16_t index, uint32_t value,
		uint32_t timeout_ms)
{
	if (!_command_in_flight) {
		_startCommand(command, index, value);
		return Progress::InProgress;
	}

	if (_reply_valid) {
		return _finishCommand();
	}

	if (!_commandTimedOut(timeout_ms)) {
		return Progress::InProgress;
	}

	if (_retries >= command_retries) {
		_failInjection(InjectError::NoReply, 0);
		return Progress::Failed;
	}

	// A word or a page sent twice is harmless, so a lost reply is asked for again.
	if (_sendBootCommand(command, index, value)) {
		_retries++;
	}

	return Progress::InProgress;
}

void InjectorFlasher::_startCommand(BootCommand command, uint16_t index, uint32_t value)
{
	if (!_sendBootCommand(command, index, value)) {
		return;
	}

	_command_in_flight = true;
	_retries = 0;
}

InjectorFlasher::Progress InjectorFlasher::_finishCommand()
{
	_command_in_flight = false;

	const bool accepted = _boot_reply.code == 0;

	if (accepted) {
		return Progress::Succeeded;
	}

	_failInjection(InjectError::TargetRefused, _boot_reply.code);

	return Progress::Failed;
}

bool InjectorFlasher::_commandTimedOut(uint32_t timeout_ms) const
{
	return (HAL_GetTick() - _command_sent_ms) >= timeout_ms;
}

void InjectorFlasher::_stepConnecting()
{
	// First the trigger, which resets a running app into its bootloader.
	if (!_trigger_sent) {
		_trigger_sent = _sendTrigger();
		return;
	}

	if (!_command_in_flight) {
		return _sendPing();
	}

	if (_reply_valid) {
		return _startSendingWords();
	}

	if (!_commandTimedOut(ping_timeout_ms)) {
		return;
	}

	_command_in_flight = false;
}

void InjectorFlasher::_sendPing()
{
	if (_ping_count >= ping_tries) {
		return _failInjection(InjectError::CannotConnect, 0);
	}

	if (!_sendBootCommand(BootCommand::Ping, 0, 0)) {
		return;
	}

	_command_in_flight = true;
	_ping_count++;
}

// The bootloader answered a ping, so the first word of the first page goes next.
void InjectorFlasher::_startSendingWords()
{
	_command_in_flight = false;
	_inject_page = 0;
	_inject_word = 0;
	_inject_state = InjectState::SendingWords;
}

// ---- the front panel ----

void InjectorFlasher::_servicePanel()
{
	const uint32_t now_ms = HAL_GetTick();

	_updateButton(_inject_button, BUTTON_INJECT_GPIO_Port, BUTTON_INJECT_Pin, now_ms);
	_updateButton(_delete_button, BUTTON_DELETE_GPIO_Port, BUTTON_DELETE_Pin, now_ms);

	_runInjectButton(now_ms);
	_runDeleteButton(now_ms);
	_updateLed(now_ms);
}

void InjectorFlasher::_runInjectButton(uint32_t now_ms)
{
	const bool pressed_long_enough = _inject_button.pressed && !_inject_button.action_done
					 && (now_ms - _inject_button.pressed_since_ms) >= button_debounce_ms;

	if (!pressed_long_enough) {
		return;
	}

	_inject_button.action_done = true;
	_startInjection();
}

void InjectorFlasher::_runDeleteButton(uint32_t now_ms)
{
	const bool held_long_enough = _delete_button.pressed && !_delete_button.action_done
				      && (now_ms - _delete_button.pressed_since_ms) >= delete_hold_ms;

	if (!held_long_enough) {
		return;
	}

	_delete_button.action_done = true;

	// The stored image is neither deleted under an injection nor under a transfer from the PC.
	if (_isInjecting() || _receiving) {
		return;
	}

	_deleteStoredImage();
}

void InjectorFlasher::_updateButton(Button &button, GPIO_TypeDef *port, uint16_t pin, uint32_t now_ms)
{
	const bool pressed_now = isPressed(port, pin);

	if (pressed_now && !button.pressed) {
		button.pressed_since_ms = now_ms;
	}

	if (!pressed_now) {
		button.action_done = false;
	}

	button.pressed = pressed_now;
}

void InjectorFlasher::_updateLed(uint32_t now_ms)
{
	// Off without an image, on with one, blinking while an injection runs.
	const bool blink_phase_on = ((now_ms / led_blink_half_period_ms) % 2) == 0;
	const bool lit = _isInjecting() ? blink_phase_on : _has_image;

	HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, lit ? led_on_level : led_off_level);
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
