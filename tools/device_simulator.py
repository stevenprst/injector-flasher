"""A software model of the injector flasher's USB link (Core/Src/mainloop.cpp), for testing the PC side without a board.

It follows the rules of the firmware: the request parser with its timeout, the image receive path (page buffer, sector
erase when a page opens a sector, header written last), the CRC check at the end, and the Busy answer during an
injection. Nothing is imported from injector_link: the protocol numbers and the CRC are written again here from the
description of the link, so a mistake in the PC side fails a test instead of agreeing with itself.

    simulator = SimulatedInjector()
    port = FakeSerial(simulator)        # give this to InjectorLink
"""

import struct

REQUEST_MAGIC = 0xA5
RESPONSE_MAGIC = 0x5A

GET_STATUS = 1
BEGIN_IMAGE = 2
WRITE_CHUNK = 3
END_IMAGE = 4
DELETE_IMAGE = 5

OK = 0
BAD_COMMAND = 1
BAD_LENGTH = 2
BAD_OFFSET = 3
FLASH_FAILED = 4
CRC_MISMATCH = 5
BUSY = 6
NO_IMAGE = 7
BAD_STATE = 8
BAD_IMAGE = 9

MAX_PAYLOAD_BYTES = 60
PAGE_SIZE_BYTES = 1024
MAX_PAGE_COUNT = 896
REQUEST_TIMEOUT_S = 0.2
LINK_VERSION = 1

IMAGE_SIZE_BYTES = 896 * 1024
SECTOR_SIZE_BYTES = 128 * 1024


def crc_bit_by_bit(words):
    """The STM32 hardware CRC, one bit at a time: polynomial 0x04C11DB7, start 0xFFFFFFFF, no reflection."""
    crc = 0xFFFFFFFF

    for word in words:
        crc ^= word

        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF

    return crc


def crc_of_bytes(data):
    return crc_bit_by_bit(struct.unpack("<%dI" % (len(data) // 4), data))


class SimulatedInjector:
    def __init__(self):
        self.now_s = 0.0

        # flash: erased bytes read 0xFF, programming can only clear bits
        self.image_area = bytearray(b"\xFF" * IMAGE_SIZE_BYTES)
        self.header = None			# (size, crc, page_count, node) or None when the header sector is erased
        self.erased_sectors = []		# image sector numbers in the order they were erased

        self.receiving = False
        self.incoming = None			# (size, crc, page_count, node)
        self.expected_offset = 0
        self.page_buffer = []
        self.pages_written = 0

        self.injecting = False			# a test sets this to model a running injection
        self.inject_state = 0
        self.inject_error = 0
        self.target_error_code = 0
        self.pages_done = 0

        # test hooks
        self.damage_word_at_offset = None	# flip a bit of the word received at this offset
        self.fail_next_header_erase = False

        self._state = "magic"
        self._header_bytes = bytearray()
        self._payload = bytearray()
        self._command = 0
        self._length = 0
        self._last_byte_s = 0.0
        self._output = bytearray()

    # ---- bytes in, bytes out ----

    def advance(self, seconds):
        self.now_s += seconds

    def receive(self, data):
        """Bytes from the PC."""
        for byte in data:
            self._feed(byte)

    def take_output(self, count):
        taken = bytes(self._output[:count])
        del self._output[:count]

        return taken

    def _feed(self, byte):
        started = self._state != "magic"

        if started and (self.now_s - self._last_byte_s) > REQUEST_TIMEOUT_S:
            self._state = "magic"

        self._last_byte_s = self.now_s

        if self._state == "magic":
            if byte == REQUEST_MAGIC:
                self._header_bytes = bytearray()
                self._state = "header"

            return

        if self._state == "header":
            self._header_bytes.append(byte)

            if len(self._header_bytes) < 3:
                return

            self._command = self._header_bytes[0]
            self._length = self._header_bytes[1] | (self._header_bytes[2] << 8)
            self._payload = bytearray()

            if self._length > MAX_PAYLOAD_BYTES:
                self._state = "magic"
                self._respond(BAD_LENGTH)
            elif self._length == 0:
                self._state = "magic"
                self._handle()
            else:
                self._state = "payload"

            return

        self._payload.append(byte)

        if len(self._payload) == self._length:
            self._state = "magic"
            self._handle()

    def _respond(self, code, payload=b""):
        self._output.extend(struct.pack("<BBBB", RESPONSE_MAGIC, self._command, code, len(payload)) + payload)

    # ---- requests ----

    def _handle(self):
        handlers = {GET_STATUS: self._get_status, BEGIN_IMAGE: self._begin_image, WRITE_CHUNK: self._write_chunk,
                    END_IMAGE: self._end_image, DELETE_IMAGE: self._delete_image}
        handler = handlers.get(self._command)

        if handler is None:
            self._respond(BAD_COMMAND)
            return

        handler()

    def _get_status(self):
        has_image = self.header is not None
        size, crc, page_count, node = self.header if has_image else (0, 0, 0, 0)
        status = struct.pack("<BBBBHHBBBII", LINK_VERSION, 1 if self.receiving else 0, 1 if has_image else 0, node,
                             page_count, self.pages_done, self.inject_state, self.inject_error,
                             self.target_error_code, size, crc)
        self._respond(OK, status)

    def _begin_image(self):
        if self.injecting:
            self._respond(BUSY)
            return

        if self._length != 11:
            self._respond(BAD_LENGTH)
            return

        size, crc, page_count, node = struct.unpack("<IIHB", bytes(self._payload))
        page_count_valid = 1 <= page_count <= MAX_PAGE_COUNT
        size_fits = (page_count - 1) * PAGE_SIZE_BYTES < size <= page_count * PAGE_SIZE_BYTES

        if not (page_count_valid and size_fits):
            self._respond(BAD_IMAGE)
            return

        # the stored image is given up here, whatever happens next
        self.header = None
        self.inject_state = 0
        self.inject_error = 0
        self.target_error_code = 0
        self.pages_done = 0

        if self.fail_next_header_erase:
            self.fail_next_header_erase = False
            self.receiving = False
            self._respond(FLASH_FAILED)
            return

        self.incoming = (size, crc, page_count, node)
        self.receiving = True
        self.expected_offset = 0
        self.page_buffer = []
        self.pages_written = 0
        self._respond(OK)

    def _write_chunk(self):
        if not self.receiving:
            self._respond(BAD_STATE)
            return

        data_bytes = self._length - 4
        carries_words = self._length > 4 and data_bytes % 4 == 0

        if not carries_words:
            self._respond(BAD_LENGTH)
            return

        (offset,) = struct.unpack("<I", bytes(self._payload[:4]))

        if offset != self.expected_offset:
            self._respond(BAD_OFFSET)
            return

        image_bytes = self.incoming[2] * PAGE_SIZE_BYTES

        if offset + data_bytes > image_bytes:
            self._respond(BAD_LENGTH)
            return

        for byte_number in range(0, data_bytes, 4):
            (word,) = struct.unpack("<I", bytes(self._payload[4 + byte_number:8 + byte_number]))

            if self.damage_word_at_offset == offset + byte_number:
                word ^= 0x00010000
                self.damage_word_at_offset = None

            self.page_buffer.append(word)

            if len(self.page_buffer) == 256:
                self._program_page()

        self.expected_offset += data_bytes
        self._respond(OK, struct.pack("<I", self.expected_offset))

    def _program_page(self):
        page_start = self.pages_written * PAGE_SIZE_BYTES

        # a page that opens a sector erases it first
        if page_start % SECTOR_SIZE_BYTES == 0:
            self.image_area[page_start:page_start + SECTOR_SIZE_BYTES] = b"\xFF" * SECTOR_SIZE_BYTES
            self.erased_sectors.append(page_start // SECTOR_SIZE_BYTES)

        page = struct.pack("<256I", *self.page_buffer)

        for index, byte in enumerate(page):
            self.image_area[page_start + index] &= byte

        self.page_buffer = []
        self.pages_written += 1

    def _end_image(self):
        if not self.receiving:
            self._respond(BAD_STATE)
            return

        image_bytes = self.incoming[2] * PAGE_SIZE_BYTES

        if self.expected_offset != image_bytes:
            self._respond(BAD_STATE)
            return

        self.receiving = False
        stored_crc = crc_of_bytes(bytes(self.image_area[:image_bytes]))

        if stored_crc != self.incoming[1]:
            self._respond(CRC_MISMATCH)
            return

        self.header = self.incoming
        self._respond(OK)

    def _delete_image(self):
        if self.injecting:
            self._respond(BUSY)
            return

        self.header = None
        self.receiving = False
        self.inject_state = 0
        self.inject_error = 0
        self.target_error_code = 0
        self.pages_done = 0
        self._respond(OK)

    def stored_image(self):
        """The bytes of the image in the simulated flash, or None when there is no valid image."""
        if self.header is None:
            return None

        return bytes(self.image_area[:self.header[2] * PAGE_SIZE_BYTES])


class FakeSerial:
    """The port InjectorLink talks to, connected to a SimulatedInjector. write() may be split into small pieces."""

    def __init__(self, simulator, write_piece_bytes=0):
        self.simulator = simulator
        self.write_piece_bytes = write_piece_bytes

    def write(self, data):
        piece = self.write_piece_bytes or len(data)

        for start in range(0, len(data), piece):
            self.simulator.receive(data[start:start + piece])

    def read(self, count):
        return self.simulator.take_output(count)

    def reset_input_buffer(self):
        self.simulator.take_output(1 << 20)
