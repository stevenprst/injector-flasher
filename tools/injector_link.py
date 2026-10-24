"""The PC side of the USB link to the injector flasher (a virtual COM port).

A request is: 0xA5, command, length (u16 little-endian), payload.
A response is: 0x5A, command, code, length (u8), payload.
The PC sends one request and waits for its response. Everything is little-endian.

    link = InjectorLink(serial.Serial("COM7", 115200, timeout=0.1))
    link.send_image(open("app.bin", "rb").read(), node_id=1)
    print(describe_status(link.get_status()))
"""

import collections
import struct
import time

REQUEST_MAGIC = 0xA5
RESPONSE_MAGIC = 0x5A

COMMAND_GET_STATUS = 1
COMMAND_BEGIN_IMAGE = 2
COMMAND_WRITE_CHUNK = 3
COMMAND_END_IMAGE = 4
COMMAND_DELETE_IMAGE = 5

PAGE_SIZE_BYTES = 1024
MAX_PAGE_COUNT = 896               # the image area of the injector is 896 KB
MAX_CHUNK_DATA_BYTES = 56          # a request carries 60 bytes: a 4 byte offset and the data
CHUNK_OFFSET_BYTES = 4

# A header sector erase (BeginImage) and the erase of a 128 KB sector (a chunk that opens one) stall the injector for
# up to a few seconds, so a reply is waited for much longer than a USB round trip.
REPLY_TIMEOUT_S = 10.0

# The image has to be a valid app for the CAN bootloader: linked at 0x08008000.
APP_ADDRESS = 0x08008000
APP_END_ADDRESS = 0x08100000
RAM_BEGIN_ADDRESS = 0x20000000
RAM_END_ADDRESS = 0x20020000
CCM_RAM_BEGIN_ADDRESS = 0x10000000
CCM_RAM_END_ADDRESS = 0x10010000

CODE_NAMES = {
    0: "ok",
    1: "unknown command",
    2: "wrong payload length",
    3: "chunk is not the next one",
    4: "flash erase or program failed",
    5: "image CRC mismatch (the image was damaged on the way)",
    6: "an injection is running",
    7: "no image stored",
    8: "wrong state (no BeginImage, or the image is incomplete)",
    9: "the sizes in BeginImage do not agree or are too big",
}

INJECT_STATE_NAMES = {
    0: "idle",
    1: "connecting to the bootloader",
    2: "sending words",
    3: "writing a page",
    4: "checking the image",
    5: "starting the app",
    6: "done",
    7: "failed",
}

INJECT_ERROR_NAMES = {
    0: "none",
    1: "no bootloader answered (is the app running with the trigger code, is the node id right?)",
    2: "a command went unanswered",
    3: "the bootloader refused a command",
    4: "the CAN bus is dead (no transmit mailbox freed up)",
    5: "CAN did not start",
}

CRC_POLYNOMIAL = 0x04C11DB7
CRC_INITIAL_VALUE = 0xFFFFFFFF

Status = collections.namedtuple(
    "Status",
    "link_version receiving has_image target_node_id page_count pages_done inject_state inject_error "
    "target_error_code image_size_bytes image_crc")

STATUS_FORMAT = "<BBBBHHBBBII"


class LinkError(Exception):
    pass


def build_crc_table():
    table = []

    for top_byte in range(256):
        crc = top_byte << 24

        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ CRC_POLYNOMIAL
            else:
                crc = crc << 1

            crc &= 0xFFFFFFFF

        table.append(crc)

    return table


CRC_TABLE = build_crc_table()


def stm32_crc(data):
    """CRC of the STM32 hardware CRC unit fed with 32-bit little-endian words.

    The unit shifts each word in from its top bit, so the bytes of a little-endian word go in reversed.
    """
    if len(data) % 4 != 0:
        raise ValueError("the CRC is calculated over whole 32-bit words")

    crc = CRC_INITIAL_VALUE

    for word_start in range(0, len(data), 4):
        word_bytes_top_first = data[word_start:word_start + 4][::-1]

        for byte in word_bytes_top_first:
            crc = ((crc << 8) & 0xFFFFFFFF) ^ CRC_TABLE[(crc >> 24) ^ byte]

    return crc


def check_image(data):
    """Refuse a file the injector or the CAN bootloader would refuse, before anything is sent."""
    page_count = (len(data) + PAGE_SIZE_BYTES - 1) // PAGE_SIZE_BYTES

    if len(data) < 8:
        raise LinkError("the file is too short to be an app image")

    if page_count > MAX_PAGE_COUNT:
        raise LinkError("the image is %d pages, the injector holds %d" % (page_count, MAX_PAGE_COUNT))

    stack_pointer, reset_vector = struct.unpack("<II", data[:8])

    stack_in_ram = RAM_BEGIN_ADDRESS <= stack_pointer <= RAM_END_ADDRESS
    stack_in_ccm_ram = CCM_RAM_BEGIN_ADDRESS <= stack_pointer <= CCM_RAM_END_ADDRESS
    if not (stack_in_ram or stack_in_ccm_ram):
        raise LinkError("the first word 0x%08X is not a RAM address, this is not an app image" % stack_pointer)

    reset_is_thumb = (reset_vector & 1) == 1
    reset_address = reset_vector & ~1
    reset_in_app = APP_ADDRESS <= reset_address < APP_END_ADDRESS
    if not (reset_is_thumb and reset_in_app):
        raise LinkError(
            "the reset handler 0x%08X is not inside 0x%08X to 0x%08X, was the app linked at 0x08000000?"
            % (reset_vector, APP_ADDRESS, APP_END_ADDRESS))


def pad_to_pages(data):
    """The image as the injector stores it: whole 1 KB pages, the last one padded with zeros."""
    padding_bytes = (-len(data)) % PAGE_SIZE_BYTES

    return bytes(data) + bytes(padding_bytes)


def describe_status(status):
    """The status as lines of text for a person."""
    if status.has_image:
        image_line = "image: %d bytes (%d pages), CRC 0x%08X, for node %d" % (
            status.image_size_bytes, status.page_count, status.image_crc, status.target_node_id)
    elif status.receiving:
        image_line = "image: being received"
    else:
        image_line = "image: none"

    inject_line = "injection: %s" % INJECT_STATE_NAMES.get(status.inject_state, "unknown")

    if status.inject_state in (1, 2, 3, 4, 5):
        inject_line += " (page %d of %d)" % (status.pages_done, status.page_count)

    lines = [image_line, inject_line]

    if status.inject_error != 0:
        reason = INJECT_ERROR_NAMES.get(status.inject_error, "unknown")

        if status.inject_error == 3:
            reason += ", code %d" % status.target_error_code

        lines.append("error: " + reason)

    return lines


class InjectorLink:
    """Requests and responses on a serial-like port: write(bytes), read(n) that waits up to its own timeout."""

    def __init__(self, port, reply_timeout_s=REPLY_TIMEOUT_S):
        self.port = port
        self.reply_timeout_s = reply_timeout_s

    def _read_exact(self, count, deadline_s):
        data = bytearray()

        while len(data) < count:
            if time.monotonic() > deadline_s:
                raise LinkError("no response from the injector")

            data.extend(self.port.read(count - len(data)))

        return bytes(data)

    def _request(self, command, payload=b""):
        """Send one request and return (code, payload) of its response."""
        if len(payload) > 60:
            raise ValueError("a payload is at most 60 bytes")

        if hasattr(self.port, "reset_input_buffer"):
            self.port.reset_input_buffer()

        self.port.write(struct.pack("<BBH", REQUEST_MAGIC, command, len(payload)) + payload)

        deadline_s = time.monotonic() + self.reply_timeout_s

        # Skip anything before the magic, then read the rest of the header and the payload.
        while self._read_exact(1, deadline_s)[0] != RESPONSE_MAGIC:
            pass

        response_command, code, length = struct.unpack("<BBB", self._read_exact(3, deadline_s))
        response_payload = self._read_exact(length, deadline_s) if length else b""

        if response_command != command:
            raise LinkError("the response is for command %d, not %d" % (response_command, command))

        return code, response_payload

    def _request_ok(self, command, payload=b""):
        code, response_payload = self._request(command, payload)

        if code != 0:
            raise LinkError("the injector answered command %d with error %d: %s"
                            % (command, code, CODE_NAMES.get(code, "unknown")))

        return response_payload

    def get_status(self):
        payload = self._request_ok(COMMAND_GET_STATUS)

        return Status(*struct.unpack(STATUS_FORMAT, payload))

    def begin_image(self, image_size_bytes, image_crc, page_count, target_node_id):
        self._request_ok(COMMAND_BEGIN_IMAGE,
                         struct.pack("<IIHB", image_size_bytes, image_crc, page_count, target_node_id))

    def write_chunk(self, offset, data):
        payload = self._request_ok(COMMAND_WRITE_CHUNK, struct.pack("<I", offset) + data)
        (next_offset,) = struct.unpack("<I", payload)

        if next_offset != offset + len(data):
            raise LinkError("the injector expects offset %d after this chunk, not %d"
                            % (next_offset, offset + len(data)))

    def end_image(self):
        self._request_ok(COMMAND_END_IMAGE)

    def delete_image(self):
        self._request_ok(COMMAND_DELETE_IMAGE)

    def send_image(self, data, node_id, progress=None):
        """Check the file, send it in chunks and let the injector verify it. progress(bytes_sent, total) is optional."""
        check_image(data)

        if not 0 <= node_id <= 255:
            raise LinkError("a node id is 0 to 255")

        padded = pad_to_pages(data)
        page_count = len(padded) // PAGE_SIZE_BYTES

        self.begin_image(len(data), stm32_crc(padded), page_count, node_id)

        for offset in range(0, len(padded), MAX_CHUNK_DATA_BYTES):
            chunk = padded[offset:offset + MAX_CHUNK_DATA_BYTES]
            self.write_chunk(offset, chunk)

            if progress is not None:
                progress(offset + len(chunk), len(padded))

        self.end_image()
