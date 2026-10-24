"""Test the PC side of the injector link against the simulated injector. No board and no USB needed.

    python test_link.py
"""

import struct

import injector_link
from device_simulator import FakeSerial, SimulatedInjector, crc_bit_by_bit, crc_of_bytes
from injector_link import InjectorLink, LinkError

PAGE_BYTES = 1024


def make_image(size_bytes, reset_vector=0x08008101):
    """An app image with a valid vector table and recognisable words after it."""
    words = [0x20020000, reset_vector]
    words += [(0xA5000000 + index) & 0xFFFFFFFF for index in range((size_bytes + 3) // 4 - 2)]

    return struct.pack("<%dI" % len(words), *words)[:size_bytes]


def make_link(write_piece_bytes=0, reply_timeout_s=2.0):
    simulator = SimulatedInjector()
    link = InjectorLink(FakeSerial(simulator, write_piece_bytes), reply_timeout_s)

    return simulator, link


def expect_error(action, text):
    try:
        action()
    except LinkError as error:
        assert text in str(error), "expected %r in %r" % (text, str(error))
        return

    raise AssertionError("expected a LinkError containing %r" % text)


def test_crc_known_values_and_bit_by_bit():
    def words_to_bytes(words):
        return b"".join(struct.pack("<I", word) for word in words)

    assert injector_link.stm32_crc(words_to_bytes([0x12345678])) == 0xDF8A8A2B
    assert injector_link.stm32_crc(words_to_bytes([0, 0])) == 0x6904BB59
    assert injector_link.stm32_crc(words_to_bytes([0xDEADBEEF, 1, 0xFFFFFFFF, 0x08008000])) == 0xBC983515

    words = [(index * 0x9E3779B1) & 0xFFFFFFFF for index in range(256)]
    assert injector_link.stm32_crc(words_to_bytes(words)) == crc_bit_by_bit(words)

    try:
        injector_link.stm32_crc(b"\x01\x02\x03")
    except ValueError:
        return

    raise AssertionError("a length that is not a multiple of 4 must be refused")


def test_empty_injector_reports_no_image():
    simulator, link = make_link()

    status = link.get_status()

    assert status.link_version == 1
    assert status.has_image == 0 and status.receiving == 0
    assert injector_link.describe_status(status)[0] == "image: none"


def test_send_image_stores_it_with_its_header():
    simulator, link = make_link()
    image = make_image(5000)

    progress_log = []
    link.send_image(image, node_id=7, progress=lambda sent, total: progress_log.append((sent, total)))

    padded = image + bytes(6 * PAGE_BYTES - len(image))
    status = link.get_status()

    assert status.has_image == 1 and status.receiving == 0
    assert status.target_node_id == 7
    assert status.page_count == 5
    assert status.image_size_bytes == 5000
    assert status.image_crc == crc_of_bytes(padded[:5 * PAGE_BYTES])
    assert simulator.stored_image() == padded[:5 * PAGE_BYTES]
    assert progress_log[-1] == (5 * PAGE_BYTES, 5 * PAGE_BYTES)
    assert "5000 bytes" in injector_link.describe_status(status)[0]


def test_image_over_several_sectors_erases_each_sector_when_its_first_page_arrives():
    simulator, link = make_link()
    simulator.image_area[:] = b"\x00" * len(simulator.image_area)  # old data that only an erase clears
    image = make_image(300 * PAGE_BYTES - 10)

    link.send_image(image, node_id=1)

    assert simulator.erased_sectors == [0, 1, 2]
    assert simulator.stored_image()[:len(image)] == image


def test_chunks_survive_a_port_that_delivers_one_byte_at_a_time():
    simulator, link = make_link(write_piece_bytes=1)
    image = make_image(3000)

    link.send_image(image, node_id=2)

    assert simulator.stored_image()[:3000] == image


def test_garbage_before_a_request_is_ignored():
    simulator, link = make_link()
    simulator.receive(b"\x00\x13\xFF\x77")

    assert link.get_status().has_image == 0


def test_a_request_that_stops_half_way_is_dropped():
    simulator, link = make_link()

    simulator.receive(bytes([injector_link.REQUEST_MAGIC, injector_link.COMMAND_BEGIN_IMAGE, 11, 0, 1, 2]))
    simulator.advance(0.3)

    assert link.get_status().has_image == 0


def test_wrong_image_is_refused_by_the_pc_before_anything_is_sent():
    simulator, link = make_link()

    expect_error(lambda: link.send_image(make_image(2000, reset_vector=0x08000101), 1), "linked at 0x08000000")
    expect_error(lambda: link.send_image(b"\x00" * 100, 1), "not a RAM address")
    expect_error(lambda: link.send_image(make_image(897 * PAGE_BYTES), 1), "897 pages")
    expect_error(lambda: link.send_image(b"\x01", 1), "too short")
    assert simulator.erased_sectors == [] and not simulator.receiving


def test_chunk_that_is_not_the_next_one_is_refused():
    simulator, link = make_link()
    link.begin_image(2000, crc_of_bytes(bytes(2048)), 2, 1)

    expect_error(lambda: link.write_chunk(56, bytes(56)), "not the next one")
    link.write_chunk(0, bytes(56))
    expect_error(lambda: link.write_chunk(0, bytes(56)), "not the next one")


def test_chunk_without_begin_and_end_without_all_bytes_are_refused():
    simulator, link = make_link()

    expect_error(lambda: link.write_chunk(0, bytes(56)), "wrong state")
    expect_error(lambda: link.end_image(), "wrong state")

    link.begin_image(1024, crc_of_bytes(bytes(1024)), 1, 1)
    link.write_chunk(0, bytes(56))
    expect_error(lambda: link.end_image(), "wrong state")


def test_chunk_that_does_not_carry_whole_words_is_refused():
    simulator, link = make_link()
    link.begin_image(1024, crc_of_bytes(bytes(1024)), 1, 1)

    expect_error(lambda: link.write_chunk(0, bytes(5)), "wrong payload length")


def test_damaged_word_is_caught_by_the_final_crc_and_leaves_no_image():
    simulator, link = make_link()
    simulator.damage_word_at_offset = 2 * PAGE_BYTES + 100

    expect_error(lambda: link.send_image(make_image(4000), 1), "CRC mismatch")

    status = link.get_status()
    assert status.has_image == 0 and status.receiving == 0


def test_begin_image_gives_up_the_stored_image_and_a_failed_erase_keeps_it_given_up():
    simulator, link = make_link()
    link.send_image(make_image(2000), 1)
    assert link.get_status().has_image == 1

    simulator.fail_next_header_erase = True
    expect_error(lambda: link.send_image(make_image(3000), 1), "erase or program failed")

    assert link.get_status().has_image == 0


def test_begin_image_refuses_sizes_that_do_not_agree():
    simulator, link = make_link()

    expect_error(lambda: link.begin_image(5000, 0, 2, 1), "do not agree")
    expect_error(lambda: link.begin_image(1024, 0, 0, 1), "do not agree")
    expect_error(lambda: link.begin_image(1024, 0, 897, 1), "do not agree")


def test_delete_image():
    simulator, link = make_link()
    link.send_image(make_image(2000), 1)

    link.delete_image()

    assert link.get_status().has_image == 0 and simulator.stored_image() is None


def test_images_cannot_be_started_or_deleted_while_an_injection_runs():
    simulator, link = make_link()
    link.send_image(make_image(2000), 1)
    simulator.injecting = True
    simulator.inject_state = 2
    simulator.pages_done = 1

    expect_error(lambda: link.begin_image(1024, 0, 1, 1), "an injection is running")
    expect_error(lambda: link.delete_image(), "an injection is running")

    status = link.get_status()
    assert status.inject_state == 2 and status.pages_done == 1
    assert "sending words (page 1 of 2)" in injector_link.describe_status(status)[1]


def test_unknown_command_is_answered():
    simulator, link = make_link()

    expect_error(lambda: link._request_ok(99), "unknown command")


def test_describe_status_shows_a_failed_injection():
    status = injector_link.Status(1, 0, 1, 3, 10, 4, 7, 3, 8, 9000, 0x1234)
    lines = injector_link.describe_status(status)

    assert lines[1] == "injection: failed"
    assert "the bootloader refused a command, code 8" in lines[2]


def test_no_response_is_reported_after_the_timeout():
    class DeadPort:
        def write(self, data):
            pass

        def read(self, count):
            return b""

    link = InjectorLink(DeadPort(), reply_timeout_s=0.05)

    expect_error(lambda: link.get_status(), "no response")


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]

    for test in tests:
        test()
        print("passed:", test.__name__)

    print("all link checks passed")
