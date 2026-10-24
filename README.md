# STM32F407VGT6 CAN Bus Injector Flasher

A small device that holds an application image and flashes it into a board on the CAN bus, over the
[CAN bootloader](https://github.com/stevenprst/stm32f405vgt6-bootloader). The PC loads the image into the injector
over USB; after that the injector flashes the board alone, by a button.

## Hardware

| Part | Pin |
|---|---|
| USB (virtual COM port) | PA11, PA12 |
| CAN1 (125 kbit/s) | PB8 (RX), PB9 (TX) |
| Status LED | PA6 |
| Inject button | PE4 (to ground) |
| Delete button | PE3 (to ground) |

The 25 MHz crystal is needed for USB. The LED and button pins are labels in the `.ioc` (`LED_STATUS`, `BUTTON_INJECT`,
`BUTTON_DELETE`): change them there and generate the code again.

## The board you flash (APP CHANGES)

The injector talks to the CAN bootloader, so the board must run it. The app must be:

1. Linked at `0x08008000` with `LENGTH = 992K`, built as `.bin`, at most 896 KB.
2. Able to reset into the bootloader when the injector asks. When the app receives CAN ID `0x6A4` with the data
   `node id, 0x07, 0xB0`, it writes `0x53544159` to the SRAM word at `0x20003004` and resets:

```c
*(volatile uint32_t *)(SRAM_BASE + 0x3004) = 0x53544159;
NVIC_SystemReset();
```

An app without it can still be flashed if the board is reset just before the inject button is pressed (the bootloader
listens for a short time after a reset), and a board with no app yet needs nothing.

## How to Use

1. Flash the injector to the board (`injector_flasher Debug` in STM32CubeIDE, or the `.elf` with ST-LINK).
2. Install the Python tool:

```bash
pip install pyserial
```

3. Plug in USB, then open the window:

```bash
python tools/injector_gui.py
```

4. Choose the COM port and press **Connect**.
5. Choose the app `.bin`, set the **node id** of the board it is for, press **Send to injector**.
   The LED turns on when the injector holds the image. It keeps it after a power cycle.
6. Connect the injector to the CAN bus and press the **inject button**. The LED blinks while it flashes.
   The window shows the page and the result (done, or why it failed).
7. To remove the image, hold the **delete button** for one second. The LED goes off.

## How the Injector Works

### 1. USB link

The PC sends one request and waits for the answer: GetStatus, BeginImage, WriteChunk, EndImage, DeleteImage.
The file is checked on the PC first (an app for `0x08008000`, at most 896 pages) and again by CRC at the end.

### 2. Store the image

Flash sectors 0 to 3 hold the firmware, sector 4 the header, sectors 5 to 11 the image. The header is written last,
after the CRC is checked, so a transfer that stops halfway leaves no image. A sector is erased when the first page
that lands on it arrives.

### 3. Inject

On the inject button the injector sends the trigger frame, pings until the bootloader answers, then sends the image
word by word (`WriteBuffer`), each page with its CRC (`WritePage`), the image CRC (`WriteCrc`) and `StartApp`.
A reply that does not come is asked for again three times. A full 896 KB image takes about 8 minutes at 125 kbit/s.

### 4. LED and buttons

The LED is off without an image, on with one, and blinks while injecting. The delete button is ignored while an
image is being received or injected.

## Test the PC side without hardware

```bash
cd tools
python test_link.py
```
