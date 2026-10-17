# Iron Man Helmet (ironman-helmet)

Firmware for a LilyGO **T-Display-S3** that drives a motorised helmet faceplate (Iron Man style) with two MG90S servos. You can open and close the faceplate from the onboard buttons, from the Apple Home app, or by asking Siri. The 1.9" screen shows the faceplate's state and position.

## Features

- **Two mirrored servos** move together to lift the faceplate. The servos sit on opposite sides, so their angles run in opposite directions.
- **Apple HomeKit** support through [HomeSpan](https://github.com/HomeSpan/HomeSpan). The faceplate shows up as a switch called *Iron Man Helmet*: on means open, off means closed.
- **Button control.** Either onboard button (BOOT / GPIO0 or GPIO14) toggles the faceplate, and the new state is sent to HomeKit so the Home app stays in step.
- **Status display** with angled eye slits that brighten as the faceplate opens, a state label (OPEN / OPENING / CLOSING / CLOSED), a progress bar, a percentage and a WiFi/HomeKit indicator.
- **Motion tuned for the mechanism:**
  - a breakaway *kick* that gets the plate moving off the closed stop
  - a *press* past the closed position so the plate seats fully with no gap
  - the servos *relax* (PWM is switched off) after closing, so they don't buzz or draw stall current
- **Serial calibration commands** for finding the real closed position.

## Hardware

| Part | Notes |
|---|---|
| LilyGO T-Display-S3 | ESP32-S3, ST7789 170×320 display (8-bit parallel) |
| 2 × MG90S servo | Any 50 Hz hobby servo works, but the timings are tuned for MG90S |
| 5 V supply for the servos | Each servo can draw about 700 mA when stalled, so don't power them from the board's 3.3 V rail |

### Wiring

| Signal | GPIO |
|---|---|
| Servo A signal | **1** |
| Servo B signal | **2** |
| Button (BOOT) | 0 (onboard) |
| Button | 14 (onboard) |
| Power hold (battery) | 15 (driven HIGH at boot) |

Connect the servo grounds to the board's GND. The servo PWM signal is 3.3 V, which MG90S servos accept.

## Building and flashing

The project uses [PlatformIO](https://platformio.org/). All dependencies are declared in `platformio.ini` and download automatically:

- `bodmer/TFT_eSPI` (configured entirely through `build_flags`, so you don't need to edit `User_Setup.h`)
- `madhephaestus/ESP32Servo`
- `homespan/HomeSpan` **1.9.x**. HomeSpan 2.x needs arduino-esp32 3.x, but this project is pinned to `espressif32@6.12.0` (arduino-esp32 2.0.x).

```sh
# Build
pio run

# Flash over USB
pio run -t upload

# Open the serial monitor (115200 baud, with the exception decoder)
pio device monitor
```

You can also use the PlatformIO extension in VS Code (Build / Upload / Monitor).

> If the upload fails, hold **BOOT**, tap **RST**, then release **BOOT** to put the board in download mode.

## First-time setup

### 1. Connect to WiFi

HomeSpan runs a command-line interface over the serial port. Open the serial monitor and type:

```
W
```

Then follow the prompts to enter your WiFi SSID and password. The credentials are saved in flash. Once the board is connected, the screen header changes from **NO WIFI** to **HOMEKIT**.

Alternatively, type `A` to start HomeSpan's setup access point and enter the credentials from your phone.

### 2. Pair with Apple Home

1. Open the **Home** app → **+** → **Add Accessory** → **More options…**
2. Choose **Iron Man Helmet**.
3. Enter the setup code: **466-37-726**

After pairing you can say things like *"Hey Siri, turn off Iron Man Helmet"* to close the faceplate. You can rename the accessory in the Home app (for example, to "Faceplate").

## Usage

| Action | Result |
|---|---|
| Press either onboard button | Toggles between open and closed |
| Switch **on** in Home / Siri | Opens the faceplate |
| Switch **off** in Home / Siri | Closes the faceplate |

- The board **boots with the faceplate open**, so the first command closes it.
- A command can reverse a move part-way through. The plate continues from wherever it is.
- When open, the servos stay powered to hold the plate up against gravity.
- When closed, the servos are released after `RELAX_DELAY_MS`, and the screen shows **SERVOS OFF**.

### Serial commands

HomeSpan's built-in commands (type `?` to list them) include:

| Command | Purpose |
|---|---|
| `W` | Set WiFi credentials |
| `A` | Start the WiFi setup access point |
| `S <code>` | Change the HomeKit setup code |
| `H` | Delete HomeKit pairing data (unpair) |
| `E` | Erase all stored data (WiFi and HomeKit) |

This project adds two calibration commands:

| Command | Purpose |
|---|---|
| `@d <deg>` | Move both servos to `<deg>` past the nominal closed angle and **hold** there. Use this to find where the plate actually seats. The command warns if a servo hits the pulse-width limit. |
| `@x` | Release both servos immediately |

> **Warning:** `@d` holds the servos stalled against the stop, where each can draw about 700 mA. Run `@x` as soon as you've taken your reading.

## Configuration

All settings are `constexpr` values at the top of `src/main.cpp`. Change them and re-flash.

### Pins

```cpp
constexpr uint8_t PIN_SERVO_A = 1;
constexpr uint8_t PIN_SERVO_B = 2;
constexpr uint8_t PIN_BTN_BOOT = 0;
constexpr uint8_t PIN_BTN_14 = 14;
constexpr uint32_t DEBOUNCE_MS = 40;
```

### Servo pulse range

| Constant | Default | Meaning |
|---|---|---|
| `SERVO_FREQ_HZ` | 50 | PWM frame rate |
| `SERVO_MIN_US` / `SERVO_MAX_US` | 500 / 2400 | Pulse widths that map to 0° and 180° |
| `SERVO_US_HEADROOM` | 160 | Extra pulse width allowed beyond 0–180°, so the closing press can go past the end. Applied **equally** to both ends. |

The headroom must stay symmetric. The servos press in opposite directions, so if one side clamps and the other doesn't, one servo travels further than the other, the plate twists, and it jams. A `static_assert` stops the build if `CLOSE_PRESS_DEG` needs more pulse width than the headroom allows.

### Travel angles

```cpp
constexpr float SERVO_A_CLOSED_DEG = 180.0f;
constexpr float SERVO_A_OPEN_DEG   = 90.0f;
constexpr float SERVO_B_CLOSED_DEG = 0.0f;
constexpr float SERVO_B_OPEN_DEG   = 90.0f;
```

If a servo moves the wrong way because its horn is mounted mirrored, swap that servo's CLOSED and OPEN values. Keep both servos' travel spans the same size.

### Speed

| Constant | Default | Meaning |
|---|---|---|
| `OPEN_TIME_MS` | 250 | Time for a full closed → open move |
| `CLOSE_TIME_MS` | 250 | Time for a full open → closed move. A quick close helps the plate carry momentum onto the stop and seat itself. |
| `UPDATE_INTERVAL_MS` | 20 | Motion and display update period |

Partial moves are scaled to their distance. The MG90S tops out around 600°/s, so values below about 180–250 ms are limited by the servo's own speed.

### Opening kick

| Constant | Default | Meaning |
|---|---|---|
| `OPEN_KICK_DEG` | 6 | Size of the first step when opening from closed. Raise it if the plate buzzes without lifting. `0` turns the kick off. |
| `OPEN_KICK_MS` | 120 | How long the kick step is held before the normal ramp takes over |

### Closing and seating

| Constant | Default | Meaning |
|---|---|---|
| `CLOSE_PRESS_DEG` | 14 | How far **past** nominal closed the servos aim, so they drive the plate home instead of drifting onto the stop. Find the right value with `@d`. |
| `RELAX_WHEN_CLOSED` | `true` | Switch off the PWM once closed |
| `RELAX_DELAY_MS` | 350 | How long the servos keep pressing before they are released. Too short and the plate isn't fully seated; too long and the servos sit stalled. |
| `CLOSE_BACKOFF_DEG` | 0 | Optional back-off after seating. Keep it at 0 on a gravity-seated plate, because backing off opens a visible gap. |
| `SEAT_TIME_MS` / `BACKOFF_TIME_MS` | 150 / 150 | Timings for the back-off sequence (only used when `CLOSE_BACKOFF_DEG > 0`) |

The press pose is a deliberate stall. It is only safe because `RELAX_WHEN_CLOSED` releases it shortly afterwards, so don't disable relaxing while `CLOSE_PRESS_DEG` is non-zero.

### HomeKit identity

These are set in `setup()`:

```cpp
homeSpan.setPairingCode("46637726");    // setup code 466-37-726
homeSpan.setQRID("HELM");
homeSpan.begin(Category::Switches, "Iron Man Helmet");
```

Change the pairing code before you rely on this anywhere other than your own network. You can also change it at runtime with the `S` serial command.

### Display

The colours are defined in `initColors()`, and the screen layout is in `drawHeader()` and `drawFace()`. The face sprite uses 8-bit colour (about 46 KB) so that enough internal RAM is left for WiFi and HomeKit.

## Calibration walkthrough

1. Flash the firmware and open the serial monitor.
2. Check the direction: press the button and make sure both servos lift the plate. If one fights the other, swap that servo's `*_CLOSED_DEG` and `*_OPEN_DEG` values.
3. Find the seat depth: run `@d 0`, then `@d 5`, `@d 10` and so on, watching for the point where the plate sits fully shut with no gap. Run `@x` between readings so the servos don't stay stalled.
4. Set `CLOSE_PRESS_DEG` to a degree or so past that point. If the build fails on the `static_assert`, raise `SERVO_US_HEADROOM`.
5. Tune `RELAX_DELAY_MS` so the plate is fully driven home before the servos are released.
6. If opening from closed stalls or buzzes, raise `OPEN_KICK_DEG`.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Serial CLI doesn't respond | Make sure the monitor is at 115200 baud. USB CDC takes about 0.5 s to come up after reset. |
| Header shows **NO WIFI** | Set the WiFi credentials with `W` (or `A`) |
| Plate jams or twists when closing | The pulse width is clamping on one side. Raise `SERVO_US_HEADROOM` or lower `CLOSE_PRESS_DEG`. |
| Gap left when closed | Raise `CLOSE_PRESS_DEG` and/or `RELAX_DELAY_MS`, or shorten `CLOSE_TIME_MS` |
| Servos buzz when closed | Make sure `RELAX_WHEN_CLOSED` is `true` |
| Plate buzzes but won't open | Raise `OPEN_KICK_DEG` |
| Board resets when the servos move | The servo supply can't deliver enough current. Use a separate 5 V supply with a shared ground. |
