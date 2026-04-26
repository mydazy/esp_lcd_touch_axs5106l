# Changelog

All notable changes to **esp_lcd_touch_axs5106l** will be documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.2] - 2026-04-26

### Changed

- All in-source comments and log strings translated from Chinese to English for global readability.
- `axs5106l_firmware.h` header annotation reformatted with copyright/license attribution; build-time metadata removed.
- README expanded with explicit "About the embedded firmware" section clarifying that the shipped image is a mydazy custom-panel reference image (Apache-2.0), and panel-specific firmware should be substituted by the integrator.

### Added

- `.clang-format` — Google C++ based style with ESP-IDF conventions.
- GitHub Actions multi-IDF-version build matrix (5.3 / 5.4 / 5.5).
- `.gitignore` rules to prevent vendor-private firmware preprocessor sources (`*.i`, `*HQR*`, `*FPC-*`, `*KY[0-9]*`) from being committed by mistake.

### Notes

- No API changes. Source-compatible drop-in for v1.0.1.

## [1.0.1] - 2026-04-26

### Changed

- Documentation rewritten for public release. No API or behavior changes.

## [1.0.0] - 2026-04-26

### Added

- Initial release.
- C++ driver for **AXS5106L** capacitive touch controller (ChipSourceTek).
- LVGL input device integration via `esp_lvgl_port` (`lv_indev_t`).
- Gesture recognition: single click, double click, long press, four-direction swipe.
- Two-phase initialization (`InitializeHardware()` + `InitializeInput()`) for panels where the touch IC shares its reset line with the LCD.
- Built-in noise filtering: INT debouncing, press-confirmation window, release-glitch filter, speed thresholding and consecutive I/O-failure suppression — designed for environments with strong wireless RF interference.
- Runtime firmware-upgrade flow with embedded firmware image.
- `Sleep()` / `Resume()` low-power APIs.
- Coordinate transform (swap-XY / mirror-X / mirror-Y) at construction time.
- Optional `AXS5106L_TOUCH_DEBUG_OVERLAY` compile-time switch for visual calibration.

### Notes

- AXS5106L is register-compatible with **AXS15231B**; existing reference code applies.
- I2C address: `0x63` (7-bit).
- Tested on ESP-IDF 5.3 / 5.4 / 5.5 with ESP32-S3.
