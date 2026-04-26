/*
 * AXS5106L touch-controller firmware upgrade module.
 */

#include "axs5106l_upgrade.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

static const char* TAG = "Axs5106lUpgrade";

// Embedded firmware image (raw byte stream included from a generated header).
static const uint8_t kFirmwareData[] = {
#include "axs5106l_firmware.h"
};

// Offset within the firmware image where the version word lives.
#define FIRMWARE_VERSION_OFFSET  0x400

// I2C transaction parameters.
#define I2C_TIMEOUT_MS           100
#define I2C_MAX_RETRIES          3

// Upgrade-flow parameters.
#define UPGRADE_RETRY_TIMES      1
#define DEBUG_MODE_RETRY_TIMES   3
#define ERASE_TIMEOUT_MS         300
#define WRITE_TIMEOUT_MS         10

Axs5106lUpgrade::Axs5106lUpgrade(i2c_master_dev_handle_t i2c_handle, gpio_num_t rst_gpio)
    : i2c_handle_(i2c_handle), rst_gpio_(rst_gpio) {
}

void Axs5106lUpgrade::DelayMs(uint16_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void Axs5106lUpgrade::DelayUs(uint16_t us) {
    uint64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < us) {
        // Busy wait: required when us < 1 RTOS tick.
    }
}

bool Axs5106lUpgrade::WriteRegister(uint8_t reg, const uint8_t* data, size_t len) {
    if (i2c_handle_ == nullptr || len > 64) return false;

    uint8_t buf[65];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    for (int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
        esp_err_t ret = i2c_master_transmit(i2c_handle_, buf, len + 1, I2C_TIMEOUT_MS);
        if (ret == ESP_OK) return true;
        DelayMs(5);
    }
    return false;
}

bool Axs5106lUpgrade::ReadRegister(uint8_t reg, uint8_t* data, size_t len) {
    if (i2c_handle_ == nullptr) return false;

    for (int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
        esp_err_t ret = i2c_master_transmit(i2c_handle_, &reg, 1, I2C_TIMEOUT_MS);
        if (ret != ESP_OK) {
            DelayMs(5);
            continue;
        }

        ret = i2c_master_receive(i2c_handle_, data, len, I2C_TIMEOUT_MS);
        if (ret == ESP_OK) return true;

        DelayMs(5);
    }
    return false;
}

bool Axs5106lUpgrade::WriteRegisters(const uint8_t* reg, size_t reg_len, const uint8_t* data, size_t data_len) {
    if (i2c_handle_ == nullptr) return false;

    // Concatenate register address bytes and payload into one transmit.
    size_t total_len = reg_len + data_len;
    if (total_len > 600) return false;

    uint8_t buf[600];
    memcpy(buf, reg, reg_len);
    memcpy(buf + reg_len, data, data_len);

    for (int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
        esp_err_t ret = i2c_master_transmit(i2c_handle_, buf, total_len, I2C_TIMEOUT_MS);
        if (ret == ESP_OK) return true;
        DelayMs(5);
    }
    return false;
}

bool Axs5106lUpgrade::ReadRegisters(const uint8_t* reg, size_t reg_len, uint8_t* data, size_t data_len) {
    if (i2c_handle_ == nullptr) return false;

    for (int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
        esp_err_t ret = i2c_master_transmit(i2c_handle_, reg, reg_len, I2C_TIMEOUT_MS);
        if (ret != ESP_OK) {
            DelayMs(5);
            continue;
        }

        ret = i2c_master_receive(i2c_handle_, data, data_len, I2C_TIMEOUT_MS);
        if (ret == ESP_OK) return true;

        DelayMs(5);
    }
    return false;
}

void Axs5106lUpgrade::HardwareReset() {
    gpio_set_level(rst_gpio_, 1);
    DelayUs(50);
    gpio_set_level(rst_gpio_, 0);
    DelayUs(50);
    DelayMs(20);
    gpio_set_level(rst_gpio_, 1);
}

void Axs5106lUpgrade::SoftwareReset() {
    uint8_t rst_cmd[5] = {0xB3, 0x55, 0xAA, 0x34, 0x01};
    WriteRegister(0xF0, rst_cmd, 5);
    HardwareReset();
}

bool Axs5106lUpgrade::GetChipFirmwareVersion(uint16_t& version) {
    uint8_t fw_ver[2] = {0};
    if (!ReadRegister(0x05, fw_ver, 2)) {
        return false;
    }
    version = (fw_ver[0] << 8) | fw_ver[1];
    return true;
}

uint16_t Axs5106lUpgrade::GetEmbeddedFirmwareVersion() const {
    if (sizeof(kFirmwareData) < FIRMWARE_VERSION_OFFSET + 2) {
        return 0;
    }
    return (kFirmwareData[FIRMWARE_VERSION_OFFSET] << 8) | kFirmwareData[FIRMWARE_VERSION_OFFSET + 1];
}

bool Axs5106lUpgrade::EnterDebugMode() {
    uint8_t debug_cmd[1] = {0x55};
    uint8_t write_buf[3] = {0x80, 0x7f, 0xd1};
    uint8_t read_buf[1] = {0x00};

    for (int retry = 0; retry < DEBUG_MODE_RETRY_TIMES; retry++) {
        // Reset chip first.
        SoftwareReset();

        // Wait window: 500 us < delay < 4 ms.
        DelayUs(800);

        // Send debug-mode entry command.
        WriteRegister(0xAA, debug_cmd, 1);

        // delay >= 50 us before the readback.
        DelayUs(100);

        // Confirm the chip acknowledges debug mode (expected reply: 0x28).
        if (ReadRegisters(write_buf, 3, read_buf, 1)) {
            if (read_buf[0] == 0x28) {
                ESP_LOGI(TAG, "Entered debug mode");
                return true;
            }
        }
    }

    ESP_LOGE(TAG, "Failed to enter debug mode");
    return false;
}

void Axs5106lUpgrade::ExitDebugMode() {
    uint8_t cmd[1] = {0x5F};
    WriteRegister(0xA0, cmd, 1);
}

bool Axs5106lUpgrade::UnlockFlash() {
    uint8_t unlock_cmd[3] = {0x6F, 0xFF, 0xFF};
    WriteRegister(0x90, unlock_cmd, 3);

    unlock_cmd[1] = 0xDA;
    unlock_cmd[2] = 0x18;
    WriteRegister(0x90, unlock_cmd, 3);

    return true;
}

bool Axs5106lUpgrade::EraseFlash() {
    uint8_t clear_flag[3] = {0x6F, 0xD9, 0x0C};
    uint8_t erase_cmd[3] = {0x6F, 0xD6, 0x77};
    uint8_t write_buf[3] = {0x80, 0x7F, 0xD9};
    uint8_t read_buf[1] = {0x00};

    WriteRegister(0x90, clear_flag, 3);
    WriteRegister(0x90, erase_cmd, 3);

    // Poll for erase completion (timeout: 300 ms).
    for (int i = 0; i < 30; i++) {
        DelayMs(WRITE_TIMEOUT_MS);

        if (ReadRegisters(write_buf, 3, read_buf, 1)) {
            if (read_buf[0] & 0x04) {  // bit 2 == 1 means done
                erase_cmd[2] = 0x00;
                WriteRegister(0x90, erase_cmd, 3);
                ESP_LOGI(TAG, "Flash erase complete");
                return true;
            }
        }
    }

    erase_cmd[2] = 0x00;
    WriteRegister(0x90, erase_cmd, 3);
    ESP_LOGE(TAG, "Flash erase timeout");
    return false;
}

bool Axs5106lUpgrade::WriteFlash(const uint8_t* data, size_t len) {
    uint8_t cmd[3] = {0x6F, 0xD4, 0x00};

    // Configure write parameters.
    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD5;
    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD2;
    cmd[2] = (len - 1) & 0xFF;
    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD3;
    cmd[2] = ((len - 1) >> 8) & 0xFF;
    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD6;
    cmd[2] = 0xF4;
    WriteRegister(0x90, cmd, 3);

    // Byte-by-byte write (slow mode, broadest compatibility).
    cmd[1] = 0xD7;
    for (size_t i = 0; i < len; i++) {
        cmd[2] = data[i];
        WriteRegister(0x90, cmd, 3);

        // Progress log every 1 KB.
        if ((i + 1) % 1024 == 0) {
            ESP_LOGI(TAG, "Write progress: %u / %u", (unsigned)(i + 1), (unsigned)len);
        }
    }

    cmd[1] = 0xD6;
    cmd[2] = 0x00;
    WriteRegister(0x90, cmd, 3);

    ESP_LOGI(TAG, "Firmware write complete (%u bytes)", (unsigned)len);
    return true;
}

bool Axs5106lUpgrade::VerifyFlash(const uint8_t* data, size_t len) {
    // Byte-by-byte readback verification.
    uint8_t cmd[3] = {0x6F, 0xD4, 0x00};
    uint8_t write_buf[3] = {0x80, 0x7F, 0xD7};
    uint8_t read_buf[1] = {0x00};

    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD5;
    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD2;
    cmd[2] = (len - 1) & 0xFF;
    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD3;
    cmd[2] = ((len - 1) >> 8) & 0xFF;
    WriteRegister(0x90, cmd, 3);

    cmd[1] = 0xD6;
    cmd[2] = 0xF1;
    WriteRegister(0x90, cmd, 3);

    for (size_t i = 0; i < len; i++) {
        if (!ReadRegisters(write_buf, 3, read_buf, 1)) {
            ESP_LOGE(TAG, "Verify read failed at offset %u", (unsigned)i);
            goto verify_exit;
        }

        if (read_buf[0] != data[i]) {
            ESP_LOGE(TAG, "Verify mismatch at offset %u: expected 0x%02X, got 0x%02X",
                     (unsigned)i, data[i], read_buf[0]);
            goto verify_exit;
        }

        // Progress log every 1 KB.
        if ((i + 1) % 1024 == 0) {
            ESP_LOGI(TAG, "Verify progress: %u / %u", (unsigned)(i + 1), (unsigned)len);
        }
    }

    cmd[1] = 0xD6;
    cmd[2] = 0x00;
    WriteRegister(0x90, cmd, 3);

    ESP_LOGI(TAG, "Firmware verification passed");
    return true;

verify_exit:
    cmd[1] = 0xD6;
    cmd[2] = 0x00;
    WriteRegister(0x90, cmd, 3);
    return false;
}

bool Axs5106lUpgrade::DoUpgrade() {
    ESP_LOGI(TAG, "Starting upgrade (firmware size: %u bytes)", (unsigned)sizeof(kFirmwareData));

    // 1. Enter debug mode.
    if (!EnterDebugMode()) {
        return false;
    }

    // 2. Unlock flash.
    if (!UnlockFlash()) {
        ESP_LOGE(TAG, "Flash unlock failed");
        return false;
    }

    // 3. Erase flash.
    if (!EraseFlash()) {
        return false;
    }

    // 4. Write new firmware.
    if (!WriteFlash(kFirmwareData, sizeof(kFirmwareData))) {
        return false;
    }

    // 5. Optional verification (omitted by default — slow on byte-by-byte path).
    // if (!VerifyFlash(kFirmwareData, sizeof(kFirmwareData))) {
    //     return false;
    // }

    return true;
}

Axs5106lUpgradeResult Axs5106lUpgrade::CheckAndUpgrade() {
    // 1. Read the version currently running on the chip.
    uint16_t chip_version = 0;
    if (!GetChipFirmwareVersion(chip_version)) {
        ESP_LOGW(TAG, "Cannot read chip firmware version; will attempt upgrade anyway "
                      "(may be a blank chip)");
    } else {
        ESP_LOGI(TAG, "Chip firmware version: V%u", chip_version);
    }

    // 2. Read the embedded version.
    uint16_t embedded_version = GetEmbeddedFirmwareVersion();
    ESP_LOGI(TAG, "Embedded firmware version: V%u", embedded_version);

    // 3. Compare; skip if equal.
    if (chip_version == embedded_version && chip_version != 0) {
        ESP_LOGI(TAG, "Chip firmware up to date; no upgrade needed");
        return Axs5106lUpgradeResult::NotNeeded;
    }

    // 4. Run the upgrade.
    ESP_LOGI(TAG, "Starting firmware upgrade: V%u -> V%u", chip_version, embedded_version);

    for (int retry = 0; retry < UPGRADE_RETRY_TIMES; retry++) {
        if (DoUpgrade()) {
            // 5. Reset chip after successful upgrade.
            SoftwareReset();
            DelayMs(50);

            // 6. Read back the version to confirm.
            uint16_t new_version = 0;
            if (GetChipFirmwareVersion(new_version)) {
                if (new_version == embedded_version) {
                    ESP_LOGI(TAG, "Firmware upgrade succeeded; new version: V%u", new_version);
                    return Axs5106lUpgradeResult::Success;
                } else {
                    ESP_LOGW(TAG, "Version mismatch after upgrade: expected V%u, got V%u",
                             embedded_version, new_version);
                }
            }

            ESP_LOGW(TAG, "Post-upgrade version verification failed; retrying...");
        }
    }

    ESP_LOGE(TAG, "Firmware upgrade failed");
    return Axs5106lUpgradeResult::Failed;
}
