/*
 * AXS5106L touch-controller firmware upgrade module.
 *
 * Verifies the firmware version reported by the chip and reflashes the MTP
 * region from an embedded image when the versions differ.
 */

#pragma once

#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <cstdint>

/// Result of a firmware upgrade attempt.
enum class Axs5106lUpgradeResult {
    Success   =  0,  ///< Upgrade completed successfully.
    NotNeeded = -1,  ///< Chip firmware already matches the embedded image.
    Failed    = -2,  ///< Upgrade flow failed (erase/write/verify error).
    I2cError  = -3,  ///< I2C communication error.
};

/**
 * @brief AXS5106L firmware upgrade helper.
 *
 * Reads the running firmware version from the chip and, when it differs from
 * the version embedded in @c axs5106l_firmware.h, performs a full MTP
 * reflash via the chip's debug-mode command sequence.
 *
 * Typical usage:
 *   1. Construct with the I2C device handle and the reset GPIO.
 *   2. Call @ref CheckAndUpgrade().
 *   3. If the result is Success, reset the chip before continuing.
 */
class Axs5106lUpgrade {
public:
    /**
     * @brief Construct the upgrade helper.
     * @param i2c_handle I2C device handle (already added to a master bus).
     * @param rst_gpio   GPIO connected to the chip reset line.
     */
    Axs5106lUpgrade(i2c_master_dev_handle_t i2c_handle, gpio_num_t rst_gpio);

    /**
     * @brief Compare versions and reflash if necessary.
     * @return Upgrade result; see @ref Axs5106lUpgradeResult.
     */
    Axs5106lUpgradeResult CheckAndUpgrade();

    /**
     * @brief Read the firmware version currently running on the chip.
     * @param[out] version Firmware version reported by the chip.
     * @return true on success.
     */
    bool GetChipFirmwareVersion(uint16_t& version);

    /// Version of the firmware image embedded in this build.
    uint16_t GetEmbeddedFirmwareVersion() const;

private:
    i2c_master_dev_handle_t i2c_handle_;
    gpio_num_t rst_gpio_;

    // Low-level I2C primitives.
    bool WriteRegister(uint8_t reg, const uint8_t* data, size_t len);
    bool ReadRegister(uint8_t reg, uint8_t* data, size_t len);
    bool WriteRegisters(const uint8_t* reg, size_t reg_len, const uint8_t* data, size_t data_len);
    bool ReadRegisters(const uint8_t* reg, size_t reg_len, uint8_t* data, size_t data_len);

    // Reset.
    void HardwareReset();
    void SoftwareReset();

    // Upgrade flow stages.
    bool EnterDebugMode();
    void ExitDebugMode();
    bool UnlockFlash();
    bool EraseFlash();
    bool WriteFlash(const uint8_t* data, size_t len);
    bool VerifyFlash(const uint8_t* data, size_t len);
    bool DoUpgrade();

    // Delay helpers.
    static void DelayMs(uint16_t ms);
    static void DelayUs(uint16_t us);
};
