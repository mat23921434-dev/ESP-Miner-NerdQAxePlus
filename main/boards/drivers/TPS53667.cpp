#include "TPS53667.h"
#include "esp_log.h"
#include "pmbus_commands.h"
#include <math.h>

static const char *TAG = "TPS53667";

TPS53667::TPS53667() : TPS53647()
{
    m_i2cAddr = 0x71;
    m_hwMinVoltage = 0.25f;
    m_initVOutMin = 0.8f;
    m_initVOutMax = 1.4f;
    m_initOnOffConfig = 0b00010111;
    m_initOtWarnLimit = 130.0f;
    m_initOtFaultLimit = 140.0f;
}

void TPS53667::set_phases(int num_phases)
{
    ESP_LOGI(TAG, "TPS53667::set_phases(%d) - enabling all phases", num_phases);
    TPS53647::set_phases(num_phases);
    write_byte(PMBUS_MFR_SPECIFIC_24, 0x00);
    ESP_LOGI(TAG, "TPS53667: PMBUS_MFR_SPECIFIC_24 set to 0x00 (all 6 phases enabled)");
}

bool TPS53667::init(int num_phases, int imax, float ifault)
{
    ESP_LOGI(TAG, "Initializing TPS53667 regulator - HYPER OVERCLOCK MODE (FIXED)");

    uint16_t device_code = get_device_code();

    if (device_code != DEVICE_CODE) {
        ESP_LOGE(TAG, "TPS53667 not found. Device code: 0x%04X", device_code);
        return false;
    }

    ESP_LOGI(TAG, "Found TPS53667 controller");

    write_command(PMBUS_CLEAR_FAULTS);
    write_command(PMBUS_RESTORE_DEFAULT_ALL);
    write_byte(PMBUS_ON_OFF_CONFIG, m_initOnOffConfig);
    write_byte(PMBUS_MFR_SPECIFIC_12, 0x20);
    write_byte(PMBUS_MFR_SPECIFIC_10, (uint8_t) imax);

#if defined(NERDOCTAXEGAMMA) || defined(Q1373)
    write_byte(PMBUS_MFR_SPECIFIC_13, 0x89);
    ESP_LOGI(TAG, "Phase shedding DISABLED (0x89) - forcing all 6 phases ON");
#else
    write_byte(PMBUS_MFR_SPECIFIC_13, 0x99);
    ESP_LOGI(TAG, "Phase shedding ENABLED (0x99) - dynamic phase control");
#endif

    set_phases(num_phases);

    // 放寬 VOUT 最大值到 1.5V 防止壓降超出上限
    write_word(PMBUS_VOUT_MAX, (uint16_t) volt_to_vid(1.50f)); 
    write_byte(PMBUS_MFR_SPECIFIC_00, 0x07); 
    write_byte(PMBUS_MFR_SPECIFIC_16, 0x01); 
    write_word(PMBUS_MFR_SPECIFIC_19, 0x0007);
    
    // ========================================================
    // 修正 1：設定 Fault Response 為 0x00 (Ignore / 忽略錯誤繼續運行)
    // ========================================================
    write_byte(0x41, 0x00); // VOUT_OV_FAULT_RESPONSE -> 忽略過壓關斷
    write_byte(0x45, 0x00); // VOUT_UV_FAULT_RESPONSE -> 忽略欠壓關斷
    write_byte(0x47, 0x00); // IOUT_OC_FAULT_RESPONSE -> 忽略輸出過流關斷
    write_byte(0x5C, 0x00); // IIN_OC_FAULT_RESPONSE  -> 忽略輸入過流關斷
    write_byte(0x50, 0x00); // OT_FAULT_RESPONSE      -> 忽略過溫關斷

    // ========================================================
    // 修正 2：使用正確的 Linear 11 格式將過流限制放寬至極限
    // 0xE3E8 代表每相大約 62.5A，多相併聯總上限可達 250A~300A+
    // 徹底解決因原先 0xF3C0 誤設為負數電流導致的 PSU Fault 報錯
    // ========================================================
    uint16_t max_current_l11 = 0xE3E8; 

    write_word(PMBUS_IOUT_OC_WARN_LIMIT, max_current_l11);
    write_word(PMBUS_IOUT_OC_FAULT_LIMIT, max_current_l11);
    write_word(PMBUS_IIN_OC_WARN_LIMIT, max_current_l11);  
    write_word(PMBUS_IIN_OC_FAULT_LIMIT, max_current_l11); 

    // 溫度保護設定維持不變 (130°C 警告 / 140°C 故障)
    write_word(PMBUS_OT_WARN_LIMIT, float_to_slinear11(130.0f));
    write_word(PMBUS_OT_FAULT_LIMIT, float_to_slinear11(140.0f));
    
    write_command(PMBUS_CLEAR_FAULTS);
    
    m_initialized = true;
    show_voltage_settings();

    return true;
}

void TPS53667::status()
{
    write_command(PMBUS_CLEAR_FAULTS);
    
    uint8_t status_byte = 0x00;
    uint16_t status_word = 0x0000;
    uint8_t status_vout = 0x00;
    uint8_t status_iout = 0x00;
    uint8_t status_input = 0x00;
    uint8_t status_mfr_specific = 0x00;
    uint8_t status_phase = 0x00;

    bool isError = false;

    if (read_byte(PMBUS_STATUS_BYTE, &status_byte)) {
        if (status_byte & 0x7C) {
            isError = true;
        }
    }

    if (read_word(PMBUS_STATUS_WORD, &status_word)) {
        if (status_word & 0xFFFF) {
            isError = true;
        }
    }

    if (read_byte(PMBUS_STATUS_VOUT, &status_vout)) {
        if (status_vout & 0x80) {
            isError = true;
        }
    }

    if (read_byte(PMBUS_STATUS_IOUT, &status_iout)) {
        if (status_iout & 0x38) {
            isError = true;
        }
    }

    if (read_byte(PMBUS_STATUS_INPUT, &status_input)) {
        if (status_input & 0x38) {
            isError = true;
        }
    }

    if (read_byte(PMBUS_STATUS_MFR_SPECIFIC, &status_mfr_specific)) {
        if (status_mfr_specific & 0xFF) {
            isError = true;
        }
    }

    if (isError) {
        ESP_LOGE(TAG, "TPS536X7 Fault status word: %04X", status_word);
    }
}

bool TPS53667::read_word(uint8_t command, uint16_t *data)
{
    return TPS53647::read_word(command, data);
}

bool TPS53667::read_byte(uint8_t command, uint8_t *data)
{
    return TPS53647::read_byte(command, data);
}
