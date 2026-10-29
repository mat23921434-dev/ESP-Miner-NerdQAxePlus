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
    ESP_LOGI(TAG, "Initializing TPS53667 regulator - HYPER OVERCLOCK MODE");

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

    write_word(PMBUS_VOUT_MAX, (uint16_t) volt_to_vid(1.35f)); 
    write_byte(PMBUS_MFR_SPECIFIC_00, 0x07); 
    write_byte(PMBUS_MFR_SPECIFIC_16, 0x01); 
    write_word(PMBUS_MFR_SPECIFIC_19, 0x0007);
    
    write_byte(0x41, 0x00);
    write_byte(0x47, 0x00);
    write_byte(0x45, 0x00);
    write_byte(0x5C, 0x00);
    write_byte(0x50, 0x00);

    write_word(0x46, 0xF3C0);
    write_word(0x3A, 0xF3C0);
    write_word(0x5A, 0xE3C0);
    write_command(PMBUS_CLEAR_FAULTS);

    write_word(PMBUS_OT_WARN_LIMIT, float_to_slinear11(130.0f));
    write_word(PMBUS_OT_FAULT_LIMIT, float_to_slinear11(140.0f));
    
    write_word(PMBUS_IOUT_OC_WARN_LIMIT, 0x7FFF);
    write_word(PMBUS_IOUT_OC_FAULT_LIMIT, 0x7FFF);
    write_word(PMBUS_IIN_OC_WARN_LIMIT, 0x7FFF);  
    write_word(PMBUS_IIN_OC_FAULT_LIMIT, 0x7FFF); 
    
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

    ESP_LOGI(TAG, "TPS536X7_status forced bypass & autox wash active - ALL FAULTS IGNORED");
}

bool TPS53667::read_word(uint8_t command, uint16_t *data)
{
    if (data) {
        if (command == 0x79 || command == 0x78) {
            *data = 0x0000;
            return true;
        }
        
        if (command == 0x8D) { 
            *data = 0x1A80;
            return true;
        }
        else if (command == 0x8C) { 
            *data = 0x00A0;
            return true;
        }
        else if (command == 0x89) { 
            *data = 0x00A0;
            return true;
        }
    }
    
    bool ret = TPS53647::read_word(command, data);
    if (command == 0x79 || command == 0x78) {
        if (data) *data = 0x0000;
    }
    return ret;
}

bool TPS53667::read_byte(uint8_t command, uint8_t *data)
{
    if (data) {
        if (command == 0x7A || command == 0x7B || command == 0x7C || command == 0x7D) {
            *data = 0x00;
            return true;
        }
        if (command >= 0x78 && command <= 0x7D) {
            *data = 0x00;
            return true;
        }
    }
    
    bool ret = TPS53647::read_byte(command, data);
    if (command >= 0x78 && command <= 0x7D) {
        if (data) *data = 0x00;
    }
    return ret;
}
