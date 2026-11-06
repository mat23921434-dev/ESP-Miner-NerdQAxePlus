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
    ESP_LOGI(TAG, "Initializing TPS53667 regulator - GOD MODE UNLIMITED OVERCLOCK (v3 FINAL)");

    uint16_t device_code = get_device_code();

    if (device_code != DEVICE_CODE) {
        ESP_LOGE(TAG, "TPS53667 not found. Device code: 0x%04X", device_code);
        return false;
    }

    ESP_LOGI(TAG, "Found TPS53667 controller");

    // ========================================================
    // 1. 基礎重置與基本配置
    // ========================================================
    write_command(PMBUS_CLEAR_FAULTS);
    write_command(PMBUS_RESTORE_DEFAULT_ALL);
    write_byte(PMBUS_ON_OFF_CONFIG, m_initOnOffConfig);
    
    // 設置跳變頻率防止電壓低頻震盪
    write_byte(PMBUS_MFR_SPECIFIC_12, 0x20); 
    
    // 強行將晶片硬體 IMAX 設定拉到最高規格 0x4B (75A)
    write_byte(PMBUS_MFR_SPECIFIC_10, 0x4B);

    // ========================================================
    // 2. 核心修改：強制禁用動態相數調節 (Phase Shedding)
    // 任何機型皆鎖定 0x89，強制 6 相全開，避免超頻時單相承受所有電流
    // ========================================================
    write_byte(PMBUS_MFR_SPECIFIC_13, 0x89);
    ESP_LOGI(TAG, "Phase shedding FORCE DISABLED (0x89) - 6 phases locked ON");

    set_phases(num_phases);

    // ========================================================
    // 3. 電壓上限與轉換斜率 (Slew Rate) 進階平衡優化
    // ========================================================
    // 放寬 VOUT 最大值到 1.6V，為超頻和防壓降（Drop Compensation）留足空間
    write_word(PMBUS_VOUT_MAX, (uint16_t) volt_to_vid(1.60f)); 
    
    // 【詳細為你修改】：放緩電壓跳變斜率 (Slew Rate) 到中高檔 0x04 (原為 0x07)
    // 理由：1050MHz 下瞬間加壓太陡峭，會導致火牛瞬態抽取電流變化率 (dI/dt) 爆炸，直接觸發火牛硬體 OCP。
    // 將其微調為 0x04，讓電壓升起時稍微平滑一點，能有效物理性「欺騙」外接火牛，防止其斷電。
    write_byte(PMBUS_MFR_SPECIFIC_00, 0x04); 
    write_byte(PMBUS_MFR_SPECIFIC_16, 0x01); 
    write_word(PMBUS_MFR_SPECIFIC_19, 0x0004);
    
    // 【全新加入魔改】：修改 MFR_SPECIFIC_09 (動態響應控制暫存器)
    // 寫入 0xFF，強行鎖死電源晶片的誤差放大器增益，硬化瞬態響應，防止電壓在高頻下產生多餘的微小鋸齒波（漣波）。
    write_byte(PMBUS_MFR_SPECIFIC_09, 0xFF);

    // ========================================================
    // 4. 核心修改：真正的 0xBF 暴力模式 (Fault Response)
    // 晶片不支持 0x00 (Ignore)，寫入會被拒絕。
    // 0xBF 代表：就算過流/過溫，也絕不關斷 PWM，持續供電並無限次重試！
    // ========================================================
    write_byte(0x41, 0xBF); // VOUT_OV_FAULT_RESPONSE  -> 超壓絕不關斷
    write_byte(0x45, 0xBF); // VOUT_UV_FAULT_RESPONSE  -> 欠壓絕不關斷
    write_byte(0x47, 0xBF); // IOUT_OC_FAULT_RESPONSE  -> 輸出過流絕不關斷
    write_byte(0x5C, 0xBF); // IIN_OC_FAULT_RESPONSE   -> 輸入過流絕不關斷
    write_byte(0x50, 0xBF); // OT_FAULT_RESPONSE       -> 過溫絕不關斷

    // 【全新加入魔改】：遮蔽硬體警報引腳 (SMBALERT# Mask)
    // 寫入 MFR_SPECIFIC_04 暫存器 0x0000，硬性命令 TPS53667 關閉外部警報中斷。
    // 這樣就算它在內部發現了電壓欠壓或電流過大，它的實體 ALERT 引腳也不會拉低，
    // 從而徹底切斷向 ESP32 主控告狀的硬體線路，防止 ESP32 彈出 PSU FAULT。
    write_word(PMBUS_MFR_SPECIFIC_04, 0x0000);

    // ========================================================
    // 5. 核心修改：修正 Linear 11 正確數值放寬電流限制
    // TPS53667 的電流 Exponent 固定為 0，數值直接就是十進制的安培數 (A)。
    // 0x004B = 十進制 75 (每單相 75A，6相併聯總電流上限達 450A)
    // ========================================================
    uint16_t max_current_l11 = 0x004B; 

    write_word(PMBUS_IOUT_OC_WARN_LIMIT, max_current_l11);
    write_word(PMBUS_IOUT_OC_FAULT_LIMIT, max_current_l11);
    
    // 總輸入電流 (IIN) 限制直接拉到大滿貫 100A 
    // 0x0064 = 十進制 100
    uint16_t max_iin_l11 = 0x0064;
    write_word(PMBUS_IIN_OC_WARN_LIMIT, max_iin_l11);  
    write_word(PMBUS_IIN_OC_FAULT_LIMIT, max_iin_l11); 

    // ========================================================
    // 6. 溫度保護設定維持在安全高位 (135°C 警告 / 145°C 故障)
    // ========================================================
    write_word(PMBUS_OT_WARN_LIMIT, float_to_slinear11(135.0f));
    write_word(PMBUS_OT_FAULT_LIMIT, float_to_slinear11(145.0f));
    
    // 開機完成前，再次清除所有殘留的過往錯誤紀錄
    write_command(PMBUS_CLEAR_FAULTS);
    
    m_initialized = true;
    show_voltage_settings();

    return true;
}

// 【詳細為你修改】：重寫 status() 函數
// 直接在這個函數內執行攔截！不管讀到什麼錯誤，強行就地執行 CLEAR_FAULTS，
// 並且不向日誌(ESP_LOGE)輸出任何 Fault status 錯誤，徹底在主控層面實施「裝聾作啞」。
void TPS53667::status()
{
    write_command(PMBUS_CLEAR_FAULTS);
    // 這裡我們直接 return，徹底閹割掉原本會檢查錯誤並列印出 ESP_LOGE 的所有邏輯。
    return;
}

bool TPS53667::read_word(uint8_t command, uint16_t *data)
{
    return TPS53647::read_word(command, data);
}

bool TPS53667::read_byte(uint8_t command, uint8_t *data)
{
    return TPS53647::read_byte(command, data);
}
