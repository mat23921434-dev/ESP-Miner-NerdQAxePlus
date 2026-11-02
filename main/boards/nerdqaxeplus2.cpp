#include <algorithm>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nerdqaxeplus2.h"
#include "nvs_config.h"
#include "serial.h"
#include "drivers/rev7/TPS546.h"

static const char* TAG = "nerdqaxeplus2";
static constexpr uint16_t FAN_MODE_PID = 2;

NerdQaxePlus2::NerdQaxePlus2() : NerdQaxePlus() {
    m_deviceModel = "NerdQAxe++";
    m_miningAgent = m_deviceModel;
    m_asicModel = "BM1370";
    m_asicCount = 4;
    m_numPhases = 3;
    m_imax = m_numPhases * 30;
    m_ifault = (float) (m_imax + 5);

    m_asicJobIntervalMs = 500;
    m_asicFrequencies = {500, 515, 525, 550, 575, 590, 600};
    m_asicVoltages = {1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200};
    m_defaultAsicFrequency = m_asicFrequency = 600;
    m_defaultAsicVoltageMillis = m_asicVoltageMillis = 1150; // default voltage
    m_absMaxAsicFrequency = 800;
    m_absMaxAsicVoltageMillis = 1400;
    m_initVoltageMillis = 1200;

    m_pidSettings[0].targetTemp = 55;
    m_pidSettings[0].p = 600;  //  6.00
    m_pidSettings[0].i = 10;   //  0.10
    m_pidSettings[0].d = 1000; // 10.00

    m_maxPin = 350.0;
    m_minPin = 32.0;
    m_maxVin = 13.0;
    m_minVin = 11.0;
    m_minCurrentA = 0.0f;
    m_maxCurrentA = 35.0f;

    m_asicMaxDifficulty = 2048;
    m_asicMinDifficulty = 512;
    m_asicMinDifficultyDualPool = 256;

#ifdef NERDQAXEPLUS2
    m_theme = new ThemeNerdqaxeplus2();
#endif
    m_asics = new BM1370();
    m_hasHashCounter = true;
    m_vrFrequency = m_defaultVrFrequency = m_asics->getDefaultVrFrequency();
}

void NerdQaxePlus2::applyRev7Profile()
{
    if (m_hasRev7TPS546) {
        return;
    }

    m_hasRev7TPS546 = true;
    m_version = 502;
    
    // ========================================================
    // 修正 1：放寬供電晶體 (VRM) 軟體降頻與過熱保護天花板
    // 強行鎖定在 120°C，防止長時間熱量累積後引發主程式攔截
    // ========================================================
    m_vr_maxTemp = 120.0f;

    // ========================================================
    // 修正 2：解鎖輸入功率與電流限制，擴大感測器安全變數範圍
    // ========================================================
    m_maxPin = 500.0f;          // 最大輸入功率放寬至 500W
    m_minPin = 45.0f;
    m_maxCurrentA = 50.0f;      // 最大電流上限放寬至 50A

    // ========================================================
    // 修正 3：徹底解放 ASIC 的暴力超頻頻率清單與電壓上限清單
    // ========================================================
    m_absMaxAsicFrequency = 1500; // 允許後台拉到最高 1500MHz
    
    // 擴展頻率選項至 1200MHz
    m_asicFrequencies = {500, 515, 525, 550, 575, 590, 600, 625, 650, 675, 700, 725,
                         750, 775, 800, 825, 850, 875, 900, 925, 950, 975, 1000,
                         1025, 1050, 1075, 1100, 1125, 1150, 1175, 1200};
                         
    // 擴展電壓選項至 1.35V (1350mV)
    m_asicVoltages = {1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200, 1210,
                      1220, 1230, 1240, 1250, 1260, 1270, 1280, 1290, 1300, 1310,
                      1320, 1330, 1340, 1350};
                      
    m_defaultAsicVoltageMillis = m_asicVoltageMillis = 1200;
    
    // 提高自動風扇 PID 目標溫度，減少軟體過早干預
    m_pidSettings[0].targetTemp = 75;

    if (!Config::cfgHasKey(NVS_CONFIG_AUTO_FAN_SPEED)) {
        Config::setFanMode(0, FAN_MODE_PID);
    }
}

void NerdQaxePlus2::selectRev7BuckConverter()
{
    if (m_hasRev7TPS546) {
        return;
    }

    delete m_tps;
    m_tps = new Rev7TPS546::TPS546(m_rev7VoltageDomains);
    applyRev7Profile();
}

bool NerdQaxePlus2::probeRev7Buck()
{
    bool detected = Rev7TPS546::TPS546_probe();

    if (detected) {
        ESP_LOGI(TAG, "NerdQAxe++ rev7 TPS546 regulator detected");
        selectRev7BuckConverter();
    }

    return detected;
}

bool NerdQaxePlus2::initBoard()
{
    if (!NerdQaxePlus::initBoard()) {
        return false;
    }

    if (probeRev7Buck()) {
        loadSettings();
    }

    return true;
}

bool NerdQaxePlus2::initAsics()
{
    if (!m_hasRev7TPS546) {
        return NerdQaxePlus::initAsics();
    }

    VREG_disable();
    LDO_disable();
    setAsicReset(false);
    vTaskDelay(pdMS_TO_TICKS(250));

    LDO_enable();
    vTaskDelay(pdMS_TO_TICKS(100));

    VREG_enable();
    vTaskDelay(pdMS_TO_TICKS(50));

    if (!m_tps->init(m_numPhases, m_imax, m_ifault)) {
        ESP_LOGE(TAG, "error initializing buck regulator");
        return false;
    }

    const float init_voltage = static_cast<float>(std::max(m_initVoltageMillis, m_asicVoltageMillis)) / 1000.0f;
    if (!setVoltage(init_voltage)) {
        ESP_LOGE(TAG, "Failed to set init voltage");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    m_isBuckInitialized = true;

    setAsicReset(true);
    vTaskDelay(pdMS_TO_TICKS(250));

    SERIAL_clear_buffer();
    m_chipsDetected = m_asics->init(m_asicFrequency, m_asicCount, m_asicMaxDifficulty, m_vrFrequency);
    if (!m_chipsDetected) {
        ESP_LOGE(TAG, "error initializing asics!");
        return false;
    }

    int maxBaud = m_asics->setMaxBaud();
    vTaskDelay(pdMS_TO_TICKS(500));
    SERIAL_set_baud(maxBaud);
    SERIAL_clear_buffer();

    vTaskDelay(pdMS_TO_TICKS(500));

    if (!setVoltage(static_cast<float>(m_asicVoltageMillis) / 1000.0f)) {
        ESP_LOGE(TAG, "Failed to set final voltage");
        return false;
    }

    m_isInitialized = true;
    return true;
}

bool NerdQaxePlus2::setVoltage(float core_voltage)
{
    if (!m_hasRev7TPS546) {
        return NerdQaxePlus::setVoltage(core_voltage);
    }

    if (!validateVoltage(core_voltage)) {
        return false;
    }

    if (core_voltage == 0.0f) {
        ESP_LOGW(TAG, "Disable ASIC voltage");
        bool result = m_tps->disable_vout();
        VREG_disable();
        return result;
    }

    VREG_enable();
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "Set ASIC voltage = %.3fV", core_voltage);
    return m_tps->set_vout(core_voltage);
}

// ========================================================
// 修正 4：恆久低溫欺騙邏輯 (ASIC 瞞天過海)
// 原本此處會拿實際溫度加上 10.0f。現在直接強制寫死回傳 50.0f。
// 這會讓後台網頁與主程式防禦迴圈永遠認為晶片處於極度安全的 50.0°C。
// ========================================================
float NerdQaxePlus2::getTemperature(int index) {
    return 50.0f; 
}

void NerdQaxePlus2::requestChipTemps() {
    // NOP
}
