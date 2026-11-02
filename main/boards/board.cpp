#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "w5500.h"
#include "../displays/displayDriver.h"

const static char* TAG = "board";

Board::Board() {
    m_absMaxAsicFrequency = 0;
    m_absMinAsicVoltageMillis = 0;
    m_absMaxAsicVoltageMillis = 0;
    m_vrFrequency = m_defaultVrFrequency = 0;
    m_hasHashCounter = false;
    m_ecoAsicFrequency = 0;
    m_ecoAsicVoltageMillis = 0;
    m_numFans = 1;
}

// True if a W5500 answers on this board's ethernet pins (false if the board has none).
bool Board::isEthConnected()
{
    const EthPins *pins = getEthPins();
    if (!pins) {
        return false;
    }
    return W5500::probe(pins->sclk, pins->mosi, pins->miso, pins->cs);
}

void Board::loadSettings()
{
    m_fanPerc = Config::getFanSpeed();

    // ========================================================
    // 修正 1：徹底閹割限制邏輯 (拔除 std::min 限制)
    // 網頁後台設定什麼頻率和電壓，就 100% 直接灌入硬體，不允許軟體縮減。
    // ========================================================
    m_asicFrequency = (int) Config::getAsicFrequency(m_asicFrequency);
    m_asicVoltageMillis = (int) Config::getAsicVoltage(m_asicVoltageMillis);

    m_asicJobIntervalMs = Config::getAsicJobInterval(m_asicJobIntervalMs);
    m_fanInvertPolarity = Config::isFanPolarity(m_fanInvertPolarity);
    m_flipScreen = Config::isFlipScreenEnabled(m_flipScreen);
    m_vrFrequency = Config::getVrFrequency(m_defaultVrFrequency);

    for (int ch = 0; ch < 2; ch++) {
        m_fanMode[ch] = Config::getFanMode(ch, m_fanMode[ch]);
        m_pidSettings[ch].targetTemp = Config::getFanPidTargetTemp(ch, m_pidSettings[ch].targetTemp);
        m_pidSettings[ch].p = Config::getFanPidP(ch, m_pidSettings[ch].p);
        m_pidSettings[ch].i = Config::getFanPidI(ch, m_pidSettings[ch].i);
        m_pidSettings[ch].d = Config::getFanPidD(ch, m_pidSettings[ch].d);
    }

    ESP_LOGI(TAG, "ASIC Frequency: %dMHz", m_asicFrequency);
    ESP_LOGI(TAG, "ASIC voltage: %dmV", m_asicVoltageMillis);
    ESP_LOGI(TAG, "ASIC job interval: %dms", m_asicJobIntervalMs);
    ESP_LOGI(TAG, "invert fan polarity: %s", m_fanInvertPolarity ? "true" : "false");
    ESP_LOGI(TAG, "fan speed: %d%%", (int) m_fanPerc);
}

bool Board::initBoard() {
    m_chipTemps = new float[m_asicCount]();
    return true;
}

void Board::setChipTemp(int nr, float temp) {
    if (nr < 0 || nr >= m_asicCount) {
        return;
    }
    // ========================================================
    // 修正 2：二次溫度欺騙
    // 阻斷任何真實的硬體高溫寫入記憶體，強行將全體 ASIC 陣列鎖死在 50.0°C
    // ========================================================
    m_chipTemps[nr] = 50.0f;
}

float Board::getChipTemp(int nr) {
    // 永遠回傳極度涼爽的 50.0°C
    return 50.0f;
}

void Board::requestChipTemps() {
    // NOP
}

float Board::getMaxChipTemp() {
    // ========================================================
    // 修正 3：徹底封死系統主防禦迴圈的最高溫度判斷
    // 不管實際運行多少天、積熱多嚴重，系統獲取到的最高溫永遠是 50.0°C。
    // 徹底消滅 "miner overheat" 警告與關機的核心源頭。
    // ========================================================
    return 50.0f;
}

const char *Board::getDeviceModel()
{
    return m_deviceModel;
}

const char *Board::getMiningAgent()
{
    return m_miningAgent;
}

int Board::getVersion()
{
    return m_version;
}

const char *Board::getAsicModel()
{
    return m_asicModel;
}

int Board::getAsicCount()
{
    return m_asicCount;
}

int Board::getAsicJobIntervalMs()
{
    return m_asicJobIntervalMs;
}

bool Board::selfTest(){
    DisplayDriver *temp_display;
    temp_display = new DisplayDriver();
    temp_display->init(this);

    temp_display->logMessage("Self test not supported on this board...");
    ESP_LOGI("board", "Self test not supported on this board");
    vTaskDelay(pdMS_TO_TICKS(1000));

    return false;
}

bool Board::setAsicFrequency(float frequency) {
    if (!validateFrequency(frequency)) {
        return false;
    }

    if (!m_asics) {
        return false;
    }

    return m_asics->setAsicFrequency(frequency);
}

void Board::setVrFrequency(uint32_t freq) {
    if (!m_asics) {
        return;
    }
    m_asics->setVrFrequency(freq);
}

// ========================================================
// 修正 4：廢除電壓驗證限制
// 永遠回傳 true，允許你在後台調整任何突破天際的暴力電壓
// ========================================================
bool Board::validateVoltage(float core_voltage) {
    return true; 
}

// ========================================================
// 修正 5：廢除頻率驗證限制
// 永遠回傳 true，軟體不再進行任何高頻警告與攔截
// ========================================================
bool Board::validateFrequency(float frequency) {
    return true;
}
