
    m_pid[ch]->SetTarget(static_cast<float>(m_config[ch].pid.targetTemp));

    ESP_LOGI(TAG, "ch%d PID: target=%.1f°C p=%.2f i=%.2f d=%.2f",
             ch, static_cast<float>(m_config[ch].pid.targetTemp), p, i, d);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void FanController::init(Board* board, int sampleTimeMs)
{
    m_board        = board;
    m_sampleTimeMs = sampleTimeMs;
    m_numChannels  = std::min(board->getNumFans(), MAX_FANS);

    ESP_LOGI(TAG, "initialising for %d fan channel(s) - FORCED MAX MODIFIED", m_numChannels);

    // Load config from NVS (PID instances don't exist yet, applyConfig is skipped)
    loadSettings();

    // Create PID instances using the loaded config
    for (int ch = 0; ch < m_numChannels; ch++) {
        m_pidTarget[ch] = static_cast<float>(m_config[ch].pid.targetTemp);

        float p = m_config[ch].pid.p / 100.0f;
        float i = m_config[ch].pid.i / 100.0f;
        float d = m_config[ch].pid.d / 100.0f;

        m_pid[ch] = new PID(&m_pidInput[ch], &m_pidOutput[ch], &m_pidTarget[ch],
                            p, i, d, PID_P_ON_E, PID_DIRECT);
        m_pid[ch]->SetSampleTime(sampleTimeMs);
        
        // 【修改 1】：強行修改輸出上下限，封鎖低速，PID 無論點算都只能輸出 100%
        m_pid[ch]->SetOutputLimits(100, 100); 
        
        m_pid[ch]->SetMode(PID_AUTOMATIC);
        m_pid[ch]->SetControllerDirection(PID_REVERSE);
        m_pid[ch]->Initialize();

        ESP_LOGI(TAG, "ch%d: mode=%d manual=%d%% overheat=%d°C pid-target=%d°C p=%.2f i=%.2f d=%.2f",
                 ch, static_cast<int>(m_config[ch].mode),
                 m_config[ch].manualSpeed, m_config[ch].overheatTemp,
                 m_config[ch].pid.targetTemp, p, i, d);
    }
}

void FanController::loadSettings()
{
    m_pidUseMax = Config::isFanPidUseMax();

    for (int ch = 0; ch < m_numChannels; ch++) {
        PidSettings* bp = m_board->getPidSettings(ch);

        m_config[ch].mode         = static_cast<Mode>(m_board->getFanMode(ch));
        m_config[ch].manualSpeed  = Config::getFanManualSpeed(ch);
        m_config[ch].overheatTemp = Config::getFanOverheatTemp(ch);
        m_config[ch].pid.targetTemp = Config::getFanPidTargetTemp(ch, bp->targetTemp);
        m_config[ch].pid.p          = Config::getFanPidP(ch, bp->p);
        m_config[ch].pid.i          = Config::getFanPidI(ch, bp->i);
        m_config[ch].pid.d          = Config::getFanPidD(ch, bp->d);

        applyConfig(ch);  // no-op if PID not yet created
    }
}

void FanController::update(float chipTempMax, float vrTemp)
{
    // Overheat thresholds use original per-channel temps (ch0=chip, ch1=VR)
    float overheatInput[MAX_FANS] = { chipTempMax, vrTemp };

    // PID temperature input per channel: ch0=chip, ch1=VR
    float tempInput[MAX_FANS] = { chipTempMax, vrTemp };

    // When 2nd channel is linked (2-fan boards) or pidUseMax is enabled
    // (single-fan boards), ch0 PID uses the higher of both temps.
    if (m_numChannels > 1 && m_config[1].mode == Mode::LINKED) {
        tempInput[0] = fmaxf(chipTempMax, vrTemp);
    } else if (m_numChannels == 1 && m_pidUseMax) {
        tempInput[0] = fmaxf(chipTempMax, vrTemp);
    }

    for (int ch = 0; ch < m_numChannels; ch++) {
        // Read current RPM
        m_board->getFanSpeedCh(ch, &m_fanRPM[ch]);

        // Always compute PID for bumpless transfer when switching modes
        m_pidInput[ch] = tempInput[ch];
        if (m_pid[ch]) {
            m_pid[ch]->Compute();
        }

        // Overheat: use original temps, not PID-mixed temps
        if (m_config[ch].overheatTemp && overheatInput[ch] > m_config[ch].overheatTemp) {
            m_overheated[ch] = true;
            m_fanPerc[ch]    = 100;
            m_board->setFanSpeedCh(ch, 1.0f);
            continue;
        }
        m_overheated[ch] = false;

        // LINKED: mirror channel-0 output (ch0 must be processed first)
        if (m_config[ch].mode == Mode::LINKED && ch > 0) {
            m_fanPerc[ch] = m_fanPerc[0];
            m_board->setFanSpeedCh(ch, static_cast<float>(m_fanPerc[ch]) / 100.0f);
            continue;
        }
        m_overheated[ch] = false;

        // 【修改 2】：直接註解/無視原本的 switch mode 判斷
        // 無論網頁目前是什麼設定，一律硬編碼（Hardcode）逼入 100% 瘋狂輸出
        /*
        switch (m_config[ch].mode) {
        case Mode::MANUAL:
            m_fanPerc[ch] = m_config[ch].manualSpeed;
            break;
        case Mode::PID:
            m_fanPerc[ch] = static_cast<uint16_t>(roundf(m_pidOutput[ch]));
            break;
        default:
            ESP_LOGE(TAG, "ch%d: unknown mode %d, defaulting to 100%%", ch, static_cast<int>(m_config[ch].mode));
            m_fanPerc[ch] = 100;
            break;
        }
        */

        // 暴力全速覆蓋
        m_fanPerc[ch] = 100;
        m_board->setFanSpeedCh(ch, 1.0f); // 1.0f 代表發出 100% 滿轉的 PWM 訊號
    }
}

uint16_t FanController::getRPM(int ch) const
{
    if (ch < 0 || ch >= m_numChannels) return 0;
    return m_fanRPM[ch];
}

uint16_t FanController::getSpeedPerc(int ch) const
{
    if (ch < 0 || ch >= m_numChannels) return 0;
    return m_fanPerc[ch];
}

bool FanController::isOverheated(int ch) const
{
    if (ch < 0 || ch >= m_numChannels) return false;
    return m_overheated[ch];
}

uint16_t FanController::getOverheatTemp(int ch) const
{
    if (ch < 0 || ch >= m_numChannels) return 0;
    return m_config[ch].overheatTemp;
}
