#pragma once

#include "TPS53647.h"

class TPS53667 : public TPS53647 {
protected:
    virtual void set_phases(int num_phases) override;

public:
    static constexpr uint16_t DEVICE_CODE = 0x01f8;

    TPS53667();
    virtual ~TPS53667() = default;

    virtual bool init(int num_phases, int imax, float ifault) override;
    virtual void status() override;

    bool read_word(uint8_t command, uint16_t *data);
    bool read_byte(uint8_t command, uint8_t *data);
};
