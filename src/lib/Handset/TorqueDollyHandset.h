#pragma once

#include "handset.h"

class TorqueDollyHandset final : public Handset
{
public:
    void Begin() override;
    void End() override;
    void handleInput() override;

private:
    float xFiltered = 0.0f;
    float yFiltered = 0.0f;
    float xDrive = 0.0f;
    float yDrive = 0.0f;
    uint32_t lastUpdate = 0;
    bool startupNeutral = true;
    bool filterInitialized = false;
};
