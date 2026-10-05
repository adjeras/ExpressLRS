#pragma once

#include "handset.h"

class TorqueDollyHandset final : public Handset
{
public:
    void Begin() override;
    void End() override;
    void handleInput() override;
    bool IsArmed() override { return false; }
};
