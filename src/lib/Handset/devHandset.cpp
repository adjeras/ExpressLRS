#include "targets.h"

#ifdef TARGET_TX

#include "CRSF.h"
#include "CRSFHandset.h"
#include "POWERMGNT.h"
#include "devHandset.h"
#if defined(TORQUE_DOLLY_HANDSET)
#include "TorqueDollyHandset.h"
#endif

#if defined(PLATFORM_ESP32)
#include "AutoDetect.h"
#endif

Handset *handset;

static void initialize()
{
#if defined(TORQUE_DOLLY_HANDSET)
    handset = new TorqueDollyHandset();
#elif defined(PLATFORM_ESP32)
    if (GPIO_PIN_RCSIGNAL_RX == GPIO_PIN_RCSIGNAL_TX)
    {
        handset = new AutoDetect();
        return;
    }
    handset = new CRSFHandset();
#else
    handset = new CRSFHandset();
#endif
}

static int start()
{
    handset->Begin();
#if defined(DEBUG_TX_FREERUN)
    handset->forceConnection();
#endif
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    handset->handleInput();
    return DURATION_IMMEDIATELY;
}

static int event()
{
    // An event should be generated every time the TX power changes
    CRSF::LinkStatistics.uplink_TX_Power = powerToCrsfPower(PowerLevelContainer::currPower());
    return DURATION_IGNORE;
}

device_t Handset_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout};
#endif
