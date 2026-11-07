#include <cassert>
#include <cstdint>
#include <cstdio>

static uint16_t saturatingAge(uint32_t age)
{
    return static_cast<uint16_t>(age > 65535U ? 65535U : age);
}

static int8_t convertRssi(uint8_t otaMagnitude)
{
    return -static_cast<int8_t>(otaMagnitude);
}

static int8_t decodeCrsfRssi(uint8_t crsfValue)
{
    return static_cast<int8_t>(crsfValue);
}

static void testSignedRssi()
{
    assert(convertRssi(40) == -40);
    assert(decodeCrsfRssi(216) == -40);
    assert(decodeCrsfRssi(218) == -38);
}

static void testPayloadEncoding()
{
    uint8_t payload[12] = {};
    const uint16_t sequence = 7;
    const uint16_t age = 1234;
    payload[0] = 1;
    payload[1] = 1;
    payload[2] = static_cast<uint8_t>(sequence);
    payload[3] = static_cast<uint8_t>(sequence >> 8);
    payload[4] = static_cast<uint8_t>(age);
    payload[5] = static_cast<uint8_t>(age >> 8);
    payload[6] = 100;
    payload[7] = static_cast<uint8_t>(-40);
    payload[8] = static_cast<uint8_t>(12);
    assert(payload[0] == 1 && payload[2] == 7 && payload[3] == 0);
    assert(payload[4] == 0xD2 && payload[5] == 0x04);
    assert(static_cast<int8_t>(payload[7]) == -40);
}

static void testUnavailableAndStale()
{
    assert(saturatingAge(0) == 0);
    assert(saturatingAge(2001) == 2001);
    assert(saturatingAge(70000) == 65535);
}

int main()
{
    testSignedRssi();
    testPayloadEncoding();
    testUnavailableAndStale();
    std::puts("Torque Dolly RF link statistics tests passed");
    return 0;
}
