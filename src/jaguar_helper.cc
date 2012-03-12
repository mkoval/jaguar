#include <cassert>
#include "jaguar_helper.h"

namespace jaguar {

int16_t double_to_s8p8(double x)
{
    return htole16(static_cast<int16_t>(x * 256));
}

int32_t double_to_s16p16(double x)
{
    return htole32(static_cast<int32_t>(x * 65536));
}

double s8p8_to_double(int16_t x)
{
    return le16toh(x) / 256.;
}

double s16p16_to_double(int32_t x)
{
    return le32toh(x) / 65536.;
}

uint32_t pack_id(uint8_t num, Manufacturer::Enum man, DeviceType::Enum type,
	                     APIClass::Enum api_class, uint8_t api_index)
{
    assert((num       & ~0x3F) == 0);
    assert((api_class & ~0x3F) == 0);
    assert((api_index & ~0x0F) == 0);
    assert((man       & ~0xFF) == 0);
    assert((type      & ~0x1F) == 0);

    return (static_cast<uint32_t>(num)       << 0)
         | (static_cast<uint32_t>(api_index) << 6)
         | (static_cast<uint32_t>(api_class) << 10)
         | (static_cast<uint32_t>(man)       << 16)
         | (static_cast<uint32_t>(type)      << 24);
}

};