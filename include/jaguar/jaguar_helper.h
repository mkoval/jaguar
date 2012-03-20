#ifndef JAGUAR_HELPER_H_
#define JAGUAR_HELPER_H_

#include <boost/detail/endian.hpp>
#include <stdint.h>
#include "jaguar_api.h"

#ifndef __GLIBC__
# ifdef BOOST_LITTLE_ENDIAN
#  define htole16(x) x
#  define htole32(x) x
#  define le16toh(x) x
#  define le32toh(x) x
# elif BOOST_BIG_ENDIAN
#  error big endian architectures are unsupported
# else
#  error unknown endiannes
# endif
#endif

namespace jaguar {

int16_t  double_to_s8p8(double x);
int32_t  double_to_s16p16(double x);

double s8p8_to_double(int16_t x);
double s16p16_to_double(int32_t x);

uint32_t pack_id(uint8_t device_num, Manufacturer::Enum man, DeviceType::Enum type,
                 APIClass::Enum api_class, uint8_t api_index);

uint32_t pack_id(uint8_t dnum, Manufacturer::Enum man, DeviceType::Enum type, uint16_t api);

uint32_t pack_ack(uint8_t device_num, Manufacturer::Enum man, DeviceType::Enum type);
};

#endif
