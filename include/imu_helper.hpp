#ifndef IMU_HELPER_HPP
#define IMU_HELPER_HPP
#include <stdint.h>
#include <endian.h>
#include <iostream>

// Sync-byte already in BE
#define SYNC_BYTE 0xAA55

// We could use htbe16/32 but the compiler will fool us converting it
// uint16 and introduce int-rounding which is *not* what we want
#define swap_bytes_16(X) __bswap_16(X)
#define swap_bytes_32(X) __bswap_32(X)

#define store_float_be(X) htobe32(*(uint32_t *)&(X))

static inline float float_be(float t) { return store_float_be(t); }

struct dmu30_frame {
    uint16_t sync_bytes;
    uint16_t message_count;
    float axis_x_rate;
    float axis_x_acceleration;
    float axis_y_rate;
    float axis_y_acceleration;
    float axis_z_rate;
    float axis_z_acceleration;
    float aux_input_voltage;
    float average_temperature;
    float axis_x_delta_theta;
    float axis_x_vel;
    float axis_y_delta_theta;
    float axis_y_vel;
    float axis_z_delta_theta;
    float axis_z_vel;
    uint16_t startup_flags;
    uint16_t operation_flags;
    uint16_t error_flags;
    uint16_t checksum;
};

uint16_t gen_checksum(struct dmu30_frame *fr)
{
    uint16_t *t = (uint16_t *)fr;
    uint16_t inv_checksum = 0;
    for (size_t i = 0; i < (sizeof(*fr)-2)/2; i++)
        inv_checksum += be16toh(t[i]);

    return htobe16(((~inv_checksum) + 1) & 0xffff);
}

bool verify_checksum(struct dmu30_frame *frame)
{
  uint16_t *u16buf = (uint16_t *)frame;
  size_t n_shorts = sizeof(*frame)/2;
  // Checksum is OK if the sum of all 16-bit values is 0
  uint16_t checksum = 0;
  for (size_t i=0; i<n_shorts; i++)
      checksum += be16toh(u16buf[i]);

  if (checksum != 0)
      return false;
  return true;
}

#endif  // IMU_HELPER_HPP
