#include "common/motor_crc_hg.h"

#include <cstring>

void get_crc(unitree_hg::msg::LowCmd &msg) {
  // memset, not `raw{}`: the robot's CRC covers the padding bytes as 0, and `{}` leaves them unspecified.
  LowCmd raw;
  std::memset(static_cast<void *>(&raw), 0, sizeof(raw));

  raw.modePr = msg.mode_pr;
  raw.modeMachine = msg.mode_machine;

  for (int i = 0; i < 35; i++) {
    raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
    raw.motorCmd[i].q = msg.motor_cmd[i].q;
    raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
    raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
    raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
    raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;

    raw.motorCmd[i].reserve = msg.motor_cmd[i].reserve;
  }

  memcpy(&raw.reserve[0], &msg.reserve[0], 4);

  // memcpy, not a uint32_t* cast: the cast breaks strict aliasing, and at -O2/-O3 the CRC
  // covered stale bytes, so the robot dropped every frame (2026-10-20, Release build).
  uint32_t words[sizeof(LowCmd) / 4];
  std::memcpy(words, &raw, sizeof(words));
  raw.crc = crc32_core(words, (sizeof(LowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}

uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
}