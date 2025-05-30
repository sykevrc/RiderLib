#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#ifndef _ASSET_H_
#define _ASSET_H_

extern "C" {

typedef struct __attribute__((__packed__)) _asset {
  uint8_t *buf;
  size_t size;
} asset;
}

#define ASSET(x)                                                               \
  static_assert(!__builtin_strcmp(__FUNCTION__, "top level"),                  \
                "Cannot use ASSET inside a function!");                        \
  extern "C" {                                                                 \
  extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];    \
  static asset x = {_binary_static_##x##_start,                                \
                    (size_t)_binary_static_##x##_size};                        \
  }

#define ASSET_LIB(x)                                                           \
  static_assert(!__builtin_strcmp(__FUNCTION__, "top level"),                  \
                "Cannot use ASSET_LIB inside a function!");                    \
  extern "C" {                                                                 \
  extern uint8_t _binary_static_lib_##x##_start[],                             \
      _binary_static_lib_##x##_size[];                                         \
  static asset x = {_binary_static_lib_##x##_start,                            \
                    (size_t)_binary_static_lib_##x##_size};                    \
  }

#endif // _ASSET_H_
