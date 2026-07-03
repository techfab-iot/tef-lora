#pragma once

#include <stddef.h>
#include <stdint.h>

namespace app::utils {
uint32_t randomDelayMs(uint32_t min_ms, uint32_t max_ms);
bool formatMacAddress(char *out, size_t out_size);
uint32_t initBootCount();
}  // namespace app::utils
