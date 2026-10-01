/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef MOWER_LOGIC_DOCKING_CONFIG_H
#define MOWER_LOGIC_DOCKING_CONFIG_H

namespace docking {

inline bool ConfigurationSupported(bool reverse, int perimeter_signal) {
  return !reverse || perimeter_signal == 0;
}

// Changing direction between behavior transitions would invalidate the physical dock pose and retry direction.
class StartupDirection {
 public:
  bool Apply(bool& requested) {
    if (!initialized_) {
      reverse_ = requested;
      initialized_ = true;
    }
    const bool accepted = requested == reverse_;
    requested = reverse_;
    return accepted;
  }

 private:
  bool initialized_ = false;
  bool reverse_ = false;
};

}  // namespace docking

#endif
