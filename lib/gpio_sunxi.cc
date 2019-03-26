// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define GPIO_REGISTER_OFFSET    0x01C20800
#define GPIO_PWM_BASE_OFFSET	0x01C21400

#define REGISTER_BLOCK_SIZE (4*1024)

#define GPIO_PIN_OUTPUT 0b001
#define GPIO_PORTA_PIN5_PWM 0b011

#define PWM_CH0_EN (1 << 4)
#define PWM_CH0_ACT_HIGH (1 << 5)
#define PWM_CH0_SCLK_GATING (1 << 6)
#define PWM_CH0_PUL_MODE (1 << 7)
#define PWM_CH0_PUL_START (1 << 8)
#define PWM_CH0_PASSTROUGH (1 << 9)

// We want to have the last word in the fifo free
#define MAX_PWM_BIT_USE 224
#define PWM_BASE_TIME_NS 2

// We're pre-mapping all the registers on first call of GPIO::Init(),
// so that it is possible to drop privileges afterwards and still have these
// usable.
static volatile uint32_t *s_GPIO_registers = NULL;
static volatile uint32_t *s_PWM_registers = NULL;

enum PWMClockDiv {
    PWMClockDiv120 = 0b0000,
    PWMClockDiv180 = 0b0001,
    PWMClockDiv240 = 0b0010,
    PWMClockDiv360 = 0b0011,
    PWMClockDiv480 = 0b0100,
    PWMClockDiv12k = 0b1000,
    PWMClockDiv24k = 0b1001,
    PWMClockDiv36k = 0b1010,
    PWMClockDiv48k = 0b1011,
    PWMClockDiv72k = 0b1100,
    PWMClockDiv1   = 0b1111,
};

static void setPinConfig(uint32_t pin, uint8_t config) {
  if (pin > 31) {
    fprintf(stderr, "Pin number not in range 0-31.\n");
    return;
  }
  auto idx = (uint8_t) (pin % 8);
  uint32_t mask = ~((uint32_t) 0b111 << (idx * 4));
  uint32_t val = (config & (uint8_t) 0b111) << (idx * 4);
  volatile uint32_t *cfg = s_GPIO_registers + pin / 8;
  *cfg = (*cfg & mask) | val;
}

namespace rgb_matrix {
/*static*/ const uint32_t GPIO::kValidBits
= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) |
   (1 << 6) | (1 << 11) | (1 << 12) |
#ifdef RGB_LED_NANOPI_NEOCORE
   (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16) |
#endif
   (1 << 17) | (1 << 18) | (1 << 19) | (1 << 20) |
   (1 << 21)
);

GPIO::GPIO() : output_bits_(0), slowdown_(1) {
}

uint32_t GPIO::InitOutputs(uint32_t outputs,
                           bool adafruit_pwm_transition_hack_needed) {
  if (s_GPIO_registers == NULL) {
    fprintf(stderr, "Attempt to init outputs but not yet Init()-ialized.\n");
    return 0;
  }

  outputs &= kValidBits;     // Sanitize: only bits on GPIO header allowed.
  outputs &= ~(output_bits_);
  for (uint32_t b = 0; b <= 21; ++b) {
    if (outputs & (1 << b)) {
      setPinConfig(b, GPIO_PIN_OUTPUT);
    }
  }
  output_bits_ |= outputs;
  return outputs;
}

static uint32_t *mmap_sunxi_register(off_t register_offset) {
  const off_t block_offset = register_offset / REGISTER_BLOCK_SIZE * REGISTER_BLOCK_SIZE;
  register_offset -= block_offset;

  int mem_fd;
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    perror("can't open /dev/mem: ");
    return NULL;
  }

  uint32_t *result =
    (uint32_t*) mmap(NULL,                  // Any adddress in our space will do
                     REGISTER_BLOCK_SIZE,   // Map length
                     PROT_READ|PROT_WRITE,  // Enable r/w on GPIO registers.
                     MAP_SHARED,
                     mem_fd,                // File to map
                     block_offset // Offset to bcm register
                     );
  close(mem_fd);

  if (result == MAP_FAILED) {
    perror("mmap error: ");
    fprintf(stderr, "%s: MMapping from base 0x%lx, offset 0x%lx\n",
            "sunxi", block_offset, register_offset);
    return NULL;
  }
  return result + (register_offset >> 2);
}

static bool mmap_all_sunxi_registers_once() {
  if (s_GPIO_registers != NULL) return true;  // alrady done.

  // The common GPIO registers.
  s_GPIO_registers = mmap_sunxi_register(GPIO_REGISTER_OFFSET);
  if (s_GPIO_registers == NULL) {
    return false;
  }

  // Hardware pin-pulser.
  s_PWM_registers  = mmap_sunxi_register(GPIO_PWM_BASE_OFFSET);
  if (s_PWM_registers == NULL) {
    return false;
  }
  return true;
}

// Based on code example found in http://elinux.org/RPi_Low-level_peripherals
bool GPIO::Init(int slowdown) {
  slowdown_ = slowdown;

  // Pre-mmap all bcm registers we need now and possibly in the future, as to
  // allow  dropping privileges after GPIO::Init() even as some of these
  // registers might be needed later.
  if (!mmap_all_sunxi_registers_once())
    return false;

  gpio_porta_base_ = s_GPIO_registers;
  gpio_porta_dat_ = gpio_porta_base_ + 4;
  return true;
}

namespace {
// A PinPulser that uses the PWM hardware to create accurate pulses.
// It only works on GPIO-18 though.
class HardwarePinPulser : public PinPulser {
public:
  static bool CanHandle(uint32_t gpio_mask) {
    return gpio_mask == (1 << 5);
  }

  HardwarePinPulser(uint32_t pins, const std::vector<int> &specs) {
    assert(CanHandle(pins));

    const int base = specs[0];
    // Get relevant registers
    pwmCtrl = s_PWM_registers;
    pwmCh0Period = pwmCtrl + 1;

    if (pins == (1<<5)) {
      setPinConfig(5, GPIO_PORTA_PIN5_PWM);
    } else {
      assert(false); // should've been caught by CanHandle()
    }

    Reset();
    SetClock(PWMClockDiv1);
    SetActiveHigh(false);
    SetMode(true);
    Enable(true);

    for (size_t i = 0; i < specs.size(); ++i) {
      pwm_range_.push_back(specs[i] / 83);
    }
  }

  virtual void SendPulse(int c) {
    SetPeriodCycles(pwm_range_[c], pwm_range_[c]);
    *pwmCtrl |= PWM_CH0_PUL_START;
  }

  virtual void WaitPulseFinished() {
    while (*pwmCtrl & PWM_CH0_PUL_START) {}
  }

private:
  volatile uint32_t *pwmCtrl;
  volatile uint32_t *pwmCh0Period;

  void Reset() {
    *pwmCtrl &= ~0x3ff;
  }

  void Enable(bool enable) {
    if (enable) {
      *pwmCtrl |= (PWM_CH0_EN | PWM_CH0_SCLK_GATING);
    } else {
      *pwmCtrl &= ~(PWM_CH0_EN | PWM_CH0_SCLK_GATING);
    }
  }

  void SetMode(bool pulseMode) {
    if (pulseMode) {
      *pwmCtrl |= PWM_CH0_PUL_MODE;
    } else {
      *pwmCtrl &= ~PWM_CH0_PUL_MODE;
    }
  }

  void SetClock(PWMClockDiv div) {
    *pwmCtrl = (*pwmCtrl & 0xf) | div;
  }

  void SetPasstrough(bool passtrough) {
    if (passtrough) {
      *pwmCtrl |= PWM_CH0_PASSTROUGH;
    } else {
      *pwmCtrl &= ~PWM_CH0_PASSTROUGH;
    }
  }

  void SetActiveHigh(bool activeHigh) {
    if (activeHigh) {
      *pwmCtrl |= PWM_CH0_ACT_HIGH;
    } else {
      *pwmCtrl &= ~PWM_CH0_ACT_HIGH;
    }
  }

  void SetPeriodCycles(uint16_t active, uint16_t total) {
    *pwmCh0Period = (uint32_t)((active & 0xffff) | ((total & 0xffff) << 16));
  }

private:
  std::vector<uint32_t> pwm_range_;
//  std::vector<int> sleep_hints_;
//  volatile uint32_t *fifo_;
//  uint32_t start_time_;
//  int sleep_hint_;
//  bool triggered_;
};

} // end anonymous namespace

// Public PinPulser factory
PinPulser *PinPulser::Create(GPIO *io, uint32_t gpio_mask,
                             bool allow_hardware_pulsing,
                             const std::vector<int> &nano_wait_spec) {
  if (allow_hardware_pulsing && HardwarePinPulser::CanHandle(gpio_mask)) {
    return new HardwarePinPulser(gpio_mask, nano_wait_spec);
  } else {
    assert(false);
  }
}

uint32_t GetMicrosecondCounter() {
  return 0;
}

} // namespace rgb_matrix
