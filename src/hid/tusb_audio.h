#pragma once

#include "hid/usb.h"
#include "sys/system.h"
#include "util/ringbuffer.h"
#include "tusb.h"

namespace daisy
{
/** @brief USB Transport for MIDI
 *  @ingroup midi
 */
class TUsbAudio
{
  public:
    void Init();
    void StartTx();
    bool IsTxActive() const;

    void PrepAudio(const float* buf, size_t size);

    class Impl;

  private:
    Impl* impl_;
};
} // namespace daisy