#include <cassert>
#include <cstdint>

#include "hid/tusb_midi.h"
#include "system.h"
#include "tusb.h"

using namespace daisy;

class MidiTUsbTransport::Impl
{
  public:
    void Init(Config config)
    {
        config_    = config;
        rx_active_ = false;

        auto port_id = config.periph == Config::INTERNAL ? 0 : 1;
        tud_init(port_id);
    }

    void StartRx(MidiRxParseCallback callback, void* context)
    {
        FlushRx();
        rx_active_      = true;
        parse_callback_ = callback;
        parse_context_  = context;
    }

    bool RxActive() { return rx_active_; }
    void FlushRx()
    {
        rx_buffer_.Flush();
        uint8_t packet[4];
        while(tud_midi_available())
        {
            tud_midi_packet_read(packet);
        }
    }
    void Tx(uint8_t* buffer, size_t size)
    {
        auto attempt_count = config_.tx_retry_count;
        bool should_retry;

        MidiToUsb(buffer, size);
        do
        {
            auto bytes_sent = tud_midi_stream_write(cable_num_, buffer, size);
            auto result     = bytes_sent == size ? UsbHandle::Result::OK
                                                 : UsbHandle::Result::ERR;

            should_retry
                = (result == UsbHandle::Result::ERR) && attempt_count--;
            if(should_retry)
                System::DelayUs(100);
        } while(should_retry);

        tx_ptr_ = 0;
    }

    void UsbToMidi(uint8_t* buffer, uint8_t length);
    void MidiToUsb(uint8_t* buffer, size_t length);
    void Parse();

  private:
    void MidiToUsbSingle(uint8_t* buffer, size_t length);

    Config config_;

    static constexpr size_t kBufferSize = 1024;
    bool                    rx_active_;
    // This corresponds to 256 midi messages
    RingBuffer<uint8_t, kBufferSize> rx_buffer_;
    MidiRxParseCallback              parse_callback_;
    void*                            parse_context_;

    // simple, self-managed buffer
    uint8_t tx_buffer_[kBufferSize];
    size_t  tx_ptr_;

    uint8_t cable_num_ = 0;

    // MIDI message size determined by the
    // code index number. You can find this
    // table in the MIDI USB spec 1.0
    const uint8_t code_index_size_[16]
        = {3, 3, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1};

    const uint8_t midi_message_size_[16]
        = {0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 1, 1, 2, 0};

    // Masks to check for message type, and byte content
    const uint8_t kStatusByteMask     = 0x80;
    const uint8_t kMessageMask        = 0x70;
    const uint8_t kDataByteMask       = 0x7F;
    const uint8_t kSystemCommonMask   = 0xF0;
    const uint8_t kChannelMask        = 0x0F;
    const uint8_t kRealTimeMask       = 0xF8;
    const uint8_t kSystemRealTimeMask = 0x07;
};

// Global Impl
static MidiTUsbTransport::Impl tusb_midi;


void ReceiveCallback(uint8_t* buffer, uint32_t* length)
{
    if(tusb_midi.RxActive())
    {
        for(uint16_t i = 0; i < *length; i += 4)
        {
            size_t  remaining_bytes = *length - i;
            uint8_t packet_length   = remaining_bytes > 4 ? 4 : remaining_bytes;
            tusb_midi.UsbToMidi(buffer + i, packet_length);
            tusb_midi.Parse();
        }
    }
}

extern "C"
{
    void tud_midi_rx_cb(uint8_t itf)
    {
        (void)itf;

        uint8_t  packet[4];
        uint32_t size = 4;
        while(tud_midi_available())
        {
            tud_midi_packet_read(packet);
            ReceiveCallback(packet, &size);
        }
    }
}


void MidiTUsbTransport::Impl::UsbToMidi(uint8_t* buffer, uint8_t length)
{
    // A length of less than four in the buffer indicates
    // a garbled message, since USB MIDI packets usually*
    // require 4 bytes per message
    if(length < 4)
        return;

    // Right now, Daisy only supports a single cable, so we don't
    // need to extract that value from the upper nibble
    uint8_t code_index = buffer[0] & 0xF;
    if(code_index == 0x0 || code_index == 0x1)
    {
        // 0x0 and 0x1 are reserved codes, and if they come up,
        // there's probably been an error. *0xF indicates data is
        // sent one byte at a time, rather than in packets of four.
        // This functionality could be supported later.
        // The single-byte mode does still come through as 32-bit messages
        return;
    }

    // Only writing as many bytes as necessary
    for(uint8_t i = 0; i < code_index_size_[code_index]; i++)
    {
        if(rx_buffer_.writable() > 0)
            rx_buffer_.Write(buffer[1 + i]);
        else
        {
            rx_active_ = false; // disable on overflow
            break;
        }
    }
}

void MidiTUsbTransport::Impl::MidiToUsbSingle(uint8_t* buffer, size_t size)
{
    if(size == 0)
        return;

    // Channel voice messages
    if((buffer[0] & 0xF0) != 0xF0)
    {
        // Check message validity
        if((buffer[0] & 0xF0) == 0xC0 || (buffer[0] & 0xF0) == 0xD0)
        {
            if(size != 2)
                return; // error
        }
        else
        {
            if(size != 3)
                return; //error
        }

        // CIN is the same as status byte for channel voice messages
        tx_buffer_[tx_ptr_ + 0] = (buffer[0] & 0xF0) >> 4;
        tx_buffer_[tx_ptr_ + 1] = buffer[0];
        tx_buffer_[tx_ptr_ + 2] = buffer[1];
        tx_buffer_[tx_ptr_ + 3] = size == 3 ? buffer[2] : 0;

        tx_ptr_ += 4;
    }
    else // buffer[0] & 0xF0 == 0xF0 aka System common or realtime
    {
        if(0xF2 == buffer[0])
        // three byte message
        {
            if(size != 3)
                return; // error

            tx_buffer_[tx_ptr_ + 0] = 0x03;
            tx_buffer_[tx_ptr_ + 1] = buffer[0];
            tx_buffer_[tx_ptr_ + 2] = buffer[1];
            tx_buffer_[tx_ptr_ + 3] = buffer[2];

            tx_ptr_ += 4;
        }
        if(0xF1 == buffer[0] || 0xF3 == buffer[0])
        // two byte messages
        {
            if(size != 2)
                return; // error

            tx_buffer_[tx_ptr_ + 0] = 0x02;
            tx_buffer_[tx_ptr_ + 1] = buffer[0];
            tx_buffer_[tx_ptr_ + 2] = buffer[1];
            tx_buffer_[tx_ptr_ + 3] = 0;

            tx_ptr_ += 4;
        }
        else if(0xF4 <= buffer[0])
        // one byte message
        {
            if(size != 1)
                return; // error

            tx_buffer_[tx_ptr_ + 0] = 0x05;
            tx_buffer_[tx_ptr_ + 1] = buffer[0];
            tx_buffer_[tx_ptr_ + 2] = 0;
            tx_buffer_[tx_ptr_ + 3] = 0;

            tx_ptr_ += 4;
        }
        else // sysex
        {
            size_t i = 0;
            // Sysex messages are split up into several 4 bytes packets
            // first ones use CIN 0x04
            // but packet containing the SysEx stop byte use a different CIN
            for(i = 0; i + 3 < size; i += 3, tx_ptr_ += 4)
            {
                tx_buffer_[tx_ptr_]     = 0x04;
                tx_buffer_[tx_ptr_ + 1] = buffer[i];
                tx_buffer_[tx_ptr_ + 2] = buffer[i + 1];
                tx_buffer_[tx_ptr_ + 3] = buffer[i + 2];
            }

            // Fill CIN for terminating bytes
            // 0x05 for 1 remaining byte
            // 0x06 for 2
            // 0x07 for 3
            tx_buffer_[tx_ptr_] = 0x05 + (size - i - 1);
            tx_ptr_++;
            for(; i < size; ++i, ++tx_ptr_)
                tx_buffer_[tx_ptr_] = buffer[i];
            for(; (tx_ptr_ % 4) != 0; ++tx_ptr_)
                tx_buffer_[tx_ptr_] = 0;
        }
    }
}

void MidiTUsbTransport::Impl::MidiToUsb(uint8_t* buffer, size_t size)
{
    // We'll assume your message starts with a status byte!
    size_t status_index = 0;
    while(status_index < size)
    {
        // Search for next status byte or end
        size_t next_status = status_index;
        for(size_t j = status_index + 1; j < size; j++)
        {
            if(buffer[j] & 0x80)
            {
                next_status = j;
                break;
            }
        }
        if(next_status == status_index)
        {
            // Either we're at the end or it's malformed
            next_status = size;
        }
        MidiToUsbSingle(buffer + status_index, next_status - status_index);
        status_index = next_status;
    }
}

void MidiTUsbTransport::Impl::Parse()
{
    if(parse_callback_)
    {
        uint8_t bytes[kBufferSize];
        size_t  i = 0;
        while(!rx_buffer_.isEmpty())
        {
            bytes[i++] = rx_buffer_.Read();
        }
        parse_callback_(bytes, i, parse_context_);
    }
}

////////////////////////////////////////////////
// MidiTUsbTransport -> MidiTUsbTransport::Impl
////////////////////////////////////////////////

void MidiTUsbTransport::Init(MidiTUsbTransport::Config config)
{
    impl_ = &tusb_midi;
    impl_->Init(config);
}

void MidiTUsbTransport::StartRx(MidiRxParseCallback callback, void* context)
{
    impl_->StartRx(callback, context);
}

bool MidiTUsbTransport::RxActive()
{
    return impl_->RxActive();
}

void MidiTUsbTransport::FlushRx()
{
    impl_->FlushRx();
}

void MidiTUsbTransport::Tx(uint8_t* buffer, size_t size)
{
    impl_->Tx(buffer, size);
}

// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

// Store example melody as an array of note values
uint8_t note_sequence[]
    = {74, 78, 81, 86,  90, 93, 98, 102, 57, 61,  66, 69, 73, 78, 81, 85,
       88, 92, 97, 100, 97, 92, 88, 85,  81, 78,  74, 69, 66, 62, 57, 62,
       66, 69, 74, 78,  81, 86, 90, 93,  97, 102, 97, 93, 90, 85, 81, 78,
       73, 68, 64, 61,  56, 61, 64, 68,  74, 78,  81, 86, 90, 93, 98, 102};

// TODO: REMOVE THIS
void MidiTUsbTransport::TestTask()
{
    static uint32_t start_ms = 0;

    uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
    uint8_t const channel   = 0; // 0 for channel 1

    // The MIDI interface always creates input and output port/jack descriptors
    // regardless of these being used or not. Therefore incoming traffic should be read
    // (possibly just discarded) to avoid the sender blocking in IO
    uint8_t packet[4];
    while(tud_midi_available())
        tud_midi_packet_read(packet);


    // send note periodically
    if(daisy::System::GetNow() - start_ms < 286)
        return; // not enough time
    start_ms += 286;

    // Previous positions in the note sequence.
    int previous = (int)(note_pos - 1);

    // If we currently are at position 0, set the
    // previous position to the last note in the sequence.
    if(previous < 0)
        previous = sizeof(note_sequence) - 1;

    // Send Note On for current position at full velocity (127) on channel 1.
    uint8_t note_on[3] = {0x90 | channel, note_sequence[note_pos], 127};
    tud_midi_stream_write(cable_num, note_on, 3);

    // Send Note Off for previous note.
    uint8_t note_off[3] = {0x80 | channel, note_sequence[previous], 0};
    tud_midi_stream_write(cable_num, note_off, 3);

    // Increment position
    note_pos++;

    // If we are at the end of the sequence, start over.
    if(note_pos >= sizeof(note_sequence))
        note_pos = 0;
}