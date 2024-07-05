#include "hid/tusb_audio.h"
#include <array>
#include "daisy_core.h"
#include "hid/logger.h"

using DBG = daisy::Logger<daisy::LOGGER_INTERNAL>;


using namespace daisy;

static constexpr size_t max_packet_size = CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ;

class TUsbAudio::Impl
{
  public:
    void Init() {}
    void StartTx() { tx_active_ = true; }
    bool IsTxActive() const { return tx_active_; }
    void PrepAudio(const float* buf, size_t size)
    {
        if(!tx_active_)
        {
            return;
        }

        // Only writing as many bytes as necessary
        // for(uint8_t i = 0; i < size; i++)
        // {
        //     if(tx_buffer_.writable() > 0)
        //         tx_buffer_.Write(f2s16(buf[i]));
        //     else
        //     {
        //         tx_active_ = false; // disable on overflow
        //         DBG::Print("tx_buffer_ OVERFLOW!: %d\n", size);

        //         // while(true) {};
        //         break;
        //     }
        // }
        // DBG::Print("Prep: %d\n", size);


        int16_t bufffff[max_packet_size] = {0};

        if(size > max_packet_size)
        {
            DBG::Print("size > max_packet_size\n");
        }

        for(uint8_t i = 0; i < size; i++)
        {
            bufffff[i] = f2s16(buf[i]);
        }
        auto buf_size = (size) * sizeof(int16_t);
        auto written  = tud_audio_write(bufffff, buf_size);
        if(written != buf_size)
        {
            DBG::Print("Error writing audio: %d\n", written);
        }
    }

    int16_t Read() { return tx_buffer_.Read(); }

    bool CanRead() { return tx_buffer_.readable() > 0; }

    bool tx_active_;


    static constexpr size_t          kBufferSize = 1024 * 10;
    RingBuffer<int16_t, kBufferSize> tx_buffer_;
};


static TUsbAudio::Impl tusb_audio;

int16_t test_buffer_audio[max_packet_size + 2] = {0};
size_t  num_samples                            = 0;

/*

// WRITE PREPARED BUFFER
bool tud_audio_tx_done_pre_load_cb(uint8_t rhport,
                                   uint8_t itf,
                                   uint8_t ep_in,
                                   uint8_t cur_alt_setting)
{
    if(tusb_audio.IsTxActive())
    {
        tud_audio_write(test_buffer_audio, (num_samples) * sizeof(int16_t));
        num_samples = 0;
    }


    return true;
}

// PREPARE NEXT BUFFER
bool tud_audio_tx_done_post_load_cb(uint8_t  rhport,
                                    uint16_t n_bytes_copied,
                                    uint8_t  itf,
                                    uint8_t  ep_in,
                                    uint8_t  cur_alt_setting)
{
    num_samples = 0;
    for(size_t cnt = 0; cnt < max_packet_size; cnt++)
    {
        if(!tusb_audio.CanRead())
        {
            break;
        }
        test_buffer_audio[cnt] = tusb_audio.Read();
        num_samples++;
    }

    // auto available_samples = tusb_audio.tx_buffer_.readable();
    // tusb_audio.tx_buffer_.ImmediateRead(test_buffer_audio, available_samples);
    // num_samples = available_samples;

    // DBG::Print("Post Load: %d\n", num_samples);

    return tusb_audio.tx_buffer_.readable() > 0;
    // return true;
}

 */

bool tud_audio_set_itf_cb(uint8_t                       rhport,
                          tusb_control_request_t const* p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    DBG::Print("Set interface %d alt %d", itf, alt);

    if(alt == 0)
    {
        // Zero bandwidth - PC has likely stopped the audio stream
        DBG::Print(" Zero bandwidth \r\n");
        tusb_audio.tx_buffer_.Flush();
        tusb_audio.tx_active_ = false;
    }
    else
    {
        // Non-zero bandwidth - PC has likely started the audio stream
        DBG::Print(" Non-zero bandwidth \r\n");
        tusb_audio.tx_buffer_.Flush();
        tusb_audio.tx_active_ = true;
    }
    return true;
}

void TUsbAudio::Init()
{
    impl_ = &tusb_audio;
    impl_->Init();
}
void TUsbAudio::StartTx()
{
    impl_->StartTx();
}

void TUsbAudio::PrepAudio(const float* buf, size_t size)
{
    impl_->PrepAudio(buf, size);
}

bool TUsbAudio::IsTxActive() const
{
    return impl_->IsTxActive();
}
