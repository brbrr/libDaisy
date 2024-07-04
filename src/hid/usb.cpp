#include "hid/usb.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "tusb.h"

#include "system.h"

using namespace daisy;

static void UsbErrorHandler();

uint8_t usb_fs_hw_initialized = 0;
uint8_t usb_hs_hw_initialized = 0;

// Externs for IRQ Handlers
extern "C"
{
    // Globals from Cube generated version:
    USBD_HandleTypeDef       hUsbDeviceHS;
    USBD_HandleTypeDef       hUsbDeviceFS;
    extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
    extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

    void DummyRxCallback(uint8_t* buf, uint32_t* size)
    {
        // Do Nothing
    }

    CDC_ReceiveCallback rxcallback;

    uint8_t usbd_mode = USBD_MODE_CDC;
}

UsbHandle::ReceiveCallback rx_callback;

static void InitFS()
{
    rx_callback = DummyRxCallback;
    if(usb_fs_hw_initialized == 0)
    {
        usb_fs_hw_initialized = 1;
        if(USBD_Init(&hUsbDeviceFS, NULL, DEVICE_FS) != USBD_OK)
        {
            UsbErrorHandler();
        }
    }
    auto result = tud_init(BOARD_TUD_RHPORT);
    if(!result)
    {
        while(true) {}
    }
    // !!! Required to make CDC work
    // while(!tud_cdc_connected())
    // {
    //     tud_task();
    // }
}

static void DeinitFS()
{
    if(USBD_DeInit(&hUsbDeviceFS) != USBD_OK)
    {
        UsbErrorHandler();
    }
}

static void InitHS()
{
    rx_callback = DummyRxCallback;
    if(usb_hs_hw_initialized == 0)
    {
        usb_hs_hw_initialized = 1;
        if(USBD_Init(&hUsbDeviceHS, NULL, DEVICE_HS) != USBD_OK)
        {
            UsbErrorHandler();
        }
    }
    tud_init(1);

    while(!tud_cdc_connected())
    {
        tud_task();
    }
}

void UsbHandle::RunTask()
{
    tud_task();
}


// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

// Store example melody as an array of note values
uint8_t note_sequence[]
    = {74, 78, 81, 86,  90, 93, 98, 102, 57, 61,  66, 69, 73, 78, 81, 85,
       88, 92, 97, 100, 97, 92, 88, 85,  81, 78,  74, 69, 66, 62, 57, 62,
       66, 69, 74, 78,  81, 86, 90, 93,  97, 102, 97, 93, 90, 85, 81, 78,
       73, 68, 64, 61,  56, 61, 64, 68,  74, 78,  81, 86, 90, 93, 98, 102};

void UsbHandle::MidiTask()
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


uint16_t test_buffer_audio[(CFG_TUD_AUDIO_EP_SZ_IN - 2) / 2];
uint16_t startVal = 0;

uint32_t sampFreq;
uint8_t  clkValid;
audio_control_range_4_n_t(1) sampleFreqRng; // Sample frequency range state

bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // +1 for master channel 0
uint16_t
    volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // +1 for master channel 0

bool inited = false;
void tud_audio__init()
{
    if(inited)
        return;
    inited   = true;
    sampFreq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    clkValid = 1;

    sampleFreqRng.wNumSubRanges    = 1;
    sampleFreqRng.subrange[0].bMin = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bMax = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bRes = 0;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport,
                                   uint8_t itf,
                                   uint8_t ep_in,
                                   uint8_t cur_alt_setting)
{
    // Prepare the next audio data to be sent
    // Return true if data is ready, false otherwise

    (void)rhport;
    (void)itf;
    (void)ep_in;
    (void)cur_alt_setting;

    tud_audio_write((uint8_t*)test_buffer_audio, CFG_TUD_AUDIO_EP_SZ_IN - 2);

    return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t  rhport,
                                    uint16_t n_bytes_copied,
                                    uint8_t  itf,
                                    uint8_t  ep_in,
                                    uint8_t  cur_alt_setting)
{
    // Called after audio data has been copied to the USB buffer
    // Return true if more data is available, false otherwise

    (void)rhport;
    (void)n_bytes_copied;
    (void)itf;
    (void)ep_in;
    (void)cur_alt_setting;

    for(size_t cnt = 0; cnt < (CFG_TUD_AUDIO_EP_SZ_IN - 2) / 2; cnt++)
    {
        test_buffer_audio[cnt] = startVal++;
    }

    return true;
}

bool tud_audio_get_req_entity_cb(uint8_t                       rhport,
                                 tusb_control_request_t const* p_request)
{
    (void)rhport;

    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    // Input terminal (Microphone input)
    if(entityID == 1)
    {
        switch(ctrlSel)
        {
            case AUDIO_TE_CTRL_CONNECTOR:
            {
                // The terminal connector control only has a get request with only the CUR attribute.
                audio_desc_channel_cluster_t ret;

                // Those are dummy values for now
                ret.bNrChannels     = 1;
                ret.bmChannelConfig = (audio_channel_config_t)0;
                ret.iChannelNames   = 0;

                TU_LOG2("    Get terminal connector\r\n");

                return tud_audio_buffer_and_schedule_control_xfer(
                    rhport, p_request, (void*)&ret, sizeof(ret));
            }
            break;

                // Unknown/Unsupported control selector
            default: TU_BREAKPOINT(); return false;
        }
    }

    // Feature unit
    if(entityID == 2)
    {
        switch(ctrlSel)
        {
            case AUDIO_FU_CTRL_MUTE:
                // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
                // There does not exist a range parameter block for mute
                TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
                return tud_control_xfer(
                    rhport, p_request, &mute[channelNum], 1);

            case AUDIO_FU_CTRL_VOLUME:
                switch(p_request->bRequest)
                {
                    case AUDIO_CS_REQ_CUR:
                        TU_LOG2("    Get Volume of channel: %u\r\n",
                                channelNum);
                        return tud_control_xfer(rhport,
                                                p_request,
                                                &volume[channelNum],
                                                sizeof(volume[channelNum]));

                    case AUDIO_CS_REQ_RANGE:
                        TU_LOG2("    Get Volume range of channel: %u\r\n",
                                channelNum);

                        // Copy values - only for testing - better is version below
                        audio_control_range_2_n_t(1) ret;

                        ret.wNumSubRanges    = 1;
                        ret.subrange[0].bMin = -90; // -90 dB
                        ret.subrange[0].bMax = 90;  // +90 dB
                        ret.subrange[0].bRes = 1;   // 1 dB steps

                        return tud_audio_buffer_and_schedule_control_xfer(
                            rhport, p_request, (void*)&ret, sizeof(ret));

                        // Unknown/Unsupported control
                    default: TU_BREAKPOINT(); return false;
                }
                break;

                // Unknown/Unsupported control
            default: TU_BREAKPOINT(); return false;
        }
    }

    // Clock Source unit
    if(entityID == 4)
    {
        switch(ctrlSel)
        {
            case AUDIO_CS_CTRL_SAM_FREQ:
                // channelNum is always zero in this case
                switch(p_request->bRequest)
                {
                    case AUDIO_CS_REQ_CUR:
                        TU_LOG2("    Get Sample Freq.\r\n");
                        return tud_control_xfer(
                            rhport, p_request, &sampFreq, sizeof(sampFreq));

                    case AUDIO_CS_REQ_RANGE:
                        TU_LOG2("    Get Sample Freq. range\r\n");
                        return tud_control_xfer(rhport,
                                                p_request,
                                                &sampleFreqRng,
                                                sizeof(sampleFreqRng));

                        // Unknown/Unsupported control
                    default: TU_BREAKPOINT(); return false;
                }
                break;

            case AUDIO_CS_CTRL_CLK_VALID:
                // Only cur attribute exists for this request
                TU_LOG2("    Get Sample Freq. valid\r\n");
                return tud_control_xfer(
                    rhport, p_request, &clkValid, sizeof(clkValid));

            // Unknown/Unsupported control
            default: TU_BREAKPOINT(); return false;
        }
    }

    TU_LOG2("  Unsupported entity: %d\r\n", entityID);
    return false; // Yet not implemented
}

// bool tud_audio_set_itf_cb(uint8_t rhport, uint8_t itf, uint8_t alt_setting)
// {
//     // Called when the host sets the interface
//     // Return true if the setting is supported, false otherwise
// }

bool tud_audio_set_itf_close_EP_cb(uint8_t                       rhport,
                                   tusb_control_request_t const* p_request)
{
    (void)rhport;
    (void)p_request;
    startVal = 0;

    return true;
}


void UsbHandle::AudioTask()
{
    tud_audio__init();
    // Check if device is ready and connected
    if(tud_audio_mounted())
    {
        // Process audio data
        // For example, if you're implementing a microphone:
        int16_t sample = INT16_MAX;
        tud_audio_write(&sample, sizeof(sample));
    }
}

uint16_t count = 0;

void UsbHandle::CdcTask()
{
    char buff[64];
    for(auto i = 0; i < 2; i++)
    {
        sprintf(buff, "[%d] tick: %d\n", i, count);
        auto size = strlen(buff);
        tud_cdc_n_write(i, buff, size);
        tud_cdc_n_write_flush(i);
    }
    count++;
}


static void DeinitHS()
{
    if(USBD_DeInit(&hUsbDeviceHS) != USBD_OK)
    {
        UsbErrorHandler();
    }
}


void UsbHandle::Init(UsbPeriph dev)
{
    switch(dev)
    {
        case FS_INTERNAL: InitFS(); break;
        case FS_EXTERNAL: InitHS(); break;
        case FS_BOTH:
            InitHS();
            InitFS();
            break;
        default: break;
    }
    // Enable USB Regulator
    HAL_PWREx_EnableUSBVoltageDetector();
}

void UsbHandle::DeInit(UsbPeriph dev)
{
    switch(dev)
    {
        case FS_INTERNAL: DeinitFS(); break;
        case FS_EXTERNAL: DeinitHS(); break;
        case FS_BOTH:
            DeinitHS();
            DeinitFS();
            break;
        default: break;
    }
    // Enable USB Regulator
    HAL_PWREx_DisableUSBVoltageDetector();
}

UsbHandle::Result UsbHandle::TransmitInternal(uint8_t* buff, size_t size)
{
    auto ret = tud_cdc_write(buff, size) == size ? Result::OK : Result::ERR;
    tud_cdc_write_flush();
    return ret;
}
UsbHandle::Result UsbHandle::TransmitExternal(uint8_t* buff, size_t size)
{
    auto ret = tud_cdc_write(buff, size) == size ? Result::OK : Result::ERR;
    tud_cdc_write_flush();
    return ret;
}

void UsbHandle::SetReceiveCallback(ReceiveCallback cb, UsbPeriph dev)
{
    // This is pretty silly, but we're working iteritavely...
    rx_callback = cb;
    rxcallback  = (CDC_ReceiveCallback)rx_callback;

    switch(dev)
    {
        case FS_INTERNAL: CDC_Set_Rx_Callback_FS(rxcallback); break;
        case FS_EXTERNAL: CDC_Set_Rx_Callback_HS(rxcallback); break;
        case FS_BOTH:
            CDC_Set_Rx_Callback_FS(rxcallback);
            CDC_Set_Rx_Callback_HS(rxcallback);
            break;
        default: break;
    }
}

// Static Function Implementation
static void UsbErrorHandler()
{
    while(1) {}
}

// IRQ Handler
extern "C"
{
    // Shared USB IRQ Handlers for USB HS peripheral are located in sys/System.cpp

    void OTG_FS_EP1_OUT_IRQHandler(void)
    {
        HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    }

    void OTG_FS_EP1_IN_IRQHandler(void)
    {
        HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    }

    void OTG_FS_IRQHandler(void)
    {
        tud_int_handler(BOARD_TUD_RHPORT);
    }
}
