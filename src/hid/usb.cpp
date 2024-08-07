#include "hid/usb.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "tusb.h"

#include "system.h"

#include "hid/logger.h"


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

    void DummyRxCallback(uint8_t *buf, uint32_t *size)
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


using DBG = daisy::Logger<daisy::LOGGER_INTERNAL>;

// const uint32_t sample_rates[] = {48000};
// const uint32_t sample_rates[]      = {44100, 48000};
static int32_t current_sample_rate = 48000;

// #define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)

audio_control_range_4_n_t(1) rangef = {
    .wNumSubRanges = 1,
    {
        {.bMin = current_sample_rate, .bMax = current_sample_rate, .bRes = 0},
    }};


// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // +1 for master channel 0
int16_t
    volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // +1 for master channel 0


// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t                        rhport,
                                        audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if(request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
    {
        if(request->bRequest == AUDIO_CS_REQ_CUR)
        {
            DBG::Print("Clock get current freq %" PRIu32 "\r\n",
                       current_sample_rate);

            audio_control_cur_4_t curf
                = {(int32_t)tu_htole32(current_sample_rate)};
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport,
                (tusb_control_request_t const *)request,
                &curf,
                sizeof(curf));
        }
        else if(request->bRequest == AUDIO_CS_REQ_RANGE)
        {
            // audio_control_range_4_n_t(N_SAMPLE_RATES) rangef
            //     = {.wNumSubRanges = tu_htole16(N_SAMPLE_RATES)};
            // DBG::Print("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            // for(uint8_t i = 0; i < 1; i++)
            // {
            //     rangef.subrange[i].bMin = (int32_t)sample_rates[i];
            //     rangef.subrange[i].bMax = (int32_t)sample_rates[i];
            //     rangef.subrange[i].bRes = 0;
            DBG::Print("Range (%d, %d, %d)\r\n",
                       rangef.subrange[0].bMin,
                       rangef.subrange[0].bMax,
                       rangef.subrange[0].bRes);
            // }

            return tud_audio_buffer_and_schedule_control_xfer(
                rhport,
                (tusb_control_request_t const *)request,
                &rangef,
                sizeof(rangef));
        }
    }
    else if(request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID
            && request->bRequest == AUDIO_CS_REQ_CUR)
    {
        audio_control_cur_1_t cur_valid = {.bCur = 1};
        DBG::Print("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport,
            (tusb_control_request_t const *)request,
            &cur_valid,
            sizeof(cur_valid));
    }
    DBG::Print(
        "Clock get request not supported, entity = %u, selector = %u, request "
        "= %u\r\n",
        request->bEntityID,
        request->bControlSelector,
        request->bRequest);
    return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t                        rhport,
                                        audio_control_request_t const *request,
                                        uint8_t const                 *buf)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if(request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
    {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

        current_sample_rate
            = (uint32_t)((audio_control_cur_4_t const *)buf)->bCur;

        DBG::Print("Clock set current freq: %" PRIu32 "\r\n",
                   current_sample_rate);

        return true;
    }
    else
    {
        DBG::Print(
            "Clock set request not supported, entity = %u, selector = %u, "
            "request = %u\r\n",
            request->bEntityID,
            request->bControlSelector,
            request->bRequest);
        return false;
    }
}

// Helper for feature unit get requests
static bool
tud_audio_feature_unit_get_request(uint8_t                        rhport,
                                   audio_control_request_t const *request)
{
    return false;
}

// Helper for feature unit set requests
static bool
tud_audio_feature_unit_set_request(uint8_t                        rhport,
                                   audio_control_request_t const *request,
                                   uint8_t const                 *buf)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if(request->bControlSelector == AUDIO_FU_CTRL_MUTE)
    {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

        mute[request->bChannelNumber]
            = ((audio_control_cur_1_t const *)buf)->bCur;

        DBG::Print("Set channel %d Mute: %d\r\n",
                   request->bChannelNumber,
                   mute[request->bChannelNumber]);

        return true;
    }
    else if(request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
    {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

        volume[request->bChannelNumber]
            = ((audio_control_cur_2_t const *)buf)->bCur;

        DBG::Print("Set channel %d volume: %d dB\r\n",
                   request->bChannelNumber,
                   volume[request->bChannelNumber] / 256);

        return true;
    }
    else
    {
        DBG::Print(
            "Feature unit set request not supported, entity = %u, selector = "
            "%u, request = %u\r\n",
            request->bEntityID,
            request->bControlSelector,
            request->bRequest);
        return false;
    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t                       rhport,
                                 tusb_control_request_t const *p_request)
{
    audio_control_request_t const *request
        = (audio_control_request_t const *)p_request;

    if(request->bEntityID == UAC2_ENTITY_CLOCK)
        return tud_audio_clock_get_request(rhport, request);
    if(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
        return tud_audio_feature_unit_get_request(rhport, request);
    else
    {
        DBG::Print(
            "Get request not handled, entity = %d, selector = %d, request = "
            "%d\r\n",
            request->bEntityID,
            request->bControlSelector,
            request->bRequest);
    }
    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t                       rhport,
                                 tusb_control_request_t const *p_request,
                                 uint8_t                      *buf)
{
    audio_control_request_t const *request
        = (audio_control_request_t const *)p_request;

    if(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    if(request->bEntityID == UAC2_ENTITY_CLOCK)
        return tud_audio_clock_set_request(rhport, request, buf);
    DBG::Print(
        "Set request not handled, entity = %d, selector = %d, request = %d\r\n",
        request->bEntityID,
        request->bControlSelector,
        request->bRequest);

    return false;
}

void UsbHandle::AudioTask() {}

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

UsbHandle::Result UsbHandle::TransmitInternal(uint8_t *buff, size_t size)
{
    auto ret = tud_cdc_write(buff, size) == size ? Result::OK : Result::ERR;
    tud_cdc_write_flush();
    return ret;
}
UsbHandle::Result UsbHandle::TransmitExternal(uint8_t *buff, size_t size)
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
