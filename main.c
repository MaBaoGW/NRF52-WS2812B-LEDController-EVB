/* Copyright (c) 2017 MaBaoGW. All Rights Reserved.
 * source link https://github.com/MaBaoGW
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "ble_flash.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_i2s.h"
#include "ws2812b_drive.h"//spi
//#include "i2s_ws2812b_drive.h"//i2s


#include "config.h"

#define led1_on nrf_gpio_pin_clear(LED_1)
#define led2_on nrf_gpio_pin_clear(LED_2)
#define led1_off nrf_gpio_pin_set(LED_1)
#define led2_off nrf_gpio_pin_set(LED_2)
#define led1_toggle nrf_gpio_pin_toggle(LED_1)
#define led2_toggle nrf_gpio_pin_toggle(LED_2)

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "MaBaoGW"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */


static uint32_t m_rgb_color=0;
bool sw_flag;
rgb_led_t led_array[NUM_LEDS];
rgb_led_t led_array1[NUM_LEDS];
rgb_led_t led_array2[NUM_LEDS];
uint8_t rxw_buff[10];
uint8_t testflag;
uint8_t have_changed=0;
uint8_t edit_direction=0;
static uint8_t direction_type=-1;

#define WBLE_PAGE_NUM_0 (NRF_FICR->CODESIZE-4)
#define WBLE_PAGE_NUM_1 (NRF_FICR->CODESIZE-5)

uint8_t word_count = 20;
static uint32_t led_num = 300;
uint32_t led_length1 = 0;
uint32_t led_length2 = 0;
uint32_t led_length3 = 0;
uint32_t led_length4 = 0;
uint32_t effect;
uint32_t led_time = 5;
uint32_t led_position = 0;

uint32_t my_red;
uint32_t my_green;
uint32_t my_blue;
uint8_t pre_position=0;
uint8_t pre_color=0;

uint8_t led_setting[20]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
															};

uint8_t led_default[20]={ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
															};												

uint8_t led_buffer_1[20]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
															};
						 
//rainbow
//static rgb_led_t *led_array_base;				// array for base color
//static rgb_led_t *led_array_work; 				// array for flash left-up to right-down

const static rgb_led_t color_list[] = {
	{0           , MAX_BRIGHTNESS, 0           }, // Red
	{MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0           }, // Yellow
	{MAX_BRIGHTNESS, 0           , 0           }, // Green
	{MAX_BRIGHTNESS, 0           , MAX_BRIGHTNESS}, // rght green
	{0           , 0           , MAX_BRIGHTNESS}, // Blue
	{0           , MAX_BRIGHTNESS, MAX_BRIGHTNESS}, // Purple
	{0           , MAX_BRIGHTNESS, 0           }, // Red
};

const static uint8_t n_color_list = sizeof(color_list)/sizeof(color_list[0]);
uint16_t timeout_count;

//scan
static ble_gap_scan_params_t m_scan_param;

static void second_counter_timeout_handler(void * p_context)
{
	uint32_t err_code;
	UNUSED_PARAMETER(p_context);
	led1_toggle;
	
}
APP_TIMER_DEF(m_led_contact_timer_id);
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_led_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                second_counter_timeout_handler);
    APP_ERROR_CHECK(err_code);

}
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_led_contact_timer_id, APP_TIMER_TICKS(500, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	uint8_t tmpx;
	//printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",p_data[0],p_data[1],p_data[2],p_data[3],p_data[4],p_data[5],p_data[6],p_data[7],p_data[8],p_data[9],p_data[10],p_data[11],p_data[12],p_data[13]);
	
	if( (p_data[0])==0xc1)
	{
		testflag=1;
		//printf("0xc1 =%d =%d =%d =%d =%d =%d =%d =%d =%d =%d =%d\n",p_data[1],p_data[2],p_data[3],p_data[4],p_data[5],p_data[6],p_data[7],p_data[8],p_data[9],p_data[10],p_data[11]);
		for(tmpx=0;tmpx<10;tmpx++)
			rxw_buff[p_data[1]*10+tmpx]=p_data[2+tmpx];
	}
	else if((p_data[0])==0xb1)
	{
		//ledb_on();
		//printf("0xb1 led_length1=%d led_length2=%d led_length3=%d led_length4=%d effect =%d led_time =%d led_position =%d\n",p_data[1],p_data[2],p_data[3],p_data[4],p_data[5],p_data[6],p_data[7]);
		
		testflag=1;
		have_changed=1;
		led_length1=p_data[1];
		led_length2=p_data[2];
		led_length3=p_data[3];
		led_length4=p_data[4];
		led_num = led_length1 + led_length2 + led_length3 + led_length4;
		effect=p_data[5];
		led_time=p_data[6];
		led_position=p_data[7];
	}
	else if((p_data[0])==0xd1)
	{
		//printf("0xd1 received\n");
		have_changed=0;
		my_red = rxw_buff[0];
		my_green = rxw_buff[1];
		my_blue = rxw_buff[2];
		testflag=2;
		//ledb_off();
	}
	else if( (p_data[0])==0xe1){
		//printf("0xe1 received - flash");
		testflag=3;
	}
	else if((p_data[0])==0xa1 && (p_data[1])==0x1a){
		//printf("0xa1 0x1a received - clear\n");
		testflag=4;
		have_changed=1;
		led_length1=p_data[2];
		led_length2=p_data[3];
		led_length3=p_data[4];
		led_length4=p_data[5];
		led_num = led_length1 + led_length2 + led_length3 + led_length4;
	}
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
	{
    uint32_t err_code;
	uint8_t tmpx;
	const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;
		
    switch (p_ble_evt->header.evt_id)
    {
		case BLE_GAP_EVT_ADV_REPORT:
            break;
		
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    //ble_advertising_on_ble_evt(p_ble_evt);
    //bsp_btn_ble_on_ble_evt(p_ble_evt);

	if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
        ble_advertising_start(BLE_ADV_MODE_FAST);
    else
        ble_advertising_on_ble_evt(p_ble_evt);
		bsp_btn_ble_on_ble_evt(p_ble_evt);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */

void ws2812_crgb(uint16_t lnum,uint8_t rrr,uint8_t ggg,uint8_t bbb)
{
		led_array[lnum].blue=bbb;
		led_array[lnum].green=ggg;
		led_array[lnum].red=rrr;
}

void ws2812_write()
{
	if (i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, STDOUT) != NRF_SUCCESS) {
		//printf("ERROR: Send to LED failed!\n");
    }
}

void ws2812_init()
{
    ws2812b_drive_set_blank(led_array,NUM_LEDS);
    i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, STDOUT);
}

void ws2812_default(uint16_t delayt)
{
	uint16_t tmpx;
	for(tmpx=0;tmpx<NUM_LEDS&&have_changed==0;tmpx++)
	{
		led_array[tmpx].blue=0x00;
		led_array[tmpx].green=0x00;
		led_array[tmpx].red=0x00;
	}
	ws2812_write();
	nrf_delay_ms(delayt);
}

void ws2812_read_flash_correct(uint16_t delayt)
{
	uint16_t tmpx;
	for(tmpx=0;tmpx<5&&have_changed==0;tmpx++)
	{
		led_array[tmpx].blue=0x00;
		led_array[tmpx].green=0x64;
		led_array[tmpx].red=0x00;
	}
	ws2812_write();
	nrf_delay_ms(delayt);
}

void ws2812_read_flash_wrong(uint16_t delayt)
{
	uint16_t tmpx;
	for(tmpx=0;tmpx<5&&have_changed==0;tmpx++)
	{
		led_array[tmpx].blue=0x00;
		led_array[tmpx].green=0x00;
		led_array[tmpx].red=0x64;
	}
	ws2812_write();
	nrf_delay_ms(delayt);
}

void ws2812_single_position(uint16_t delayt,uint8_t rrr,uint8_t ggg,uint8_t bbb,uint32_t position)
{
	uint16_t tmpx;

	for(tmpx=0;tmpx<led_num&&have_changed==0;tmpx++)
	{
		if(tmpx==position){
			led_array[tmpx].blue=bbb;
			led_array[tmpx].green=ggg;
			led_array[tmpx].red=rrr;
		}
		else{
			led_array[tmpx].blue=0x00;
			led_array[tmpx].green=0x00;
			led_array[tmpx].red=0x00;
		}
	}
	ws2812_write();
	nrf_delay_ms(delayt);
}

void ws2812_single_color(uint16_t delayt,uint8_t rrr,uint8_t ggg,uint8_t bbb)
{
	uint16_t tmpx;

	for(tmpx=0;tmpx<led_num&&have_changed==0;tmpx++)
	{
		led_array[tmpx].blue=bbb;
		led_array[tmpx].green=ggg;
		led_array[tmpx].red=rrr;
	}
	ws2812_write();
	nrf_delay_ms(delayt);
}

void ws2812_star(uint8_t rrr,uint8_t ggg,uint8_t bbb,uint16_t delayt)
{
	uint16_t tmpx,position=0;
	uint16_t tmpr=0,tmpg=0,tmpb=0;
	
	//printf("\r\nws2812_star %d,%d,%d\r\n",rrr,ggg,bbb);
	
	for(position=0;position<led_num&&have_changed==0;position++){
		for(tmpx=0;tmpx<led_num&&have_changed==0;tmpx++){
			ws2812_crgb(tmpx,tmpr,tmpg,tmpb);
			
			if(tmpx==position){
				ws2812_crgb((position)%led_num,rrr-((rrr/7)*0),ggg-((ggg/7)*0),bbb-((bbb/7)*0));
			}
			else if(tmpx==position-1){
				ws2812_crgb((position-1)%led_num,rrr-((rrr/7)*1),ggg-((ggg/7)*1),bbb-((bbb/7)*1));
			}
			else if(tmpx==position-2){
				ws2812_crgb((position-2)%led_num,rrr-((rrr/7)*2),ggg-((ggg/7)*2),bbb-((bbb/7)*2));
			}
			else if(tmpx==position-3){
				ws2812_crgb((position-3)%led_num,rrr-((rrr/7)*3),ggg-((ggg/7)*3),bbb-((bbb/7)*3));
			}
			else if(tmpx==position-4){
				ws2812_crgb((position-4)%led_num,rrr-((rrr/7)*4),ggg-((ggg/7)*4),bbb-((bbb/7)*4));
			}
			else if(tmpx==position-5){
				ws2812_crgb((position-5)%led_num,rrr-((rrr/7)*5),ggg-((ggg/7)*5),bbb-((bbb/7)*5));
			}
			else if(tmpx==position-6){
				ws2812_crgb((position-6)%led_num,rrr-((rrr/7)*6),ggg-((ggg/7)*6),bbb-((bbb/7)*6));
			}
			else if(tmpx==position-7){
				ws2812_crgb((position-7)%led_num,rrr-((rrr/7)*7),ggg-((ggg/7)*7),bbb-((bbb/7)*7));
			}
		}
			ws2812_write();	
			nrf_delay_ms(delayt);
	}
	
		for(position=led_num;position>0&&have_changed==0;position--){
		for(tmpx=0;tmpx<led_num&&have_changed==0;tmpx++){
			ws2812_crgb(tmpx,tmpr,tmpg,tmpb);
			
			if(tmpx==position){
				ws2812_crgb((position)%led_num,rrr-((rrr/7)*0),ggg-((ggg/7)*0),bbb-((bbb/7)*0));
			}
			else if(tmpx==position+1){
				ws2812_crgb((position+1)%led_num,rrr-((rrr/7)*1),ggg-((ggg/7)*1),bbb-((bbb/7)*1));
			}
			else if(tmpx==position+2){
				ws2812_crgb((position+2)%led_num,rrr-((rrr/7)*2),ggg-((ggg/7)*2),bbb-((bbb/7)*2));
			}
			else if(tmpx==position+3){
				ws2812_crgb((position+3)%led_num,rrr-((rrr/7)*3),ggg-((ggg/7)*3),bbb-((bbb/7)*3));
			}
			else if(tmpx==position+4){
				ws2812_crgb((position+4)%led_num,rrr-((rrr/7)*4),ggg-((ggg/7)*4),bbb-((bbb/7)*4));
			}
			else if(tmpx==position+5){
				ws2812_crgb((position+5)%led_num,rrr-((rrr/7)*5),ggg-((ggg/7)*5),bbb-((bbb/7)*5));
			}
			else if(tmpx==position+6){
				ws2812_crgb((position+6)%led_num,rrr-((rrr/7)*6),ggg-((ggg/7)*6),bbb-((bbb/7)*6));
			}
			else if(tmpx==position+7){
				ws2812_crgb((position+7)%led_num,rrr-((rrr/7)*7),ggg-((ggg/7)*7),bbb-((bbb/7)*7));
			}
		}
			ws2812_write();	
			nrf_delay_ms(delayt);
	}
}

void my_rainbow_init(uint16_t num_leds)
{
	/* Initialize the LED colors based upon the color list
	 * Originated from running_rainbow.c (c) Takafumi Naka, 2016
	 */
	uint32_t iregion;
	float ratio;

	// allocate buffers
	//led_array_base		= malloc(num_leds * sizeof(rgb_led_t));
	//led_array_work		= malloc(num_leds * sizeof(rgb_led_t));

	// initialize led_array (base color array)
	for(uint16_t i=0;i<led_num&&have_changed==0;i++)
	{

		iregion = (n_color_list-1)*i/led_num;
		ratio = (i - iregion*(float)led_num/(n_color_list-1))/((float)led_num/(n_color_list-1));
		led_array1[i].green = color_list[iregion].green*(1-ratio) + color_list[iregion+1].green * ratio;
		led_array1[i].red   = color_list[iregion].red*(1-ratio)   + color_list[iregion+1].red   * ratio;
		led_array1[i].blue  = color_list[iregion].blue*(1-ratio)  + color_list[iregion+1].blue  * ratio;
	}
}

void my_rainbow_uninit()
{
		//free(led_array_base);
		//free(led_array_work);
}

void my_rainbow(rgb_led_t * led_array_out)
{	
	// update led_array_base
	{
		for(uint16_t i=0;i<led_num&&have_changed==0;i++)
		{
			led_array2[i] = led_array1[i];
		}
	}
	// Update led color
	{
		uint16_t i;
		rgb_led_t tmp;

		for( i=0;i<led_num-1&&have_changed==0;i++)
		{
			if (i==0) {
				tmp=led_array2[i];
			}
			led_array2[i]=led_array2[i+1];				
		}	
		led_array2[i] = tmp;	
	}
	// Update output array
	{
		for(uint16_t i=0;i<led_num&&have_changed==0;i++)
		{
			led_array_out[i] = led_array2[i];
			// Also update the base color table
			led_array1[i]=led_array2[i];
		}	
	}
}

void do_my_rainbow(){
	    //rgb_led_t led_array[led_num];	
//    uint32_t current_limit;

//    LEDS_INVERT(GOODLED);
    
    my_rainbow_init(led_num);
    /*
    //int32_t time_left = DEMO_PERIOD;
    //int32_t step_time = CYCLE_TIME;
    while (have_changed==0&&testflag!=3)
    {
//        LEDS_INVERT(GOODLED);   // Beat the heart

        // work out the LED colors for each cycle 
        my_rainbow(led_array);

        // Adjust total current used by LEDs to below the max allowed 
//        current_limit = MAX_TOTAL_CURRENT;
//        ws2812b_drive_current_cap(led_array, NUM_LEDS, current_limit);

        if (i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, STDOUT) != NRF_SUCCESS) {
//            LEDS_ON(BADLED);    // Lit error indicator up
//            SEGGER_RTT_WriteString(0,"ERROR: Send to LED failed!\n");
        } else {
//            LEDS_OFF(BADLED);   // Reset bad status indicator
        }

        nrf_delay_ms(CYCLE_TIME);
        //time_left -= step_time;
    }
	*/
	
	while (have_changed==0&&testflag!=3)
	{
		my_rainbow(led_array);
		ws2812_write();
		nrf_delay_ms(20);
	}
	
    //my_rainbow_uninit();
    
	//ws2812_default_color(1000,0,16,0);
    // Turn off LEDs
//    ws2812b_drive_set_blank(led_array,NUM_LEDS);
//    i2s_ws2812b_drive_xfer(led_array, NUM_LEDS, STDOUT);
//    LEDS_ON(BLANKLED);
//    nrf_delay_ms(2000);
//    LEDS_OFF(BLANKLED);

}

void ReadFlashSettings(){
	
	if(ble_flash_page_read(WBLE_PAGE_NUM_0,(uint32_t*) & led_setting,&word_count )==NRF_SUCCESS)
	{
			//printf("led_length1 = %d led_length2 = %d led_length3 = %d led_length4 = %d effect = %d led_time = %d\n",led_setting[0],led_setting[1],led_setting[2],led_setting[3],led_setting[4],led_setting[5]);
			led_length1 = led_setting[0];
			led_length2 = led_setting[1];
			led_length3 = led_setting[2];
			led_length4 = led_setting[3];
			led_num = led_length1 + led_length2 + led_length3 + led_length4;
			effect = led_setting[4];
			led_time = led_setting[5];
			led_position = led_setting[6];
		    testflag = 0 ;
	}	
	else		
	{
		//printf("flash 1 read err!\n");
		testflag = 101;
	}
	
}

void ReadFlashBuffers(){
	
	int tmpx;
	
	//printf("******** Buffer 1\n\n");
	if(ble_flash_page_read(WBLE_PAGE_NUM_1,(uint32_t*) & led_buffer_1,&word_count )==NRF_SUCCESS)
	{/*
			for(tmpx=0;tmpx<484;tmpx++)
			{
				//if(tmpx%44==0)nrf_delay_ms(5);
				printf("[%d]%X ",tmpx,led_buffer_1[tmpx]);
				nrf_delay_ms(2);
			}
			printf("\n\n");
		*/
	}	
	else		
	{
		//printf("flash 1 read err!\n");
		testflag = 101;
	}
	
}

void gpio_init()
{
	uint8_t tmpx;
	//nrf_gpio_cfg_input(2,GPIO_PIN_CNF_PULL_Pullup);
	nrf_gpio_cfg_output(LED_1);
	
	led1_on;
	for(tmpx=0;tmpx<6;tmpx++)
	{
		led1_toggle;
		nrf_delay_ms(50);
	}
	for(tmpx=0;tmpx<6;tmpx++)
	{
		led1_toggle;
		nrf_delay_ms(100);
	}

	/*
	sw_flag=nrf_gpio_pin_read(2);
	if(sw_flag)led1_on;
	else led1_off;
	*/
}
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
	uint16_t tmpx=0;
	effect=0;
	testflag=101;
	gpio_init();
    // Initialize.
	timers_init();
	
	//uart_init();
	//printf("\r\nUART init!\r\n");
	
	/*
	if(ble_flash_page_write(WBLE_PAGE_NUM_0,(uint32_t*) & led_default,word_count)==NRF_SUCCESS)	//printf("write sucess!\n");
	if(ble_flash_page_write(WBLE_PAGE_NUM_1,(uint32_t*) & led_default,word_count)==NRF_SUCCESS)	//printf("write sucess!\n");
	*/

	ReadFlashSettings();
	ReadFlashBuffers();
	
	//led_array_base	= malloc(NUM_LEDS * sizeof(rgb_led_t));
	//led_array_work	= malloc(NUM_LEDS * sizeof(rgb_led_t));
	
    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);
	
	ws2812_init();
	application_timers_start();

    // Enter main loop.
    while(1)
    {
			if(testflag==0){
					if(effect==0){//flash read correct
						ws2812_read_flash_correct(50);
					}
					else if(effect==1){
						ws2812_single_position(1000,led_buffer_1[0],led_buffer_1[1],led_buffer_1[2],led_position);
					}
					else if(effect==2){
						ws2812_single_color(1000,led_buffer_1[0],led_buffer_1[1],led_buffer_1[2]);
					}
					else if(effect==3){
						ws2812_star(led_buffer_1[0],led_buffer_1[1],led_buffer_1[2],led_time*10);
					}
					else if(effect==4){
						do_my_rainbow();
					}
			}

			else if(testflag==2){
				//printf("buff\n");
				
				//ws2812_init();
				led_setting[0]=led_length1;
				led_setting[1]=led_length2;
				led_setting[2]=led_length3;
				led_setting[3]=led_length4;
				led_setting[4]=effect;
				led_setting[5]=led_time;
				led_setting[6]=led_position;
				led_num = led_length1 + led_length2 + led_length3 + led_length4;
				
				led_buffer_1[0] = my_red;
				led_buffer_1[1] = my_green;
				led_buffer_1[2] = my_blue;
				
				//listBuffers();
				testflag=0;
			}//if(testflag==2)
			
			else if(testflag==3){
				//printf("flash\n");
				
				sd_softdevice_disable();
				
				if(ble_flash_page_write(WBLE_PAGE_NUM_0,(uint32_t*) & led_setting,word_count)==NRF_SUCCESS)//printf("write sucess!\n");
				if(ble_flash_page_write(WBLE_PAGE_NUM_1,(uint32_t*) & led_buffer_1,word_count)==NRF_SUCCESS)//printf("write sucess!\n");
				testflag=0;
				
				NVIC_SystemReset();
			}//if(testflag==3)

			else if(testflag==4){
				have_changed=0;
				ws2812_default(1000);
			}
			else if(testflag==101){//flash read wrong
				ws2812_read_flash_wrong(50);
			}
    }//while

}


/**
 * @}
 */
