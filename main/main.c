#include <stdio.h>
#include <time.h>
#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mavlink/common/mavlink.h"
#include "pioneer_sdk/pioneer_esp32.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "lwip/sockets.h"


#define TAG                       "PIONEER SDK"
#define PIONEER_IP                "192.168.4.1"
#define PIONEER_PORT              8001
#define PIONEER_PASS              "12345678"
#define PIONEER_SSID              "PioneerMiniac67b21ff0a8"
#define WIFI_FAIL_BIT             BIT1
#define WIFI_CONNECTED_BIT        BIT0
#define EXAMPLE_ESP_MAXIMUM_RETRY 15

int sock_fd;
static int s_retry_num = 0;
struct sockaddr_in pioneer_addr;
Pioneer mini = {PIONEER_PORT, 1, PIONEER_IP};
static EventGroupHandle_t s_wifi_event_group;

void arm_pioneer();
void land_pioneer();
void disarm_pioneer();
void takeoff_pioneer();
void wifi_init_sta(void);
void send_heartbeats(void * argv);
void send_mavlink_message(mavlink_message_t * message);
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void send_long_command(uint8_t target_system,
                       uint8_t target_component,
                       uint8_t confirmation,
                       uint16_t command,
                       float p1,
                       float p2,
                       float p3,
                       float p4,
                       float p5,
                       float p6,
                       float p7);

void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
    	ESP_LOGE(TAG, "socket() error");
    }
    else
    {
    	ESP_LOGI(TAG, "socket() success");
    }

    pioneer_addr.sin_family = AF_INET;
    pioneer_addr.sin_port = htons(mini.port);
    pioneer_addr.sin_addr.s_addr = inet_addr(mini.ip);

    xTaskCreate(send_heartbeats, "send_heartbeats", 65536, NULL, 5, NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

        ESP_LOGW(TAG,"connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = PIONEER_SSID,
            .password = PIONEER_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", PIONEER_SSID, PIONEER_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s, password:%s", PIONEER_SSID, PIONEER_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void send_heartbeats(void * argv)
{
	while (1)
	{
        static int arm = 0;
		mavlink_message_t message;
        const uint8_t system_id = 1;
        const uint8_t base_mode = 0;
        const uint8_t custom_mode = 0;

        mavlink_msg_heartbeat_pack_chan(
            system_id,
            0,
            MAVLINK_COMM_0,
            &message,
            MAV_TYPE_GENERIC,
            0,
            base_mode,
            custom_mode,
            MAV_STATE_STANDBY);

        send_mavlink_message(&message);

        if (arm == 5)
        {
            arm_pioneer(&mini);
        }
        if (arm == 10)
        {
            takeoff_pioneer();
        }
        if (arm == 20)
        {
            land_pioneer(&mini);
        }
        if (arm == 30)
        {
            disarm_pioneer();
        }

        arm++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void send_long_command(uint8_t target_system,
                       uint8_t target_component,
                       uint8_t confirmation,
                       uint16_t command,
                       float p1,
                       float p2,
                       float p3,
                       float p4,
                       float p5,
                       float p6,
                       float p7)
{
    mavlink_command_long_t command_long_to_send = {
        .param1 = p1,
        .param2 = p2,
        .param3 = p3,
        .param4 = p4,
        .param5 = p5,
        .param6 = p6,
        .param7 = p7,
        .command = command,
        .target_system = target_system,
        .target_component = target_component,
        .confirmation = confirmation
    };

    mavlink_message_t message_to_send;
    mavlink_msg_command_long_encode(1, 0, &message_to_send, &command_long_to_send);
    send_mavlink_message(&message_to_send);
}

void send_mavlink_message(mavlink_message_t * message)
{
    uint8_t buffer[2048];
    memset(buffer, 0, sizeof(buffer));

    const int len = mavlink_msg_to_send_buffer(buffer, message);
    int ret = sendto(sock_fd, buffer, len, 0, (struct sockaddr *)&pioneer_addr, sizeof(pioneer_addr));
    if (ret != len)
    {
        ESP_LOGW(TAG, "send_mavlink_message() error");
    }
    else
    {
        ESP_LOGI(TAG, "send_mavlink_message() success ret = %d", ret);
    }
}

void arm_pioneer()
{
    send_long_command(mini.system_id, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196, 0, 0, 0, 0, 0);
    ESP_LOGW(TAG, "arm()");
}

void disarm_pioneer()
{
    send_long_command(mini.system_id, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0);
    ESP_LOGW(TAG, "disarm()");
}

void takeoff_pioneer()
{
    send_long_command(mini.system_id, 0, 0, MAV_CMD_NAV_TAKEOFF_LOCAL, 0, 0, 0.5, 0, 0, 0, 0);
    ESP_LOGW(TAG, "takeoff_pioneer()");
}

void land_pioneer()
{
    send_long_command(mini.system_id, 0, 0, MAV_CMD_NAV_LAND_LOCAL, 0, 0, 0, 0, 0, 0, 0);
    ESP_LOGW(TAG, "land_pioneer()");
}