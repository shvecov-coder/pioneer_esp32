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
static EventGroupHandle_t s_wifi_event_group;

void arm_pioneer();
void wifi_init_sta(void);
void send_heartbeats(void * argv);
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void app_main(void)
{
	Pioneer mini = {PIONEER_PORT, PIONEER_IP};

	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    printf("ip: %s\nport: %d\n", mini.ip, mini.port);
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
    	ESP_LOGE(TAG, "socket() error");
    }
    else
    {
    	ESP_LOGI(TAG, "socket() success");
    }

    xTaskCreate(send_heartbeats, "send_heartbeats", 4096, NULL, 5, NULL);
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

        ESP_LOGI(TAG,"connect to the AP fail");
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

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

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
		struct sockaddr_in dest_addr;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PIONEER_PORT);
        dest_addr.sin_addr.s_addr = inet_addr(PIONEER_IP);

        mavlink_message_t message;
        const uint8_t system_id = 49;
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

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const int len = mavlink_msg_to_send_buffer(buffer, &message);

        int ret = sendto(sock_fd, buffer, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (ret != len) 
        {
            ESP_LOGE(TAG, "sendto error: %s\n", strerror(errno));
        }
        else
        {
            ESP_LOGW(TAG, "Sent heartbeat\n");
        }

        if (arm == 5)
        {
            arm_pioneer(&dest_addr, sizeof(dest_addr));
            arm = 0;
        }

        arm++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void arm_pioneer(struct sockaddr_in * dst_pioneer, size_t size_dst_pioneer)
{
    mavlink_command_long_t command_long_to_send = {
        .param1 = 1,
        .param2 = 21196,
        .param3 = 0,
        .param4 = 0,
        .param5 = 0,
        .param6 = 0,
        .param7 = 0,
        .command = MAV_CMD_COMPONENT_ARM_DISARM,
        .target_system = 0,
        .target_component = 0,
        .confirmation = 0
    };

    mavlink_message_t message_to_send;
    mavlink_msg_command_long_encode(42, 0, &message_to_send, &command_long_to_send);

    uint8_t buffer[2048] = {0};
    const int len = mavlink_msg_to_send_buffer(buffer, &message_to_send);

    int ret = sendto(sock_fd, buffer, len, 0, (struct sockaddr *)dst_pioneer, size_dst_pioneer);
    if (ret != len)
    {
        ESP_LOGE(TAG, "sendto() error");
    }
    else
    {
        ESP_LOGI(TAG, "ARM");
    }
}