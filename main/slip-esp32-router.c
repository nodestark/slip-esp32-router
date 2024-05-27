#include <stdio.h>
#include <string.h>

#include <driver/uart.h>
#include <driver/gpio.h>

#include <nvs_flash.h>
#include "esp_private/wifi.h"

struct flow_wifi2eth_msg_t {
	void *packet, *eb;
	uint16_t length;
};

static QueueHandle_t net0_queue;

static int pkt_wifi2eth(void *buffer, uint16_t len, void *eb) {

	struct flow_wifi2eth_msg_t msg= { .length= len, .packet= buffer, .eb= eb};

	xQueueSend(net0_queue, &msg, portMAX_DELAY);

	return 0;
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {

	switch (event_id) {
	case WIFI_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case WIFI_EVENT_STA_CONNECTED:
		esp_wifi_internal_reg_rxcb(WIFI_IF_STA, pkt_wifi2eth);
		break;
	case WIFI_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		break;
	}
}

void vTaskCode(void *pvParameters) {

	struct flow_wifi2eth_msg_t msg;

	while (1) {

		if (xQueueReceive(net0_queue, &msg, portMAX_DELAY)) {

			char* test_str = "This is a test string.\n";
			uart_write_bytes(UART_NUM_0, (const char*) test_str, strlen(test_str));

			esp_wifi_internal_free_rx_buffer(msg.eb);
		}
	}
}

void app_main(void) {

	ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 256, 0, (void *) 0, 0) );
	ESP_ERROR_CHECK( uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );

	uart_config_t uart_config_0 = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

	ESP_ERROR_CHECK( uart_param_config(UART_NUM_0, &uart_config_0) );

	net0_queue = xQueueCreate(21 /*queue-length*/, sizeof(struct flow_wifi2eth_msg_t));

	ESP_ERROR_CHECK( esp_event_loop_create_default() );
	ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, (void*) 0, (void*) 0) );

	ESP_ERROR_CHECK( nvs_flash_init() );

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );

	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = "ALLREDE-SOARES-2G",
			.password = "julia2012",
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,
		},
	};
	ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

	ESP_ERROR_CHECK( esp_wifi_start() );

	xTaskCreate( vTaskCode, "cpu_loop", 6765, (void *) 0, 1, (void*) 0 );
}
