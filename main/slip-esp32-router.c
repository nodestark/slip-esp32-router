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

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

struct slip_packet {
	uint8_t data[65535 * 2]; 	// Buffer for SLIP-encoded data (worst case scenario)
	uint16_t length;            // Length of SLIP-encoded data
};

void slip_encode(const uint8_t *packet, uint32_t packet_len, struct slip_packet *slip) {

	uint32_t j = 0;
	slip->data[j++] = SLIP_END; // Start with an END character

	for (uint32_t i = 0; i < packet_len; i++) {
		switch (packet[i]) {
		case SLIP_END:
			slip->data[j++] = SLIP_ESC;
			slip->data[j++] = SLIP_ESC_END;
			break;
		case SLIP_ESC:
			slip->data[j++] = SLIP_ESC;
			slip->data[j++] = SLIP_ESC_ESC;
			break;
		default:
			slip->data[j++] = packet[i];
		}
	}

	slip->data[j++] = SLIP_END; // End with an END character
	slip->length = j;
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

//	################################################################
	ESP_ERROR_CHECK( uart_driver_install(UART_NUM_1, 256, 256, 0, (void *) 0, 0) );
	ESP_ERROR_CHECK( uart_set_pin(UART_NUM_1, GPIO_NUM_27, GPIO_NUM_14, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );

	uart_config_t uart_config_1 = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

	ESP_ERROR_CHECK( uart_param_config(UART_NUM_1, &uart_config_1) );

//	################################################################
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

//	################################################################
	struct slip_packet _slip= {
			.length = 0
	};

	struct flow_wifi2eth_msg_t msg;
	while (1) {

		if (xQueueReceive(net0_queue, &msg, 0)) {

			struct slip_packet slip_;
		    slip_encode(msg.packet, msg.length, &slip_);

		    uart_write_bytes(UART_NUM_1, slip_.data, slip_.length);
		}

		uint8_t buff;
		if(uart_read_bytes(UART_NUM_1, &buff, sizeof(buff), 0)){

			if(buff == SLIP_END){

				esp_wifi_internal_tx(WIFI_IF_STA, &_slip.data, _slip.length);
				_slip.length = 0;

			} else if(buff == SLIP_ESC){

				if(uart_read_bytes(UART_NUM_1, &buff, sizeof(buff), 10 / portTICK_PERIOD_MS)){
					if(buff == SLIP_ESC_END){
						_slip.data[ _slip.length++ ] = SLIP_END;
					} else if(buff == SLIP_ESC_ESC){
						_slip.data[ _slip.length++ ] = SLIP_ESC;
					}
				}

			} else _slip.data[ _slip.length++ ] = buff;
		}
	}
}
