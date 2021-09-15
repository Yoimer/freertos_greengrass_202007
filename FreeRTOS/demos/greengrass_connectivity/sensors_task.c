#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <dht.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "aws_application_version.h"
#include "esp_log.h"
#include "iot_ble_config.h"
#include "driver/uart.h"
// #include "minmea.h"
#include "sensors.h"

char version[VERSION_STRING_LENGTH];
char string_latitude[LATITUDE_STRING_LENGTH];
char string_longitude[LONGITUDE_STRING_LENGTH];
extern char sensorsPayload[PAYLOAD_STRING_LENGTH];

void sensors_task(void *pvParameters)
{

    while (1)
    {
        /* dht11 reading */
        static const dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;
        static const gpio_num_t dht_gpio = 4;

        int16_t temperature = -2000;
        int16_t humidity = -2000;

        if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK) {
            printf("Humidity: %d%% Temp: %dC\n", humidity / 10, temperature / 10);
        }else {
            printf("Could not read data from dht11 sensor\n");
        }
        sprintf(sensorsPayload, "{\"T2\":%d,\"H\":%d}",temperature/10, humidity/10);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
} 