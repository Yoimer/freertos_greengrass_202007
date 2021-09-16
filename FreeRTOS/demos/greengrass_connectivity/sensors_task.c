#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp180.h>
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

#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"

#define SDA_GPIO 21
#define SCL_GPIO 22

char version[VERSION_STRING_LENGTH];
char string_latitude[LATITUDE_STRING_LENGTH];
char string_longitude[LONGITUDE_STRING_LENGTH];
extern char sensorsPayload[PAYLOAD_STRING_LENGTH];

#define UART_NUM                UART_NUM_2
#define UART_RX_PIN             33
#define UART_RX_BUF_SIZE        (1024)

//static char tag[] = "gps";
float latitude = -200.00;
float longitude = -200.00;

static void uart_setup()
{
    uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM,
                    UART_PIN_NO_CHANGE, UART_RX_PIN,
                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

static void read_and_parse_nmea()
{
    // Configure a temporary buffer for the incoming data
    char *buffer = (char*) malloc(UART_RX_BUF_SIZE + 1);
    //char fmt_buf[32];

    latitude = -200.00;
    longitude = -200.00;

    size_t total_bytes = 0;
    while (1) {
        // Read data from the UART
        int read_bytes = uart_read_bytes(UART_NUM, (uint8_t*) buffer + total_bytes, UART_RX_BUF_SIZE - total_bytes, 100 / portTICK_RATE_MS);

        printf("read_bytes: %d\n", read_bytes);

        if (read_bytes <= 0) {
            break;
        }

        nmea_s *data;
        total_bytes += read_bytes;

        /* find start (a dollar sign) */
        char* start = memchr(buffer, '$', total_bytes);
        if (start == NULL) {
            total_bytes = 0;
            break;
        }

        /* find end of line */
        char* end = memchr(start, '\r', total_bytes - (start - buffer));
        if (NULL == end || '\n' != *(++end)) {
            break;
        }
        end[-1] = NMEA_END_CHAR_1;
        end[0] = NMEA_END_CHAR_2;

        /* handle data */
        data = nmea_parse(start, end - start + 1, 0);
        if (data == NULL) {
            printf("Failed to parse the sentence!\n");
            printf("  Type: %.5s (%d)\n", start+1, nmea_get_type(start));
        } else {
            if (data->errors != 0) {
                printf("WARN: The sentence struct contains parse errors!\n");
            }

            if (NMEA_GPGLL == data->type) {
                printf("GPGLL sentence\n");
                nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                int degrees = pos->longitude.degrees;
                printf("  Minutes: %f\n", pos->longitude.minutes / 60.0);
                float minutes = (pos->longitude.minutes / 60.0);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);

                char longitude_cardinal = pos->longitude.cardinal;
                printf("longitude_cardinal--: %c\n", longitude_cardinal);

                if ((longitude_cardinal == 'W') || (longitude_cardinal == 'S')) {
                    longitude = degrees + minutes;
                    longitude = - longitude;
                    printf("longitude: %f\n", longitude);
                }
                else{
                    longitude = degrees + minutes;
                    printf("longitude: %f\n", longitude);
                }

                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                degrees = pos->latitude.degrees;
                printf("  Minutes: %f\n", pos->latitude.minutes / 60.0);
                minutes = (pos->latitude.minutes / 60.0);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);

                char latitude_cardinal = pos->latitude.cardinal;
                printf("latitude_cardinal--: %c\n", latitude_cardinal);

                if ((latitude_cardinal == 'W') || (latitude_cardinal == 'S')) {
                    latitude = degrees + minutes;
                    latitude = - latitude;
                    printf("latitude: %f\n", latitude);
                }
                else{
                    latitude = degrees + minutes;
                    printf("latitude: %f\n", latitude);
                }
            }

            if (NMEA_GPRMC == data->type) {
                printf("GPRMC sentence\n");
                nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes / 60.0);
                int degrees = pos->longitude.degrees;
                float minutes = (pos->longitude.minutes / 60.0);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);

                char longitude_cardinal = pos->longitude.cardinal;
                printf("longitude_cardinal--: %c\n", longitude_cardinal);

                if ((longitude_cardinal == 'W') || (longitude_cardinal == 'S')) {
                    longitude = degrees + minutes;
                    longitude = - longitude;
                    printf("longitude: %f\n", longitude);
                }
                else{
                    longitude = degrees + minutes ;
                    printf("longitude: %f\n", longitude);
                }

                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                degrees = pos->latitude.degrees;
                printf("  Minutes: %f\n", pos->latitude.minutes / 60.0);
                minutes = (pos->latitude.minutes / 60.0);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);

                char latitude_cardinal = pos->latitude.cardinal;
                printf("latitude_cardinal--: %c\n", latitude_cardinal);

                if ((latitude_cardinal == 'W') || (latitude_cardinal == 'S')) {
                    latitude = degrees + minutes;
                    latitude = - latitude;
                    printf("latitude: %f\n", latitude);
                }
                else{
                    latitude = degrees + minutes;
                    printf("latitude: %f\n", latitude);
                }

            }

            nmea_free(data);
        }

        /* buffer empty? */
        if (end == buffer + total_bytes) {
            total_bytes = 0;
            break;
        }

        /* copy rest of buffer to beginning */
        if (buffer != memmove(buffer, end, total_bytes - (end - buffer))) {
            total_bytes = 0;
            break;
        }

        total_bytes -= end - buffer;
    }
    free(buffer);
}

void sensors_task(void *pvParameters)
{

    uart_setup();

    bmp180_dev_t dev;
    memset(&dev, 0, sizeof(bmp180_dev_t)); // Zero descriptor

    ESP_ERROR_CHECK_WITHOUT_ABORT(bmp180_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmp180_init(&dev));

    while (1)
    {

        read_and_parse_nmea();

        /* bmp180 reading */
        float temp = -200;
        uint32_t pressure = -200;

        esp_err_t res = bmp180_measure(&dev, &temp, &pressure, BMP180_MODE_STANDARD);
        if (res != ESP_OK)
            printf("Could not read data from bmp 180 sensor: %d\n", res);
        else
            /* float is used in printf(). you need non-default configuration in
             * sdkconfig for ESP8266, which is enabled by default for this
             * example. see sdkconfig.defaults.esp8266
             */
            printf("Temperature: %.2f degrees Celsius; Pressure: %d Pa\n", temp, pressure);

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

        sprintf(string_latitude, "%.6f", latitude);
        sprintf(string_longitude, "%.6f", longitude);
        // makes version a char array
        sprintf(version, "%u.%u.%u", xAppFirmwareVersion.u.x.ucMajor, xAppFirmwareVersion.u.x.ucMinor, xAppFirmwareVersion.u.x.usBuild);
        sprintf(sensorsPayload, "{\"d\":\"%s\",\"v\":\"%s\",\"T1\":%.2f,\"P\":%d,\"T2\":%d,\"H\":%d,\"lat\":\"%s\",\"lon\":\"%s\"}", IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME, version, temp, pressure, temperature/10, humidity/10, string_latitude, string_longitude);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}