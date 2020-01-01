#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

#include "wifi.h"
#include "mqtt.h"
#include "as3935.h"
#include "utils.h"
#include "bme280.h"
#include "tsl2561.h"
#include "pcf8574.h"

#define MSG_BUF_SIZE 16
#define _PUB(pub, buf, value, format) do {                  \
        snprintf((buf), (MSG_BUF_SIZE), (format), (value)); \
        mqtt_pub_publish((pub), (buf), strlen(buf));        \
    } while (0)
#define PUB_F(pub, buf, value) _PUB(pub, buf, value, "%f")
#define PUB_U(pub, buf, value) _PUB(pub, buf, value, "%u")

#define IO_OFF 1
#define IO_ON  0


typedef struct
{
    float temperature;
    float humidity;
    float pressure;
    uint32_t luminosity;
    uint32_t telemetry_pub_interval_ms;
} APP_telemetry_t;

typedef struct
{
    bool t1;
    bool t2;
    bool rl1;
    bool rl2;
} APP_io_t;

const char* MAIN_TAG = "MAIN";

static APP_telemetry_t telemetry = {
        .temperature = 0.0f,
        .humidity = 0.0f,
        .pressure = 0.0f,
        .luminosity = 0,
        .telemetry_pub_interval_ms = 5000
};
static APP_io_t io = {
        .t1 = IO_OFF,
        .t2 = IO_OFF,
        .rl1 = IO_OFF,
        .rl2 = IO_OFF
};

void main_task(void *pv_param);
#ifdef CONFIG_I2C_GPIO_ENABLED
void gpio_task(void __unused *pv_param);
#endif
#ifdef CONFIG_AS3935_ENABLED
void as3935_task(void *pv_param);
#endif
#ifdef CONFIG_BME280_ENABLED
void bme280_task(void *pv_param);
#endif
#ifdef CONFIG_TSL2561_ENABLED
void tsl2561_task(void *pv_param);
#endif
esp_err_t _setup_i2c();

esp_err_t mqtt_cmd_set_telemetry_interval_cb(MQTT_callback_event_t *event);
esp_err_t mqtt_cmd_io_cb(MQTT_callback_event_t *event);


void app_main()
{
    esp_err_t ret;

    ret = uart_set_baudrate(UART_NUM_0, CONFIG_UART_BAUDRATE);
    ESP_ERROR_CHECK(ret);

    ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);

    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGW(MAIN_TAG, "RESET REASON = %s", reset_reason_map[(size_t)reason]);

    ESP_LOGI(MAIN_TAG, "Initializing WiFi...");
    init_wifi();

    ESP_LOGI(MAIN_TAG, "Initializing I2C...");
    ret = _setup_i2c();
    ESP_ERROR_CHECK(ret);

    ret = gpio_install_isr_service(0);
    ESP_ERROR_CHECK(ret);

#ifdef CONFIG_AS3935_ENABLED
    xTaskCreate(&as3935_task, "as3935_task", 2048, NULL, 5, NULL);
#endif
#ifdef CONFIG_BME280_ENABLED
    xTaskCreate(&bme280_task, "bme280_task", 2048, NULL, 5, NULL);
#endif
#ifdef CONFIG_TSL2561_ENABLED
    xTaskCreate(&tsl2561_task, "tsl2561_task", 2048, NULL, 5, NULL);
#endif
#ifdef CONFIG_I2C_GPIO_ENABLED
    xTaskCreate(&gpio_task, "gpio_task", 2048, NULL, 4, NULL);
#endif

    xTaskCreate(&main_task, "main_task", 2084, NULL, 3, NULL);
}

void main_task(void __unused *pv_param)
{
#if (CONFIG_BME280_ENABLED | CONFIG_TSL2561_ENABLED)
    char msg_buf[MSG_BUF_SIZE];
#endif
    MQTT_sub_config_t config;
    MQTT_Client_t mqtt = mqtt_init();

    ESP_LOGI(MAIN_TAG, "Creating publishers...");
#ifdef CONFIG_BME280_ENABLED
    MQTT_Publisher_t temp_pub = mqtt_pub_create(mqtt, "tdesp/temperature", 0, 0);
    MQTT_Publisher_t hum_pub = mqtt_pub_create(mqtt, "tdesp/humidity", 0, 0);
    MQTT_Publisher_t press_pub = mqtt_pub_create(mqtt, "tdesp/pressure", 0, 0);
#endif
#ifdef CONFIG_TSL2561_ENABLED
    MQTT_Publisher_t lux_pub = mqtt_pub_create(mqtt, "tdesp/luminosity", 0, 0);
#endif
    MQTT_Publisher_t telem_interval_pub = mqtt_pub_create(mqtt, "tdesp/telemetry_interval", 0, 1);

    ESP_LOGI(MAIN_TAG, "Creating subscribers...");

    config.topic = "tdesp/cmd/telemetry_interval";
    config.qos = 1;
    config.callback = mqtt_cmd_set_telemetry_interval_cb;
    config.context.data = telem_interval_pub;
    MQTT_Subscriber_t __unused cmd_set_telemetry_interval_sub = mqtt_sub_create(mqtt, &config);

    config.callback = mqtt_cmd_io_cb;
    config.context.data = NULL;
    config.topic = "tdesp/cmd/t1";
    config.context.value = 1;
    MQTT_Subscriber_t __unused cmd_io_t1_sub = mqtt_sub_create(mqtt, &config);

    config.topic = "tdesp/cmd/t2";
    config.context.value = 2;
    MQTT_Subscriber_t __unused cmd_io_t2_sub = mqtt_sub_create(mqtt, &config);

    config.topic = "tdesp/cmd/rl1";
    config.context.value = 3;
    MQTT_Subscriber_t __unused cmd_io_rl1_sub = mqtt_sub_create(mqtt, &config);

    config.topic = "tdesp/cmd/rl2";
    config.context.value = 4;
    MQTT_Subscriber_t __unused cmd_io_rl2_sub = mqtt_sub_create(mqtt, &config);

    while(true)
    {
        vTaskDelay(telemetry.telemetry_pub_interval_ms / portTICK_RATE_MS);

#ifdef CONFIG_BME280_ENABLED
        PUB_F(temp_pub, msg_buf, telemetry.temperature);
        PUB_F(hum_pub, msg_buf, telemetry.humidity);
        PUB_F(press_pub, msg_buf, telemetry.pressure);
#endif
#ifdef CONFIG_TSL2561_ENABLED
        PUB_U(lux_pub, msg_buf, telemetry.luminosity);
#endif
    }
}

esp_err_t mqtt_cmd_set_telemetry_interval_cb(MQTT_callback_event_t *event)
{
    int32_t value;
    char buf[MSG_BUF_SIZE];
    MQTT_Publisher_t telem_interval_pub = event->context.data;
    char *msg = calloc(event->data_len + 1, sizeof(char));
    strncpy(msg, event->data, event->data_len);
    value = atoi(msg);

    ESP_LOGI(MAIN_TAG, "Got cmd telemetry interval. Data: '%s', value: '%d'", msg, value);
    free(msg);

    if (event->data_len == 0 || value == 0)
    {
        PUB_U(telem_interval_pub, buf, telemetry.telemetry_pub_interval_ms);
    }
    else if (value > 0)
    {
        ESP_LOGI(MAIN_TAG, "Updating telemetry interval to: %u", value);
        telemetry.telemetry_pub_interval_ms = value;
    }
    else
    {
        ESP_LOGW(MAIN_TAG, "Telemetry interval can't be negative!");
    }

    return ESP_OK;
}

esp_err_t mqtt_cmd_io_cb(MQTT_callback_event_t *event)
{
    int8_t value;
    char *msg = calloc(event->data_len + 1, sizeof(char));
    strncpy(msg, event->data, event->data_len);
    value = atoi(msg);
    free(msg);

    ESP_LOGI(MAIN_TAG, "Got CMD IO, topic: '%s', context: '%d', value: '%d'",
            event->topic, event->context.value, value);

    if (value < 0)
    {
        ESP_LOGW(MAIN_TAG, "Value could not be less than 0!");
        return ESP_OK;
    }

    switch (event->context.value)
    {
        case 1:
            io.t1 = value ? IO_ON : IO_OFF;
            break;
        case 2:
            io.t2 = value ? IO_ON : IO_OFF;
            break;
        case 3:
            io.rl1 = value ? IO_ON : IO_OFF;
            break;
        case 4:
            io.rl2 = value ? IO_ON : IO_OFF;
            break;
        default:
            ESP_LOGW(MAIN_TAG, "Unknown context value: %d", event->context.value);
            break;
    }

    return ESP_OK;
}

#ifdef CONFIG_I2C_GPIO_ENABLED

void gpio_task(void __unused *pv_param)
{
    PCF8574_handle_t pcf = pcf8574_init(PCF8574_ADDR_000);
    APP_io_t prev_io = io;

    pcf8574_set_pin(pcf, CONFIG_IO_T1_PIN, io.t1);
    pcf8574_set_pin(pcf, CONFIG_IO_T2_PIN, io.t2);
    pcf8574_set_pin(pcf, CONFIG_IO_RL1_PIN, io.rl1);
    pcf8574_set_pin(pcf, CONFIG_IO_RL2_PIN, io.rl2);

    while (true)
    {
        if (io.t1 != prev_io.t1)
        {
            ESP_LOGI(MAIN_TAG, "Updating T1 state to %s",
                    (io.t1 == IO_ON) ? "ON" : "OFF");
            pcf8574_set_pin(pcf, CONFIG_IO_T1_PIN, io.t1);
            prev_io = io;
        }
        if (io.t2 != prev_io.t2)
        {
            ESP_LOGI(MAIN_TAG, "Updating T2 state to %s",
                    (io.t2 == IO_ON) ? "ON" : "OFF");
            pcf8574_set_pin(pcf, CONFIG_IO_T2_PIN, io.t2);
            prev_io = io;
        }
        if (io.rl1 != prev_io.rl1)
        {
            ESP_LOGI(MAIN_TAG, "Updating RL1 state to %s",
                    (io.rl1 == IO_ON) ? "ON" : "OFF");
            pcf8574_set_pin(pcf, CONFIG_IO_RL1_PIN, io.rl1);
            prev_io = io;
        }
        if (io.rl2 != prev_io.rl2)
        {
            ESP_LOGI(MAIN_TAG, "Updating RL2 state to %s",
                    (io.rl2 == IO_ON) ? "ON" : "OFF");
            pcf8574_set_pin(pcf, CONFIG_IO_RL2_PIN, io.rl2);
            prev_io = io;
        }

        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

#endif // CONFIG_I2C_GPIO_ENABLED

#ifdef CONFIG_AS3935_ENABLED

void _as3935_print_registers(AS3935_handle_t as);
void _as3935_irq_handler(void *params);

bool as3935_irq_triggered = false;

void as3935_task(void __unused *pv_param)
{
    ESP_LOGI(MAIN_TAG, "Initializing...");
    gpio_isr_handler_add(CONFIG_AS3935_IRQ_PIN, _as3935_irq_handler, NULL);
    AS3935_handle_t as = as3935_init(AS3935_ADDR_11, CONFIG_AS3935_IRQ_PIN);
    ESP_LOGI(MAIN_TAG, "Power up...");
    as3935_reset(as);
    vTaskDelay(300 / portTICK_RATE_MS);
    as3935_calibrate(as);
    as3935_set_afe_indoors(as);
    as3935_set_disturbers_mask(as, false);

    _as3935_print_registers(as);

    as3935_irq_triggered = false;

    uint8_t irq_source;

    while (true)
    {
        if (as3935_irq_triggered) {
            vPortEnterCritical();
            as3935_irq_triggered = false;

            irq_source = as3935_get_interrupt_source(as);

            ESP_LOGI(MAIN_TAG, "IRQ_PIN state is %d", gpio_get_level(CONFIG_AS3935_IRQ_PIN));

            if (irq_source & 0b0001) {
                ESP_LOGI(MAIN_TAG, "Noise level too high, try adjusting noise floor");
            }
            if (irq_source & 0b0100) {
                ESP_LOGI(MAIN_TAG, "Disturber detected");
            }
            if (irq_source & 0b1000) {
                uint8_t distance = as3935_get_lighting_distance_km(as);
                ESP_LOGI(MAIN_TAG, "Lighting detected - distance = %d", distance);
            }
            vPortExitCritical();
        }
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

void _as3935_print_registers(AS3935_handle_t as)
{
    uint8_t noise_floor = as3935_get_noise_floor(as);
    uint8_t spike_rejection = as3935_get_spike_rejection(as);
    uint8_t wdth = as3935_get_watchdog_threshold(as);
    ESP_LOGI(MAIN_TAG, "Noise floor is: %d", noise_floor);
    ESP_LOGI(MAIN_TAG, "Spike rejection is: %d", spike_rejection);
    ESP_LOGI(MAIN_TAG, "Watchdog threshold is: %d", wdth);
}

void _as3935_irq_handler(void __unused *params)
{
    as3935_irq_triggered = true;
}

#endif // CONFIG_AS3935_ENABLED

#ifdef CONFIG_BME280_ENABLED

void bme280_task(void *pv_param)
{
    ESP_LOGI(MAIN_TAG, "Initializing BME280...");

    BME280_config_t config = {
            .mode = BME280_MODE_NORMAL,
            .filter = BME280_FILTER_4,
            .ov_hum = BME280_OVx1,
            .ov_press = BME280_OVx1,
            .ov_temp = BME280_OVx1,
            .t_standby = BME280_T_SB_1000
    };

    BME280_handle_t bme = bme280_init(BME280_ADDR_0, &config);

    while (true)
    {
        vTaskDelay(2000 / portTICK_RATE_MS);
        bme280_read_temperature(bme, &telemetry.temperature);
        bme280_read_humidity(bme, &telemetry.humidity);
        bme280_read_pressure(bme, &telemetry.pressure);
    }
}

#endif // CONFIG_BME280_ENABLED

#ifdef CONFIG_TSL2561_ENABLED

void tsl2561_task(void *pv_param)
{
    ESP_LOGI(MAIN_TAG, "Initializing TSL2561...");

    TSL2561_config_t config = {
            .power = TSL2561_PWR_ON,
            .gain = TSL2561_GAIN_x16,
            .integ = TSL2561_INTEG_402ms,
            .intr = TSL2561_INTR_OFF,
            .persist = TSL2561_PERSIST_THRESH1
    };

    TSL2561_handle_t tsl = tsl2561_init(TSL2561_ADDR_SEL_FLOAT, &config);
    tsl2561_enable_auto_range(tsl, true);

    while (true)
    {
        vTaskDelay(2000 / portTICK_RATE_MS);
        tsl2561_read_calculated_lux(tsl, &telemetry.luminosity);
    }
}

#endif // CONFIG_TSL2561_ENABLED

esp_err_t _setup_i2c()
{
    esp_err_t ret;
    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER);
    ESP_ERROR_CHECK(ret)

    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .clk_stretch_tick = 500,
            .scl_io_num = CONFIG_I2C_SCL_PIN,
            .sda_io_num = CONFIG_I2C_SDA_PIN,
#ifdef CONFIG_I2C_INT_PULLUP
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .sda_pullup_en = GPIO_PULLUP_ENABLE
#else
            .scl_pullup_en = GPIO_PULLUP_DISABLE,
            .sda_pullup_en = GPIO_PULLUP_DISABLE
#endif
    };

    ret = i2c_param_config(I2C_NUM_0, &conf);
    ESP_ERROR_CHECK(ret)
    return ESP_OK;
}
