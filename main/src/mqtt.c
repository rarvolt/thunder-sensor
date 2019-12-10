#include "mqtt.h"

#include <stdlib.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "utlist.h"


struct _MQTT_Publisher
{
    esp_mqtt_client_handle_t esp_client;
    char *topic;
    uint8_t qos;
    bool retain;
    bool connected;
    MQTT_Publisher_t prev;
    MQTT_Publisher_t next;
};

struct _MQTT_Subscriber
{
    esp_mqtt_client_handle_t esp_client;
    char *topic;
    uint8_t qos;
    mqtt_message_callback_t message_callback;
    MQTT_context_t context;
    MQTT_Subscriber_t prev;
    MQTT_Subscriber_t next;
};

struct _MQTT_Client
{
    esp_mqtt_client_handle_t esp_client;
    bool connected;
    MQTT_Subscriber_t subscribers;
    MQTT_Publisher_t publishers;
};

static const char *TAG_MQTT = "MQTT";

esp_err_t _mqtt_process_message(MQTT_Client_t client,
        const char *topic, size_t topic_len,
        const char *data, size_t data_len);


static esp_err_t _mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    //esp_mqtt_client_handle_t client = event->client;
    //int msq_id;

    MQTT_Client_t client = (MQTT_Client_t)event->user_context;
    MQTT_Publisher_t pub;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
            client->connected = true;
            DL_FOREACH(client->publishers, pub)
            {
                pub->connected = true;
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
            client->connected = false;
            DL_FOREACH(client->publishers, pub)
            {
                pub->connected = false;
            }
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
            _mqtt_process_message(client, event->topic, event->topic_len,
                    event->data, event->data_len);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}


MQTT_Client_t mqtt_init()
{
    MQTT_Client_t client = calloc(1, sizeof(struct _MQTT_Client));

    esp_mqtt_client_config_t config = {
            .uri = CONFIG_BROKER_URL,
            .event_handle = _mqtt_event_handler,
            .user_context = client
    };

    client->esp_client = esp_mqtt_client_init(&config);
    client->publishers = NULL;
    client->subscribers = NULL;
    client->connected = false;
    esp_mqtt_client_start(client->esp_client);

    while (!client->connected)
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    return client;
}

MQTT_Publisher_t mqtt_pub_create(MQTT_Client_t client, const char *topic, uint8_t qos, bool retain)
{
    MQTT_Publisher_t pub = malloc(sizeof(struct _MQTT_Publisher));

    pub->topic = malloc(strlen(topic) + 1);
    strcpy(pub->topic, topic);
    pub->esp_client = client->esp_client;
    pub->qos = qos;
    pub->retain = retain;
    pub->connected = client->connected;

    DL_APPEND(client->publishers, pub);

    return pub;
}

esp_err_t mqtt_pub_delete(MQTT_Client_t client, MQTT_Publisher_t pub)
{
    DL_DELETE(client->publishers, pub);
    free(pub->topic);
    free(pub);

    return ESP_OK;
}

esp_err_t mqtt_pub_publish(MQTT_Publisher_t pub, const char *data, size_t data_len)
{
    if (!pub->connected)
        return ESP_OK;

    ESP_LOGI(TAG_MQTT, "Publishing data: '%.*s', len: %u", data_len, data, data_len);

    esp_mqtt_client_publish(pub->esp_client, pub->topic, data, data_len, pub->qos, pub->retain);

    return ESP_OK;
}

MQTT_Subscriber_t mqtt_sub_create(MQTT_Client_t client,
        MQTT_sub_config_t *config)
{
    MQTT_Subscriber_t sub = malloc(sizeof(struct _MQTT_Subscriber));

    sub->topic = malloc(strlen(config->topic) + 1);
    strcpy(sub->topic, config->topic);
    sub->esp_client = client->esp_client;
    sub->qos = config->qos;
    sub->message_callback = config->callback;
    sub->context = config->context;

    esp_mqtt_client_subscribe(sub->esp_client, sub->topic, sub->qos);

    DL_APPEND(client->subscribers, sub);

    return sub;
}

esp_err_t mqtt_sub_delete(MQTT_Client_t client, MQTT_Subscriber_t sub)
{
    DL_DELETE(client->subscribers, sub);
    esp_mqtt_client_unsubscribe(sub->esp_client, sub->topic);

    free(sub->topic);
    free(sub);

    return ESP_OK;
}

esp_err_t _mqtt_process_message(MQTT_Client_t client,
        const char *topic, size_t topic_len,
        const char *data, size_t data_len)
{
    char *_topic = malloc(topic_len + 1);
    strncpy(_topic, topic, topic_len);
    _topic[topic_len] = '\0';

    MQTT_Subscriber_t sub;
    bool found_callback = false;
    esp_err_t ret = ESP_OK;
    MQTT_callback_event_t event = {
            .topic = _topic,
            .data = data,
            .data_len = data_len
    };

    DL_FOREACH(client->subscribers, sub)
    {
        if (strcmp(_topic, sub->topic) == 0)
        {
            found_callback = true;
            event.context = sub->context;
            ret = sub->message_callback(&event);
            break;
        }
    }

    if (!found_callback)
    {
        ret = ESP_ERR_NOT_FOUND;
    }

    free(_topic);

    return ret;
}
