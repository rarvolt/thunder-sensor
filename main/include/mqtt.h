#ifndef THUNDER_DETECT_MQTT_H
#define THUNDER_DETECT_MQTT_H

#include <mqtt_client.h>

typedef struct
{
    uint8_t value;
    void *data;
} MQTT_context_t;

typedef struct
{
    const char *topic;
    const char *data;
    size_t data_len;
    MQTT_context_t context;
} MQTT_callback_event_t;

typedef esp_err_t (* mqtt_message_callback_t)(MQTT_callback_event_t *event);

typedef struct _MQTT_Publisher* MQTT_Publisher_t;
typedef struct _MQTT_Subscriber* MQTT_Subscriber_t;
typedef struct _MQTT_Client* MQTT_Client_t;

typedef struct {
    char *topic;
    uint8_t qos;
    mqtt_message_callback_t callback;
    MQTT_context_t context;
} MQTT_sub_config_t;


extern MQTT_Client_t mqtt_init();
extern MQTT_Publisher_t mqtt_pub_create(MQTT_Client_t client, const char *topic,
        uint8_t qos, bool retain);
extern esp_err_t mqtt_pub_delete(MQTT_Client_t client, MQTT_Publisher_t pub);
extern esp_err_t mqtt_pub_publish(MQTT_Publisher_t pub, const char *data, size_t data_len);
extern MQTT_Subscriber_t mqtt_sub_create(MQTT_Client_t client, MQTT_sub_config_t *config);
extern esp_err_t mqtt_sub_delete(MQTT_Client_t client, MQTT_Subscriber_t sub);


#endif //THUNDER_DETECT_MQTT_H
