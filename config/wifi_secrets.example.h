/**
 * @file wifi_secrets.example.h
 * @brief Example WiFi and MQTT configuration.
 *
 * Copy this file to wifi_secrets.h and replace the placeholder values.
 * wifi_secrets.h is ignored by Git and must not be committed.
 */
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* MQTT=1 also requires WIFI=cyw43 and NET=lwip. */
#define MQTT_BROKER_IP   "192.168.1.10"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "p2-08-car-01"
#define MQTT_TOPIC       "p2-08/p2-08-car-01/heartbeat"

/* Subsystem 1 command input and binary telemetry output. */
#define S1_MQTT_COMMAND_TOPIC   "p2-08/p2-08-car-01/command"
#define S1_MQTT_TELEMETRY_TOPIC "p2-08/p2-08-car-01/telemetry"

#endif /* WIFI_SECRETS_H */

/* End of file. */
