/**
 * @file wifi_config.h
 * @brief WiFi and MQTT Configuration
 * 
 * IMPORTANT: Update these values with your actual credentials!
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// ============================================================================
// WiFi Credentials
// ============================================================================
#define WIFI_SSID       "---"        // Your WiFi network name
#define WIFI_PASSWORD   "---"    // Your WiFi password

// ============================================================================
// MQTT Broker Configuration
// ============================================================================
#define MQTT_BROKER_IP  "192.168.1.93"         // Your MQTT broker IP address
#define MQTT_BROKER_PORT 1883                    // MQTT broker port (default 1883)
#define MQTT_CLIENT_ID  "spotmicro_pico"        // Unique client ID

// ============================================================================
// MQTT Topics
// ============================================================================
#define MQTT_TOPIC_COMMAND  "spotmicro/command"  // Subscribe: Receive commands
#define MQTT_TOPIC_STATUS   "spotmicro/status"   // Publish: Send status updates
#define MQTT_TOPIC_SENSOR   "spotmicro/sensors"  // Publish: Send sensor data

#endif // WIFI_CONFIG_H
