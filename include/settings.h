/**
 * @file settings.h
 * @author Pingoin (p.drente@gmx.de)
 * @brief Gesammelte Einstellungen
 * @version 0.1
 * @date 2019-12-03
 * 
 * @copyright Copyright (c) 2019
 * 
 */

//Auskommentieren, wenn keine Private.h existiert und WIFI Daten direkt eingertagen werden
//#include <private.h>

/**
 * @brief Netzwerk SSID Angabe in Anführungszeichen: "SSID"
 */
#define SSID "mySSID"

/**
 * @brief WPA-Passwort Angabe in Anführungszeichen: "Passwort"
 * 
 */
#define PSK "myPSK"

/**
 * @brief Bezeichnung des Controllers
 * 
 * Wird benutzt als bezeichnung für MQTT und OTA
 */
#define hostname  "Raumsensor"

/**
 * @brief Adresse des MQTT-Brokers
 * 
 * Angabe in den Byte-Gruppen getrennt durch komma (z.B.: 192,168,178,111)
 */
#define MQTT_BROKER 192, 168, 178, 110

/**
 * @brief Statische IP des MCU
 * 
 * Wenn DHCP-Benutzt wird bitte auskommentieren
 * Angabe in den Byte-Gruppen getrennt durch komma (z.B.: 192,168,178,111)
 */
#define staticIP 192, 168, 178, 190

/**
 * @brief Gateway des MCU
 * 
 * Wenn DHCP-Benutzt wird bitte auskommentieren
 * Angabe in den Byte-Gruppen getrennt durch komma (z.B.: 192,168,178,111)
 */
#define staticGateway 192, 168, 178, 1

/**
 * @brief Subnetzsmaske des MCU
 * 
 * Wenn DHCP-Benutzt wird bitte auskommentieren
 * Angabe in den Byte-Gruppen getrennt durch komma (z.B.: 255,255,255,0)
 */
#define staticSubnet 255, 255, 255, 0

/**
 * @brief DNS-SeverIP des MCU
 * 
 * Wenn DHCP-Benutzt wird bitte auskommentieren
 * Angabe in den Byte-Gruppen getrennt durch komma (z.B.: 192,168,178,111)
 */
#define staticDNS 192, 178, 168, 1

/**
 * @brief i2CAdresse des BME280
 * 
 */
#define bme280I2C 0x76

/**
 * @brief Zeit zwischen den Sendungen in Millisekunden
 * 
 * angabe in minuten*60e6 möglich (z.B.: 5*60e6)
 */
#define sleepTime 5*60e3

/**
 * @brief 
 * 
 */
#define httpServerWiFi "http://192.168.178.110:1880/wettersensor"