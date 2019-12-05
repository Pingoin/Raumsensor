/*************************************************************/
/**  
 *  @file ads111x.h
 *    
 *  @author Pingoin
 *  @date 25.09.2019 – erste Implementierung  
 *  @date 26.09.2019 – Dokumentation  
 *  
 *  @version 1   
 *************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
/**
 * @brief Titel für die Gain Einstellungen
 * 
 * Geben den Messbereich an. ein FSR6144 bedeutet einen Messbereich von +-6,144 V Messbereich Hier ist die Zulässige Spannung des Chips zu beachten (Vcc+0,3V)
 * 
 */
typedef enum
{
    ///+-6,114 V (max Vcc+0,3 V pro Pin)
    FSR6144 = 0b000,
    ///+-4,096 V (max Vcc +0,3 V pro Pin)
    FSR4096 = 0b001,
    ///+-2,048 V
    FSR2048 = 0b010,
    ///+- 1,024 V
    FSR1024 = 0b011,
    ///+- 0,512 V
    FSR0512 = 0b100,
    ///+- 0,256 V
    FSR0256 = 0b101
} adsGain_t;

/**
 * @brief der zu messende Kanal
 * 
 * Der Sensor kann nur einen Kanal/eine Differenz auf mal auslesen. Dieser ist über diese Liste zu definieren
 * 
 */
typedef enum
{
    /// Misst A0 zu GND
    A0GND = 0b100,
    ///Misst A1 zu GND
    A1GND = 0b101,
    ///Misst A2 zu GND
    A2GND = 0b110,
    ///Misst A3 zu GND
    A3GND = 0b111,
    ///Misst die Differenz von A0 zu A1
    A0A1 = 0b000,
    ///Misst die Differenz von A0 zu A3
    A0A3 = 0b001,
    ///Misst die Differenz von A1 zu A3
    A1A3 = 0b010,
    ///Misst die Differenz von A2 zu A3
    A2A3 = 0b011
} adsChan_t;

/**
 * @brief Maximale Ausleserate
 * 
 * bei einem SPS8 können 8 Messungen in der Sekunde gemacht werden (die retliche zeit wird zum glätten genutzt)
 * SPS860 bedeutet bis zu 860 Messungen in der Sekunde, hier ist Größeres rauschen zu erwarten als bei kleineren Werten. Der Stromverbrauch ist hier besonders gering.
 * 
 */
typedef enum
{
    ///8 Messungen pro Sekunde (Messdauer 125 ms)
    SPS8 = 0b000,
    ///16 Messungen pro Sekunde (Messdauer 62,5 ms)
    SPS16 = 0b001,
    ///32 Messungen pro Sekunde (Messdauer 31,25 ms)
    SPS32 = 0b010,
    ///64 Messungen pro Sekunde (Messdauer 15,625 ms)
    SPS64 = 0b011,
    ///128 Messungen pro Sekunde (Messdauer 7,813 ms)
    SPS128 = 0b100,
    ///250 Messungen pro Sekunde (Messdauer 4 ms)
    SPS250 = 0b101,
    ///475 Messungen pro Sekunde (Messdauer 2,105 ms)
    SPS475 = 0b110,
    ///860 Messungen pro Sekunde (Messdauer 1,163 ms)
    SPS860 = 0b111
} adsSPS_t;

/**
 * @brief Gesamtklasse des ADS1115 Sensors
 * 
 */
class ADS1115
{
public:
    /**
     * @brief Erstellt ein neues ADS1115 Objekt
     * 
     * Als Standardwerte werden die Werte aus dem datenblatt übernommen, als Standard Kanal wird \ref A0GND eingestellt, dieser Wird beim ersten lesen überschrieben.
     * 
     * @param i2cAddr die per Schaltung einestellte I2C Adresse.
     */
    ADS1115(uint8_t i2cAddr);
    /**
     * @brief Leere Destroyer-Funktion
     * 
     */
    ~ADS1115();
    /**
     * @brief Startet die I2C Verbindung.
     * 
     * Wenn bereits ein Wire.begin() ausgeführt wurde nicht weiter nötig
     * 
     */
    void begin();
    /**
     * @brief Liefert zurück, on der Definierte Sensor vorhanden ist
     * 
     * @return true Sensor ist vorhanden
     * @return false Sensor ist NICHT vorhanden
     */
    bool conneted();
    /**
     * @brief Stellt den Gain Wert ein
     * 
     * @param gain Ein Wert aus \ref adsGain_t ist hier zu wählen
     */
    void setGain(adsGain_t gain);
    /**
     * @brief Stellt die Messrate/Dauer ein
     * 
     * @param dataRate Siehe \ref adsSPS_t
     */
    void setDataRate(adsSPS_t dataRate);
    /**
     * @brief Misst die Spannung und gibt den Wert aus
     * 
     * Eine Berechnung mit Spannungsteiler ist Möglich wenn dieser per setVoltageDivider(float r1, float r2) eingetragen wird
     * Die Dauer der messung ist Abhängig von dem gewählten Gain
     * 
     * @param channel Der zu messende Kanal/die zu vergleichenden Kanäle als \ref adsChan_t
     * @return float die gemessene Spannung in Volt
     */
    float readVoltage(adsChan_t channel);
    /**
     * @brief Misst die Spannung und gibt den Wert aus
     * 
     * Eine Berechnung mit Spannungsteiler ist Möglich wenn dieser per setVoltageDivider(float r1, float r2) eingetragen wird
     * Die Dauer der messung ist Abhängig von dem gewählten Gain
     * 
     * @param channel Der zu messende Kanal/die zu vergleichenden Kanäle als \ref adsChan_t
     * @param R1 Widerstand zwischen Spannung und sensor
     * @param R2 Widerstand ziwschen Sensor und GNG
     * @return float die gemessene Spannung in Volt
     */
    float readVoltage(adsChan_t channel,float R1,float R2);
protected:
    /**
     * @brief I2C-Konfiguration des Status
     * 
     * Kann wird zum Starten einer Messung benutzt
     */
    uint8_t confOS;
    /**
     * @brief I2C-Konfiguration für die Kanalauswahl (\ref adsChan_t)
     * 
     */
    uint8_t confMUX;
    /**
     * @brief I2C-Konfiguration für den Spannugnsbereich/gain (\ref adsGain_t)
     * 
     */
    uint8_t confPGA;
    /**
     * @brief  I2C-Konfiguration für den arbeitsmodus
     * 
     * @todo weitere Einstellmöglichkeiten erstellen
     */
    uint8_t confMODE;
    /**
     * @brief  I2C-Konfiguration für die Samplerate (\ref adsSPS_t)
     *
     *  
     */
    uint8_t confDR;
    /**
     * @brief I2C-Konfiguration
     * 
     */
    uint8_t confCompMode;
    /**
     * @brief I2C-Konfiguration
     * 
     */
    uint8_t confCompPol;
    /**
     * @brief I2C-Konfiguration
     * 
     */
    uint8_t confCompLat;
    /**
     * @brief I2C-Konfiguration
     * 
     */
    uint8_t confCompQue;
    /**
     * @brief I2C-Adresse des Sensors
     * 
     */
    uint8_t i_i2cAddr;
    /**
     * @brief Wartezeit nach Beauftragung der Messung
     * 
     */
    uint8_t delayTime;
    /**
     * @brief Spannungswert den ein Bit ausmacht.
     * 
     * Ist abhängig vom gewählten Spannungsbereich \see adsGain_t
     * 
     */
    float i_lsb;
    /**
     * @brief Interne Funktion für die neuberechnung von Variablen nach anpassung anderer Variablen
     * 
     */
    void calcInternals();

};