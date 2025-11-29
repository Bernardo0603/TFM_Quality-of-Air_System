/**
 * @file      loramac.cpp
 * LMIC library only support SX1276 Radio
 */


#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Tomoto_HM330X.h>
#include <Wire.h>
#include "PMsystem.h"
#include "PMplatform.h"
#include "PMutility.h"
#include "LoRaBoards.h"
#include "utilities.h"



// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x50, 0xA3, 0xF3, 0x39, 0xC5, 0x33, 0x53, 0x4D, 0x30, 0x24, 0xEA, 0x53, 0x06, 0xBA, 0x68, 0xAE };
// LoRaWAN AppSKey, application session key
static const PROGMEM u1_t APPSKEY[16] = { 0xCE, 0xB6, 0x14, 0x9A, 0x60, 0xE3, 0x70, 0xB1, 0x18, 0x26, 0x19, 0x70, 0xCD, 0x4D, 0xF1, 0x2B };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x27FC038A; 


// Pin mapping
#ifdef STM32L073xx
const lmic_pinmap lmic_pins = {
    .nss =  RADIO_CS_PIN,
    .rxtx = RADIO_SWITCH_PIN,
    .rst =  RADIO_RST_PIN,
    .dio = {RADIO_DIO0_PIN, RADIO_DIO1_PIN, RADIO_DIO2_PIN},
    .rx_level = HIGH
};
#else
const lmic_pinmap lmic_pins = {
    .nss =  RADIO_CS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst =  RADIO_RST_PIN,
    .dio = {RADIO_DIO0_PIN, RADIO_DIO1_PIN, RADIO_DIO2_PIN}
};
#endif

static osjob_t sendjob;
unsigned long lastGpsSendTime = 0;
const unsigned long intervaloGps = 120000; // 2 minutos
static int spreadFactor = DR_SF7;
static int joinStatus = EV_JOINING;
static const unsigned TX_INTERVAL = 30;
static String lora_msg = "";

// ----------------------------------- GPS e Sensores ---------------------------------------------------------
// -------------------------- GPS ---------------------------
TinyGPSPlus gps;
HardwareSerial hs(1); // UART#1 do LilyGo

// ---------------------- Hum e Temp ------------------------
#define DHTPIN  25
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);

// -------------------- PM2.5 e PM 10 -----------------------
Tomoto_HM330X sensor;
uint64_t failureStartTime = 0;
const uint64_t RESTART_TIMEOUT = 20000;
void initSystem() {}
void haltSystem() {while (true);}
void restartSystem () {}

// -------------------- CO2 -----------------------
#define MG_PIN 35
#define BOOL_PIN 2
#define DC_GAIN 8.5
#define READ_SAMPLE_INTERVAL 50
#define READ_SAMPLE_TIMES 5
#define ZERO_POINT_VOLTAGE 0.346 //resultante de medição de 2.94 V / 8.5 V --> DC Gain
#define REACTION_VOLTAGE 0.030 //valor com atmosfera em 1000 ppm (Valor testado em laboratório)
float CO2Curve[3] = {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTAGE/(2.602-3))};

// ----------------------------------- OTAA - Não necessário --------------------------------------------------
void os_getArtEui (u1_t *buf){}
void os_getDevEui (u1_t *buf){}
void os_getDevKey (u1_t *buf){}
// ------------------------------------------------------------------------------------------------------------


float MGRead(int mg_pin)
{
    int i;
    float v=0;

    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) * 3.3/4095.0 ;
    return v;
}

float MGGetPercentage(float volts, float *pcurve){
    if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}

bool my_payload(uint8_t *payload) {
    // Leitura GPS
    int32_t lat = gps.location.lat() * 1e6;
    int32_t lng = gps.location.lng() * 1e6;
    uint8_t sats = gps.satellites.value();

    // Leitura DHT
    sensors_event_t tempEvent, humEvent;
    dht.temperature().getEvent(&tempEvent);
    dht.humidity().getEvent(&humEvent);

    if (isnan(tempEvent.temperature) || isnan(humEvent.relative_humidity)) {
        Serial.println("Erro ao ler DHT22");
        return false;
    }

    int16_t temp_int = round(tempEvent.temperature); // décimos de °C
    int16_t hum_int  = round(humEvent.relative_humidity);    // décimos de %

    //Leitura HM3301
    uint16_t pm10 = sensor.atm.getPM10();
    uint16_t pm2_5 = sensor.atm.getPM2_5();

    //Leitura CO2
    float volts, percentage;
    //int percentage;       //only after 48 hours
    volts = MGRead(MG_PIN);
    percentage = volts*(-400/2.94)+800;
    uint16_t co2_ppm = percentage;

   /*/ // Debug
    Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
    Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
    Serial.print("Satélites: "); Serial.println(sats);
    Serial.print("Temperatura: "); Serial.print(tempEvent.temperature); Serial.println(" °C");
    Serial.print("Humidade: "); Serial.print(humEvent.relative_humidity); Serial.println(" %");
    Serial.print("PM2.5: "); Serial.print(sensor.atm.getPM2_5()); Serial.println(" ug/m^3");
    Serial.print("PM10: "); Serial.print(sensor.atm.getPM10()); Serial.println(" ug/m^3");
*/
    // GPS
    payload[0] = (lat >> 24) & 0xFF;
    payload[1] = (lat >> 16) & 0xFF;
    payload[2] = (lat >> 8) & 0xFF;
    payload[3] = lat & 0xFF;
    payload[4] = (lng >> 24) & 0xFF;
    payload[5] = (lng >> 16) & 0xFF;
    payload[6] = (lng >> 8) & 0xFF;
    payload[7] = lng & 0xFF;
    payload[8] = sats;

    // Temperatura (2 bytes, int16)
    payload[9]  = (temp_int >> 8) & 0xFF;
    payload[10] = temp_int & 0xFF;

    // Humidade (2 bytes, int16)
    payload[11] = (hum_int >> 8) & 0xFF;
    payload[12] = hum_int & 0xFF;

    //PM2.5 (2 bytes, int16)
    payload[13] = (pm2_5 >> 8) & 0xFF;
    payload[14] = pm2_5 & 0xFF;

    //PM10 (2 bytes, int16)
    payload[15] = (pm10 >> 8) & 0xFF;
    payload[16] = pm10 & 0xFF;

    //CO2 (2 bytes, int16)
    payload[17] = (co2_ppm >> 8) & 0xFF;
    payload[18] = co2_ppm & 0xFF;
    return true;
}

void send_payload() {
    // Verificar se dados do GPS são válidos
    if (!gps.location.isValid()) {
        Serial.println("Sem fix GPS");
        return;
    }

    // Gerar payload combinado (GPS + DHT + PM2.5 + PM10)
    uint8_t payload[19];
    if (!my_payload(payload)) {
        Serial.println("Erro ao gerar payload");
    return;
    }

    // Enviar via LoRa
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println("Payload GPS + DHT + Sensores enviado!");
}

void send_onlySensors_payload() {
    sensors_event_t tempEvent, humEvent;
    dht.temperature().getEvent(&tempEvent);
    dht.humidity().getEvent(&humEvent);

    if (isnan(tempEvent.temperature) || isnan(humEvent.relative_humidity)) {
        Serial.println("Erro ao ler DHT22");
        return;
    }

    int16_t temp_int = round(tempEvent.temperature * 100);
    int16_t hum_int = round(humEvent.relative_humidity * 100);
    //Leitura HM3301
    uint16_t pm10 = sensor.atm.getPM10();
    uint16_t pm2_5 = sensor.atm.getPM2_5();
    //Leitura CO2
    float volts, percentage;
    //int percentage;       //only after 48 hours
    volts = MGRead(MG_PIN);
    percentage = volts*(-400/2.94)+800;
    uint16_t co2_ppm = percentage;

    uint8_t payload[10];
    payload[0] = (temp_int >> 8) & 0xFF;
    payload[1] = temp_int & 0xFF;
    payload[2] = (hum_int >> 8) & 0xFF;
    payload[3] = hum_int & 0xFF;
    payload[4] = (pm2_5 >> 8) & 0xFF;
    payload[5] = pm2_5 & 0xFF;
    payload[6] = (pm10 >> 8) & 0xFF;
    payload[7] = pm10 & 0xFF;
    payload[8] =(co2_ppm >> 8) & 0xFF;
    payload[9] = co2_ppm & 0xFF;

    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println("Payload Sensores enviado!");
    Serial.print("TEMP: ");
    Serial.println(temp_int);

    Serial.print("HUM: ");
    Serial.println(hum_int);
}



/*
void send_gps_payload() {
    if (!gps.location.isValid()) {
        Serial.println("Sem fix GPS");
        return;
    }
    int32_t lat = gps.location.lat() * 1e6;
    int32_t lng = gps.location.lng() * 1e6;
    uint8_t sats = gps.satellites.value();

    uint8_t payload[9];
    payload[0] = (lat >> 24) & 0xFF;
    payload[1] = (lat >> 16) & 0xFF;
    payload[2] = (lat >> 8) & 0xFF;
    payload[3] = lat & 0xFF;
    payload[4] = (lng >> 24) & 0xFF;
    payload[5] = (lng >> 16) & 0xFF;
    payload[6] = (lng >> 8) & 0xFF;
    payload[7] = lng & 0xFF;
    payload[8] = sats;

    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println("Payload GPS enviado!");
}
*/

void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }

    Serial.println(F("OP_TXRXPEND, sending ..."));

    unsigned long now = millis();

    if ((now - lastGpsSendTime) >= intervaloGps || lastGpsSendTime == 0) {
        if (gps.location.isValid()) {
            send_payload(); // envia GPS + DHT
            lastGpsSendTime = now;
        } else {
            Serial.println(F("GPS inválido, enviando só sensores"));
            send_onlySensors_payload();
        }
    } else {
        send_onlySensors_payload(); // envia só DHT durante os 2 minutos
    }

    if (u8g2) {
        char buf[256];
        u8g2->clearBuffer();
        snprintf(buf, sizeof(buf), "[%lu]data sending!", millis() / 1000);
        u8g2->drawStr(0, 12, buf);
        u8g2->sendBuffer();
    }
}
/* 
-------------------------------- DO SEND SÓ LoRa ------------------------------------------------------------
void do_send(osjob_t *j)
{
   /* if (joinStatus == EV_JOINING) {
        Serial.println(F("Not joined yet"));
        // Check if there is not a current TX/RX job running
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

    } else if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        Serial.println(F("OP_TXRXPEND,sending ..."));
        static uint8_t mydata[] = "Hello, world!";
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        if (u8g2) {
            char buf[256];
            u8g2->clearBuffer();
            snprintf(buf, sizeof(buf), "[%lu]data sending!", millis() / 1000);
            u8g2->drawStr(0, 12, buf);
            u8g2->sendBuffer();
        }
    }
*/

void onEvent (ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println(F("Received ack"));
            lora_msg =  "Received ACK.";
        }

        lora_msg = "rssi:" + String(LMIC.rssi) + " snr: " + String(LMIC.snr);

        if (LMIC.dataLen) {
            // data received in rx slot after tx
            Serial.print(F("Data Received: "));
            // Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            // Serial.println();
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_JOINING:
         Serial.println(F("EV_JOINING"));
        /*Serial.println(F("EV_JOINING: -> Joining..."));
        lora_msg = "OTAA joining....";
        joinStatus = EV_JOINING;

        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "OTAA joining....");
            u8g2->sendBuffer();
        }*/
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOINING_FAILED"));
        /*Serial.println(F("EV_JOIN_FAILED: -> Joining failed"));
        lora_msg = "OTAA Joining failed";
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "OTAA joining failed");
            u8g2->sendBuffer();
        }*/
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        /*Serial.println(F("EV_JOINED"));
        lora_msg = "Joined!";
        joinStatus = EV_JOINED;

        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "Joined TTN!");
            u8g2->sendBuffer();
        }
        delay(3);
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);*/

        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

u1_t readReg (u1_t addr)
{
    hal_pin_nss(0);
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    hal_pin_nss(1);
    return val;
}

uint32_t delayMS;

void setupLMIC(void)
{

    Serial.begin(115200);

    //GPS receiver
    hs.begin(9600, SERIAL_8N1, 34, 12);
    Serial.println(F("Iniciar GPS..."));

    //DHT22 sensor
    dht.begin();
    Serial.println(F("Iniciar DHT22 sensor..."));

    //for Dust Sensor
    initSystem();
    delay(100);
    if (!sensor.begin()) {
        Serial.println("Failed to initialize the sensor.");
        haltSystem();
    }
    Serial.println("Sensor initialized.");
    
    //for CO2
    pinMode(BOOL_PIN, INPUT);
    digitalWrite(BOOL_PIN, HIGH);


    //setupLMIC();
    #ifdef  RADIO_TCXO_ENABLE
        pinMode(RADIO_TCXO_ENABLE, OUTPUT);
        digitalWrite(RADIO_TCXO_ENABLE, HIGH);
    #endif

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    /*LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI); */     // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setAdrMode(0);
    LMIC_setDrTxpow(DR_SF7, 14);


    /*Serial.println("LMIC_startJoining");
    // Start job
    LMIC_startJoining();*/
    sensor_t sensor;
    delayMS = sensor.min_delay/1000;

    do_send(&sendjob);     // Will fire up also the join
}

void loopLMIC(void)
{
    while (hs.available()) {
        char c = hs.read();
        Serial.write(c);  // mostra dados brutos
        gps.encode(c);
    }

    if (gps.location.isUpdated()) {
        Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
        Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
    }
    if (millis() > 15000 && gps.charsProcessed() < 10) {
        Serial.println(F("No GPS detected: check wiring."));
        delay(15000);
    }

    State sensorState;
    if (!sensor.readSensor()) {
        Serial.println("Failed to read the sensor.");
        sensorState = State::BAD;

    if (failureStartTime) {
        // Restart the system if the sensor has not responded for a while
        if (millis() - failureStartTime > RESTART_TIMEOUT) {
        Serial.println("Sensor is not responding. Restarting...");
        restartSystem();
        }
    } else {
        failureStartTime = millis();  // Failure started
    }
    } else {
    sensorState = millis() < 30 * 1000
                        ? State::WARN  // Sensor needs 30 seconds to become stable
                        : State::GOOD;

    failureStartTime = 0;  // Clear failure
    }

    os_runloop_once();
}