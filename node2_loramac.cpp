/**
 * @file      node2_loramac.cpp
 * LMIC library only support SX1276 Radio
 */


#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "DFRobot_MultiGasSensor.h"
#include "LoRaBoards.h"
#include "utilities.h"

/*
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0xB0, 0x4C, 0x37, 0x11, 0x57, 0xC2, 0x7F, 0xF5, 0x6A, 0x7B, 0xD8, 0x90, 0xCD, 0x7E, 0x0E, 0xAB };
// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0x36, 0xEF, 0xC6, 0x91, 0x35, 0xD3, 0x21, 0xA5, 0x85, 0xF2, 0x04, 0xCE, 0x94, 0x1E, 0x9D, 0x6B };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x27FC038D; 
*/

// LoRaWAN NwkSKey, network session key NODE 1
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
static int spreadFactor = DR_SF7;
static int joinStatus = EV_JOINING;
static const unsigned TX_INTERVAL = 30;
static String lora_msg = "";
//Potência de 6dBm para ser o melhor valor possível

// ----------------------------------- GPS e Sensores ---------------------------------------------------------
// -------------------------- GPS ---------------------------
TinyGPSPlus gps;
HardwareSerial hs(1);

// -------------------------- O2 ----------------------------
#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x74
DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
#else
#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
/**
  UNO:pin_2-----RX
      pin_3-----TX
*/
  SoftwareSerial mySerial(2,3);
  DFRobot_GAS_SoftWareUart gas(&mySerial);
#else
/**
  ESP32:IO16-----RX
        IO17-----TX
*/
  DFRobot_GAS_HardWareUart gas(&Serial2); //ESP32HardwareSerial
#endif
#endif

// -------------------------- NO2 ---------------------------
int No2_pin = 35;

// -------------------------- CO ----------------------------
int CO_pin = 32;

// ----------------------------------- OTAA - Não necessário --------------------------------------------------
void os_getArtEui (u1_t *buf){}
void os_getDevEui (u1_t *buf){}
void os_getDevKey (u1_t *buf){}
// ------------------------------------------------------------------------------------------------------------

static uint8_t Payload[14];
//static uint8_t Payload[8];

/*void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        Serial.println(F("Sending GPS data..."));

        // Atualiza leitura do GPS
        while (Serial1.available() > 0) {
            gps.encode(Serial1.read());
        }

        if (gps.location.isValid()) {
            // Converte lat/lon em inteiros para compactar no payload (exemplo: graus * 10000)
            int32_t lat = (int32_t)(gps.location.lat() * 10000);
            int32_t lon = (int32_t)(gps.location.lng() * 10000);
            int16_t alt = (int16_t)gps.altitude.meters();

            // Coloca latitude (3 bytes) no payload (por simplicidade aqui usamos 4 bytes, mas pode ser 3 bytes com shift)
            Payload[0] = (lat >> 24) & 0xFF;
            Payload[1] = (lat >> 16) & 0xFF;
            Payload[2] = (lat >> 8) & 0xFF;
            Payload[3] = lat & 0xFF;

            // Longitude (4 bytes)
            Payload[4] = (lon >> 24) & 0xFF;
            Payload[5] = (lon >> 16) & 0xFF;
            Payload[6] = (lon >> 8) & 0xFF;
            Payload[7] = lon & 0xFF;

            //LMIC_setTxData2(1, Payload, 8, 0);
            //Serial.printf("Enviando GPS: lat=%.5f lon=%.5f alt=%.1f\n", gps.location.lat(), gps.location.lng());

        } else {
            Serial.println(F("GPS sem fix"));
            // Envia um payload vazio ou outro dado qualquer enquanto não tem fix
            //uint8_t noFixPayload[] = {0};
            //LMIC_setTxData2(1, noFixPayload, sizeof(noFixPayload), 0);
        }

        uint8_t o2_con = gas.readGasConcentrationPPM();
        Payload[8] = (o2_con >> 8) & 0xFF;
        Payload[9] = o2_con & 0xFF;

        uint8_t no2_value = 0;
        no2_value = analogRead(No2_pin);
        Payload[10] = (no2_value >> 8) & 0xFF;
        Payload[11] = no2_value & 0xFF;

        Serial.print("Ambient ");
        Serial.print(gas.queryGasType());
        Serial.print(" concentration is: ");
        Serial.print(gas.readGasConcentrationPPM());
        Serial.println(" %vol");
        Serial.print("No2 value: ");
        Serial.print(no2_value + "ppm");
        LMIC_setTxData2(1, Payload, sizeof(Payload), 0);
        Serial.printf("Enviando GPS: lat=%.5f lon=%.5f alt=%.1f\n", gps.location.lat(), gps.location.lng());

    }
}*/

void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return;
    }

    Serial.println(F("Preparing data..."));

    // Atualiza GPS
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    bool hasFix = gps.location.isValid();
    int32_t lat = 0;
    int32_t lon = 0;

    if (hasFix) {
        lat = gps.location.lat() * 10000;
        lon = gps.location.lng() * 10000;
    }

    // Preenche payload com zeros se não houver fix
    Payload[0] = (lat >> 24) & 0xFF;
    Payload[1] = (lat >> 16) & 0xFF;
    Payload[2] = (lat >> 8) & 0xFF;
    Payload[3] = lat & 0xFF;

    Payload[4] = (lon >> 24) & 0xFF;
    Payload[5] = (lon >> 16) & 0xFF;
    Payload[6] = (lon >> 8) & 0xFF;
    Payload[7] = lon & 0xFF;
///////////////////////////////////////////////////////////////////////

    // Leitura do O2
    uint8_t o2_con = 0;
    if (gas.begin()) {
        o2_con = gas.readGasConcentrationPPM();
    }
    Payload[8] = 0;
    Payload[9] = o2_con;
    //Payload[8]  = (o2_con >> 8) & 0xFF; // parte alta
    //Payload[9] = o2_con & 0xFF;

    // Leitura do NO2
    int no2_raw = analogRead(No2_pin); //leitura RAW
    float Vadc = no2_raw * (5 / 4095.0);  // para ESP32
    // Divisor de tensão:
    float R1 = 10000.0;
    float R2 = 6800.0;
    float Vsensor = Vadc * (R1 + R2) / R2;
    // Conversão para ppm (DFRobot SEN0574)
    float ppm = (Vsensor - 0.1) / 100;
    uint16_t No2_payload = (uint16_t)(ppm*1000);

    Payload[10] = (No2_payload >> 8) & 0xFF;
    Payload[11] = No2_payload & 0xFF;

    //Leitura CO
    int co_raw = analogRead(CO_pin);
    float co_adc = co_raw * (5/ 4095.0); 
    float CO_Vsensor = co_adc * (R1 + R2) / R2;
    float CO_ppm = (Vsensor - 0.1) / 100;
    uint16_t CO_payload = (uint16_t)(CO_ppm*1000);
    //uint16_t CO_payload = co_raw;

    Payload[12] = (CO_payload >> 8) & 0xFF;
    Payload[13] = CO_payload & 0xFF;


    // Verificação final antes de enviar
    Serial.print(F("Enviando payload: "));
    /*for (int i = 0; i < sizeof(Payload); i++) {
        Serial.print(Payload[i], HEX);
        Serial.println(" ");
    }*/
    /*Serial.print("Ambient ");
    Serial.print(gas.queryGasType());
    Serial.print(" concentration is: ");
    Serial.print(gas.readGasConcentrationPPM());
    Serial.println(" %vol");
    Serial.print("No2 value: ");
    Serial.print(ppm);
    Serial.print(" ppm");
    Serial.println();
    Serial.print("CO value: ");
    Serial.print(co_raw);
    Serial.print(" ppm");
    Serial.println();*/

    LMIC_setTxData2(1, Payload, sizeof(Payload), 0);
}

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

void setupLMIC(void)
{
    Serial.begin(115200);

    //GPS receiver
    hs.begin(9600, SERIAL_8N1, 34, 12);
    Serial.println(F("Iniciar GPS..."));

    //O2 Sensor
    gas.changeAcquireMode(gas.PASSIVITY);
    delay(1000);
    gas.setTempCompensation(gas.ON);


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
    LMIC_setDrTxpow(spreadFactor, 14);


    /*Serial.println("LMIC_startJoining");
    // Start job
    LMIC_startJoining();*/


    do_send(&sendjob);     // Will fire up also the join
}

void loopLMIC(void)
{
    os_runloop_once();
}