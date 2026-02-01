/*
MASTER 5.20.00090
*/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SH110X.h>

// Libreria per CANbus J1939 con controller STM32 integrato
#include <STM32_CAN.h>
/*#define i2c_Address 0x3c // I2C address for the OLED
#define SCREEN_WIDTH 128  // OLED display width
#define SCREEN_HEIGHT 64  // OLED display height
#define OLED_RESET -1     // OLED reset pin
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);*/


#include <TCA9555.h>
//Set i2c multiplexer address=============================================
TCA9555 multi_Input_20(0x20);
TCA9555 multi_Output_21(0x21);

#define INTERRUPT_PIN PB1 
volatile bool interruptFlag = false;

#include <HardwareSerial.h>
// Definizione della porta USART1 su PA9 (TX) e PA10 (RX)
HardwareSerial Serial1(PA10, PA9);
#define BAUD_RATE 19200  // 9600, 19200, 38400, 57600, 115200
#define INT_Pin PB0   // Pin per DE/RE
#define LED_ERROR PC15 // Pin per LED_ERROR
#define LED_Diagnosi PC13
#define LED_Diagnosi1 PC14
#define TIMEOUT 2000
#define TIMEOUT_SLAVE 1000
#define DELAY_BETWEEN_TRANSMISSION 40
#define DELAY_INT 100

#define RELE_DOWN PC0
#define RELE_UP PC1
#define RELE_RESUME PC2
#define RELE_PTO PC3
#define O_CE PB14
#define O_RADIO PB15
#define OUT_1 PB12
#define OUT_2 PB13

#define IDip0 PC4
#define IDip1 PC5
bool Dip0, Dip1;
int SLAVE_ID = 0;       // viene configurato con Dip0 e Dip1

int send = 0;
int update = 0;
byte skip = 0; 
bool skip3 = 0;
bool statusSL1 = false;
bool statusSL2 = false;
bool statusSL3 = false;
unsigned long currentMillis;
const int intervalON = 250;
const int intervalOFF = 250;
unsigned long previousLEDMillis = 0;
unsigned long lastReceivedTime = 0;
unsigned long lastSend = 0;
unsigned long slave1Time = 0;
unsigned long slave2Time = 0;
unsigned long slave3Time = 0;
int error = 0;

uint8_t ID = 0;
uint8_t receiveID = 0;

int nextion_pagina = 0;

bool bt1,bt2,bt3,bt4,bt5,bt6,bt7,bt8,bt9,bt10,bt11,bt12,bt13,bt14,bt15,bt16,bt17,bt18;
bool prev_bt1, prev_bt2, prev_bt3, prev_bt4, prev_bt5, prev_bt6, prev_bt7, prev_bt8, prev_bt9, prev_bt10, prev_bt11, prev_bt12;
bool prev_bt13, prev_bt14, prev_bt15, prev_bt16, prev_bt17, prev_bt18;

bool fari_tutti, fari_g_1, fari_g_2, strobo_tutti, bagagliera, strobo_anteriore, strobo_posteriori, luci_cassette;
bool pre_fari_tutti, pre_fari_g_1, pre_fari_g_2, pre_strobo_tutti, pre_bagagliera, pre_strobo_anteriore, pre_strobo_posteriori, pre_luci_cassette;

bool OUT1,OUT2,OUT3,OUT4,OUT5,OUT6,OUT7,OUT8,OUT9,OUT10,OUT11,OUT12,OUT13,OUT14,OUT15,OUT16,OUT17,OUT18;

bool STATO_PTO,ERRORE_PTO,STATO_OUT3,ERRORE_OUT3,STATO_OUT4,ERRORE_OUT4,STATO_OUT5,ERRORE_OUT5,STATO_OUT6,ERRORE_OUT6,STATO_OUT7,ERRORE_OUT7;
bool STATO_OUT8,ERRORE_OUT8,STATO_OUT9,ERRORE_OUT9,STATO_OUT10,ERRORE_OUT10,STATO_OUT11,ERRORE_OUT11,STATO_OUT12,ERRORE_OUT12,STATO_OUT13,ERRORE_OUT13;
bool STATO_OUT14,ERRORE_OUT14,STATO_OUT15,ERRORE_OUT15,STATO_OUT16,ERRORE_OUT16,STATO_OUT17,ERRORE_OUT17,STATO_OUT18,ERRORE_OUT18;

bool CONFIG_OUTPTO,CONFIG_OUT3,CONFIG_OUT4,CONFIG_OUT5,CONFIG_OUT6,CONFIG_OUT7,CONFIG_OUT8,CONFIG_OUT9,CONFIG_OUT10,CONFIG_OUT11,CONFIG_OUT12;
bool CONFIG_OUT13,CONFIG_OUT14,CONFIG_OUT15,CONFIG_OUT16,CONFIG_OUT17,CONFIG_OUT18;

bool MEM_OUT3,MEM_OUT4,MEM_OUT5,MEM_OUT6,MEM_OUT7,MEM_OUT8,MEM_OUT9,MEM_OUT10,MEM_OUT11,MEM_OUT12,MEM_OUT13,MEM_OUT14,MEM_OUT15,MEM_OUT16,MEM_OUT17,MEM_OUT18;

uint32_t btnMode = 0;
uint32_t btnMem = 0;

#define EEPROM_SIZE 64
#define EEPROM_ADDR_BTNMEM  0      // indirizzo iniziale
#define EEPROM_ADDR_BTNMODE 4      // subito dopo btnMem (4 byte)

// Valori di default
const uint32_t DEFAULT_BTNMEM = 0;
const uint32_t DEFAULT_BTNMODE = 52308;

bool BULBO_PTO,ACC_SU,ACC_GIU,KEY,HB,LP; 
bool PRE_BULBO_PTO, PRE_HB;

int count = 0;
int stato = 0;

int rit_verifica_bulbo = 5000;
int ritardo_inversione = 500;
int ritardo_spinta = 500;
int ritardo_resume = 6000;
int impulso_resume = 1500;

unsigned long timerOUT1 = 0;
unsigned long timerOUT2 = 0;
unsigned long timerSCAMBIO = 0;
unsigned long timerRESUME = 0;
unsigned long timerT_RESUME = 0;
unsigned long timerSpinta = 0;

// ========== CANbus J1939 - Definizioni ==========
// Pin CAN (hardware defined - non servono definizioni esplicite)
// PB8 = CAN1_RX (pin alternativo)
// PB9 = CAN1_TX (pin alternativo)

// PGN J1939 per Iveco Daily
#define PGN_VEHICLE_ELECTRICAL_POWER  0xFEE0  // 65280 - Stato luci principali
#define PGN_DASH_DISPLAY              0xFEEC  // 65276 - Display e segnalazioni
#define PGN_CAB_MESSAGE_1             0xFDBC  // 64972 - Messaggi cabina
#define PGN_LIGHTING_COMMAND          0xFEB5  // 65205 - Comandi luci
#define PGN_VEHICLE_POSITION          0xFEB1  // 65201 - Include frecce

// Struttura stato luci Iveco
struct IvecoLightsStatus {
    bool parking_lights;      // Luci di posizione
    bool low_beam;            // Anabbaglianti
    bool high_beam;           // Abbaglianti
    bool front_fog_lights;    // Fendinebbia anteriori
    bool rear_fog_lights;     // Fendinebbia posteriori
    bool left_turn_signal;    // Freccia sinistra
    bool right_turn_signal;   // Freccia destra
    bool hazard_warning;      // Emergenza/4 frecce
    bool brake_lights;        // Stop
    bool reverse_lights;      // Retromarcia
    bool work_lights;         // Luci da lavoro
    bool beacon;              // Lampeggiante/strobo
    unsigned long last_update; // Timestamp ultimo aggiornamento
};

// Variabili globali CANbus - STM32 integrato con SN65HVD230
STM32_CAN Can(CAN1, ALT);  // ALT per usare pin alternativi PB8/PB9
IvecoLightsStatus iveco_lights;

// Dichiarazione funzioni CANbus J1939
void setupCANbus();
uint32_t getPGN(uint32_t can_id);
void readJ1939Messages();
void processIvecoLights(uint32_t pgn, uint8_t* data, uint8_t len, uint8_t source);
bool isLightsDataValid();
void printLightsStatus();
void mapIvecoLightsToOutputs();
// ================================================


// Dichiarazione delle funzioni
void blinkLed();
void interruptPB1();
void handleInterrupt();
void gestOut();
void checkOut();
void attesa();
void fase1();
void fase2();
void fase3();
void fase4();
void fase5();
void fase6();
void fase7();
void fase8();
void fase9();
void fase10();

// -------------------------------------------

#define number_of_bits 40
#define number_of_ints 20

// Arrays for slave 1 data
bool slave1_bits[number_of_bits];
int slave1_ints[number_of_ints];

// Arrays for slave 2 data
bool slave2_bits[number_of_bits];
int slave2_ints[number_of_ints];

// Arrays for slave 2 data
bool slave3_bits[number_of_bits];
int slave3_ints[number_of_ints];

// Master data to broadcast
bool master_bits[number_of_bits];
int master_ints[number_of_ints];

unsigned long lastRequestTime = 0;
bool slave1Error = false;
bool slave2Error = false;
bool slave3Error = false;

uint16_t calculateCRC16(uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; // Valore iniziale
    uint16_t polynomial = 0x1021; // Polinomio CRC-CCITT

    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8; // XOR il byte corrente
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc & 0xFFFF; // Ritorna i 16 bit del CRC
}



void setup() {
    Wire.begin();
    multi_Input_20.begin();
    multi_Output_21.begin();
    // Set pinMode to INPUT
    for(int i=0;i<16;i++) {
        multi_Input_20.pinMode(i, INPUT);
    } 
    // Set pinMode to OUTPUT
    for(int i=0;i<16;i++) {
        multi_Output_21.pinMode(i, OUTPUT);  
    }  
    for(int i=7;i<16;i++) {
    multi_Output_21.digitalWrite(i, LOW); 
    }

    SystemClock_Config();
    
    // Inizializza CANbus J1939
    setupCANbus();
    
    //Serial.begin(115200);
    Serial1.begin(BAUD_RATE);
    pinMode(INT_Pin, OUTPUT);
    pinMode(LED_ERROR, OUTPUT);
    pinMode(LED_Diagnosi, OUTPUT);   
    pinMode(LED_Diagnosi1, OUTPUT); 
    pinMode(RELE_DOWN, OUTPUT);
    pinMode(RELE_UP, OUTPUT);   
    pinMode(RELE_RESUME, OUTPUT);
    pinMode(RELE_PTO, OUTPUT);      

    pinMode(O_CE, OUTPUT);
    pinMode(O_RADIO, OUTPUT);
    pinMode(OUT_1, OUTPUT);
    pinMode(OUT_2, OUTPUT);    

    digitalWrite(INT_Pin, LOW);
    digitalWrite(LED_Diagnosi, LOW);    
    digitalWrite(LED_Diagnosi1, LOW);
    digitalWrite(LED_ERROR, LOW);

    digitalWrite(RELE_DOWN, LOW);
    digitalWrite(RELE_UP, LOW);   
    digitalWrite(RELE_RESUME, LOW);
    digitalWrite(RELE_PTO, LOW); 

    digitalWrite(O_CE, LOW);
    digitalWrite(O_RADIO, LOW);
    digitalWrite(OUT_1, LOW);
    digitalWrite(OUT_2, LOW); 

    pinMode(IDip0, INPUT);
    pinMode(IDip1, INPUT);

    Dip0 = digitalRead(IDip0);
    Dip1 = digitalRead(IDip1); 

    if(Dip0 and Dip1) SLAVE_ID = 0;

    pinMode(INTERRUPT_PIN, INPUT); 
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptPB1, FALLING); // Interrupt su FALLING fronte di discesa / RISING fronte di salita
    interruptFlag = false;

    // Configurazione priorità NVIC
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);  // Priorità massima per USART1
    //HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);   // Priorità inferiore per PB0
    HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 1);   // Priorità inferiore per PB1
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    //HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);


    //TEST LED e uscite
    int ritardo = 15;
    // Set pinMode to OUTPUT
    for(int i=0;i<7;i++) {
    multi_Output_21.digitalWrite(i, LOW); 
    multi_Output_21.digitalWrite(i-1, HIGH);     
    delay(ritardo);
    }

    multi_Output_21.digitalWrite(6, HIGH); 

    for(int i=7;i<16;i++) {
    multi_Output_21.digitalWrite(i, LOW); 
    }

    digitalWrite(LED_Diagnosi, HIGH); 
    delay(ritardo);   
    digitalWrite(LED_Diagnosi, LOW);    
    digitalWrite(LED_Diagnosi1, HIGH);
    delay(ritardo);  
    digitalWrite(LED_Diagnosi1, LOW);
    digitalWrite(LED_ERROR, HIGH);  
    delay(ritardo);     
    digitalWrite(LED_ERROR, LOW); 


    digitalWrite(OUT_2, HIGH); 
    delay(ritardo);
    digitalWrite(OUT_2, LOW);    
    digitalWrite(OUT_1, HIGH); 
    delay(ritardo);
    digitalWrite(OUT_1, LOW);    


    multi_Output_21.digitalWrite(14, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(14, LOW);     

    multi_Output_21.digitalWrite(11, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(11, LOW);   

    multi_Output_21.digitalWrite(10, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(10, LOW);   

    multi_Output_21.digitalWrite(12, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(12, LOW);     

    multi_Output_21.digitalWrite(13, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(13, LOW);  

    multi_Output_21.digitalWrite(7, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(7, LOW);  

    multi_Output_21.digitalWrite(9, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(9, LOW);  

    multi_Output_21.digitalWrite(8, HIGH); 
    delay(ritardo);
    multi_Output_21.digitalWrite(8, LOW); 


    digitalWrite(O_CE, HIGH); 
    delay(ritardo);
    digitalWrite(O_CE, LOW);    
    digitalWrite(O_RADIO, HIGH); 
    delay(ritardo);
    digitalWrite(O_RADIO, LOW);   

    OUT1 = 0;
    OUT2 = 0;   
    OUT3 = 0;
    OUT4 = 0;
    OUT5 = 0;   
    OUT6 = 0;
    OUT7 = 0;
    OUT8 = 0;   
    OUT9 = 0;
    OUT10 = 0;
    OUT11 = 0;
    OUT12 = 0;   
    OUT13 = 0;
    OUT14 = 0;
    OUT15 = 0;   
    OUT16 = 0;
    OUT17 = 0;
    OUT18 = 0; 
    stato = 0;
    gestOut();

    // Verifica se EEPROM è "vergine" (4 byte tutti a 0xFF)
    uint32_t checkVal;
    EEPROM.get(EEPROM_ADDR_BTNMEM, checkVal);
    if (checkVal == 0xFFFFFFFF) {
        // EEPROM vergine: imposta valori di default e salva
        btnMem = DEFAULT_BTNMEM;
        btnMode = DEFAULT_BTNMODE;
        EEPROM.put(EEPROM_ADDR_BTNMEM, btnMem);
        EEPROM.put(EEPROM_ADDR_BTNMODE, btnMode);
      } else {
        // EEPROM già inizializzata: leggi i valori salvati
        EEPROM.get(EEPROM_ADDR_BTNMEM, btnMem);
        EEPROM.get(EEPROM_ADDR_BTNMODE, btnMode);
      }
      // stesse righe anche in receiveslavedata
        CONFIG_OUTPTO = (btnMode >> 0) & 1;
        CONFIG_OUT3   = (btnMode >> 1) & 1;
        CONFIG_OUT4   = (btnMode >> 2) & 1;
        CONFIG_OUT5   = (btnMode >> 3) & 1;
        CONFIG_OUT6   = (btnMode >> 4) & 1;
        CONFIG_OUT7   = (btnMode >> 5) & 1;
        CONFIG_OUT8   = (btnMode >> 6) & 1;
        CONFIG_OUT9   = (btnMode >> 7) & 1;
        CONFIG_OUT10  = (btnMode >> 8) & 1;
        CONFIG_OUT11  = (btnMode >> 9) & 1;
        CONFIG_OUT12  = (btnMode >> 13) & 1;
        CONFIG_OUT13  = (btnMode >> 10) & 1;
        CONFIG_OUT14  = (btnMode >> 14) & 1;
        CONFIG_OUT15  = (btnMode >> 11) & 1;
        CONFIG_OUT16  = (btnMode >> 15) & 1;
        CONFIG_OUT17  = (btnMode >> 12) & 1;
        CONFIG_OUT18  = (btnMode >> 16) & 1;
}


void requestSlaveData(uint8_t slaveId) {
        uint16_t crc = calculateCRC16(&slaveId, 1);    
        digitalWrite(INT_Pin, HIGH);
        delayMicroseconds(DELAY_INT);
        Serial1.write(slaveId);
        //uint16_t crc = calculateCRC16(&slaveId, 1);
        Serial1.write((uint8_t)(crc >> 8));
        Serial1.write((uint8_t)(crc & 0xFF));
        Serial1.flush();
        delayMicroseconds(DELAY_INT); 
        digitalWrite(INT_Pin, LOW);
}


void receiveSlaveData() {
    const size_t dataSize = number_of_bits / 8 + number_of_ints * 2;
    uint8_t buffer[dataSize];
    if (Serial1.available() >= dataSize + 3) { 
    receiveID = Serial1.read();  
    ID = receiveID;    
    Serial1.readBytes(buffer, dataSize); 
    uint16_t receivedCRC = (Serial1.read() << 8) | Serial1.read();
    uint16_t calculatedCRC = calculateCRC16(buffer, dataSize);

    while (Serial1.available()) {
      byte temp = Serial1.read();
    }

    if (receivedCRC != calculatedCRC) {
            return;           
        }
        else {
            bool* bits = nullptr;
            int* ints = nullptr;
            if (receiveID == 1) {
                bits = slave1_bits;
                ints = slave1_ints;
                slave1Error = false;
                slave1Time = currentMillis;
                lastReceivedTime = currentMillis;
            } else if (receiveID == 2) {
                bits = slave2_bits;
                ints = slave2_ints;
                slave2Error = false; 
                slave2Time = currentMillis; 
                lastReceivedTime = currentMillis;             
            } else if (receiveID == 3) {
                bits = slave3_bits;
                ints = slave3_ints;
                slave3Error = false; 
                slave3Time = currentMillis; 
                lastReceivedTime = currentMillis;            
            } 
            // Spacchetta i byte in bit
            for (int i = 0; i < number_of_bits; i++) {
                int byteIndex = i / 8;
                int bitIndex = 7 - (i % 8);
                bits[i] = (buffer[byteIndex] >> bitIndex) & 0x01;
            }
            // Spacchetta gli int
            for (int i = 0; i < number_of_ints; i++) {
                ints[i] = (int16_t)((buffer[number_of_bits/8 + i * 2] << 8) | buffer[number_of_bits/8 + i * 2 + 1]);
            }
            bt1 = slave1_bits[0]; 
            bt2 = slave1_bits[1]; 
            bt3 = slave1_bits[2]; 
            bt4 = slave1_bits[3]; 
            bt5 = slave1_bits[4];   
            bt6 = slave1_bits[5];  
            bt7 = slave1_bits[6]; 
            bt8 = slave1_bits[7]; 
            bt9 = slave1_bits[8];  
            bt10 = slave1_bits[9]; 
            bt11 = slave1_bits[10];
            bt12 = slave1_bits[11];
            bt13 = slave1_bits[12];
            bt14 = slave1_bits[13];
            bt15 = slave1_bits[14];
            bt16 = slave1_bits[15];
            bt17 = slave1_bits[16];
            bt18 = slave1_bits[17];   

            fari_tutti = slave2_bits[2];
            fari_g_1 = slave2_bits[3];
            fari_g_2 = slave2_bits[4];
            strobo_tutti = slave2_bits[5];
            bagagliera = slave2_bits[6];
            strobo_anteriore = slave2_bits[7]; 
            strobo_posteriori = slave2_bits[8];
            luci_cassette = slave2_bits[9];

            nextion_pagina = slave1_ints[4];
            master_ints[4] = slave1_ints[4];

            if(slave1_bits[37]){
                btnMem = ((uint32_t)slave1_ints[3] << 16) | (uint16_t)slave1_ints[2]; 
                EEPROM.put(EEPROM_ADDR_BTNMEM, btnMem);             
            } 
            if(slave1_bits[38]){
                btnMode = ((uint32_t)slave1_ints[1] << 16) | (uint16_t)slave1_ints[0];     
                EEPROM.put(EEPROM_ADDR_BTNMODE, btnMode);
                // stesse righe anche in setup
                CONFIG_OUTPTO = (btnMode >> 0) & 1;
                CONFIG_OUT3   = (btnMode >> 1) & 1;
                CONFIG_OUT4   = (btnMode >> 2) & 1;
                CONFIG_OUT5   = (btnMode >> 3) & 1;
                CONFIG_OUT6   = (btnMode >> 4) & 1;
                CONFIG_OUT7   = (btnMode >> 5) & 1;
                CONFIG_OUT8   = (btnMode >> 6) & 1;
                CONFIG_OUT9   = (btnMode >> 7) & 1;
                CONFIG_OUT10  = (btnMode >> 8) & 1;
                CONFIG_OUT11  = (btnMode >> 9) & 1;
                CONFIG_OUT12  = (btnMode >> 13) & 1;
                CONFIG_OUT13  = (btnMode >> 10) & 1;
                CONFIG_OUT14  = (btnMode >> 14) & 1;
                CONFIG_OUT15  = (btnMode >> 11) & 1;
                CONFIG_OUT16  = (btnMode >> 15) & 1;
                CONFIG_OUT17  = (btnMode >> 12) & 1;
                CONFIG_OUT18  = (btnMode >> 16) & 1;
            }   
        } 
        return;   
    }
    else{  

        // Aspetta un attimo prima di abortire
        delay(2); 
        if (Serial1.available() < dataSize + 3) return;
    }
}

void broadcastMasterData() {
    /*update++;   // usato per generare dati da scambiare
    if(update >=50){
        update = 0;
        for( int x=0; x<number_of_bits; x++)
        {
        master_bits[x] = random(0, 2); 
        }
        for( int x=0; x<number_of_ints; x++)
        {
        master_ints[x] = random(-1000, 1000); 
        }
    } */
    const size_t dataSize = number_of_bits/8 + number_of_ints*2; 
    uint8_t buffer[dataSize];
    // Impacchetta i bit in byte
    for (int i = 0; i < number_of_bits/8; i++) {
        buffer[i] = 0;
        for (int bit = 0; bit < 8; bit++) {
            int index = i * 8 + bit;
            if (index < number_of_bits) {
                buffer[i] |= (master_bits[index] << (7 - bit)); // MSB first
            }
        }
    }
    master_ints[0] = btnMode & 0xFFFF;            // parte bassa (bit 0–15)
    master_ints[1] = (btnMode >> 16) & 0xFFFF;    // parte alta (bit 16–31)
    //master_ints[2] =
    //master_ints[3] =
    //master_ints[4] = dichiarato nella parte di ricezione

    // Impacchetta gli int in 2 byte ciascuno
    for (int i = 0; i < number_of_ints; i++) {
        int16_t value = (int16_t)master_ints[i]; // Cast esplicito
        buffer[number_of_bits/8 + i*2] = (value >> 8) & 0xFF; // Byte alto
        buffer[number_of_bits/8 + i*2 + 1] = value & 0xFF;    // Byte basso
    }

    digitalWrite(INT_Pin, HIGH);
    digitalWrite(LED_Diagnosi1, HIGH); 
    delayMicroseconds(DELAY_INT);

    uint8_t broadcast = 0xFF;
    uint16_t crc1 = calculateCRC16(&broadcast, 1);
    uint16_t crc = calculateCRC16(buffer, dataSize);

    Serial1.write(broadcast); // Broadcast address 
    //uint16_t crc1 = calculateCRC16(&broadcast, 1);
    Serial1.write((uint8_t)(crc1 >> 8));
    Serial1.write((uint8_t)(crc1 & 0xFF));

    Serial1.write(buffer, dataSize);
    //uint16_t crc = calculateCRC16(buffer, dataSize);
    Serial1.write((uint8_t)(crc >> 8));
    Serial1.write((uint8_t)(crc & 0xFF));
    Serial1.flush();
    //delayMicroseconds(DELAY_INT); 
    digitalWrite(INT_Pin, LOW);
    digitalWrite(LED_Diagnosi1, LOW); 
}


void loop() {
    currentMillis = millis();
    blinkLed();  
    handleInterrupt();

    // Leggi messaggi J1939 dal bus CAN Iveco
    readJ1939Messages();

    // Mappa le luci Iveco ai tuoi output (opzionale)
    mapIvecoLightsToOutputs();

    if (currentMillis - lastSend >= DELAY_BETWEEN_TRANSMISSION) {
            lastSend = currentMillis;
            switch (send) {
                case 0: //
                    requestSlaveData(1);
                    receiveSlaveData();                                 
                    break;

                case 1: // 
                    requestSlaveData(2);
                    receiveSlaveData();                  
                    break;

                case 2:  
                    requestSlaveData(3);
                    receiveSlaveData(); 
                    if(!slave1Error) gestOut();                                                                             
                    break;

                case 3: 
                    checkOut();     
                    broadcastMasterData(); 
                    if(slave1Error){
                        digitalWrite(LED_ERROR, HIGH); 
                    }
                    else{
                        digitalWrite(LED_ERROR, LOW); 
                    }                                 
                    break;

                default:
                    break;
            }
            send = (send + 1) % 4;
    }


    /*if (currentMillis - lastReceivedTime > TIMEOUT) {
        // Riavvia Serial1
        error++;
        Serial1.end();     
        delay(50);      
        Serial1.begin(BAUD_RATE); 
        lastReceivedTime = millis();
        lastSend = millis();
    }*/

    if (currentMillis - lastReceivedTime > TIMEOUT) {
        while (Serial1.available()) Serial1.read(); // Svuota il buffer seriale
        error++;
        lastReceivedTime = millis();
        lastSend = millis();
    }

    if ((currentMillis - slave1Time > TIMEOUT_SLAVE)) {
        slave1Error = true;

        digitalWrite(OUT_1, LOW);         
        digitalWrite(OUT_2, LOW);    
        multi_Output_21.digitalWrite(14, LOW);     
        multi_Output_21.digitalWrite(11, LOW);   
        multi_Output_21.digitalWrite(10, LOW);   
        multi_Output_21.digitalWrite(12, LOW);     
        multi_Output_21.digitalWrite(13, LOW);  
        multi_Output_21.digitalWrite(7, LOW);  
        multi_Output_21.digitalWrite(9, LOW);  
        multi_Output_21.digitalWrite(8, LOW); 
        digitalWrite(O_CE, LOW);    
        digitalWrite(O_RADIO, LOW);  

        OUT1 = LOW;
        OUT2 = LOW;
        OUT3 = LOW;
        OUT4 = LOW;
        OUT5 = LOW;
        OUT6 = LOW;
        OUT7 = LOW;
        OUT8 = LOW;       
        OUT9 = LOW;
        OUT10 = LOW;
        OUT11 = LOW;
        OUT12 = LOW;       
        OUT13 = LOW;
        OUT14 = LOW;
        OUT15 = LOW;
        OUT16 = LOW;        

        for (int i = 0; i < number_of_bits; i++) {
            slave1_bits[i] = 0;
        }
        for (int i = 0; i < number_of_ints; i++) {
            slave1_ints[i] = 0;
        }

        for (int i = 0; i < number_of_bits; i++) {
            master_bits[i] = 0;
        }
    }

    if (currentMillis - slave2Time > TIMEOUT_SLAVE) {
        slave2Error = true;
        for (int i = 0; i < number_of_bits; i++) {
            slave2_bits[i] = 0;
        }
        for (int i = 0; i < number_of_ints; i++) {
            slave2_ints[i] = 0;
        }
    }

    if (currentMillis - slave3Time > TIMEOUT_SLAVE) {
        slave3Error = true;
        for (int i = 0; i < number_of_bits; i++) {
            slave3_bits[i] = 0;
        }
        for (int i = 0; i < number_of_ints; i++) {
            slave3_ints[i] = 0;
        }
    }   

    // Debug: stampa stato luci ogni 5 secondi (opzionale - decommentare Serial.begin per abilitare)
    /*
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 5000) {
        printLightsStatus();
        lastPrint = millis();
    }
    */
     
    switch(stato){
        case 0:
          attesa();
          break;
        case 1:
          //
          fase1();
          break;
        case 2:
        //
          fase2();     
          break; 
        case 3:
        //
          fase3();     
          break; 
        case 4:
        //
          fase4();     
          break; 
        case 5:
        //
          fase5();     
          break; 
        case 6:
        //
            fase6();     
            break; 
        case 7:
        //
            fase7();     
            break; 
        case 8:
        //
            fase8();     
            break; 
        case 9:
        //
            fase9();     
            break; 
        case 10:
        //
            fase10();     
            break; 
      }   
}


void interruptPB1() {  
      interruptFlag = true;
}

void handleInterrupt() {
      if(interruptFlag){
      //Leggo gli ingressi fisici della scheda  
      BULBO_PTO = !multi_Input_20.digitalRead(0);
      ACC_SU = !multi_Input_20.digitalRead(1);
      ACC_GIU = !multi_Input_20.digitalRead(2);
      KEY = !multi_Input_20.digitalRead(3);
      HB = !multi_Input_20.digitalRead(4);
      LP = !multi_Input_20.digitalRead(5);

      //Led di segnalazione per gli ingressi
      multi_Output_21.digitalWrite(0, !KEY); 
      multi_Output_21.digitalWrite(1, !HB); 
      multi_Output_21.digitalWrite(4, !BULBO_PTO); 
      multi_Output_21.digitalWrite(5, !LP); 
      multi_Output_21.digitalWrite(15, LP);

      //Attivo le uscite locali
      digitalWrite(RELE_PTO,BULBO_PTO); 
      digitalWrite(O_CE,BULBO_PTO);      
      digitalWrite(O_RADIO,BULBO_PTO);

      //Setto i bit per la trasmissione broadcast
      master_bits[35] = ACC_SU;
      master_bits[36] = ACC_GIU;
      master_bits[37] = KEY;
      master_bits[38] = HB;
      master_bits[39] = LP;

      if((!PRE_BULBO_PTO && BULBO_PTO) && stato == 0) {
        timerRESUME = millis();
        stato = 6;
      }  

      if((PRE_HB && !HB) && BULBO_PTO){
        if(CONFIG_OUTPTO == 0){
                OUT1 = LOW;    
                OUT2 = HIGH;
                timerOUT2 = millis();
                stato = 2;
        }
        else{
                OUT1 = LOW;
                OUT2 = LOW; 
                if(BULBO_PTO){
                    timerOUT2 = millis();
                    stato = 4;
                }           
        }        
      }


      PRE_BULBO_PTO = BULBO_PTO;
      PRE_HB = HB;

      interruptFlag = false;

      }
}

void gestOut(){

            if(nextion_pagina > 1) {
                digitalWrite(OUT_1, LOW);         
                digitalWrite(OUT_2, LOW);    
                multi_Output_21.digitalWrite(14, LOW);     
                multi_Output_21.digitalWrite(11, LOW);   
                multi_Output_21.digitalWrite(10, LOW);   
                multi_Output_21.digitalWrite(12, LOW);     
                multi_Output_21.digitalWrite(13, LOW);  
                multi_Output_21.digitalWrite(7, LOW);  
                multi_Output_21.digitalWrite(9, LOW);  
                multi_Output_21.digitalWrite(8, LOW); 
                digitalWrite(O_CE, LOW);    
                digitalWrite(O_RADIO, LOW);  
                OUT1 = LOW;
                OUT2 = LOW;
                OUT3 = LOW;
                OUT4 = LOW;
                OUT5 = LOW;
                OUT6 = LOW;
                OUT7 = LOW;
                OUT8 = LOW;       
                OUT9 = LOW;
                OUT10 = LOW;
                OUT11 = LOW;
                OUT12 = LOW;       
                OUT13 = LOW;
                OUT14 = LOW;
                OUT15 = LOW;
                OUT16 = LOW;  
                multi_Output_21.digitalWrite(15, LOW);      
              
                for (int i = 0; i < number_of_bits; i++) {
                    master_bits[i] = 0;
                }

            stato = 0;
            return;
            }

            if(ACC_SU or slave2_bits[0]){ 
                multi_Output_21.digitalWrite(2, LOW); 
                digitalWrite(RELE_UP, HIGH); 
            }
            else{
                multi_Output_21.digitalWrite(2, HIGH);
                digitalWrite(RELE_UP, LOW); 
            }
        
            if(ACC_GIU or slave2_bits[1]){ 
                multi_Output_21.digitalWrite(3, LOW); 
                digitalWrite(RELE_DOWN, HIGH);
            }
            else{
                multi_Output_21.digitalWrite(3, HIGH);
                digitalWrite(RELE_DOWN, LOW);
            }
        
            // Gestione uscita PTO      
            if(bt1 && !prev_bt1){ //&& CONFIG_OUTPTO == 0){
                if(BULBO_PTO) return;
                if(CONFIG_OUTPTO == 0){ // inversione
                    if(KEY && HB){ 
                    ERRORE_PTO = 0;       
                    OUT1 = HIGH;
                    OUT2 = LOW;
                    timerOUT1 = millis();
                    stato = 1;
                    }
                    else{
                        OUT1 = LOW; 
                        OUT2 = LOW; 
                        ERRORE_PTO = 1;    
                    }                                       
                }    
                else{  // sempre ON
                    if(KEY && HB){ 
                        ERRORE_PTO = 0;       
                        OUT1 = HIGH;
                        OUT2 = LOW;
                        timerOUT1 = millis();  
                        stato = 3;  
                    }    
                    else{
                        OUT1 = LOW;
                        OUT2 = LOW;  
                        ERRORE_PTO = 1;    
                    }   
                }    
            }


            if(bt2 && !prev_bt2){
                ERRORE_PTO = 0;
                if(!BULBO_PTO) return;
                if(CONFIG_OUTPTO == 0){
                    if(KEY && HB){
                        OUT1 = LOW;    
                        OUT2 = HIGH;
                        timerOUT2 = millis();
                        stato = 2;
                    }
                    else{
                        OUT1 = LOW;
                        OUT2 = LOW;  
                        ERRORE_PTO = 1;  
                    }                                       
                }
                else{                  
                    OUT1 = LOW;
                    OUT2 = LOW; 
                    if(BULBO_PTO){
                        timerOUT2 = millis();
                        stato = 4;
                    }
                }
            }     
        
            //Gestione uscite OUT3
            if (CONFIG_OUT3 == 1) {
                OUT3 = bt3; // momentaneo
            } else {
                if ((bt3 && !prev_bt3) || (bagagliera && !pre_bagagliera)) {
                OUT3 = !OUT3; // toggle su fronte di salita
                }
            }
        
            //Gestione uscite OUT4
            if (CONFIG_OUT4 == 1) {
                OUT4 = bt4; // momentaneo
            } else {
                if (bt4 && !prev_bt4) {
                OUT4 = !OUT4; // toggle su fronte di salita
                }
            }
        
            //Gestione uscite OUT5
            if (CONFIG_OUT5 == 1) {
                OUT5 = bt5; // momentaneo
            } else {
                if ((bt5 && !prev_bt5) || (strobo_anteriore && !pre_strobo_anteriore)) {
                OUT5 = !OUT5; // toggle su fronte di salita
                }
            }
        
            //Gestione uscite OUT6
            if (CONFIG_OUT6 == 1) {
                OUT6 = bt6; // momentaneo
            } else {
                if (bt6 && !prev_bt6) {
                OUT6 = !OUT6; // toggle su fronte di salita
                }
            }
        
            //Gestione uscite OUT7
            if (CONFIG_OUT7 == 1) {
                OUT7 = bt7; // momentaneo
                } else {
                if ((bt7 && !prev_bt7) || (strobo_posteriori && ! pre_strobo_posteriori)){
                    OUT7 = !OUT7; // toggle su fronte di salita
                }
            }
        
            //Gestione uscite OUT8
                if (CONFIG_OUT8 == 1) {
                OUT8 = bt8; // momentaneo
                } else {
                if (bt8 && !prev_bt8) {
                    OUT8 = !OUT8; // toggle su fronte di salita
                }
            } 
            
            //Gestione uscite OUT9
                if (CONFIG_OUT9 == 1) {
                OUT9 = bt9; // momentaneo
                } else {
                if ((bt9 && !prev_bt9) || (luci_cassette && !pre_luci_cassette)){
                    OUT9 = !OUT9; // toggle su fronte di salita
                }
            }  
        
            //Gestione uscite OUT10
                if (CONFIG_OUT10 == 1) {
                OUT10 = bt10; // momentaneo
                } else {
                if (bt10 && !prev_bt10) {
                    OUT10 = !OUT10; // toggle su fronte di salita
                }
            } 

            //Gestione uscite OUT11
            if (CONFIG_OUT11 == 1) {
                OUT11 = bt11; // momentaneo
                } else {
                if (bt11 && !prev_bt11) {
                    OUT11 = !OUT11; // toggle su fronte di salita
                }
            } 

            //Gestione uscite OUT12
            if (CONFIG_OUT12 == 1) {
                OUT12 = bt12; // momentaneo
                } else {
                if (bt12 && !prev_bt12) {
                    OUT12 = !OUT12; // toggle su fronte di salita
                }
            }            

            //Gestione uscite OUT13
            if (CONFIG_OUT13 == 1) {
                OUT13 = bt13; // momentaneo
                } else {
                if (bt13&& !prev_bt13) {
                    OUT13 = !OUT13; // toggle su fronte di salita
                }
            }              

            //Gestione uscite OUT14
            if (CONFIG_OUT14 == 1) {
                OUT14 = bt14; // momentaneo
                } else {
                if (bt14 && !prev_bt14) {
                    OUT14 = !OUT14; // toggle su fronte di salita
                }
            }

            //Gestione uscite OUT15
            if (CONFIG_OUT15 == 1) {
                OUT15 = bt15; // momentaneo
                } else {
                if (bt15 && !prev_bt15) {
                    OUT15 = !OUT15; // toggle su fronte di salita
                }
            }            
        
            //Gestione uscite OUT16
            if (CONFIG_OUT16 == 1) {
                OUT16 = bt16; // momentaneo
                } else {
                if (bt16 && !prev_bt16) {
                    OUT16 = !OUT16; // toggle su fronte di salita
                }
            } 

            //Gestione uscite OUT17
            if (CONFIG_OUT17 == 1) {
                OUT17 = bt17; // momentaneo
                } else {
                if (bt17 && !prev_bt17) {
                    OUT17 = !OUT17; // toggle su fronte di salita
                }
            } 

            //Gestione uscite OUT18
            if (CONFIG_OUT18 == 1) {
                OUT18 = bt18; // momentaneo
                } else {
                if (bt18 && !prev_bt18) {
                    OUT18 = !OUT18; // toggle su fronte di salita
                }
            }   
            
            

            //Gestione gruppi tasti
            if (fari_tutti && !pre_fari_tutti){
                if(OUT11 && OUT12 && OUT13 && OUT14){
                    OUT11 = 0;
                    OUT12 = 0;
                    OUT13 = 0;
                    OUT14 = 0;
                }
                else if(!OUT11 && !OUT12 && !OUT13 && !OUT14){
                    OUT11 = 1;
                    OUT12 = 1;
                    OUT13 = 1;
                    OUT14 = 1;                   
                }
                else{
                    OUT11 = 1;
                    OUT12 = 1;
                    OUT13 = 1;
                    OUT14 = 1;                   
                }
            }

            if (fari_g_1 && !pre_fari_g_1){
                if(OUT11 && OUT12){
                    OUT11 = 0;
                    OUT12 = 0;
                }
                else if(!OUT11 && !OUT12){
                    OUT11 = 1;
                    OUT12 = 1;             
                }
                else{
                    OUT11 = 1;
                    OUT12 = 1;                 
                }
            }

            if (fari_g_2 && !pre_fari_g_2){
                if(OUT13 && OUT14){
                    OUT13 = 0;
                    OUT14 = 0;
                }
                else if(!OUT13 && !OUT14){
                    OUT13 = 1;
                    OUT14 = 1;             
                }
                else{
                    OUT13 = 1;
                    OUT14 = 1;                 
                }
            } 
            
            if (strobo_tutti && !pre_strobo_tutti){
                if(OUT3 && OUT5 && OUT7){
                    OUT3 = 0;
                    OUT5 = 0;
                    OUT7 = 0;                   
                }
                else if(!OUT3 && !OUT5 && !OUT7){
                    OUT3 = 1;
                    OUT5 = 1;
                    OUT7 = 1;            
                }
                else{
                    OUT3 = 1;
                    OUT5 = 1;
                    OUT7 = 1;                 
                }
            }             
       
            digitalWrite(OUT_1, OUT1);
            digitalWrite(OUT_2, OUT2);   
            multi_Output_21.digitalWrite(7, OUT3);
            multi_Output_21.digitalWrite(8, OUT4);
            multi_Output_21.digitalWrite(9, OUT5);
            multi_Output_21.digitalWrite(10, OUT6);
            multi_Output_21.digitalWrite(11, OUT7);
            multi_Output_21.digitalWrite(12, OUT8);
            multi_Output_21.digitalWrite(13, OUT9);
            multi_Output_21.digitalWrite(14, OUT10);

            // Scheda aggiuntiva SLAVE 2
            master_bits[18] =  OUT11;
            master_bits[20] =  OUT12;
            master_bits[22] =  OUT13;
            master_bits[24] =  OUT14;   
            master_bits[26] =  OUT15;   
            master_bits[28] =  OUT16;


            // Aggiorna stato precedente
            prev_bt1 = bt1;
            prev_bt2 = bt2;
            prev_bt3 = bt3;
            prev_bt4 = bt4;
            prev_bt5 = bt5;
            prev_bt6 = bt6;
            prev_bt7 = bt7;
            prev_bt8 = bt8;
            prev_bt9 = bt9;
            prev_bt10 = bt10;
            prev_bt11 = bt11;
            prev_bt12 = bt12;
            prev_bt13 = bt13;
            prev_bt14 = bt14;
            prev_bt15 = bt15;
            prev_bt16 = bt16;
            prev_bt17 = bt17;
            prev_bt18 = bt18;

            pre_fari_tutti = fari_tutti;
            pre_fari_g_1 = fari_g_1;
            pre_fari_g_2 = fari_g_2;
            pre_strobo_tutti = strobo_tutti;
            pre_bagagliera = bagagliera;
            pre_strobo_anteriore = strobo_anteriore; 
            pre_strobo_posteriori = strobo_posteriori;
            pre_luci_cassette = luci_cassette;
}


void attesa(){
 return;
}

void fase1(){
    if(BULBO_PTO){
        timerSpinta = millis();
        stato = 8;  
    }  
    else{
        if (millis() - timerOUT1 >= rit_verifica_bulbo){
            OUT1 = LOW;  
            ERRORE_PTO = 1;   
            timerSCAMBIO = millis();
            stato = 5;      // devo fare inversione 
        }        
    }
}

void fase2(){
    if(!BULBO_PTO){
        timerSpinta = millis();
        stato = 9;
    }  
    else{
        if (millis() - timerOUT2 >= rit_verifica_bulbo){
            OUT2 = LOW;  
            ERRORE_PTO = 1;   
            stato = 0;       
        }        
    }
}

void fase3(){
    if(bt2 && !prev_bt2){
        if(BULBO_PTO){
            OUT1 = LOW; 
            OUT2 = LOW;   
            stato = 4;
        }
        else{
            OUT1 = LOW; 
            OUT2 = LOW;   
            stato = 0;   
            return;        
        } 
    }                     
    if(BULBO_PTO){
        ERRORE_PTO = 0;  
        timerRESUME = millis(); 
        stato = 6;        
    }
    if (millis() - timerOUT1 >= rit_verifica_bulbo){
        OUT1 = LOW;  
        ERRORE_PTO = 1;   
        stato = 0;       
    }
}

void fase4(){
    if(!BULBO_PTO){
        ERRORE_PTO = 0; 
        stato = 0;         
    }    
    if (millis() - timerOUT2 >= rit_verifica_bulbo){
        OUT2 = LOW;  
        ERRORE_PTO = 1; 
        skip3 = false;    
        stato = 0;     
    }

}

void fase5(){
    if (millis() - timerSCAMBIO >= ritardo_inversione){ 
        OUT1 = LOW; 
        OUT2 = HIGH; 
        timerOUT2 = millis();
        stato = 2;     
    }
}

void fase6(){
    if (millis() - timerRESUME >= ritardo_resume){
        timerT_RESUME = millis();
        digitalWrite(RELE_RESUME, HIGH);
        stato = 7;
    }
}

void fase7(){
    if (millis() - timerT_RESUME >= impulso_resume){
        digitalWrite(RELE_RESUME, LOW);
        stato = 0;
    }
}

void fase8(){
    if (millis() - timerSpinta >= ritardo_spinta){
    OUT1 = LOW;  
    ERRORE_PTO = 0;
    timerRESUME = millis();  
    stato = 6; 
    }
}

void fase9(){
    if (millis() - timerSpinta >= ritardo_spinta){
    OUT2 = LOW;  
    ERRORE_PTO = 0;  
    stato = 0; 
    }
}

void fase10(){
}




void checkOut(){
    STATO_PTO = BULBO_PTO;
    master_bits[0] = STATO_PTO;
    master_bits[1] = ERRORE_PTO; 

    STATO_OUT3 = !multi_Input_20.digitalRead(6);
    if(STATO_OUT3 != OUT3) {
        ERRORE_OUT3 = true;
    }
    else{
        ERRORE_OUT3 = false;        
    }
    master_bits[2] =  STATO_OUT3;
    master_bits[3] =  ERRORE_OUT3;

    STATO_OUT4 = !multi_Input_20.digitalRead(7);
    if(STATO_OUT4 != OUT4) {
        ERRORE_OUT4 = true;
    }
    else{
        ERRORE_OUT4 = false;       
    }    
    master_bits[4] =  STATO_OUT4;
    master_bits[5] =  ERRORE_OUT4;

    STATO_OUT5 = !multi_Input_20.digitalRead(8);
    if(STATO_OUT5 != OUT5) {
        ERRORE_OUT5 = true;
    }
    else{
        ERRORE_OUT5 = false;        
    }           
    master_bits[6] =  STATO_OUT5;
    master_bits[7] =  ERRORE_OUT5;

    STATO_OUT6 = !multi_Input_20.digitalRead(9);
    if(STATO_OUT6 != OUT6) {
        ERRORE_OUT6 = true; 
    }
    else{
        ERRORE_OUT6 = false;         
    }
    master_bits[8] =  STATO_OUT6;
    master_bits[9] =  ERRORE_OUT6;

    STATO_OUT7 = !multi_Input_20.digitalRead(10);
    if(STATO_OUT7 != OUT7) {
        ERRORE_OUT7 = true;
    }
    else{
        ERRORE_OUT7 = false;        
    }
    master_bits[10] =  STATO_OUT7;
    master_bits[11] =  ERRORE_OUT7;

    STATO_OUT8 = !multi_Input_20.digitalRead(11);
    if(STATO_OUT8 != OUT8) {
        ERRORE_OUT8 = true;
    }
    else{
        ERRORE_OUT8 = false;        
    }
    master_bits[12] =  STATO_OUT8;
    master_bits[13] =  ERRORE_OUT8;

    STATO_OUT9 = !multi_Input_20.digitalRead(12);
    if(STATO_OUT9 != OUT9) {
        ERRORE_OUT9 = true;
    }
    else{
        ERRORE_OUT9 = false;       
    }
    master_bits[14] =  STATO_OUT9;
    master_bits[15] =  ERRORE_OUT9;

    STATO_OUT10 = !multi_Input_20.digitalRead(13);
    if(STATO_OUT10 != OUT10) {
        ERRORE_OUT10 = true;
    }
    else{
        ERRORE_OUT10 = false;        
    }
    master_bits[16] =  STATO_OUT10;
    master_bits[17] =  ERRORE_OUT10;

    // Scheda aggiuntiva SLAVE 2
    master_bits[19] =  slave2_bits[11]; //ERRORE_OUT11
    master_bits[21] =  slave2_bits[13]; //ERRORE_OUT12
    master_bits[23] =  slave2_bits[15]; //ERRORE_OUT13
    master_bits[25] =  slave2_bits[17]; //ERRORE_OUT14
    master_bits[27] =  slave2_bits[19]; //ERRORE_OUT15
    master_bits[29] =  slave2_bits[21]; //ERRORE_OUT16
}




void blinkLed() {
  if ((currentMillis - previousLEDMillis >= intervalON) && (skip == 0)) {
    previousLEDMillis = currentMillis;
    digitalWrite(LED_Diagnosi, HIGH);  
    skip = 1;
  }
  if (currentMillis - previousLEDMillis >= intervalOFF) {
    previousLEDMillis = currentMillis;
    digitalWrite(LED_Diagnosi, LOW); 
    skip = 0;
  } 
}

// ========== Implementazione funzioni CANbus J1939 ==========

/**
 * Inizializza il bus CAN J1939
 * Configura MCP2515 a 250 kbps con ID estesi 29-bit
 */
void setupCANbus() {
    // Inizializza CAN1 con pin alternativi (PB8/PB9)
    if (!Can.begin()) {
        // Errore inizializzazione CAN - continua comunque
        // Il sistema RS485 funzionerà normalmente
        return;
    }
    
    if (!Can.setBaudRate(250000)) {  // 250 kbps standard J1939
        // Errore impostazione baudrate - continua comunque
        return;
    }
    
    // Imposta filtri per i PGN Iveco Daily
    CAN_filter_t filter;
    
    // Filtro 0: PGN 0xFEE0 - Vehicle Electrical Power (principale per luci)
    filter.id = 0x18FEE000;
    filter.mask = 0x1FFFF00;  // Maschera PGN (bit 8-24) + permetti qualsiasi source address
    filter.flags.extended = 1;
    filter.flags.rtr = 0;
    Can.setFilter(filter, 0);
    
    // Filtro 1: PGN 0xFEEC - Dash Display
    filter.id = 0x18FEEC00;
    filter.mask = 0x1FFFF00;
    filter.flags.extended = 1;
    Can.setFilter(filter, 1);
    
    // Filtro 2: PGN 0xFDBC - Cab Message
    filter.id = 0x18FDBC00;
    filter.mask = 0x1FFFF00;
    filter.flags.extended = 1;
    Can.setFilter(filter, 2);
    
    // Filtro 3: PGN 0xFEB1 - Vehicle Position (frecce)
    filter.id = 0x18FEB100;
    filter.mask = 0x1FFFF00;
    filter.flags.extended = 1;
    Can.setFilter(filter, 3);
    
    // Inizializza struttura dati luci
    memset(&iveco_lights, 0, sizeof(IvecoLightsStatus));
    iveco_lights.last_update = 0;
    
    // Debug (opzionale)
    // Serial.println("CANbus J1939 inizializzato su PB8/PB9 con SN65HVD230");
}

/**
 * Estrae il PGN da un CAN ID esteso 29-bit secondo lo standard J1939
 * @param can_id CAN ID esteso 29-bit
 * @return PGN estratto
 */
uint32_t getPGN(uint32_t can_id) {
    // J1939 29-bit CAN ID structure:
    // [28:26] Priority (3 bits)
    // [25:24] Reserved (2 bits) 
    // [23:16] PDU Format (8 bits)
    // [15:8]  PDU Specific (8 bits) - può essere destination o group extension
    // [7:0]   Source Address (8 bits)
    
    uint8_t pdu_format = (can_id >> 16) & 0xFF;
    uint8_t pdu_specific = (can_id >> 8) & 0xFF;
    
    uint32_t pgn;
    if (pdu_format < 240) {
        // PDU1 format (destination specific)
        // PGN = [Reserved][PDU Format][00]
        pgn = ((can_id >> 8) & 0x03FF00);
    } else {
        // PDU2 format (broadcast)
        // PGN = [Reserved][PDU Format][PDU Specific]
        pgn = ((can_id >> 8) & 0x03FFFF);
    }
    
    return pgn;
}

/**
 * Legge messaggi J1939 dal bus CAN
 * Chiama processIvecoLights per decodificarli
 */
void readJ1939Messages() {
    CAN_message_t msg;
    
    // Leggi messaggio se disponibile
    if (Can.read(msg)) {
        // Verifica che sia extended frame (29-bit J1939)
        if (msg.flags.extended) {
            // Estrai informazioni J1939
            uint32_t pgn = getPGN(msg.id);
            uint8_t source_address = msg.id & 0xFF;
            
            // Processa i messaggi delle luci
            processIvecoLights(pgn, msg.buf, msg.len, source_address);
        }
    }
}

/**
 * Processa messaggi J1939 per estrarre lo stato delle luci Iveco
 * @param pgn Parameter Group Number
 * @param data Dati del messaggio (max 8 byte)
 * @param len Lunghezza dati
 * @param source Source address del mittente
 */
void processIvecoLights(uint32_t pgn, uint8_t* data, uint8_t len, uint8_t source) {
    // Helper macro per estrarre valori da 2 bit
    #define GET_2_BITS(byte, start_bit) (((byte) >> (start_bit)) & 0x03)
    
    switch (pgn) {
        case PGN_VEHICLE_ELECTRICAL_POWER:  // 0xFEE0
            if (len >= 4) {
                // Byte 0, bit 2-3: Hazard warning
                iveco_lights.hazard_warning = (GET_2_BITS(data[0], 2) == 0x01);
                
                // Byte 1, bit 0-1: Left turn signal
                iveco_lights.left_turn_signal = (GET_2_BITS(data[1], 0) == 0x01);
                
                // Byte 1, bit 2-3: Right turn signal
                iveco_lights.right_turn_signal = (GET_2_BITS(data[1], 2) == 0x01);
                
                // Byte 2, bit 0-1: Parking lights
                iveco_lights.parking_lights = (GET_2_BITS(data[2], 0) == 0x01);
                
                // Byte 2, bit 2-3: Low beam
                iveco_lights.low_beam = (GET_2_BITS(data[2], 2) == 0x01);
                
                // Byte 2, bit 4-5: High beam
                iveco_lights.high_beam = (GET_2_BITS(data[2], 4) == 0x01);
                
                // Byte 2, bit 6-7: Front fog lights
                iveco_lights.front_fog_lights = (GET_2_BITS(data[2], 6) == 0x01);
                
                // Byte 3, bit 0-1: Rear fog lights
                iveco_lights.rear_fog_lights = (GET_2_BITS(data[3], 0) == 0x01);
                
                // Byte 3, bit 2-3: Brake lights
                iveco_lights.brake_lights = (GET_2_BITS(data[3], 2) == 0x01);
                
                // Byte 3, bit 4-5: Reverse lights
                iveco_lights.reverse_lights = (GET_2_BITS(data[3], 4) == 0x01);
                
                iveco_lights.last_update = millis();
            }
            break;
            
        case PGN_DASH_DISPLAY:  // 0xFEEC
            if (len >= 6) {
                // Byte 5, bit 4-5: High beam indicator (conferma)
                bool high_beam_conf = (GET_2_BITS(data[5], 4) == 0x01);
                if (high_beam_conf) {
                    iveco_lights.high_beam = true;
                }
                iveco_lights.last_update = millis();
            }
            break;
            
        case PGN_CAB_MESSAGE_1:  // 0xFDBC
            if (len >= 7) {
                // Byte 6, bit 0-1: Work lights
                iveco_lights.work_lights = (GET_2_BITS(data[6], 0) == 0x01);
                iveco_lights.last_update = millis();
            }
            break;
            
        case PGN_VEHICLE_POSITION:  // 0xFEB1
            if (len >= 1) {
                // Byte 0, bit 0-1: Left turn signal (conferma)
                iveco_lights.left_turn_signal = (GET_2_BITS(data[0], 0) == 0x01);
                
                // Byte 0, bit 2-3: Right turn signal (conferma)
                iveco_lights.right_turn_signal = (GET_2_BITS(data[0], 2) == 0x01);
                
                iveco_lights.last_update = millis();
            }
            break;
            
        default:
            // PGN non gestito - ignora
            break;
    }
    
    #undef GET_2_BITS
}

/**
 * Verifica se i dati delle luci sono validi (ricevuti recentemente)
 * @return true se i dati sono stati ricevuti negli ultimi 2000ms
 */
bool isLightsDataValid() {
    if (iveco_lights.last_update == 0) {
        return false;
    }
    return (millis() - iveco_lights.last_update) < 2000;
}

/**
 * Stampa lo stato delle luci su Serial (per debug)
 * Decommentare Serial.begin(115200) nel setup() per abilitare
 */
void printLightsStatus() {
    if (!isLightsDataValid()) {
        // Serial.println("J1939: Dati non validi (timeout)");
        return;
    }
    
    // Serial.println("=== Stato Luci Iveco J1939 ===");
    // Serial.print("Luci posizione: "); Serial.println(iveco_lights.parking_lights ? "ON" : "OFF");
    // Serial.print("Anabbaglianti: "); Serial.println(iveco_lights.low_beam ? "ON" : "OFF");
    // Serial.print("Abbaglianti: "); Serial.println(iveco_lights.high_beam ? "ON" : "OFF");
    // Serial.print("Fendinebbia ant.: "); Serial.println(iveco_lights.front_fog_lights ? "ON" : "OFF");
    // Serial.print("Fendinebbia post.: "); Serial.println(iveco_lights.rear_fog_lights ? "ON" : "OFF");
    // Serial.print("Freccia SX: "); Serial.println(iveco_lights.left_turn_signal ? "ON" : "OFF");
    // Serial.print("Freccia DX: "); Serial.println(iveco_lights.right_turn_signal ? "ON" : "OFF");
    // Serial.print("Emergenza: "); Serial.println(iveco_lights.hazard_warning ? "ON" : "OFF");
    // Serial.print("Stop: "); Serial.println(iveco_lights.brake_lights ? "ON" : "OFF");
    // Serial.print("Retromarcia: "); Serial.println(iveco_lights.reverse_lights ? "ON" : "OFF");
    // Serial.print("Luci lavoro: "); Serial.println(iveco_lights.work_lights ? "ON" : "OFF");
    // Serial.println("==============================");
}

/**
 * Mappa lo stato delle luci Iveco agli output master_bits
 * Solo se i dati CAN sono validi (ricevuti negli ultimi 2 secondi)
 */
void mapIvecoLightsToOutputs() {
    if (!isLightsDataValid()) {
        // Dati non validi - non modificare gli output
        return;
    }
    
    // Mappa le luci Iveco ai bit del master
    // Nota: master_bits[30-34] sono disponibili (non usati dal codice esistente)
    master_bits[30] = iveco_lights.parking_lights;
    master_bits[31] = iveco_lights.low_beam;
    master_bits[32] = iveco_lights.high_beam;
    master_bits[33] = iveco_lights.left_turn_signal;
    master_bits[34] = iveco_lights.right_turn_signal;
    
    // Altri bit potrebbero essere mappati se necessario:
    // master_bits[xx] = iveco_lights.front_fog_lights;
    // master_bits[xx] = iveco_lights.rear_fog_lights;
    // master_bits[xx] = iveco_lights.hazard_warning;
    // master_bits[xx] = iveco_lights.brake_lights;
    // master_bits[xx] = iveco_lights.reverse_lights;
    // master_bits[xx] = iveco_lights.work_lights;
}

// ========== Fine implementazione CANbus J1939 ==========
