/*
* ADXL362:n toiminnan esimerkkikoodi ja selitykset (englanniksi):
* https://ez.analog.com/mems/w/documents/4503/adxl362-arduino-using-interrupts-to-implement-motion-activated-sleep
*/
#include <LowPower.h>
#include <SPI.h>
#include <ADXL362.h>
//#include <avr/sleep.h>
#include <lmic.h>
#include <hal/hal.h>
#include "keys.h"

#define DEBUG false// Debuggausviestien tulostus päälle tai pois päältä
#define SLEEPTIME 15// Anturin nukkumisaika lähetyksen jälkeen

#if DEBUG
  #define DEBUG_SERIALPRINTLN(x) Serial.println(x)
  #define DEBUG_SERIALPRINT(x) Serial.print(x)
#else
  #define DEBUG_SERIALPRINTLN(x)
  #define DEBUG_SERIALPRINT(x)
#endif
 
ADXL362 xl;

void interruptFunction();
String createPayload();
int selftest();
float readBattery();
void readValues();
void sleep_min(int mins);

int16_t interruptPin = 3;//ADXL362:n keskeytys
int16_t chipSelectPin = 4;
int16_t interruptStatus = 0;//0 = ADXL nukkuu, 1 = ADXL on hereillä
int16_t XValue, YValue, ZValue, Temperature;//Anturin arvot

/* LoRa ja lähetys */
// onko lähetys käynnissä?
bool sending = false;

// TX:n lähetysväli (60 sekuntia)
const unsigned TX_INTERVAL = 60;

// Näitä käytetään ainoastaan OTAA-lähetykseen, joten tässä tyhjiä.
void os_getArtEui(u1_t *buf){}
void os_getDevEui(u1_t *buf){}
void os_getDevKey(u1_t *buf){}

// LMIC-kirjaston lähetysfunktiota varten
static osjob_t sendjob;

 // Draginon pinnikartta
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

//Kynnysarvoja
int activity_threshold = 50;  // Liikkeen voimakkuuden kynnys. Tämän arvon pitää ylittyä, jotta laite aktivoituu
int activity_time = 10;       // Kuinka kauan (ms) liikettä pitää havaita, ennen kuin herätään
int inactivity_threshold = 40;// 
int inactivity_time = 600;    // Kuinka kauan (ms) ilman liikettä laittaa laitteen uneen

//Selftestin muuttujia
int16_t DEVID_AD, DEVID_MST, PARTID;
bool test_passed = false;
int avg_test_X, avg_test_Y, avg_test_Z, avg_X, avg_Y, avg_Z;
int X_ST, Y_ST, Z_ST;


// Funktio, joka käsittelee erilaisia tapahtumia ja tulostaa ilmoituksia monitoriin debuggausta varten.
// Tapahtumaa "EV_TXCOMPLETE" käytetään lähetyksen onnistumisen tarkastukseen.
void onEvent(ev_t ev)
{
  DEBUG_SERIALPRINT(os_getTime());
  DEBUG_SERIALPRINT(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    DEBUG_SERIALPRINTLN(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    DEBUG_SERIALPRINTLN(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    DEBUG_SERIALPRINTLN(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    DEBUG_SERIALPRINTLN(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    DEBUG_SERIALPRINTLN(F("EV_JOINING"));
    break;
  case EV_JOINED:
    DEBUG_SERIALPRINTLN(F("EV_JOINED"));
    break;
  case EV_RFU1:
    DEBUG_SERIALPRINTLN(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    DEBUG_SERIALPRINTLN(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    DEBUG_SERIALPRINTLN(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    DEBUG_SERIALPRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    os_radio(RADIO_RST);
    // After successful send, set sending=0 and sleep.
    DEBUG_SERIALPRINTLN("Sending message completed.");
    sending = 0;
    sleep_min(SLEEPTIME);
    break;
  case EV_LOST_TSYNC:
    DEBUG_SERIALPRINTLN(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    DEBUG_SERIALPRINTLN(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    DEBUG_SERIALPRINTLN(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    DEBUG_SERIALPRINTLN(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    DEBUG_SERIALPRINTLN(F("EV_LINK_ALIVE"));
    break;
  default:
    DEBUG_SERIALPRINTLN(F("Unknown event"));
    break;
  }
}

//Fuktio, joka lähettää LoRa-viestin ja asettaa siihen datan.
void do_send(osjob_t *j)
{
  // Tarkistetaan, onko TX/RX-työ käynnissä. Jos on, ajastetaan seuraava lähetys.
  if (LMIC.opmode & OP_TXRXPEND)
  {
    DEBUG_SERIALPRINTLN(F("OP_TXRXPEND, not sending"));
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
  }
  else
  {
    // Lisätään anturiarvot viestiin ja laitetaan viesti lähetysjonoon.
    String values = createPayload();
    uint8_t payload[values.length()+1];
    values.getBytes(payload, values.length());
    DEBUG_SERIALPRINTLN("Sending LoRa message.");
    LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
    DEBUG_SERIALPRINTLN(F("Packet queued"));
  }
}

/* Muita funktioita */
//Funktio, jota kutsutaan keskeytyksen tapahtuessa. Tässä tyhjä, koska toimintalogiikka hoidetaan muualla.
void interruptFunction() {
  //empty function
}

//Luekee ADXL362:n arvot; x-,y- ja z-akselit sekä (anturin sisäinen) lämpötila.
void readValues() {
  xl.readXYZTData(XValue, YValue, ZValue, Temperature);
}

// Luekee pariston varaus volteissa.
float readBattery() {
  int value = analogRead(A2);
  return value * (3.3 / 1023.00);
}

//Luo anturiarvoista ja pariston varaustilasta lähetystä varten merkkijono
String createPayload() {
  readValues();
  return (String)XValue + "|" + (String)YValue + "|" + (String)ZValue + "|" + (String)Temperature + "|" + (String)readBattery();
}

//Laittaa anturin nukkumaan. Tässä tilassa anturi herää keskeytykseen.
void sleep_forever() {
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
}

//Laittaa anturin nukkumaan tietyksi ajaksi (minuuteissa).
void sleep_min(int mins) {
  DEBUG_SERIALPRINT("Sleeping for ");
  DEBUG_SERIALPRINT((String)mins);
  DEBUG_SERIALPRINTLN(" minutes");
  delay(100);
  unsigned int sleepCounter;
  for( sleepCounter = 7.5 * mins; sleepCounter > 0; sleepCounter--){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  DEBUG_SERIALPRINT("Woke up from ");
  DEBUG_SERIALPRINT((String)mins);
  DEBUG_SERIALPRINTLN(" minute sleep.");
}

//-----------itse toimintalogiikka alkaa tästä------------------------------------------------------------//
//Setupissa määritetään laitteiston toiminta. Tämä ajetaan vain kerran.
void setup(){
  Serial.begin(9600);//Käynnistetään monitori debuggausta varten.
  xl.begin(chipSelectPin);//Kytketään ADXL362-anturi
  delay(100);

  selftest();//Ajetaan selftest
   
  xl.setupDCActivityInterrupt(activity_threshold, activity_time);
  xl.setupDCInactivityInterrupt(inactivity_threshold, inactivity_time);
    
  DEBUG_SERIALPRINTLN();
   //
   // ADXL362:n nukkumisen konfigurointi
   //
  xl.SPIwriteOneRegister(0x2A, 0x40);   
  xl.SPIwriteOneRegister(0x27, 0x3F);   
  // kytketään "autosleep" päälle, jolloin laite menee itsestään nukkumaan, jos liikettä ei havaita
  byte POWER_CTL_reg = xl.SPIreadOneRegister(0x2D);
  POWER_CTL_reg = POWER_CTL_reg | (0x04);
  xl.SPIwriteOneRegister(0x2D, POWER_CTL_reg);
  // kytketään mittausmoodi päälle
  xl.beginMeasure();                        
  xl.checkAllControlRegs();
  delay(100);

  //LoRan asetus
  // LMICin alustus
  os_init();
  // Tyhjennetään MAC-tila. Kaikki istuntotiedot ja lähetettävät paketit poistetaan.
  LMIC_reset();

  #ifdef PROGMEM
  // AVR:ssä nämä tallennetaan flash-muistiin ja kopioidaan RAMiin vain kerran.
  // Arvot tallennetaan tässä väliaikaisesti ja LMIC_setSession kopioi ne 
  // sitten omaan bufferiinsa.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
  #else
  // Jos ei ole AVR-mikro-ohjainta käytössä, käytetään suoraan taulukkoa.
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  //Kytketään linkin tarkitus pois päältä.
  LMIC_setLinkCheckMode(0);

  // The Things Network käyttää SF9 RX2 ikkunnassa
  LMIC.dn2Dr = DR_SF9;

  //Asetetaan lähetyksen vahvuus (SF)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Keskeytyksen päällekytkentä
  pinMode(interruptPin, INPUT);    
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptFunction, RISING);// A high on output of ADXL interrupt means ADXL is awake, and wake up Arduino
}

//Tässä tapahtuu varsinainen toimintalogiikka. Tämä ajetaan jatkuvana silmukkana aina kun laite on hereillä.
void loop(){

  // Tarkistetaan ADXL362:n keskeytystila.
  interruptStatus = digitalRead(interruptPin);
 
  // Jos ADXL362 nukkuu eikä lähetys ole käynnissä, laitetaan Dragino nukkumaan
  if(interruptStatus == 0 && sending == 0) {
    sleep_forever();
  }
  // Jos ADXL362 on hereillä ja lähetys ei ole käynnisttä, laitetaan LoRa lähetys aluilleen
  else if (interruptStatus == 1 && sending == 0) {
    sending = 1;
    do_send(&sendjob);
    DEBUG_SERIALPRINTLN("Sending...");
  }
  os_runloop_once(); // Tämä lähettää jonossa olevat viestit.
  delay(10);
}

//-----------------------------------------------------------------------//

int selftest()
{
    avg_test_X = 0;
    avg_test_Y = 0;
    avg_test_Z = 0;
    avg_X = 0;
    avg_Y = 0;
    avg_Z = 0;
    xl.SPIwriteOneRegister(0x1F, 0x52);      //Tehdään soft reset.
    delay(100);                              //Odotetaan 100 millisekuntia.
    DEVID_AD = xl.SPIreadOneRegister(0x00);  //Luetaan rekisteristä DEVID_AD.
    DEVID_MST = xl.SPIreadOneRegister(0x01); //Luetaan rekisteristä DEVID_MST.
    PARTID = xl.SPIreadOneRegister(0x02);    //Luetaan rekisteristä PARTID.
    //Tarkistetaan onko rekistereistä luetut arvot oikeat, jos ei niin testi ei ole läpi.
    if (DEVID_AD != 0xAD || DEVID_MST != 0x1D || PARTID != 0xF2) //Tarkista tähän oikeat luvut!
    {
        test_passed = false;
    }
    // Jos arvot täsmäävät, testiä jatketaan.
    else
    {
        xl.SPIwriteOneRegister(0x2C, 0x83); //Valitaan ±8G alue ja 100Hz ODR FILTER_CLT-rekisteristä.
        xl.SPIwriteOneRegister(0x2D, 0x02); // Otetaan mittaus käyttöön kirjoittamalla 2 POWER_CTL-reskisteriin.
        delay(200);                         //Odotetaan 200 millisekuntia.
        //Luetaan kaikkien akselien kiihtyvyyksien tiedostot burst-metodilla 20ms välein ja tallennetaan tiedot muuttujaan.
        for (int i = 0; i < 16; i ++)
        {
            xl.readXYZTData(XValue, YValue, ZValue, Temperature);
            avg_test_X += XValue;
            avg_test_Y += YValue;
            avg_test_Z += ZValue;
            delay(20);
        }
        xl.SPIwriteOneRegister(0x2D, 0x00); //Laitetaan kiihtyvyysanturi standby-tilaan kirjoittamalla 0 POWER_CTL-rekisteriin.
        xl.SPIwriteOneRegister(0x2E, 0x01); //Otetaan self_test käyttöön kirjoittamalla 1 SELF_TEST-rekisteriin.
        xl.SPIwriteOneRegister(0x2D, 0x02); //Otetaan mittaus käyttöön kirjottamalla 2 POWER_CTL-rekisteriin.
        delay(200);                         //Odotetaan 200 millisekuntia.
        //Luetaan kaikkien akselien kiihtyvyyksien tiedostot burst-metodilla 20ms välein ja tallennetaan tiedot muuttujaan.
        for (int i = 0; i < 16; i++)
        {
            xl.readXYZTData(XValue, YValue, ZValue, Temperature);
            avg_X += XValue;
            avg_Y += YValue;
            avg_Z += ZValue;
            delay(20);
        }
        xl.SPIwriteOneRegister(0x2E, 0x00); //Otetaan self_test pois käytöstä kirjoittamalla 0 SELF_TEST-rekisteriin.
        xl.SPIwriteOneRegister(0x2D, 0x00); //Laitetaan kiihtyvyysanturi standby-tilaan kirjoittamalla 0 POWER_CTL-rekisteriin.
        //Lasketaan keskiarvot mitatuista tuloksista.
        avg_test_X = avg_test_X / 16;
        avg_test_Y = avg_test_Y / 16;
        avg_test_Z = avg_test_Z / 16;
        avg_X = avg_X / 16;
        avg_Y = avg_Y / 16;
        avg_Z = avg_Z / 16;
        //Lasketaan tietojen välinen ero silloin kun selftest on päällä siihen kun se ei ole päällä.
        X_ST = (avg_X - avg_test_X);
        Y_ST = (avg_Y - avg_test_Y);
        Z_ST = (avg_Z - avg_test_Z);
        // Tarkastellaan saatuja lukemia ja katsotaan menikö testi läpi.
        if (50 <= X_ST && X_ST <= 700 && -700 <= Y_ST && Y_ST <= -50 && 50 <= Z_ST && Z_ST <= 700)
        {
            test_passed = true;
        }
        else
        {
            test_passed = false;
        }
    }
  DEBUG_SERIALPRINTLN("Selftest completed.");
  return test_passed;
}