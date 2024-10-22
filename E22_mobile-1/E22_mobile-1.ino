/*
 * LoRa E22 / ESP32 тест дальности при передачи пакетов
 * Эксперимент 2
 * Мобильная часть
 * 
 * 01.2024 Игонин Алексей 
 * https://www.youtube.com/@RadioFromRussia
 * https://dzen.ru/radiofromrussia
 * https://vk.com/radiofromrussia
 *
 * Подключение модуля
 * E22       ESP32
 * M0    ----- 25
 * M1    ----- 26
 * TX    ----- 18
 * RX    ----- 19
 * AUX   ----- 23 
 *
 */
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#define FREQUENCY_868
#define E22_30
#include "LoRa_E22.h"

// rf module pins
#define RX1 19
#define TX1 18
#define AUX1 23
// button
#define BTN1 5

LoRa_E22 rfm(&Serial1, AUX1, 25, 26);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// #pragma pack (push, 1) для эксперимента пока не меняю выравнивание
struct msg_t
{
  char      str[10];   // текстовое представление значения счетчика
  uint32_t  num;       // двоичное представление счетчика
  uint32_t  rx_num;    // значение счетчика из предыдущего принятого пакета
  char      tmp[34];   // дополнение до нужных размеров пакета
};
// #pragma pack (pop)


uint32_t    rx_count = 0;
uint32_t    lost_count2 = 0;
msg_t       tx_msg1;
msg_t       rx_msg1;

uint8_t     mode_num = 2;
uint32_t    start_num;
bool        start_num_ok = false;
int64_t     pktsend_prev_timer;

//-------------------------------------------------------------------

void printParameters(struct Configuration configuration) 
{
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
  Serial.print(F("NetID : "));  Serial.println(configuration.NETID, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));

  Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeTransContr: "));  Serial.print(configuration.TRANSMISSION_MODE.WORTransceiverControl, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORTransceiverControlDescription());
  Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeEnabRepeat: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRepeater, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRepeaterModeEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

  Serial.println("----------------------------------------");

}
void printModuleInformation(struct ModuleInformation moduleInformation) 
{
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD: "));  Serial.print(moduleInformation.COMMAND, HEX);Serial.print(" ");Serial.print(moduleInformation.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(moduleInformation.LENGHT, DEC);
  Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, HEX);
  Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}


void radioSetup()
{
  ResponseStatus rs;
  ResponseStructContainer c;
  ResponseStructContainer cMi;
  int rate;

  while(digitalRead(AUX1) == 0) delay(1); // ждём, если модуль что-от передаёт в компорт
  Serial1.end();
  Serial1.begin(9600, SERIAL_8N1, RX1, TX1); // скорость 9600 для работы в режиме конфигурации

  delay(200);
  while(Serial1.available()) Serial1.read(); // вычитываем левые байты из компорта, если они там образовались после его передергивания

  c = rfm.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;
  Serial.print("Read conf from RF module = ");
  Serial.print(c.status.code);
  Serial.print(" (");
  Serial.print(c.status.getResponseDescription());
  Serial.println(")");

  if (c.status.code != 1) 
  {
  Serial.println("Try again...");
  while(Serial1.available()) Serial1.read(); // вычитываем левые байты из компорта, если они там образовались
  c = rfm.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;
  Serial.print("Read conf from RF module = ");
  Serial.print(c.status.code);
  Serial.print(" (");
  Serial.print(c.status.getResponseDescription());
  Serial.println(")");
  }

  if (c.status.code != 1) 
  {
    Serial.println("return!!!");
    c.close();
    return;
  }

/*
  cMi = rfm.getModuleInformation();
  // It's important get information pointer before all other operation
  ModuleInformation mi = *(ModuleInformation*)cMi.data;

  Serial.print("Read module info from RF module ");  
  Serial.print(cMi.status.code);
  Serial.print(" (");
  Serial.print(cMi.status.getResponseDescription());
  Serial.println(")");

  if (cMi.status.code != 1) 
  {
    c.close();
    cMi.close();
    return;
  }

  printModuleInformation(mi);
  cMi.close();
*/


  configuration.ADDL = 0x00;
  configuration.ADDH = 0x00;
  configuration.NETID = 0x00;
  configuration.CHAN = 21;
  
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;                    // listen before transmit (проверка занятости канала, может увеличить задержку)
//  configuration.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
  configuration.TRANSMISSION_MODE.WORTransceiverControl = WOR_TRANSMITTER;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

//  configuration.OPTION.transmissionPower = POWER_21;
  configuration.OPTION.transmissionPower = POWER_30;
  configuration.OPTION.subPacketSetting = SPS_240_00;
//  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_ENABLED;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;


  configuration.SPED.uartParity = MODE_00_8N1;
  configuration.SPED.uartBaudRate = UART_BPS_115200;
//  configuration.SPED.airDataRate = AIR_DATA_RATE_000_03;
//  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
//  configuration.SPED.airDataRate = AIR_DATA_RATE_011_48;
  configuration.SPED.airDataRate = AIR_DATA_RATE_101_192;
//  configuration.SPED.airDataRate = AIR_DATA_RATE_110_384
//  configuration.SPED.airDataRate =  AIR_DATA_RATE_111_625;


  switch(mode_num)
  {
      case 0:
        configuration.SPED.airDataRate = AIR_DATA_RATE_111_625;
        rate = 62500;
        break;
      case 1:
        configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
        rate = 2400;
        break;
      case 2:
        configuration.SPED.airDataRate = AIR_DATA_RATE_101_192;
        rate = 19200;
        break;
  }

  lcd.setCursor(0, 0);
  lcd.print("Rate:");
  lcd.print(rate);
  lcd.print("   ");

  rs = rfm.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
  Serial.print("Write conf to RF module ");  
  Serial.print(rs.code);
  Serial.print(" (");
  Serial.print(rs.getResponseDescription());
  Serial.println(")");
  if (rs.code != 1) 
  {
    c.close();
    return;
  }

  printParameters(configuration);
  c.close();

  Serial1.end();
  Serial1.begin(115200, SERIAL_8N1, RX1, TX1);
}



//----------------------------------------------------------------
void setup()
{
  pinMode(BTN1, INPUT_PULLUP);
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX1, TX1);
  delay(300);
  
  lcd.init();
  lcd.clear();
  lcd.backlight();
  rfm.begin();

  radioSetup();

	Serial.println();
  Serial.print("pkt size=");
  Serial.println(sizeof(msg_t));
  Serial.println("Start");
}


void loop()
{


// receive processing
  int16_t len = rfm.available();

	if (len == sizeof(msg_t))
	{
      rx_count++;
      ResponseStructContainer rsc = rfm.receiveMessage(sizeof(msg_t));
      rx_msg1 = *(msg_t*) rsc.data;
      rsc.close();

      if (!start_num_ok)
      {
          start_num = rx_msg1.num;
          start_num_ok = true;
      }

      if ((rx_count != 1) && (rx_msg1.rx_num != (rx_count -1))) // при первом пакете не считаем, так передающей стороне еще не известен номер передаваемого нами пакета
      {
          lost_count2++;       
      }

// update LCD 
int   lost = (rx_msg1.num - rx_count + 1 - start_num);
int   total = rx_count + lost;
      lcd.setCursor(0, 0);  // общее кол-во принятых и потеряных пакетов
      lcd.print(total);
      lcd.print("    ");

      lcd.setCursor(5, 0);  // кол-во принятых пакетов
      lcd.print(rx_count);
      lcd.print("/");
      lcd.print(lost);
      lcd.print("   ");

      lcd.setCursor(12, 0);  // процент потярянных при приёме пакетов
      lcd.print( (int)((100 / (float)total) * lost) );
      lcd.print("% ");


      lcd.setCursor(0, 1);  // кол-во переданных пакетов (равно кол-ву принятых пакетов)
      lcd.print(rx_count);
      lcd.print("  ");

      lcd.setCursor(5, 1);  // кол-во принятых пакетов
      lcd.print(rx_count - lost_count2);
      lcd.print("/");
      lcd.print(lost_count2);
      lcd.print("   ");

      lcd.setCursor(12, 1);  // процент потярянных на той стороне переданных отсюда пакетов
      lcd.print( (int)((100 / (float)rx_count) * lost_count2) );
      lcd.print("% ");

// transmit pkt
      sprintf(tx_msg1.str, "%06d", rx_count);
      tx_msg1.num = rx_count;
      ResponseStatus rs = rfm.sendMessage(&tx_msg1, sizeof(msg_t));

	}
  else
  {
      if (len > 0) // Принят пакет не нашей длины (левый). Вычитываем его просто так.
      {
        Serial.print("received len = ");
        Serial.println(len);

        ResponseStructContainer rsc = rfm.receiveMessage(len);
        rsc.close();
      }
  }




// button press processing
  if (digitalRead(BTN1) == 0)
  {
          while(digitalRead(BTN1) == 0) delay(10);
          mode_num++;
          if (mode_num > 2) mode_num = 0;
          lcd.clear();
          radioSetup();
          start_num_ok = false;
          lost_count2 = 0;
          rx_count = 0;
  }

}
