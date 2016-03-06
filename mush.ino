//#include <SoftwareSerial.h>
//#include <sim900_Suli.h>
//#include <GPRS_Shield_Arduino.h>
//#include <Suli.h>

#include <LiquidCrystal.h>
#include <DHT.h>
#include <ModbusRtu.h>
#include <EEPROM.h>
#include <mushnet2.h>

#define MASTERHOST 0
#define I1_HOST 1
#define BZ_TF_IN_HOST 7
#define LOCALHOST  0
#define ISMASTER   (LOCALHOST==MASTERHOST)

//GSM gsmAccess; // include a 'true' parameter for debug enabled
//GSM_SMS sms;
//GPRS gprs(8, 7, 9600);

uint16_t au16data[AU16DATA_SIZE];

DeviceManager deviceManager(LOCALHOST, ISMASTER, au16data);

byte id = 0;

// BZ Relays
Relay bz_water_in(id++, "BZ_WI", MASTERHOST, LOCALHOST, 39, au16data);
Relay bz_water_re(id++, "BZ_WR", MASTERHOST, LOCALHOST, 41, au16data);
Relay bz_water_cool(id++, "BZ_WC", MASTERHOST, LOCALHOST, 43, au16data);
Relay bz_heater_valve_re_o(id++, "BZ_HVRO", MASTERHOST, LOCALHOST, 31, au16data);
Relay bz_heater_valve_re_c(id++, "BZ_HVRC", MASTERHOST, LOCALHOST, 33, au16data);
Relay bz_vent_re(id++, "BZ_VR", MASTERHOST, LOCALHOST, 35, au16data);
Relay bz_heater_valve_in_c(id++, "BZ_HVIC", MASTERHOST, LOCALHOST, 22, au16data); //change
Relay bz_heater_valve_in_e(id++, "BZ_HVIE", MASTERHOST, LOCALHOST, 24, au16data); //enable
//BZ DTH
SensorDHT22Temperature bz_t1(id++, "BZ_T1", MASTERHOST, LOCALHOST, 42, au16data);
SensorDHT22Humidity bz_h1(id++, "BZ_H1", MASTERHOST, LOCALHOST, bz_t1.get_dht(), au16data);
SensorDHT22Temperature bz_t2(id++, "BZ_T2", MASTERHOST, LOCALHOST, 44, au16data);
SensorDHT22Humidity bz_h2(id++, "BZ_H2", MASTERHOST, LOCALHOST, bz_t2.get_dht(), au16data);
//BZ FT
TransformerFrequency bz_tf_in(id++, "BZ_FT", BZ_TF_IN_HOST, LOCALHOST, au16data);
//BZ Diffrent
Switch bz_pc(id++, "BZ_PC", MASTERHOST, LOCALHOST, 40, au16data);

//I1 Relays
Relay i1_valve_air_in(id++, "I1_VAI", I1_HOST, LOCALHOST, 23, au16data);
Relay i1_valve_air_re(id++, "I1_VAR", I1_HOST, LOCALHOST, 25, au16data);
Relay i1_valve_water(id++, "I1_VW", I1_HOST, LOCALHOST, 27, au16data);
Relay i1_valve_water_cool(id++, "I1_VWC", I1_HOST, LOCALHOST, 29, au16data);
Relay i1_vent_out(id++, "I1_VO", I1_HOST, LOCALHOST, 31, au16data);
Relay i1_vent_in(id++, "I1_VI", I1_HOST, LOCALHOST, 33, au16data);
Relay i1_heater_a1(id++, "I1_H1", I1_HOST, LOCALHOST, 39, au16data);
Relay i1_heater_a2(id++, "I1_H2", I1_HOST, LOCALHOST, 41, au16data);
Relay i1_heater_a3(id++, "I1_H3", I1_HOST, LOCALHOST, 43, au16data);
//I1 DTH
SensorDHT22Temperature i1_t1(id++, "I1_T1", I1_HOST, LOCALHOST, 22, au16data);
SensorDHT22Humidity i1_h1(id++, "I1_H1", I1_HOST, LOCALHOST, i1_t1.get_dht(), au16data);
SensorDHT22Temperature i1_t2(id++, "I1_T2", I1_HOST, LOCALHOST, 24, au16data);
SensorDHT22Humidity i1_h2(id++, "I1_H2", I1_HOST, LOCALHOST, i1_t2.get_dht(), au16data);
SensorDHT22Temperature i1_t3(id++, "I1_T3", I1_HOST, LOCALHOST, 26, au16data);
SensorDHT22Humidity i1_h3(id++, "I1_H3", I1_HOST, LOCALHOST, i1_t3.get_dht(), au16data);
SensorDHT22Temperature i1_t4(id++, "I1_T4", I1_HOST, LOCALHOST, 28, au16data);
SensorDHT22Humidity i1_h4(id++, "I1_H4", I1_HOST, LOCALHOST, i1_t4.get_dht(), au16data);
CO2Sensor i1_co1(id++, "I1_CO1", MASTERHOST, LOCALHOST, 45, au16data);

byte arg_id = 0;
Argument bz_min_t(arg_id++, "BZ_MIN_T", 1500);
Argument bz_max_t(arg_id++, "BZ_MAX_T", 1700);
Argument bz_min_h(arg_id++, "BZ_MIN_H", 7000);
Argument bz_max_h(arg_id++, "BZ_MAX_H", 8000);

Argument i1_min_t(arg_id++, "I1_MIN_T", 1700);
Argument i1_max_t(arg_id++, "I1_MAX_T", 1800);
Argument i1_min_h(arg_id++, "I1_MIN_H", 9000);
Argument i1_max_h(arg_id++, "I1_MAX_H", 9500);
Argument i1_min_co2(arg_id++, "I1_MIN_CO2", 150);
Argument i1_max_co2(arg_id++, "I1_MAX_CO2", 200);
Argument bz_ft_freq(arg_id++, "BZ_FT_FREQ", 150);

int i;
byte i1_mode = 0;
unsigned long i1_mode_ts = 0;
unsigned long ts;

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting");
  
//  gprs.powerUpDown(5);
//  while (!gprs.init()) {
//    // если связи нет, ждём 1 секунду
//    // и выводим сообщение об ошибке
//    // процесс повторяется в цикле
//    // пока не появится ответ от GPRS устройства
//    delay(1000);
//    Serial.print("Init error\r\n");
//  }
  
  deviceManager.add_executor(&bz_water_in);
  deviceManager.add_executor(&bz_water_re);
  deviceManager.add_executor(&bz_water_cool);
  deviceManager.add_executor(&bz_heater_valve_re_o);
  deviceManager.add_executor(&bz_heater_valve_re_c);
  deviceManager.add_executor(&bz_vent_re);
  deviceManager.add_executor(&bz_heater_valve_in_c);
  deviceManager.add_executor(&bz_heater_valve_in_e);
  deviceManager.add_executor(&bz_tf_in);
  
  deviceManager.add_sensor(&bz_pc);
  deviceManager.add_sensor(&bz_t1);
  deviceManager.add_sensor(&bz_h1);
  deviceManager.add_sensor(&bz_t2);
  deviceManager.add_sensor(&bz_h2);
  
  deviceManager.add_executor(&i1_vent_in);
  deviceManager.add_executor(&i1_vent_out);
  deviceManager.add_executor(&i1_valve_air_in);
  deviceManager.add_executor(&i1_valve_air_re);
  deviceManager.add_executor(&i1_valve_water);
  deviceManager.add_executor(&i1_heater_a1);
  deviceManager.add_executor(&i1_heater_a2);
  deviceManager.add_executor(&i1_heater_a3);

  deviceManager.add_sensor(&i1_t1);
  deviceManager.add_sensor(&i1_h1);
  deviceManager.add_sensor(&i1_t2);
  deviceManager.add_sensor(&i1_h2);
  deviceManager.add_sensor(&i1_t3);
  deviceManager.add_sensor(&i1_h3);
  deviceManager.add_sensor(&i1_t4);
  deviceManager.add_sensor(&i1_h4);
  deviceManager.add_sensor(&i1_co1);
    
  deviceManager.add_argument(&bz_min_t);
  deviceManager.add_argument(&bz_max_t);
  deviceManager.add_argument(&bz_min_h);
  deviceManager.add_argument(&bz_max_h);
  
  deviceManager.add_argument(&i1_min_t);
  deviceManager.add_argument(&i1_max_t);
  deviceManager.add_argument(&i1_min_h);
  deviceManager.add_argument(&i1_max_h);
  deviceManager.add_argument(&i1_min_co2);
  deviceManager.add_argument(&i1_max_co2);
  deviceManager.add_argument(&bz_ft_freq);
  i1_min_t.reset();
  i1_max_t.reset();
  i1_min_h.reset();
  i1_max_h.reset();
  i1_max_co2.reset();
  bz_ft_freq.reset();
}

void loop()
{
  ts = millis();
  
  deviceManager.poll();
  deviceManager.log();
  
  if (!ISMASTER) {
    return;
  }
    
    //deviceManager.display();
    
  bz_tf_in.off();
    
    /*if (bz_pc.enabled())
      bz_tf_in.on(bz_t1.get()/10);
    else
      bz_tf_in.off();*/
    //bz_tf_in.off();
   
  //BZ Temperature
  int bz_t_val = bz_t2.get();
  if (bz_t_val < (bz_min_t.get()+bz_max_t.get())/2)
    bz_vent_re.on();
  else if (bz_t_val > bz_max_t.get())
    bz_vent_re.off();
    
  //BZ Humidity
  int bz_h_val = bz_h1.get();
  if (bz_h_val < (bz_min_h.get()+bz_max_h.get())/2) {
    if (bz_tf_in.enabled())
      bz_water_in.on();
    if (bz_vent_re.enabled())
      bz_water_re.on();
  }
  else if (bz_h_val > bz_max_h.get()) {
    bz_water_in.off();
    bz_water_re.off();
  }

  int i1_t_val = (i1_t1.get()+i1_t2.get()+i1_t3.get()+i1_t4.get())/4;
  int i1_h_val = (i1_h1.get()+i1_h2.get()+i1_h3.get()+i1_h4.get())/4;
  int i1_co_val = i1_co1.get();
    
    
  Serial.println(i1_t_val);
  Serial.println(i1_h_val);
  Serial.println(i1_co_val);
    
  i1_vent_in.on();
    
  if (i1_mode == 0) {
    //RE mode
    Serial.print("RE mode; ");
    bz_tf_in.off();
    i1_vent_out.off();
    i1_valve_air_in.off();
    i1_valve_air_re.on();
      
    if (ts < i1_mode_ts || ts - i1_mode_ts > 300000)
      if (i1_t_val>i1_max_t.get() || i1_h_val>i1_max_h.get() || i1_co_val>i1_max_co2.get()) {
        i1_mode_ts = ts;
        i1_mode = 1; 
      }
  }
  else {
    //VENT mode
    Serial.print("VENT mode; ");
    bz_tf_in.on(bz_ft_freq.get());
    i1_vent_out.on();
    i1_valve_air_in.on();
    i1_valve_air_re.off();
      
    if (ts < i1_mode_ts || ts - i1_mode_ts > 120000)
      if (i1_t_val<i1_max_t.get() && i1_h_val<i1_max_h.get() && i1_co_val<i1_min_co2.get()) {
        i1_mode_ts = ts;
        i1_mode = 0;
      }
  }
  
  //Heating  
  if (i1_t_val < (i1_min_t.get()+i1_max_t.get())/2) {
    Serial.print("Heating; ");
    int period = 21000;
    float percentage = 0.8;
    if ((i1_min_t.get()+i1_max_t.get())/2 - i1_t_val < 100)
      percentage = ((i1_min_t.get()+i1_max_t.get())/2 - i1_t_val)/150+0.1;
    if ((ts % period) < period*percentage)
      i1_heater_a1.off();
    else
      i1_heater_a1.on();
    if (((ts+period/3) % period) < period*percentage)
      i1_heater_a2.off();
    else
      i1_heater_a2.on();
    if (((ts+2*period/3) % period) < period*percentage)
      i1_heater_a3.off();
    else
      i1_heater_a3.on();
  }
  else {
    i1_heater_a1.on();
    i1_heater_a2.on();
    i1_heater_a3.on();
  }
  
  //Humiditing
  if (i1_h_val < (i1_min_h.get()+i1_max_h.get())/2)
    i1_valve_water.on();
  else if (i1_h_val > i1_max_h.get())
    i1_valve_water.off();
}
