#include "Arduino.h"
#include "mushnet.h"
#include <DHT.h>
#include <EEPROM.h>
//#include <ModbusRtu.h>

#define SSerialRX        50
#define SSerialTX        52
#define SSerialTxControl 34
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define RS485StartByte   27
#define ANALOG_BUTTONS_PIN	 A0
#define DISPLAY_SENSOR_MODE      0
#define DISPLAY_EXECUTOR_MODE      1
#define DISPLAY_CONFIG_MODE      2
#define DISPLAY_RESET_TO_DEFAULTS_MODE      3
#define DISPLAY_LIGHT_PIN   10
#define LED_PIN          13
#define MAX_SKIP_COUNT   20
#define TRANSFER_WAIT    5000
#define DEVICE_TIMEOUT   300000

/* ========================== */
/*           BASICS           */
/* ========================== */


Device::Device(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: id(id)
	, name(name)
	, host(host)
	, localhost(localhost)
	, pin(pin)
	, au16ptr(au16data + id)
	, is_local(host == localhost)
	, ts(0)
	, waiting(true)
	, device_type(Device::NONE)
{}

int16_t Device::get()
{
	return *(this->au16ptr);
}

void Device::mark_ts()
{
	this->ts = millis();
}

bool Device::is_waiting()
{
	if (this->ts > millis())
		this->mark_ts();
	if (this->waiting)
		return true;
	return ((millis() - this->ts) > DEVICE_TIMEOUT);
}

Executor::Executor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: Device(id, name, host, localhost, pin, au16data)
	, state(Executor::NONE)
{}

void Executor::set(int16_t val)
{
	if (this->is_local) {
		digitalWrite(this->pin, val);
	}
	else if (*(this->au16ptr) != val) {
		this->mark_ts();
		this->waiting = true;
	}
	*(this->au16ptr) = val;
}

void Executor::write()
{
	digitalWrite(this->pin, this->get());
}

Sensor::Sensor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: Device(id, name, host, localhost, pin, au16data)
{
	device_type = Device::SENSOR;
}

/* ========================== */
/*         REALIZATION        */
/* ========================== */

Relay::Relay(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: Executor(id, name, host, localhost, pin, au16data)
{
	if (this->is_local)
		pinMode(pin, OUTPUT);
	this->off();
}

void Relay::on()
{
	this->set(LOW);
}

void Relay::off()
{
	this->set(HIGH);
}

bool Relay::enabled()
{
	return (this->get() == LOW);
}

TransformerFrequency::TransformerFrequency(byte id, char* name, byte host, byte localhost, uint16_t* au16data)
	: Executor(id, name, host, localhost, 0, au16data)
{
	this->device_type = Device::TRANSFORMAERFREQUENCY;
	this->off();
}

void TransformerFrequency::on(int16_t freq) {
	this->set(freq);
}

void TransformerFrequency::off() {
	this->set(0xFF00);
}

bool TransformerFrequency::enabled()
{
	return (this->get() != 0xFF00);
}

Switch::Switch(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: Sensor(id, name, host, localhost, pin, au16data)
{
	if (this->is_local)
		pinMode(pin, INPUT_PULLUP);
}

void Switch::read() {
	*(this->au16ptr) = digitalRead(this->pin);
}

bool Switch::enabled() {
	return (this->get() == LOW);
}

SensorDHT22Temperature::SensorDHT22Temperature(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: Sensor(id, name, host, localhost, pin, au16data)
	, dht(pin, DHT22)
{
	if (this->is_local)
		dht.begin();
}

void SensorDHT22Temperature::read() {
	if (!this->is_local)
		return;
	float t = this->dht.readTemperature();
	if (isnan(t))
		return;
	*(this->au16ptr) = int(t*100);
}

DHT* SensorDHT22Temperature::get_dht() {
	return &(this->dht);
}

SensorDHT22Humidity::SensorDHT22Humidity(byte id, char* name, byte host, byte localhost, DHT* dht, uint16_t* au16data)
	: Sensor(id, name, host, localhost, 0, au16data)
{
	this->dht = dht;
}

void SensorDHT22Humidity::read() {
	float h = this->dht->readHumidity();
	if (isnan(h)) {
		return;
	}
	*(this->au16ptr) = int(h*100);
}

AnalogSensor::AnalogSensor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: Sensor(id, name, host, localhost, pin, au16data)
{
	if (this->is_local)
		analogReference(INTERNAL2V56);
}

void AnalogSensor::read()
{
	*(this->au16ptr) = analogRead(this->pin);
}

CO2Sensor::CO2Sensor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data)
	: Sensor(id, name, host, localhost, pin, au16data)
{
	if (this->is_local)
		pinMode(pin, INPUT_PULLUP);
}

void CO2Sensor::read()
{
	unsigned long pwm_in = pulseIn(this->pin, HIGH, 2000000)/1000;
	if (pwm_in == 0)
		pwm_in = 2000;
	//*(this->au16ptr) = pwm_in;
	*(this->au16ptr) = ((*(this->au16ptr))*9 + pwm_in)/10;
}

Argument::Argument(byte id, char* name, int default_value)
	: id(id)
	, name(name)
	, default_value(default_value)
	, val(0)
{
	int addr = id*4;
	byte b;
	for (int i=0; i<4; ++i) {
		b = EEPROM.read(addr+i);
		val = val + (b << (8*i));
	}
}

int Argument::get()
{
	return val;
}

void Argument::set(int new_val)
{
	if (this->val == new_val)
		return;
	this->val = new_val;
	for (int i=0; i<4; ++i) {
		byte b = (new_val >> 8*i) & 0xFF;
		EEPROM.write(id*4+i, b);
	}
}

void Argument::reset()
{
	this->set(this->default_value);
}

AnalogButton::AnalogButton(byte id)
	: id(id)
	, pressed(AnalogButton::NONE)
	, clicked(AnalogButton::NONE)
{}

void AnalogButton::refresh()
{
	int input = analogRead(id);
	if (input > 1000) {
		if (pressed != AnalogButton::NONE)
			clicked = pressed;
		else
			clicked = AnalogButton::NONE;
		pressed = AnalogButton::NONE;
		return;
	}

	if (input < 100)
		pressed = AnalogButton::RIGHT;
	else if (input < 200)
		pressed = AnalogButton::UP;
	else if (input < 400)
		pressed = AnalogButton::DOWN;
	else if (input < 650)
		pressed = AnalogButton::LEFT;
	else
		pressed = AnalogButton::MODE;

	//Serial.println(pressed);
}

DeviceManager::DeviceManager(byte host, bool is_master, uint16_t* au16data)
	: lcd(8, 9, 4, 5, 6, 7)
	, display_wait(millis())
	, buttons(ANALOG_BUTTONS_PIN)
	, display_mode(DISPLAY_SENSOR_MODE)
	, display_position(0)
	, executors_count(0)
    , sensors_count(0)
	, executors_rotation(0)
	, sensors_read_rotation(0)
	, sensors_rotation(0)
    , arguments_count(0)
	, host(host)
	, is_master(is_master)
	, au16data(au16data)
	, transfer_state(0)
	, modbus(host, 1, SSerialTxControl)
	, transfer_wait(millis() + TRANSFER_WAIT)
{
	if (is_master) {
		pinMode(DISPLAY_LIGHT_PIN, OUTPUT);
		digitalWrite(DISPLAY_LIGHT_PIN, HIGH);
		lcd.begin(16, 2);
		lcd.setCursor(0,0);
		lcd.print("   9Lab v0.1");
		lcd.setCursor(0,1);
		lcd.print("================");
	}
	modbus.begin(1200);
	modbus.setTimeOut(2000);
	pinMode(LED_PIN, OUTPUT);
	pinMode(SSerialTxControl, OUTPUT);
}

void DeviceManager::add_executor(Executor *executor)
{
	this->executors[this->executors_count] = executor;
	++this->executors_count;
}

void DeviceManager::add_sensor(Sensor *sensor)
{
	this->sensors[this->sensors_count] = sensor;
	++this->sensors_count;
}

void DeviceManager::add_argument(Argument *argument)
{
	this->arguments[this->arguments_count] = argument;
	++this->arguments_count;
}

void DeviceManager::poll()
{
	digitalWrite(LED_PIN, HIGH);

	if (this->is_master) {
		bool br;
		switch( this->transfer_state ) {
		case 0:
			if (millis() > this->transfer_wait)
				this->transfer_state = 1;
			break;
		case 1:
			br = false;
			for (int i=0; i<this->executors_count; ++i) {
				Executor* executor = this->executors[i];
				if (executor->is_local || !executor->is_waiting())
					continue;

				executor->waiting = false;
				modbus_t telegram;
				telegram.u8id = executor->host; // slave address
				telegram.u8fct = 6; // function code
				if (executor->device_type == Device::TRANSFORMAERFREQUENCY) {
					if (executor->get()==0xFF00) {
						Serial.println("TRANSFORMAERFREQUENCY STOP");
						telegram.u8fct = 5;
						telegram.u16RegAdd = 1;
						executor->state = Executor::STOPPED;
					}
					else if (executor->state!=Executor::RUNNED && executor->get()!=0xFF00) {
						Serial.println("TRANSFORMAERFREQUENCY START");
						telegram.u8fct = 5;
						telegram.u16RegAdd = 0;
						executor->state = Executor::RUNNED;
						executor->waiting = true;
					}

					else {
						Serial.println("TRANSFORMAERFREQUENCY FREQ");
						telegram.u16RegAdd = 2;
						executor->state = Executor::ADJUSTED;
					}
				}
				else {
					telegram.u16RegAdd = executor->id; // start address in slave
				}
				telegram.u16CoilsNo = 1; // number of elements (coils or registers)
				telegram.au16reg = this->au16data + executor->id;
				this->modbus.query(telegram);
				br = true;
				//Serial.println("SENT");
				//Serial.println(executor->id);
				break;
			}
			if (!br) {
				this->sensors_rotation += 1;
				this->sensors_rotation %= this->sensors_count;
				for (int i=this->sensors_rotation; i<this->sensors_count; ++i) {
					Sensor* sensor = this->sensors[i];
					this->sensors_rotation = i;
					if (sensor->is_local)
						continue;
					modbus_t telegram;
					telegram.u8id = sensor->host; // slave address
					telegram.u8fct = 4; // function code
					telegram.u16RegAdd = sensor->id; // start address in slave
					telegram.u16CoilsNo = 1; // number of elements (coils or registers)
					telegram.au16reg = this->au16data + sensor->id;
					this->modbus.query(telegram);
					//Serial.println("REQUESTED");
					//Serial.println(sensor->id);
					break;
				}
			}
			this->transfer_state = 2;
			break;
		case 2:
			this->modbus.poll(); // check incoming messages
			//Serial.println(this->modbus.getInCnt());
			//Serial.println(this->modbus.getOutCnt());
			if (this->modbus.getState() == COM_IDLE) {
		    	this->transfer_wait = millis() + TRANSFER_WAIT;
		    	this->transfer_state = 0;
		    }
			break;
		}
	}
	else
		this->modbus.poll(this->au16data, AU16DATA_SIZE);

	this->sensors_read_rotation += 1;
	this->sensors_read_rotation %= this->sensors_count;
	for (int i=this->sensors_read_rotation; i<this->sensors_count; ++i) {
		this->sensors_read_rotation = i;
		if (!this->sensors[i]->is_local)
			continue;
		this->sensors[i]->read();
		break;
	}
	for (int i=0; i<this->executors_count; ++i) {
		if (this->executors[i]->is_local)
			this->executors[i]->write();
	}
	digitalWrite(SSerialTxControl, RS485Receive);
	digitalWrite(LED_PIN, LOW);
}

void DeviceManager::log() {
	Serial.print("Executors: ");
	for (int i=0; i<this->executors_count; ++i) {
		Executor* e = this->executors[i];
		Serial.print(e->name);
		Serial.print(": ");
		Serial.print(e->get());
		Serial.print("; ");
	}
	Serial.print("\n");
	Serial.print("Sensors: ");
	for (int i=0; i<this->sensors_count; ++i) {
		Sensor* e = this->sensors[i];
		Serial.print(e->name);
		Serial.print(":");
		Serial.print(e->get());
		Serial.print("; ");
	}
	Serial.print("\n");
}

void DeviceManager::display()
{
//	if (!is_master)
//		return;
//
//	buttons.refresh();
//
//	if (buttons.pressed != AnalogButton::NONE) {
//		this->display_wait = millis();
//		digitalWrite(DISPLAY_LIGHT_PIN, HIGH);
//	}
//	if (millis() > this->display_wait + 600000) {
//		digitalWrite(DISPLAY_LIGHT_PIN, LOW);
//		return;
//	}
//
//	if (buttons.clicked == AnalogButton::MODE) {
//		this->display_mode = (this->display_mode+1) % 4;
//		display_position = 0;
//	}
//	else if (buttons.pressed == AnalogButton::UP) {
//		--display_position;
//	}
//	else if (buttons.pressed == AnalogButton::DOWN) {
//		++display_position;
//	}
//	else if (buttons.pressed == AnalogButton::LEFT) {
//		if (this->display_mode == DISPLAY_CONFIG_MODE) {
//			Argument* arg = this->arguments[display_position % arguments_count];
//			int v = arg->get();
//			v = v - 50;
//			arg->set(v);
//		}
//	}
//	else if (buttons.pressed == AnalogButton::RIGHT) {
//		if (this->display_mode == DISPLAY_RESET_TO_DEFAULTS_MODE) {
//			for (byte i=0; i<arguments_count; ++i) {
//				this->arguments[i]->reset();
//			}
//			this->display_mode = DISPLAY_SENSOR_MODE;
//			display_position = 0;
//		}
//		else if (this->display_mode == DISPLAY_CONFIG_MODE) {
//			Argument* arg = this->arguments[display_position % arguments_count];
//			int v = arg->get();
//			v = v + 50;
//			arg->set(v);
//		}
//	}
//
//	lcd.clear();
//	for (byte r=0; r<2; ++r) {
//		if (this->display_mode == DISPLAY_SENSOR_MODE) {
//			byte i = (display_position*2+r) % sensors_count;
//			lcd.setCursor(0,r); lcd.print("S");
//			lcd.setCursor(1,r); lcd.print(i);
//			lcd.setCursor(3,r); lcd.print(this->sensors[i]->name);
//			lcd.setCursor(10,r); lcd.print(this->sensors[i]->get());
//		}
//		else if (this->display_mode == DISPLAY_EXECUTOR_MODE) {
//			byte i = (display_position*2+r) % executors_count;
//			lcd.setCursor(0,r); lcd.print("E");
//			lcd.setCursor(1,r); lcd.print(i);
//			lcd.setCursor(3,r); lcd.print(this->executors[i]->name);
//			lcd.setCursor(10,r); lcd.print(this->executors[i]->get());
//		}
//	}
//	if (this->display_mode == DISPLAY_CONFIG_MODE) {
//		byte i = display_position % arguments_count;
//		lcd.setCursor(0,0); lcd.print("C");
//		lcd.setCursor(1,0); lcd.print(i);
//		lcd.setCursor(3,0); lcd.print(this->arguments[i]->name);
//		lcd.setCursor(0,1); lcd.print("<-");
//		lcd.setCursor(4,1); lcd.print(this->arguments[i]->get());
//		lcd.setCursor(14,1); lcd.print("->");
//	}
//	else if (this->display_mode == DISPLAY_RESET_TO_DEFAULTS_MODE) {
//		lcd.setCursor(0,0); lcd.print("| PRESS  RIGHT |");
//		lcd.setCursor(0,1); lcd.print("| FOR DEFAULTS |");
//	}
}
