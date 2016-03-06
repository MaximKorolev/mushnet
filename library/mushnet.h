#ifndef mushnet_h
#define mushnet_h

#define AU16DATA_SIZE    100

//#include "Arduino.h"
//#include <ModbusRtu.h>
//#include <DHT.h>
//#include <LiquidCrystal.h>

fhfhgfhhgf

class Device
{
public:
	enum Type {
			NONE, SENSOR, EXECUTOR, TRANSFORMAERFREQUENCY
	};
	Device(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
	int16_t get();
	bool is_waiting();
	void mark_ts();

	const byte id, host, localhost, pin;
	const bool is_local;
	unsigned long ts;
	bool waiting;
	char* name;
	Type device_type;
	uint16_t* au16ptr;
};

class Executor: public Device
{
public:
	enum States {
		NONE, STOPPED, RUNNED, ADJUSTED
	};
	Executor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
	void set(const int16_t val);
	void write();
	byte state;
};

class Sensor: public Device
{
public:
    Sensor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
    virtual void read() {};
};

class Relay: public Executor
{
public:
	Relay(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
    void on();
    void off();
    bool enabled();
};

class TransformerFrequency: public Executor
{
public:
	TransformerFrequency(byte id, char* name, byte host, byte localhost, uint16_t* au16data);
    void on(int16_t freq);
    void off();
    bool enabled();
};

class Switch: public Sensor
{
public:
	Switch(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
	void read();
	bool enabled();
};

class SensorDHT22Temperature: public Sensor
{
public:
	SensorDHT22Temperature(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
	void read();
	DHT* get_dht();
private:
	DHT dht;
};

class SensorDHT22Humidity: public Sensor
{
public:
	SensorDHT22Humidity(byte id, char* name, byte host, byte localhost, DHT* dht, uint16_t* au16data);
	void read();
private:
	DHT* dht;
};

class AnalogSensor: public Sensor
{
public:
	AnalogSensor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
	void read();
};

class CO2Sensor: public Sensor
{
public:
	CO2Sensor(byte id, char* name, byte host, byte localhost, byte pin, uint16_t* au16data);
	void read();
};

class Argument
{
public:
	Argument(byte id, char* name, int default_value);
	int get();
	void set(int val);
	void reset();
	char* name;
private:
	int val, default_value;
	byte id;
};

class AnalogButton
{
public:
	enum BTN {
			RIGHT, UP, DOWN, LEFT, MODE, NONE
		};
	AnalogButton(byte id);
	void refresh();
	byte clicked, pressed;
private:
	byte id;
};

class DeviceManager
{
	public:
		DeviceManager(byte host, bool is_master, uint16_t* au16data);
		void add_executor(Executor *executor);
		void add_sensor(Sensor *sensor);
		void add_argument(Argument *argument);

		void poll();
		void log();
		void display();
		void reset_to_default();

	private:
		Executor* executors[500];
		Sensor* sensors[500];
		Argument* arguments[100];
		size_t executors_count, executors_rotation;
		size_t sensors_count, sensors_rotation, sensors_read_rotation;
		size_t arguments_count;
		bool is_master;
		int host;
		LiquidCrystal lcd;
		unsigned long display_wait;
		AnalogButton buttons;
		byte display_mode, display_position, transfer_state;
		unsigned long transfer_wait;

		uint16_t* au16data;
		Modbus modbus;
};

#endif device_h
