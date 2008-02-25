#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
#include <avrlib/portio.h>
#include <avrlib/spi.h>
#include <avrlib/interrupt.h>

//
// Kicker-1.0
// 

using namespace AVRLIB;

#define CLOCK F_OSC    /* clock rate of microcontroller (in Hz) */
#define SECOND_OCR ((uint16_t) (((unsigned long)(CLOCK) / 256L) / 10L))
#define CHECK_TIME(us) ((uint8_t) ((us) *((((unsigned long)(CLOCK) / 256L) / 8L) / 1000L)))
#define CHECK_LONG_TIME(us) ((uint8_t) ((us) *((((unsigned long)(CLOCK) / 256L) / 8L) / 10L)))

// Short Freq = 1/(889 us) = 1124 Hz
// Long  Freq = 1/(1778 us) = 562 Hz
#define IRC_SHORT ((uint8_t) ((((unsigned long)(CLOCK) / 256L) / 1124L)))
#define IRC_LONG ((uint8_t) ((((unsigned long)(CLOCK) / 256L) / 562L)))
#define IRC_MAX ((uint8_t) (((((((unsigned long)(CLOCK) / 256L) / 8L) / 562L) + 1))))

using IO::Bit;
using IO::Port;

struct Fifo
{
	static const uint8_t max = 14;
	uint8_t buff[max];
	uint8_t tail;
	uint8_t head;
	
	Fifo (): tail(0), head(0) {}
	
	void fix_head()
	{
		head = head < max ? head : 0;
	}
	void fix_tail()
	{
		tail = tail < max ? tail : 0;
	}
	bool empty()
	{
		return head == tail;
	}
	void push(uint8_t data)
	{
		buff[head++] = data;
		fix_head();
	}
	uint8_t pop()
	{
		
		uint8_t ret = buff[tail++];
		fix_tail();
		return ret;
	}
};

//Fifo fifo;

template <class Strobe, class Enable>
class Indicator
{
	static const uint8_t Leds[];
	volatile int8_t Line[3*6];
	volatile bool flag;
	volatile bool overflow;
	volatile bool spi_in_progress;
	volatile bool spi_done;
	
	uint8_t send_index;
	uint8_t send_number;
	uint8_t send_digit;
public:
	static void init ()
	{
		Spi::init (Spi::mode_master, Spi::rate_64, Spi::isr_enable);	
	}
	Indicator (): flag(true), overflow(false), spi_in_progress(false)
	{
		Strobe::init();
		Enable::init();
	}
	void display ();
	void display_next ();
	void display_stop ();
	void inc (uint8_t index, int8_t middle = 5);
	void inc10 (uint8_t index, int8_t middle = 5);
	void inc100 (uint8_t index);
	void dec (uint8_t index, int8_t middle = 5);
	void dec100 (uint8_t index, int8_t middle = 5);
	bool ready ()
	{
		if (overflow) {
			overflow = false;
			return true;
		}

		return false;
	}
	bool display_in_progress ()
	{
		return spi_in_progress;
	}
	int16_t get (uint8_t index, uint8_t high = 100);
	void put (uint8_t index, int16_t digit);
	int16_t total ()
	{
		return get(0) + get(1) + get(2) + get(3) + get(4);
	}
	int16_t time ()
	{
		return get(5, 60);
	}
	void time (int16_t t)
	{
		return put(5, t*100/60);
	}
	void zero (uint8_t index)
	{
		Line[index * 3] = 0;
		Line[index * 3 + 1] = 0;
		Line[index * 3 + 2] = 0;

		flag = true;
	}	
	void minus (uint8_t index, uint8_t middle = 0x11)
	{
		Line[index * 3] = 0x11;
		Line[index * 3 + 1] = middle;
		Line[index * 3 + 2] = 0x11;

		flag = true;
	}	
	void mode (uint8_t mode)
	{
		minus (5, mode + 0xA);
	}	
	void reset ()
	{
		for (int i = 0; i < 6*3; i++)
			Line[i] = 0;
		overflow = false;
		flag = true;
		spi_in_progress = false;
		Spi::stop();
	}	
	void kick (uint8_t index) { inc (index, 9); }
};

class Indicators
{
public:
	static void init ()
	{
		Spi::init (Spi::mode_master, Spi::rate_64, Spi::isr_enable);	
	}
};

template <class Strobe, class Enable>
const uint8_t Indicator<Strobe, Enable>::Leds[] =
{
	0x5f,
	0x42,
	0x37,
	0x76,
	0x6a,
	0x7c,
	0x7d,
	0x46,
	0x7f,
	0x7e,
	0x6f,
	0x79,
	0x1d,
	0x73,
	0x3d,
	0x2d,
    	0x00,
	0x20
};

template <Port port,
          IO::Bit  bit1,
          IO::Bit  bit2,
          IO::Bit  bit3,
          IO::Bit  bit4,
          IO::Bit  bit5>
class Sensor
{
	typedef uint8_t sensor_type;
	static const sensor_type sensor1 = MAKEBIT(bit1);
	static const sensor_type sensor2 = MAKEBIT(bit2);
	static const sensor_type sensor3 = MAKEBIT(bit3);
	static const sensor_type sensor4 = MAKEBIT(bit4);
	static const sensor_type sensor5 = MAKEBIT(bit5);
	static const sensor_type sensors_mask = sensor1|sensor2|sensor3|sensor4|sensor5;
protected:
	sensor_type checkSensors ()
	{
		sensor_type mask = sensors::get();

		if ((mask & sensor1) == 0) 
			return sensor1;
		if ((mask & sensor2) == 0) 
			return sensor2;
		if ((mask & sensor3) == 0) 
			return sensor3;
		if ((mask & sensor4) == 0) 
			return sensor4;
		if ((mask & sensor5) == 0) 
			return sensor5;

		return 0;
	}

	typedef Inputs<port, sensors_mask, false> sensors;
public:
	Sensor (): state(Wait)
	{
		sensors::init(true);
	}

	template <class IndicatorType>
	void poll (IndicatorType &indicator);
	void check ()
	{
		if(checkTimer != 0)
			--checkTimer;
	}
	void reset ()
	{
		state = Wait;
		last_sensor = 0;
		checkTimer = 0;
	}

private:
	enum State
	{
		Wait,
		Check,
		Kick,
		Dead
	};

	State state;
	sensor_type last_sensor;
	uint8_t checkTimer;
};

template <class input>
class Button
{
public:
	Button (bool pullout = true): state(Wait)
	{
		input::init(pullout);
	}

	void poll ();
	void check ()
	{
		if(checkTimer != 0)
			--checkTimer;
	}
	void reset () 
	{
		pressed = false;
		state = Wait;
	}
	bool get (bool reinit = true)
	{
		if (pressed) {
			if (reinit)
//				reset();
				pressed = false;
			return true;
		}
		
		return false;
	}

private:
	enum State
	{
		Wait,
		Check,
		Press,
		Dead
	};

	volatile State state;
	volatile bool pressed;
	volatile uint8_t checkTimer;
};


class Dynamic
{
public:
	static void init()
	{
		checkTimer = 0;
		TCCR2 = _BV(COM20) | _BV(CTC2);
		DDRD |= _BV(PD7);
	}

	static void check ()
	{
		if (checkTimer == 0) {
			if (checkNum == 0) {
				stop();
				return;
			}
			
			if (--checkNum & 1)
				restart();
			else
				stop();
		} else
			--checkTimer;
	}

	static void reset()
	{
		init();
	}

	static void start (uint8_t time = 10, uint8_t num = 1)
	{
	//	if (time == 1 && checkTimer == 1 && num == 1)
	//		checkTimer = 2, lastCheckTimer = 1;
	//	else 
			lastCheckTimer = checkTimer = time;
		checkNum = num;
		run();
	}

	static void restart ()
	{
		checkTimer = lastCheckTimer;
		run();
	}
	
	static void run () {
		//// Prescale 64 - 8000000/64/125 = 1000Hz
		//TCCR2 |= _BV(CS22);

		//// Prescale 32 - 8000000/32/125 = 2000Hz
		TCCR2 |= _BV(CS21) | _BV(CS20);		
		OCR2 = 125;
		
		//// Prescale 32 - 8000000/8/250 = 4000Hz
		//TCCR2 |= _BV(CS21);
		//OCR2 = 250;
	}
	
	static void stop () 
	{
		TCCR2 &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20));
	}
private:
	static volatile uint8_t checkTimer;
	static uint8_t lastCheckTimer;
	static volatile uint8_t checkNum;
};

volatile uint8_t Dynamic::checkTimer;
uint8_t Dynamic::lastCheckTimer;
volatile uint8_t Dynamic::checkNum;

class IRController
{
	// message size in this IR Controller Extended RC-5 protocol is 14 bits
	//  1 start bit (second bit in this implemetation is high bit of command)
	// +
	//  1 toggle bit
	// +
	//  5 address bits
	// +
	//  7 command bits
	// ---------------
	// 14 message size
	enum Bits
	{
		InitBit,
		CmdBit_6 = InitBit,  // bit S2 in extended RC5 uses as 6 bit of command
		ToggleBit, // toggle bit
		AddrBit_4, // MSB of address bit
		AddrBit_3, //
		AddrBit_2, //
		AddrBit_1, //
		AddrBit_0, // LSB of address bit
		CmdBit_5,  // MSB of standart command bit
		CmdBit_4,  //
		CmdBit_3,  //
		CmdBit_2,  //
		CmdBit_1,  //
		CmdBit_0,  // LSB of command bit
		LastBit = CmdBit_0
	};

	enum Codes
	{
		Digit_0 = 0x40, // 0
		Digit_1 = 0x41, // 1
		Digit_2 = 0x42, // 2
		Digit_3 = 0x43, // 3
		Digit_4 = 0x44, // 4
		Digit_5 = 0x45, // 5
		Digit_6 = 0x46, // 6
		Digit_7 = 0x47, // 7
		Digit_8 = 0x48, // 8
		Digit_9 = 0x49, // 9
		OneTwo  = 0x4A, // -/-- 
		Power   = 0x4C, // POWER (StandBy)
		Mute    = 0x4D, // MUTE
		PStd    = 0x4E, // P.STD
		Display = 0x4F, // DISPLAY
		Menu    = 0x12, // MENU
		PSize   = 0x56, // P.SIZE
		Sleep   = 0x66, // SLEEP
		Video   = 0x78, // VIDEO
		TV      = 0x7F, // TV
		Up      = 0x10, // Arrow Up    - CH/P
		Down    = 0x11, // Arrow Down  - CH/P
		Left    = 0x15, // Arrow Left  - Vol-
		Right   = 0x16, // Arrow Right - Vol+
		Unknown = 0xFF
	};

public:
	IRController (): state(Wait)
	{
		resetAll();
asm ("; --start-- IRController");
//		ExternalInterrupt0::init (Interrupt::falling_edge);
//		ExternalInterrupt0::init (Interrupt::rising_edge);
asm ("; --xxx-- IRController");
//		ExternalInterrupt1::init (Interrupt::rising_edge);
//		ExternalInterrupt1::init (Interrupt::falling_edge);

//	GIMSK |= _BV(INT0);
//	MCUCR |= _BV(ISC01);
//	MCUCR &= ~_BV(ISC00);
//	GIMSK |= _BV(INT1);
//	MCUCR |= _BV(ISC11);
//	MCUCR |= _BV(ISC10);

	GIMSK = 0xc0;
	MCUCR = 0xe;
asm ("; --end-- IRController");
	}

	void poll ();
	void check ()
	{
		if (checkTimer != 0xff)
			checkTimer++;
	}
	void reset () 
	{
		state = Wait;
		offset = InitBit;
//		data.data = 0;
	}
	void resetAll () 
	{
		reset();
		last_cmd = 0;
		last_toggle = 0;
	}
	void signal (uint8_t);
	bool ready (bool reinit = true)
	{
		if (state == Ready) {
			if (reinit)
				reset();
			return true;
		}

		return false;
	}
private:
	bool readyButton (Codes code, bool again = false)
	{
		if (ready(false) && cmd == code) {
			if (!again && last_cmd == cmd && last_toggle == toggle)
				return false;
			last_cmd = cmd;
			last_toggle = toggle;
			reset();
			return true;
		}

		return false;
	}
public:
	bool readyStart ()
	{
		return readyButton (Menu);
	}
	bool readyReset ()
	{
		return readyButton (Power);
	}
	bool readyMinutePlus ()
	{
		return readyButton (Up);
	}
	bool readyMinuteMinus ()
	{
		return readyButton (Down);
	}
	bool readyMute ()
	{
		return readyButton (Mute, true);
	}
	bool readyMode ()
	{
		return readyButton (PSize);
	}

private:
	enum State
	{
		Wait,
		Mid1,
		Start1,
		Mid2,
		Start2,
		Ready,
		Error
	};

	enum Signal
	{
		Short,
		Long,
		VeryLong,
		VeryShort
	};
	
	State delta (Signal sig)
	{
		if (sig == VeryLong) {
			reset ();
			return Mid1;
		}

		if (sig == VeryShort)
			return Error;
		
asm ("; --start-- IRController::delta");
		if (state == Wait)
			return Wait;
		else if (state == Ready)
			return Ready;
		else if (state == Error)
			return Error;
		else if (state == Mid1) {
			if (sig == Long) {
				if (emit (false))
					return Ready;
				return Mid2;
			}
			if (sig == Short)
				return Start1;
		} else if (state == Start1) {
			if (sig == Short) {
				if (emit (true))
					return Ready;
				return Mid1;
			}
		} else if (state == Mid2) {
			if (sig == Long) {
				if (emit (true))
					return Ready;
				return Mid1;
			}
			if (sig == Short)
				return Start2;
		} else if (state == Start2) {
			if (sig == Short) {
				if (emit (false))
					return Ready;
				return Mid2;
			}
		}
asm ("; --end-- IRController::delta");
		
		return Error;		
	}

	bool emit (bool);	


//public:

//	union raw {
//		uint16_t data;
//		uint8_t byte[2];
//	};

	volatile uint8_t cnt;
	volatile uint8_t addr;
	volatile uint8_t cmd;
	volatile bool toggle;
	volatile uint8_t last_cmd;
	volatile bool last_toggle;
	volatile State state;
	volatile uint8_t checkTimer;
	volatile uint8_t offset;
//	volatile raw data;
};

class System
{
	typedef Input<IO::port_B, IO::bit_0, false> StartInput;
	typedef Input<IO::port_D, IO::bit_4, false> ResetInput;
	typedef Input<IO::port_D, IO::bit_5, false> ModeInput;
	typedef Input<IO::port_D, IO::bit_6, false> MinuteInput;

//	typedef Input<IO::port_D, IO::bit_0, false> UART_rxd;
//	typedef Input<IO::port_D, IO::bit_1, false> UART_txd;
//	typedef Input<IO::port_D, IO::bit_2, false> IR_int0;
//	typedef Input<IO::port_D, IO::bit_3, false> IR_int1;
	typedef Output<IO::port_D, IO::bit_7, false> DynamicOutput;

	typedef Output<IO::port_B, IO::bit_4, true> RedStrobe;
	typedef Output<IO::port_B, IO::bit_2, true> GreenStrobe;
	typedef Output<IO::port_B, IO::bit_3, true> IndicatorEnable;

	typedef Indicator<RedStrobe, IndicatorEnable> RedIndicator;
	typedef Indicator<GreenStrobe, IndicatorEnable> GreenIndicator;

	typedef Button<StartInput> StartButton;
	typedef Button<ResetInput> ResetButton;
	typedef Button<ModeInput> ModeButton;
	typedef Button<MinuteInput> MinuteButton;

	typedef Sensor<	IO::port_C,
			IO::bit_4,
			IO::bit_6,
			IO::bit_7,
			IO::bit_5,
			IO::bit_3> RedSensor;
	typedef Sensor<	IO::port_A,
			IO::bit_3,
			IO::bit_1,
			IO::bit_0,
			IO::bit_2,
			IO::bit_4> GreenSensor;

	RedIndicator redIndicator;
	GreenIndicator greenIndicator;

	RedSensor redSensor;
	GreenSensor greenSensor;
	
	StartButton startButton;
	ResetButton resetButton;
	ModeButton modeButton;
	MinuteButton minuteButton;

	IRController ircontroller;
	
public:
	System (): state(Wait), mode(0)
	{
		Indicators::init();
		Dynamic::init();
	}

	void poll_sensors()
	{
		if (state == Box) {
			greenSensor.poll(greenIndicator);
			redSensor.poll(redIndicator);
		} else {
			greenSensor.reset();
			redSensor.reset();
		}
		
		startButton.poll();
		resetButton.poll();
		modeButton.poll();
		minuteButton.poll();
	}

	void poll_display()
	{
		if (!redIndicator.display_in_progress())
			greenIndicator.display();
		if (!greenIndicator.display_in_progress())
			redIndicator.display();
	}

	void logic ();
	bool timeset ();
	bool pressed ();
	void result ()
	{
		int16_t red = redIndicator.total();
		int16_t green = greenIndicator.total();
		if (red >= green) {
			redIndicator.put(5,red);
			greenIndicator.put(5,red - green);
			last_result_best = red;
			last_result_worst = green;
		} else {
			redIndicator.put(5,green - red);
			greenIndicator.put(5,green);		
			last_result_best = green;
			last_result_worst = red;
		}
	}
	
	void poll ()
	{
		poll_sensors();

		if (muteEvent())
			Dynamic::start(2);
		
		logic();

//		if (ircontroller.ready()) {
//			redIndicator.put(1,ircontroller.toggle);
//			redIndicator.put(2,ircontroller.addr);
//			redIndicator.put(3,ircontroller.cmd);
//			redIndicator.put(0,ircontroller.cnt);
//			redIndicator.put(4,ircontroller.offset);
//		}
		
		poll_display();

	}
	
	void checkSensors ()
	{
		redSensor.check();
		greenSensor.check();
		
		startButton.check();
		resetButton.check();
		modeButton.check();
		minuteButton.check();

//DynTimer//		if(checkTimer != 0) {
//DynTimer//			if (--checkTimerHelper == 0)
//DynTimer//				--checkTimer;
//DynTimer//		}
//DynTimer//		if(checkTimerHelper == 0)
//DynTimer//			checkTimerHelper = 100;

		ircontroller.check();
	}

	void checkIndicators ()
	{
		greenIndicator.display_next();	
		redIndicator.display_next();	
	}

	void signalIRController ()
	{
		uint16_t cnt = TCNT1;
		static uint16_t last = 0;
		uint16_t dT = 0;
		uint8_t dt;
		
		if (cnt > last)
			dT = cnt - last;
		else
			dT = (SECOND_OCR - last + cnt);
		
		dt = dT > 0xff ? 0xff : dT;
		
		last = cnt;
		static uint8_t max = 0;
		static uint8_t min = 0;
		//if (dt > 0 && dt != 0xff) {
		//	if (dt > max) {
		//		min = max;
		//		max = dt;
		//	} else if (dt > min)
		//		min = dt;
		//}
		//static uint8_t ck = 0;
		//if (ircontroller.checkTimer == 0) {
		//	ck++;
		//	return;
		//}
		//if (ircontroller.checkTimer != ck) {
		//	greenIndicator.put(2,ircontroller.checkTimer);
		//	greenIndicator.put(0,max);
		//	greenIndicator.put(1,min);
		//	greenIndicator.put(0,ircontroller.toggle);
		
		//	if (dt > ((IRC_LONG + IRC_SHORT) / 2L) && max < 5) {
		//		greenIndicator.put(max,dt);
		//		redIndicator.put(max,min);
		//		max++;
		//	}
		//	if (((ircontroller.checkTimer > IRC_MAX || dt > (2L * IRC_LONG))) && max >= 5) min = max = 0;
		//	else min++;
		
		//	redIndicator.put(3,ircontroller.checkTimer);
		//	redIndicator.put(2,((IRC_LONG + IRC_SHORT) / 2L));
		//	redIndicator.put(1,IRC_SHORT);
		//	redIndicator.put(0,IRC_LONG);
		
		//}
		//ck = 
		//ircontroller.checkTimer = 0;
		//ircontroller.signal();

//		if (USR & _BV(UDRE))
//			UDR = dt;
//		else
//			fifo.push (dt);
		ircontroller.signal(dt);
	}

//DynTimer//	void checkInit (uint16_t time = 0)
//DynTimer//	{
//DynTimer//		checkTimer = time;
//DynTimer//		checkTimerHelper = 100;
//DynTimer//	}

	void secondTimer ()
	{
		if (state == Box) {
			--timer;
			redIndicator.inc(5);
			greenIndicator.dec(5);
			if (mode == Battle && (timer % 30) == 0)
				Dynamic::start(3, 3);
		}
	}

	void dynamicTimer ()
	{
		Dynamic::check();
	}
	
	void reset ()
	{
		state = Wait;
		//mode = Pair;
		
		Dynamic::reset();
		
		greenIndicator.reset();
		redIndicator.reset();
		
		startButton.reset();
		modeButton.reset();
		minuteButton.reset();

		timer = 0;
//DynTimer//		checkInit();
	}

	void reset_all ()
	{
		resetButton.reset();		
		reset();
	}
	
	void reinit ()
	{
		redIndicator.reset();
		greenIndicator.reset();
		greenIndicator.put(5, last_result_best);
	}

private:

	bool startEvent ()
	{
		if (startButton.get() || ircontroller.readyStart())
			return true;
		return false;
	}
	bool resetEvent ()
	{
		if (resetButton.get() || ircontroller.readyReset())
			return true;
		return false;
	}
	bool minutePlusEvent ()
	{
		if (minuteButton.get() || ircontroller.readyMinutePlus())
			return true;
		return false;
	}
	bool minuteMinusEvent ()
	{
		if (ircontroller.readyMinuteMinus())
			return true;
		return false;
	}
	bool muteEvent ()
	{
		if (ircontroller.readyMute())
			return true;
		return false;
	}
	bool modeEvent ()
	{
		if (modeButton.get() || ircontroller.readyMode())
			return true;
		return false;
	}

	enum State
	{
		Wait,
		Box,
		Pause,
		Result,
		Dead
	};

	enum Mode
	{
		Trainee,
		Battle,
		LastMode = Battle
	};

	volatile State state;
	volatile uint8_t mode;
	volatile uint16_t timer;
//DynTimer//	volatile uint16_t checkTimer;
//DynTimer//	volatile uint8_t checkTimerHelper;
	int16_t last_result_best;
	int16_t last_result_worst;
};

template <Port port,
          IO::Bit  bit1,
          IO::Bit  bit2,
          IO::Bit  bit3,
          IO::Bit  bit4,
          IO::Bit  bit5>
template <class IndicatorType>
void Sensor<port,bit1,bit2,bit3,bit4,bit5>::poll (IndicatorType &indicator)
{
	switch (state)
	{
		case Wait:
			last_sensor = checkSensors();
			if (last_sensor)
			{
				checkTimer = CHECK_TIME(1);
				state = Check;
			}
			break;
		case Check:
			if (checkTimer > 0) {
				if ((sensors::get() & last_sensor) != 0)
					state = Wait;
				break;
			} else
				state = Kick;
		case Kick:
			if ((sensors::get() & last_sensor) != 0) {		
				if ((last_sensor & sensor1) != 0) 
					indicator.kick(0);
				if ((last_sensor & sensor2) != 0) 
					indicator.kick(1);
				if ((last_sensor & sensor3) != 0) 
					indicator.kick(2);
				if ((last_sensor & sensor4) != 0) 
					indicator.kick(3);
				if ((last_sensor & sensor5) != 0) 
					indicator.kick(4);
				
				checkTimer = CHECK_TIME(50);
				state = Dead;
			}			
			break;
		case Dead:
			if (checkTimer > 0)
				break;
		default:
			state = Wait;
	}	
}

template <class input>
void Button<input>::poll ()
{
	switch (state)
	{
		case Wait:
			if (input::get())
			{
				checkTimer = CHECK_TIME(10);
				state = Check;
			}
			break;
		case Check:
			if (checkTimer > 0) {
				if (!input::get())
					state = Wait;
				break;
			} else {
				state = Press;
				pressed = true;
			}
		case Press:
			if (!input::get()) {		
				checkTimer = CHECK_TIME(10);
				state = Dead;
			}			
			break;
		case Dead:
			if (checkTimer > 0)
				break;
		default:
			state = Wait;
	}	
}

static System system;

ISR(UART_UDRE_vect)
{
//	if (!fifo.empty())
//		UDR = fifo.pop();
}

ISR(TIMER0_OVF_vect)
{
	system.checkSensors();
}

ISR(TIMER1_COMPA_vect)
{
	static uint8_t s = 0;
	if (s == 0)
		system.secondTimer();
	system.dynamicTimer();
	s = s < 9 ? s + 1 : 0;
}

ISR(SPI_STC_vect)
{
	system.checkIndicators();
}

ISR(INT1_vect)
{
	system.signalIRController();
}

ISR(INT0_vect)
{
	system.signalIRController();
}

bool System::pressed ()
{
	if (modeButton.get(false) ||
	    minuteButton.get(false) ||
	    startButton.get(false)  ||
	    ircontroller.ready(false)) {
		return true;
	}
	
	return false;
}

bool System::timeset ()
{
	if (minutePlusEvent())
		redIndicator.inc100(5);
	if (minuteMinusEvent())
		redIndicator.dec100(5);
	if (modeEvent()) {
		mode = mode < LastMode ? mode + 1 : 0;
		greenIndicator.mode (mode);
	}
	if (startEvent()) {
		timer = redIndicator.time();
		if (timer == 0)
			timer = 60*3;
		redIndicator.time(timer);
		greenIndicator.put(5, redIndicator.get(5));
		redIndicator.zero(5);
		return true;
	}
	
	return false;
}

void System::logic ()
{
	if (resetEvent())
		reset();
	
	switch (state)
	{
		case Wait:
			if (timeset())
				state = Box;
			break;
		case Box:
			if (timer == 0)
				state = Result;
			else if (startEvent())
				state = Pause;
			break;
		case Pause:
			if (startEvent())
				state = Box;
			break;
		case Result:
			result ();
			Dynamic::start();
//DynTimer//			checkInit(CHECK_LONG_TIME(10));
			state = Dead;
		case Dead:
//DynTimer//			if (checkTimer > 0)
//DynTimer//				break;
//DynTimer//			Dynamic::stop();
			if (!pressed())
				break;
			reinit();
		default:
			state = Wait;
	}

}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::display() 
{
	if (!flag) return;
	if (spi_in_progress) return;

	Spi::reinit (Spi::mode_master);
	Enable::set(true);
	
	spi_in_progress = true;
	spi_done = false;

	send_index  = 0;
	send_number = 0;
	send_digit  = 0;
	
	display_next();
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::display_next()
{
	if (!spi_in_progress) return;

	if (spi_done) {
		display_stop();
		return;
	}

	register uint8_t digit = Line[send_index];
	static bool zero_digit = true;
	if (zero_digit && digit == 0 && (send_digit < 2))
		digit = 0x10;
	else
		zero_digit = false;

	if ((send_number == 5 ) && (send_digit == 0 && digit < 10))
		Spi::send(0xff - (Leds[digit] + 0x80));
	else
		Spi::send(0xff - Leds[digit]);

	if(++send_index >= sizeof(Line))
		spi_done = true;

	if(++send_digit >= 3) {
		send_digit = 0;
		zero_digit = true;
		++send_number;
	}
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::display_stop()
{
	if (!spi_in_progress) return;

	spi_in_progress = false;
	Spi::stop();

	Strobe::set(true);
	flag = 0;
	flag = 1;
	flag = 0;
	Strobe::set(false);

	Enable::set(false);
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::inc (uint8_t index, int8_t middle)
{
	if (++Line[index * 3 + 2] > 9) {
		Line[index * 3 + 2] = 0;
		if (++Line[index * 3 + 1] > middle) {
			Line[index * 3 + 1] = 0;
			if (++Line[index * 3] > 9)
				Line[index * 3] = 0;
		}
	}

	flag = 1;
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::inc10 (uint8_t index, int8_t middle)
{
	if (++Line[index * 3 + 1] > middle) {
		Line[index * 3 + 1] = 0;
		if (++Line[index * 3] > 9)
			Line[index * 3] = 0;
	}

	flag = 1;
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::inc100 (uint8_t index)
{
	if (++Line[index * 3] > 9)
		Line[index * 3] = 0;

	flag = 1;
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::dec (uint8_t index, int8_t middle)
{
//asm ("; --start-- Indicator<Strobe, Enable>::dec");
	if (--Line[index * 3 + 2] < 0) {
		Line[index * 3 + 2] = 9;
		if (--Line[index * 3 + 1] < 0) {
			Line[index * 3 + 1] = middle;
			if (--Line[index * 3] < 0) {
				Line[index * 3] = 9;
				overflow = true;
			}
		}
	}

	flag = 1;
//asm ("; --end-- Indicator<Strobe, Enable>::dec");
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::dec100 (uint8_t index, int8_t middle)
{
	if (--Line[index * 3] < 0)
		Line[index * 3] = 9;

	flag = 1;
}

template <typename Strobe, typename Enable>
int16_t Indicator<Strobe, Enable>::get (uint8_t index, uint8_t high)
{
	return high * Line[index * 3] + 10 * Line[index * 3 + 1] + Line[index * 3 + 2];
}

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::put (uint8_t index, int16_t digit)
{
//asm ("; --start-- Indicator<Strobe, Enable>::put");
	int8_t sign = digit % 10;
	Line[index * 3 + 2] = sign;

	digit /= 10;
	sign = digit % 10;
	Line[index * 3 + 1] = sign;

	digit /= 10;
	sign = digit % 10;
	Line[index * 3] = sign;
	if (sign == 0 && digit < 0)
		Line[index * 3] = 0x11;

	flag = true;
//asm ("; --end-- Indicator<Strobe, Enable>::put");
}

bool IRController::emit (bool bit)
{
	bool done = false;

asm ("; --start-- IRController::emit");
/*	data.data << 1;
	if (bit) data.data |= 1;
	offset++;
	if (offset == LastBit) {

		cmd = data.byte[0] & 0x3f;
		if (data.byte[1] & 0x10) cmd |= 0x40;
		toggle = (data.byte[1] & 0x08) != 0;
		addr = (data.data >> 5) & 0x1f;
		done = true;
	}*/
//	if (USR & _BV(UDRE))
//		UDR = state;
//	else
//		fifo.push (state);

	switch (offset)
	{
		case ToggleBit:
			toggle = bit;
			offset = AddrBit_4;
			break;
		case AddrBit_4:
			bit ? addr |= _BV(4) : addr &= ~_BV(4);
			offset = AddrBit_3;
			break;
		case AddrBit_3:
			bit ? addr |= _BV(3) : addr &= ~_BV(3);
			offset = AddrBit_2;
			break;
		case AddrBit_2:
			bit ? addr |= _BV(2) : addr &= ~_BV(2);
			offset = AddrBit_1;
			break;
		case AddrBit_1:
			bit ? addr |= _BV(1) : addr &= ~_BV(1);
			offset = AddrBit_0;
			break;
		case AddrBit_0:
			bit ? addr |= _BV(0) : addr &= ~_BV(0);
			offset = CmdBit_5;
			break;
		case CmdBit_6:
			bit ? cmd |= _BV(6) : cmd &= ~_BV(6);
			offset = ToggleBit;
			break;
		case CmdBit_5:
			bit ? cmd |= _BV(5) : cmd &= ~_BV(5);
			offset = CmdBit_4;
			break;
		case CmdBit_4:
			bit ? cmd |= _BV(4) : cmd &= ~_BV(4);
			offset = CmdBit_3;
			break;
		case CmdBit_3:
			bit ? cmd |= _BV(3) : cmd &= ~_BV(3);
			offset = CmdBit_2;
			break;
		case CmdBit_2:
			bit ? cmd |= _BV(2) : cmd &= ~_BV(2);
			offset = CmdBit_1;
			break;
		case CmdBit_1:
			bit ? cmd |= _BV(1) : cmd &= ~_BV(1);
			offset = CmdBit_0;
			break;
		case CmdBit_0:
			bit ? cmd |= _BV(0) : cmd &= ~_BV(0);
			done = true;
			cnt++;
		default:
			offset = InitBit;
	}
asm ("; --end-- IRController::emit");

	return done;
}

void IRController::signal (uint8_t dt)
{
	Signal sig;
	if (checkTimer > IRC_MAX || dt > (2L * IRC_LONG))
		sig = VeryLong;
	else if (dt > (((IRC_LONG + IRC_SHORT) / 2L) + 5L))
		sig = Long;
	else if (dt > (IRC_SHORT / 2L))
		sig = Short;
	else
		sig = VeryShort;
//	if (USR & _BV(UDRE))
//		UDR = 0x80 | sig;
//	else
//		fifo.push (0x80 | sig);
	state = delta (sig);
	checkTimer = 0;
}

#define BAUD 38400
#define MYUBRR (F_OSC/16/BAUD-1)

void USART_Init( unsigned int baud )
{
	/* Set baud rate */
	//UBRRH = (unsigned char)(baud>>8);
	//UBRRL = (unsigned char)baud;
	UBRR = (unsigned char)baud;
	/* Enable receiver and transmitter */
	//UCSRB = _BV(UDRIE)|_BV(TXEN);
	UCR = _BV(UDRIE)|_BV(TXEN);
	/* Set frame format: 8data, 2stop bit */
	//UCSRC = _BV(URSEL)|_BV(USBS)|(3<<UCSZ0);
}

int main (void) 
{
	TIMSK  |= _BV(OCIE1A) | _BV(TOIE0);
	TCCR1A = 0x00;
	TCCR1B = 0x0C;
	TCCR0  = 0x02;
	OCR1A  = SECOND_OCR;

	USART_Init (MYUBRR);		
	sei();
	
	for (;;)
	{
		system.poll();
	}
	
	return 0;
}
