#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
 
#include <avrlib/portio.h>
#include <avrlib/spi.h>
#include <avrlib/interrupt.h>
#include <string.h>

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

/*struct Fifo
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
};*/

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
	int16_t total ();
	int16_t time ()
	{
		return get(5, 60);
	}
	void time (int16_t t)
	{
		return put(5, t*100/60);
	}
	void zero (uint8_t index);
	void minus (uint8_t index, uint8_t middle = 0x11);
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
	const uint8_t* data () { return (uint8_t*)Line; }
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
	bool readyButton (Codes code, bool again = false);
	
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
		switch (state) {
		case Wait:
			return Wait;
		case Ready:
			return Ready;
		case Error:
			return Error;
		case Mid1:
			if (sig == Long) {
				if (emit (false))
					return Ready;
				return Mid2;
			}
			if (sig == Short)
				return Start1;
			break;	
		case Start1:
			if (sig == Short) {
				if (emit (true))
					return Ready;
				return Mid1;
			}
			break;
		case Mid2:
			if (sig == Long) {
				if (emit (true))
					return Ready;
				return Mid1;
			}
			if (sig == Short)
				return Start2;
			break;
		case Start2:
			if (sig == Short) {
				if (emit (false))
					return Ready;
				return Mid2;
			}
		}

/*		
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
		}*/
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

class UART
{
public:
	static void init(unsigned int baud)
	{
		/* Set baud rate */
		//UBRRH = (unsigned char)(baud>>8);
		//UBRRL = (unsigned char)baud;
		UBRR = (unsigned char)baud;
		/* Enable receiver and transmitter */
		//UCSRB = _BV(UDRIE)|_BV(TXEN);
		//UCR = _BV(UDRIE)|_BV(TXEN);
		//UCR = _BV(TXEN)|_BV(RXEN)|_BV(RXCIE);
		UCR = _BV(TXEN)|_BV(RXEN);
		/* Set frame format: 8data, 2stop bit */
		//UCSRC = _BV(URSEL)|_BV(USBS)|(3<<UCSZ0);
	}
	static bool send_ready()
	{
		return USR & _BV(UDRE);
	}
	static bool recv_ready()
	{
		return USR & _BV(RXC);
	}
	static void send(uint8_t data)
	{
		UDR = data;
	}
	static uint8_t recv()
	{
		return UDR;
	}
};

class Terminal;
class TerminalReceiver
{
	enum State
	{
		Wait,
		Cmd,
		Data1,
		Data2,
		Data3,
		Data4,
		Checksum1,
		Checksum2,
		Ready,
		Error
	};

	enum Command
	{
		Start = 'B',
		Reset = 'H',
		Plus  = 'I',
		Minus = 'D',
		Mute  = 'S',
		Mode  = 'M',
		Time  = 'T'
	};

public:
	TerminalReceiver(): state(Wait) {}

	void reset () 
	{
		state = Wait;
		data.data16 = 0;
	}

	bool ready (bool reinit = true)
	{
		if (state == Ready) {
			if (reinit)
				reset();
			return true;
		}
		return false;
	}

	bool ready (Command cmd)
	{
		if (ready(false) && cmd == command) {
			reset();
			return true;
		}
		return false;
	}

	bool readyStart ()
	{
		return ready (Start);
	}
	bool readyReset ()
	{
		return ready (Reset);
	}
	bool readyMinutePlus ()
	{
		return ready (Plus);
	}
	bool readyMinuteMinus ()
	{
		return ready (Minus);
	}
	bool readyMute ()
	{
		return ready (Mute);
	}
	bool readyMode ()
	{
		return ready (Mode);
	}
	bool readyTime ()
	{
		return ready (Time);
	}

	uint8_t getData(uint8_t index)
	{
		if (index > 1)
			return 0;
		return data.data8[index];
	}
	uint16_t getData16()
	{
		return data.data16;
	}
	
	State delta(uint8_t byte)
	{
		if (byte == 0)
			return Wait;
		
		switch (state) {
		case Error:
			reset();
		case Ready:
		case Wait:
			if (byte == ':')
				return Cmd;
			break;
		case Cmd:
			if (is_command(byte)) {
				command = byte;
				return Data1;
			}
			break;
		case Data1:
			if (is_data(byte)) {
				data.data8[0] = to_hex(byte);
				return Data2;
			}
			break;
		case Data2:
			if (is_data(byte)) {
				data.data8[0] |= to_hex(byte)<<4;
				return Data3;
			}
			break;
		case Data3:
			if (is_data(byte)) {
				data.data8[1] = to_hex(byte);
				return Data4;
			}
			break;
		case Data4:
			if (is_data(byte)) {
				data.data8[1] |= to_hex(byte)<<4;
				return Checksum1;
			}
			break;
		case Checksum1:
			if (is_data(byte)) {
				checksum = to_hex(byte);
				return Checksum2;
			}
			break;
		case Checksum2:
			if (is_data(byte)) {
				checksum |= to_hex(byte)<<4;
				if (test_checksum())
					return Ready;
			}
			break;
		default:
			break;
		}
		
		return Error;
	}
	
	void poll()
	{
		if (!UART::recv_ready())
			return;
		
		state = delta(UART::recv());
	}
	
private:
	bool is_digit (uint8_t byte)
	{
		return byte >= '0' && byte <= '9';
	}
	bool is_alpha (uint8_t byte, uint8_t last = 'Z')
	{
		return byte >= 'A' && byte <= last;
	}
	bool is_command (uint8_t byte)
	{
		return is_alpha(byte);
	}
	bool is_data (uint8_t byte)
	{
		return is_alpha(byte, 'F') || is_digit(byte);
	}
	uint8_t to_hex (uint8_t byte)
	{
		if (byte > '9')
			return byte - 'A' + 0xA;
				
		return byte - '0';
	}

	bool test_checksum ()
	{
		register uint8_t byte = command;
		byte += getData(0);
		byte += getData(1);

		if (byte == checksum)
			return true;

		return false;
	}

	State state;
	union Data {
		uint8_t data8[2];
		uint16_t data16;
	};
	uint8_t checksum;
	uint8_t command;
	Data data;
	
	friend class Terminal;
};

#define BAUD 38400
#define MYUBRR (F_OSC/16/BAUD-1)

class Terminal
{
	static const uint8_t indicator_size = 3*5;
	static const uint8_t mode_size      = 1;
	static const uint8_t time_size      = 3;
	static const uint8_t status_size    = 1;
	static const uint8_t checksum_size  = 2;
	static const uint8_t buffer_size    = indicator_size*2+mode_size+time_size+status_size+checksum_size;
	static const uint8_t red_index      = 0;
	static const uint8_t green_index    = indicator_size;
	static const uint8_t mode_index     = indicator_size*2;
	static const uint8_t time_index     = indicator_size*2+mode_size;
	static const uint8_t status_index   = indicator_size*2+mode_size+time_size;
	static const uint8_t checksum_index = indicator_size*2+mode_size+time_size+status_index;

	enum State
	{
		Wait,
		Send,
		Done,
		Error
	};

	TerminalReceiver receiver;
	
public:
	Terminal(): state(Wait) {
		UART::init (MYUBRR);
	}
	
	bool update (const uint8_t *redLine,
		     const uint8_t *greenLine,
		     uint8_t mode, uint16_t time, uint8_t status)
	{
		if (state == Send && UART::send_ready())
			return false;
		
		for (register uint8_t i = 0; i < indicator_size; i++) {
			buffer[i] = normalize_digit(redLine[i]);
			buffer[i+green_index] = normalize_digit(greenLine[i]);
		}
		buffer[mode_index] = normalize_alpha(mode);
		fill3digits(buffer+time_index, time);
		buffer[status_index] = check_alpha(status);

		register uint8_t checksum = 0;
		for (register uint8_t i = 0; i < buffer_size-checksum_size; i++)
			checksum += buffer[i];
		buffer[checksum_index] = normalize_hex(checksum);
		buffer[checksum_index+1] = normalize_hex(checksum>>4);

		state = Send;
		index = 0;
		UART::send(':');

		return true;
	}

	void poll() {
		next();
		receiver.poll();
	}

	bool readyStart ()
	{
		return receiver.readyStart();
	}
	bool readyReset ()
	{
		return receiver.readyReset();
	}
	bool readyMinutePlus ()
	{
		return receiver.readyMinutePlus();
	}
	bool readyMinuteMinus ()
	{
		return receiver.readyMinuteMinus();
	}
	bool readyMute ()
	{
		return receiver.readyMute();
	}
	bool readyMode ()
	{
		return receiver.readyMode();
	}
	bool readyTime ()
	{
		return receiver.readyTime();
	}
	uint16_t getTime ()
	{
		return receiver.getData16();
	}

private:
	void next() {
		if (state != Send || !UART::send_ready())
			return;

		if (index < buffer_size) {
			UART::send(buffer[index++]);
		} else {
			state = Done;
			UART::send(buffer[0]);
		}
	}
	uint8_t normalize_digit (uint8_t byte) {
		if (byte > 9)
			return '.';
		return byte + '0';	
	}
	uint8_t normalize_alpha (uint8_t byte) {
		if (byte > 'Z'-'A')
			return '.';
		return byte - 0xA + 'A';
	}
	uint8_t normalize_hex (uint8_t byte) {
		byte &= 0xf;
		if (byte > 9)
			return byte - 0xA + 'A';
		return byte + '0';
	}
	uint8_t check_alpha (uint8_t byte) {
		if (byte < 'A' || byte > 'Z')
			return '.';
		return byte;
	}
	void fill3digits (uint8_t *buff, uint16_t data) {
		register uint8_t a, b;
		a = data%10;
		buff[0] = a + '0';
		b = data/10;
		a = b%10;
		buff[1] = a + '0';
		b = b/10;
		a = b%10;
		buff[2] = a + '0';
	}

	uint8_t buffer[buffer_size];
	uint8_t index;
	volatile State state;
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
	Terminal terminal;
	
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

	void poll_terminal()
	{
		if (need_terminal_update) {
			if (terminal.update(redIndicator.data(), greenIndicator.data(), mode, timer, terminal_status))
				need_terminal_update = false;
		}
		terminal.poll();
	}

	void terminal_update(uint8_t status = 'E', bool force = false)
	{
		if (!force && need_terminal_update)
			return;
		
		need_terminal_update = true;
		terminal_status = status;
	}

	void logic ();
	bool timeset ();
	bool pressed ();
	void result ()
	{
		//uint16_t red = redIndicator.total();
		//uint16_t green = greenIndicator.total();
		//redIndicator.put(5,red);
		//greenIndicator.put(5,green);
		redIndicator.put(5,redIndicator.total());
		greenIndicator.put(5,greenIndicator.total());
	}
	
	void poll ()
	{
		poll_sensors();
		poll_terminal();

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

//	void checkTerminal ()
//	{
//		terminal.check();	
//	}

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

//	void checkTerminal ()
//	{
//		terminal.next();	
//	}

//DynTimer//	void checkInit (uint16_t time = 0)
//DynTimer//	{
//DynTimer//		checkTimer = time;
//DynTimer//		checkTimerHelper = 100;
//DynTimer//	}

	void secondTimer ()
	{
		if (state == Box) {
			terminal_update('T');
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
	}

private:

	bool startEvent ()
	{
		if (startButton.get() || ircontroller.readyStart() || terminal.readyStart())
			return true;
		return false;
	}
	bool resetEvent ()
	{
		if (resetButton.get() || ircontroller.readyReset() || terminal.readyReset())
			return true;
		return false;
	}
	bool minutePlusEvent ()
	{
		if (minuteButton.get() || ircontroller.readyMinutePlus() || terminal.readyMinutePlus())
			return true;
		return false;
	}
	bool minuteMinusEvent ()
	{
		if (ircontroller.readyMinuteMinus() || terminal.readyMinuteMinus())
			return true;
		return false;
	}
	bool muteEvent ()
	{
		if (ircontroller.readyMute() || terminal.readyMute())
			return true;
		return false;
	}
	bool modeEvent ()
	{
		if (modeButton.get() || ircontroller.readyMode() || terminal.readyMode())
			return true;
		return false;
	}
	bool timeEvent ()
	{
		if (terminal.readyTime())
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
	//int16_t red;
	//int16_t green;
	volatile bool need_terminal_update;
	uint8_t terminal_status;
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

//ISR(UART_RX_vect)
//{
//	system.checkTerminal();
//}

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
	if (minutePlusEvent()) {
		redIndicator.inc100(5);
		terminal_update('I');
	}
	if (minuteMinusEvent()) {
		redIndicator.dec100(5);
		terminal_update('D');
	}
	if (modeEvent()) {
		mode = mode < LastMode ? mode + 1 : 0;
		greenIndicator.mode (mode);
		terminal_update('M');
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
	State current = state;

	if (resetEvent()) {
		reset();
		terminal_update('H', true);
	}
	
	switch (state)
	{
		case Wait:
			if (timeset()) {
				state = Box;
				terminal_update('B', true);
			}
			break;
		case Box:
			if (timer == 0) {
				state = Result;
			} else if (startEvent()) {
				state = Pause;
				terminal_update('P', true);
			}
			break;
		case Pause:
			if (startEvent()) {
				state = Box;
				terminal_update('B', true);
			}
			break;
		case Result:
			result ();
			terminal_update('R', true);
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

	if (current != state)
		terminal_update();
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
	register uint8_t send_digit_tmp = send_digit;
	static bool zero_digit = true;
	if (zero_digit && digit == 0 && (send_digit_tmp < 2))
		digit = 0x10;
	else
		zero_digit = false;

	if ((send_number == 5 ) && (send_digit_tmp == 0 && digit < 10))
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
int16_t Indicator<Strobe, Enable>::total ()
{
	return get(0) + get(1) + get(2) + get(3) + get(4);
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

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::zero (uint8_t index)
{
	Line[index * 3] = 0;
	Line[index * 3 + 1] = 0;
	Line[index * 3 + 2] = 0;

	flag = true;
}	

template <typename Strobe, typename Enable>
void Indicator<Strobe, Enable>::minus (uint8_t index, uint8_t middle)
{
	Line[index * 3] = 0x11;
	Line[index * 3 + 1] = middle;
	Line[index * 3 + 2] = 0x11;

	flag = true;
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
/*
	if (offset == ToggleBit) {
		toggle = bit;
		offset = AddrBit_4;
	} else if (offset == AddrBit_4) {
		bit ? addr |= _BV(4) : addr &= ~_BV(4);
		offset = AddrBit_3;
	} else if (offset == AddrBit_3) {
		bit ? addr |= _BV(3) : addr &= ~_BV(3);
		offset = AddrBit_2;
	} else if (offset == AddrBit_2) {
		bit ? addr |= _BV(2) : addr &= ~_BV(2);
		offset = AddrBit_1;
	} else if (offset == AddrBit_1) {
		bit ? addr |= _BV(1) : addr &= ~_BV(1);
		offset = AddrBit_0;
	} else if (offset == AddrBit_0) {
		bit ? addr |= _BV(0) : addr &= ~_BV(0);
		offset = CmdBit_5;
	} else if (offset == CmdBit_6) {
		bit ? cmd |= _BV(6) : cmd &= ~_BV(6);
		offset = ToggleBit;
	} else if (offset == CmdBit_5) {
		bit ? cmd |= _BV(5) : cmd &= ~_BV(5);
		offset = CmdBit_4;
	} else if (offset == CmdBit_4) {
		bit ? cmd |= _BV(4) : cmd &= ~_BV(4);
		offset = CmdBit_3;
	} else if (offset == CmdBit_3) {
		bit ? cmd |= _BV(3) : cmd &= ~_BV(3);
		offset = CmdBit_2;
	} else if (offset == CmdBit_2) {
		bit ? cmd |= _BV(2) : cmd &= ~_BV(2);
		offset = CmdBit_1;
	} else if (offset == CmdBit_1) {
		bit ? cmd |= _BV(1) : cmd &= ~_BV(1);
		offset = CmdBit_0;
	} else if (offset == CmdBit_0) {
		bit ? cmd |= _BV(0) : cmd &= ~_BV(0);
		done = true;
		cnt++;
		offset = InitBit;
	} else {
		offset = InitBit;
	}*/
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
	state = delta (sig);
	checkTimer = 0;
}

bool IRController::readyButton (Codes code, bool again)
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

int main (void) 
{
	TIMSK  |= _BV(OCIE1A) | _BV(TOIE0);
	TCCR1A = 0x00;
	TCCR1B = 0x0C;
	TCCR0  = 0x02;
	OCR1A  = SECOND_OCR;

	sei();
	
	for (;;)
	{
		system.poll();
	}
	
	return 0;
}
