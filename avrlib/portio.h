#ifndef __AVRLIB_PORTIO_H__INCLUDED__
#define __AVRLIB_PORTIO_H__INCLUDED__

#ifndef __cplusplus
#error C++ compilation required
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avrlib/system.h>

namespace AVRLIB {

namespace IO {

/**
 * Определения доступных имен и типов портов и номеров битов
 **/

enum Port {port_A, port_B, port_C, port_D, port_E, port_F, port_G};
enum Reg {reg_PORT, reg_DDR, reg_PIN};

/**
 * Адресация портов В/В
 **/

// Шаблон функции, определяющей требуемые адреса в пространстве В/В
template <Port port, Reg reg> volatile uint8_t& port_ref();

// Определение специализаций шаблонов для портов в виде:
// template <> inline volatile uint8_t& port_ref<port_X, reg_PORT>() { return PORTX; }
// template <> inline volatile uint8_t& port_ref<port_X, reg_DDR >() { return DDRX ; }
// template <> inline volatile uint8_t& port_ref<port_X, reg_PIN >() { return PINX ; }
// Где X используется в качестве параметра имени порта, для которого уже объявлены:
// PORTX, DDRX и PINX (например, для порта A, это PORTA, DDRA и PINA)

#define AVRLIB_SPECIALIZE_PORT_REF(_port_) \
template <> inline volatile uint8_t& port_ref<port_##_port_, reg_PORT>() { return PORT##_port_; } \
template <> inline volatile uint8_t& port_ref<port_##_port_, reg_DDR >() { return DDR##_port_ ; } \
template <> inline volatile uint8_t& port_ref<port_##_port_, reg_PIN >() { return PIN##_port_ ; }

#define AVRLIB_PORTIO_ENABLED
#if defined (__AVR_ATtiny15__)
#include <avr/attiny15.1.h>
AVRLIB_SPECIALIZE_PORT_REF(B)
#elif defined (__AVR_ATmega8535__) || defined (__AVR_AT90S8535__)
AVRLIB_SPECIALIZE_PORT_REF(A)
AVRLIB_SPECIALIZE_PORT_REF(B)
AVRLIB_SPECIALIZE_PORT_REF(C)
AVRLIB_SPECIALIZE_PORT_REF(D)
#else
#  undef AVRLIB_PORTIO_ENABLED
#  warning "AVRLIB::PORTIO: unknown device type"
#endif

//AVRLIB_SPECIALIZE_PORT_REF(E)
//AVRLIB_SPECIALIZE_PORT_REF(F)
//AVRLIB_SPECIALIZE_PORT_REF(G)

#undef AVRLIB_SPECIALIZE_PORT_REF

/**
 * Бит порта В/В
 **/

template <Port port, Bit bit>
class Pxn
{
public:
	static void init(bool output)
	{
		output ?
			port_ref<port, reg_DDR>() |= makebit(bit) :
			port_ref<port, reg_DDR>() &= (uint8_t)(~makebit(bit));
	}
	static void set()
	{
		port_ref<port, reg_PORT>() |= makebit(bit);
	}
	static void clr()
	{
		port_ref<port, reg_PORT>() &= (uint8_t)(~makebit(bit));
	}
	static bool state()
	{
		return port_ref<port, reg_DDR>() & makebit(bit);
	}
//	static bool get()
	static uint8_t get()
	{
		return port_ref<port, reg_PIN>() & makebit(bit);
	}

// ЗАМЕЧАНИЕ. Было бы корректнее использовать вариант, приведенный ниже, однако
// в текущей версии WinAVR (20040404) компилятор не может правильно
// соптимизировать такой код, генерируя массу излишних инструкций. Тем не менее,
// рекомендуется интерпретировать возвращаемое значение исключительно как bool
// в целях совместимости со следующими версиями компиляторов. Следовательно,
// использование возвращенного значения в арифметических и битовых операциях
// не рекомендуется.
//
//	static bool get()
//	{
//		return (port_ref<port, reg_PIN>() & makebit(bit)) != 0;
//	}
};

/**
 * Набор битов порта В/В по маске
 **/

template <Port port, Mask mask>
class Pxns
{
public:
	static void init(bool output)
	{
		output ?
			port_ref<port, reg_DDR>() |= mask :
			port_ref<port, reg_DDR>() &= ~mask;
	}
	static void set(Mask data = mask)
	{
		port_ref<port, reg_PORT>() |= data;
	}
	static void clr(Mask data = mask)
	{
		port_ref<port, reg_PORT>() &= ~data;
	}
	static Mask get()
	{
		return port_ref<port, reg_PIN>() & mask;
	}
};
} // namespace IO

/**
 * Bit: бит - вход или выход, статический класс
 *
 * Настройка и управление битом порта на вход и на выход.
 *
 * Определяется тремя параметрами:
 * - имя порта;
 * - номер бита;
 * - значение по умолчанию - вход или выход:
 * -- true  - устанавливает бит по умолчанию в режим на выход;
 * -- false - устанавливает бит по умолчанию в режим на вход.
 *
 * При инициализации можно изменить режим по умолчанию на обратный,
 * установкой параметра инициализации в true.
 **/

template <IO::Port port, IO::Bit bit, bool out>
class Bit
{
	typedef IO::Pxn<port, bit> pio;

public:
	static void init(bool reverse = false)
	{
		(out == reverse) ? pio::init(false) : pio::init(true);
	}
	static void set(bool state)
	{
		state ? pio::set() : pio::clr();
	}
//	static bool get()
	static uint8_t get()
	{
		return pio::state() ? false : pio::get();
	}
};

/**
 * Bits: биты - входы/выходы по маске, статический класс
 *
 * Настройка и управление битами порта на вход и на выход.
 *
 * Определяется тремя параметрами:
 * - имя порта;
 * - маска битов;
 * - значение по умолчанию - входы или выходы:
 * -- true  - устанавливает биты по умолчанию в режим на выход;
 * -- false - устанавливает биты по умолчанию в режим на вход.
 *
 * При инициализации можно изменить режим по умолчанию на обратный,
 * установкой параметра инициализации в true.
 **

template <IO::Port port, IO::Mask mask, bool out>
class Bits
{
	typedef IO::Pxns<port, mask> pio;

public:
	static void init(bool reverse = false)
	{
		(out == reverse) ? pio::init(false) : pio::init(true);
	}
	static void set(bool state)
	{
		state ? pio::set() : pio::clr();
	}
//	static bool get()
	static uint8_t get()
	{
		return pio::state() ? false : pio::get();
	}
};*/

/**
 * Output: выход, статический класс
 *
 * Настройка и управление выходом порта.
 *
 * Определяется тремя параметрами:
 * - имя порта;
 * - номер бита;
 * - значение, устанавливаемое функцией set(true):
 * -- true  - set(true) устанавливает выход в логическую единицу;
 * -- false - set(true) устанавливает выход в логический ноль.
 *
 * При инициализации можно задать значение, в которое будет установлен бит
 * после иницализации, в соответствии с семантикой, заданной для функции set().
 **/

template <IO::Port port, IO::Bit bit, bool on>
class Output
{
	typedef IO::Pxn<port, bit> pout;

public:
	static void init(bool state = false)
	{
		pout::init(true);
		set(state);
	}
	static void set(bool state)
	{
		(on == state) ? pout::set() : pout::clr();
	}
};

/**
 * Outputs: выходы по маске, статический класс
 *
 * Настройка и управление выходом порта.
 *
 * Определяется тремя параметрами:
 * - имя порта;
 * - маска битов;
 * - значение, устанавливаемое функцией set(true):
 * -- true  - set(true) устанавливает выходы в логическую единицу;
 * -- false - set(true) устанавливает выходы в логический ноль.
 *
 * При инициализации можно задать значение, в которое будет установлены биты
 * после иницализации, в соответствии с семантикой, заданной для функции set().
 **/

template <IO::Port port, IO::Mask mask, bool on>
class Outputs
{
	typedef IO::Pxns<port, mask> pout;

public:
	static void init(bool state = false)
	{
		pout::init(true);
		set(state);
	}
	static void set(bool state)
	{
		(on == state) ? pout::set() : pout::clr();
	}
	static void set(IO::Mask data)
	{
		on ? pout::set(data) : pout::clr(data);
	}
};

/**
 * Input: вход, статический класс
 *
 * Настройка и определение состояния входа порта.
 *
 * Определяется тремя параметрами:
 * - имя порта;
 * - номер бита;
 * - значение, возвращаемое функцией get():
 * -- true  - get() возвращает truе, если на входе логическая единица;
 * -- false - get() возвращает truе, если на входе логический ноль.
 *
 * При инициализации можно установить "подтяжку" (pull-up) вывода к питанию 
 * микроконтроллера через резистор 40 КОм, по умолчанию вывод подключен как
 * высокоомный вход (находится в, так называемом, Z-состоянии).
 **/

template <IO::Port port, IO::Bit bit, bool on>
class Input
{
	typedef IO::Pxn<port, bit> pin;

public:
	static void init(bool pullup = false)
	{
		pin::init(false);
		pullup ? pin::set() : pin::clr();
	}
	static bool get()
	{
		return pin::get() ? on : !on;
	}
};

/**
 * Inputs: входы по маске, статический класс
 *
 * Настройка и определение состояния входа порта.
 *
 * Определяется тремя параметрами:
 * - имя порта;
 * - маска битов;
 * - значение, возвращаемое функцией get():
 * -- true  - get() возвращает биты, по заданной маске;
 * -- false - get() возвращает инвертированное значение битов по маске.
 *
 * При инициализации можно установить "подтяжку" (pull-up) выводов к питанию 
 * микроконтроллера через резистор 40 КОм, по умолчанию выводы подключен как
 * высокоомные входы (находятся в, так называемом, Z-состоянии).
 **/

template <IO::Port port, IO::Mask mask, bool on>
class Inputs
{
	typedef IO::Pxns<port, mask> pin;

public:
	static void init(bool pullup = false)
	{
		pin::init(false);
		pullup ? pin::set() : pin::clr();
	}
	static IO::Mask get()
	{
		return on ? pin::get() : ~pin::get();
	}
};

} // namespace AVRLIB

#endif//__AVRLIB_PORTIO_H__INCLUDED__
