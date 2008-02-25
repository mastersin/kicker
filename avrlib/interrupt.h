#ifndef __AVRLIB_INTERRUPT_H__INCLUDED__
#define __AVRLIB_INTERRUPT_H__INCLUDED__

#include <inttypes.h>
#include <avr/interrupt.h>
#include <avrlib/portio.h>

#ifndef __cplusplus
#error C++ compilation required
#endif

namespace AVRLIB {

namespace Interrupt {

/**
 * Определения доступных внешних прерываний
 **/

enum Ext { int_0, int_1, int_2 };
enum Reg { reg_control, reg_sense, reg_flag };
enum Sense { low_level, logical_change, falling_edge, rising_edge };

/**
 * Адресация регистров внешних прерываний
 **/

// Шаблон функции, определяющей требуемые адреса регистров внешних прерываний
template <Reg reg, Ext ext> volatile uint8_t& port_ref();

// Определение специализаций шаблонов для регистров внешних прерываний:
#define AVRLIB_EXTINTERRUPT_ENABLED
#if defined (__AVR_ATmega8535__) || defined (__AVR_AT90S8535__)
#  define EXTERNAL_INTERRUPT0_ENABLED
#  define EXTERNAL_INTERRUPT1_ENABLED
typedef Bit<IO::port_D,IO::bit_2,true>  INT0_bit;
typedef Bit<IO::port_D,IO::bit_3,false> INT1_bit;
#if   defined (__AVR_ATmega8535__)
#    define EXTERNAL_INTERRUPT2_ENABLED
typedef Bit<IO::port_B,IO::bit_3,false> INT2_bit;
template <> inline volatile uint8_t& port_ref<reg_control, int_0>() { return GICR; }
template <> inline volatile uint8_t& port_ref<reg_control, int_1>() { return GICR; }
template <> inline volatile uint8_t& port_ref<reg_control, int_2>() { return GICR; }
template <> inline volatile uint8_t& port_ref<reg_sense, int_2>  () { return MCUCSR; }
template <> inline volatile uint8_t& port_ref<reg_flag, int_2>   () { return GIFR; }
#else
template <> inline volatile uint8_t& port_ref<reg_control, int_0>() { return GIMSK; }
template <> inline volatile uint8_t& port_ref<reg_control, int_1>() { return GIMSK; }
#endif
template <> inline volatile uint8_t& port_ref<reg_sense, int_0>  () { return MCUCR; }
template <> inline volatile uint8_t& port_ref<reg_flag, int_0>   () { return GIFR; }
template <> inline volatile uint8_t& port_ref<reg_sense, int_1>  () { return MCUCR; }
template <> inline volatile uint8_t& port_ref<reg_flag, int_1>   () { return GIFR; }
#else
#  undef AVRLIB_EXTINTERRUPT_ENABLED
#  warning "AVRLIB::EXTINTERRUPT: unknown device type"
#endif

//template <Ext ext>
//inline IO::Mask sense_mask (Sense sense)
//{
//
//}
//
//template <IO::reg_ref reg, Ext ext>
//class RegSense: public IO::RegBits<port_ref<reg_sense, ext>, (IO::Mask)(MAKEBIT(ISC00)|MAKEBIT(ISC01))>
//{
//	static void set (Sense sense)
//	{
//		
//	}
//};
//typedef IO::RegBits<port_ref<reg_sense, int_0>, (IO::Mask)(MAKEBIT(ISC10)|MAKEBIT(ISC11))> ExternalSense1;

} // namespace Interrupt

#if defined (AVRLIB_EXTINTERRUPT_ENABLED)

template <Interrupt::Ext ext, IO::Bit ISCn0, IO::Bit ISCn1, IO::Bit INTx>
class External
{
protected:
	static volatile uint8_t& Flag () { return Interrupt::port_ref<Interrupt::reg_flag, ext>(); }
	static volatile uint8_t& StatusControl () { return Interrupt::port_ref<Interrupt::reg_sense, ext>(); }
	static volatile uint8_t& GeneralControl () { return Interrupt::port_ref<Interrupt::reg_control, ext>(); }
//	typedef typename IO::RegBits<StatusControl, (IO::Mask)(MAKEBIT(ISCn0)|MAKEBIT(ISCn1))> RegSense;
//	typedef Interrupt::ExternalSense0 RegSense;
//	typedef Interrupt::RegSense<ext> RegSense;
	static void set_sense (Interrupt::Sense sense)
	{
		switch (ext) {
		case Interrupt::low_level:
			StatusControl () &= ~_BV (ISCn0);
			StatusControl () &= ~_BV (ISCn1);
			break;
		case Interrupt::logical_change:
			StatusControl () |= _BV (ISCn0);
			StatusControl () &= ~_BV (ISCn1);
			break;
		case Interrupt::falling_edge:
			StatusControl () &= ~_BV (ISCn0);
			StatusControl () |= _BV (ISCn1);
			break;
		case Interrupt::rising_edge:
			StatusControl () |= _BV (ISCn0);
			StatusControl () |= _BV (ISCn1);
			break;
		}
	}

//	typedef IO::RegBit<GeneralControl, (IO::Bit)(MAKEBIT(INTx))> enable_bit;
public:
	static void init (Interrupt::Sense sense)
	{
		set_sense(sense);
		GeneralControl () |= makebit (INTx);
//		enable_bit::set();
	}
	static void stop ()
	{
		GeneralControl () &= ~makebit (INTx);
//		enable_bit::clr();
	}
};

// ЗАМЕЧАНИЕ. Было бы корректнее использовать вариант, приведенный ниже, однако
// в текущей версии avr-gcc (3.4.4) компилятор не может правильно распознать код.
//
//	template <Interrupt::Ext ext, IO::Bit ISCn0, IO::Bit ISCn1>
//	class External
//	{
//	 ...
//		typedef typename IO::RegBits<StatusControl, (IO::Mask)(MAKEBIT(ISCn0)|MAKEBIT(ISCn1))> RegSense;
//	 ...
//		typedef typename IO::RegBit<GeneralControl, (IO::Bit)(MAKEBIT(INTx))> enable_bit;
//	 ...
//	};
//
// $ make
//  ...
// Compiling: main.cpp
// avr-g++ -c -mmcu=at90s8535 -I. -gstabs   -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wa,-adhlns=main.lst -I/usr/lib/avr/include -DF_OSC=8000000 -Wall -MD -MP -MF .dep/main.o.d main.cpp -o main.o
// In file included from main.cpp:7:
// ./avrlib/interrupt.h:71: internal compiler error: in uses_template_parms, at cp/pt.c:4823
// Please submit a full bug report,
// with preprocessed source if appropriate.
// See <URL:http://gcc.gnu.org/bugs.html> for instructions.
// make: *** [main.o] Ошибка 1

#if defined (EXTERNAL_INTERRUPT0_ENABLED)

ISR(INT0_vect);

class ExternalInterrupt0: public External<Interrupt::int_0, (IO::Bit)ISC00, (IO::Bit)ISC01, (IO::Bit)INT0>
{
	friend void INT0_vect(void);
};

#endif

#if defined (EXTERNAL_INTERRUPT1_ENABLED)

ISR(INT1_vect);

class ExternalInterrupt1: public External<Interrupt::int_1, (IO::Bit)ISC10, (IO::Bit)ISC11, (IO::Bit)INT1>
{
	friend void INT1_vect(void);
};

#endif

#if defined (EXTERNAL_INTERRUPT2_ENABLED)

//ISR(INT2_vect);

#endif

#endif

} // namespace AVRLIB

#endif//__AVRLIB_INTERRUPT_H__INCLUDED__

