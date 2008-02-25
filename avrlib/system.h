#ifndef __AVRLIB_SYSTEM_H__INCLUDED__
#define __AVRLIB_SYSTEM_H__INCLUDED__

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

namespace AVRLIB {

#define MAKEBIT(bit) ((uint8_t)(_BV(bit)))

#define ASM(str) __asm__ __volatile__ (str)

#define NOP() ASM("nop")

#define SIGNAL_PROTOTYPE(signame) void signame (void)

#ifdef __cplusplus
#define SIGNAL_DECLARATION(signame) extern "C" SIGNAL_PROTOTYPE(signame)
#else
#define SIGNAL_DECLARATION(signame) SIGNAL_PROTOTYPE(signame)
#endif

#ifdef __cplusplus

namespace IO {

enum Bit {bit_0, bit_1, bit_2, bit_3, bit_4, bit_5, bit_6, bit_7};
typedef uint8_t Mask;

// Преобразование номера бита в битовую маску
inline uint8_t makebit(Bit bit) { return _BV((uint8_t)(bit)); }
inline uint8_t makebit(uint8_t bit) { return makebit((Bit)bit); }

// Установка и обнуление бита по маске
inline void setbit(volatile uint8_t& ref, Bit bit) { ref |= makebit(bit); }
inline void setbit(volatile uint8_t& ref, uint8_t bit) { setbit(ref, (Bit)bit); }
inline void clrbit(volatile uint8_t& ref, Bit bit) { ref &= ~makebit(bit); }
inline void clrbit(volatile uint8_t& ref, uint8_t bit) { clrbit(ref, (Bit)bit); }

typedef volatile uint8_t& (*reg_ref)();

template <reg_ref ref, Bit bit>
class RegBit
{
public:
	static void set(bool state)
	{
		state ? set() : clr();
	}
	static void set ()
	{
		ref() |= makebit(bit);
	}
	static void clr ()
	{
		ref() &= ~makebit(bit);
	}
	static uint8_t get ()
//	static bool get ()
	{
		return ref() & makebit(bit);
	}
};

template <reg_ref ref, Mask mask>
class RegBits
{
public:
	static void set(Mask data = mask)
	{
		ref() |= data & mask;
	}
	static void clr(Mask data = mask)
	{
		ref() &= ~(data & mask);
	}
	static void reset(Mask data)
	{
		clr();
		set(data);
	}
	static Mask get()
	{
		return ref() & mask;
	}
};

} // namespace IO

class Lockint
{
public:
	Lockint()
	{
		cli();
	}
	~Lockint()
	{
		sei();
	}
};

//inline uint8_t MAKEBIT(IO::Bit bit) { return IO::makebit(bit); }
//inline uint8_t MAKEBIT(uint8_t bit) { return MAKEBIT((IO::Bit)bit); }
//
//#else
//
//#define MAKEBIT(bit) ((uint8_t)(_BV(bit)))

#endif

} // namespace AVRLIB

#endif//__AVRLIB_SYSTEM_H__INCLUDED__
