#ifndef __AVRLIB_SPI_H__INCLUDED__
#define __AVRLIB_SPI_H__INCLUDED__

#ifndef __cplusplus
#error C++ compilation required
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avrlib/system.h>
#include <avrlib/portio.h>

namespace AVRLIB {

namespace SPI {

/**
 * Определения доступных типов регистров
 **/

enum Reg {Control, Status, Data};

/**
 * Адресация регистров SPI
 **/

// Шаблон функции, определяющей требуемые адреса регистров SPI
template <Reg reg> volatile uint8_t& port_ref();

// Определение специализаций шаблонов для регистров SPI:
template <> inline volatile uint8_t& port_ref<Control>() { return SPCR; }
template <> inline volatile uint8_t& port_ref<Status> () { return SPSR; }
template <> inline volatile uint8_t& port_ref<Data>   () { return SPDR; }

#define AVRLIB_SPI_ENABLED
#if defined (__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || defined (__AVR_AT90S8535__)
typedef Bit<IO::port_B,IO::bit_7,true>  SCK_bit;
typedef Bit<IO::port_B,IO::bit_6,false> MISO_bit;
typedef Bit<IO::port_B,IO::bit_5,true>  MOSI_bit;
typedef Bit<IO::port_B,IO::bit_4,true>  SS_bit;
#else
#  undef AVRLIB_SPI_ENABLED
#  warning "AVRLIB::SPI: unknown device type"
#endif

} // namespace SPI

#if defined (AVRLIB_SPI_ENABLED)

#if defined (__AVR_ATmega8535__) || defined (__AVR_ATmega16__)
#  define SPI_DOUBLE_SPEED
#elif defined (__AVR_AT90S8535__)
#  undef SPI_DOUBLE_SPEED
#else
#  warning "unknown device type"
#endif

ISR(SPI_STC_vect);

class Spi
{
public:
	enum Order {order_msb, order_lsb};
	enum Mode  {mode_slave, mode_master};
	enum Idle  {idle_low, idle_high};
	enum Edge  {edge_lead, edge_trail};
	enum Isr   {isr_disable, isr_enable};
#if defined (SPI_DOUBLE_SPEED)
	enum Speed {speed_normal, speed_double};
	enum Rate  {rate_4, rate_16, rate_64, rate_128, rate_2, rate_8, rate_32, rate_64_2X};
#else
	enum Rate  {rate_4, rate_16, rate_64, rate_128};
#endif

	static volatile uint8_t& Data () { return SPI::port_ref<SPI::Data>(); }
	static volatile uint8_t& Status () { return SPI::port_ref<SPI::Status>(); }
	static volatile uint8_t& Control () { return SPI::port_ref<SPI::Control>(); }
	typedef IO::RegBit<Status, (IO::Bit)SPIF> SPIF_bit;
	typedef IO::RegBit<Control, (IO::Bit)SPE> SPE_bit;
	typedef IO::RegBit<Control, (IO::Bit)SPIE> SPIE_bit;
	typedef IO::RegBit<Control, (IO::Bit)MSTR> MSTR_bit;
	static void init(
		Mode mode,
		Rate rate   = rate_16,
		Isr isr     = isr_disable, 
#if defined (SPI_DOUBLE_SPEED)
		Speed speed = speed_normal,
#endif
		Idle idle   = idle_low,
		Edge edge   = edge_lead,
		Order order = order_msb)
	{
		if (mode == mode_master)
		{
			SPI::SCK_bit::init();
			SPI::MISO_bit::init();
			SPI::MOSI_bit::init();
		}
		else // mode == mode_slave
		{
			SPI::SCK_bit::init(true);
			SPI::MISO_bit::init(true);
			SPI::MOSI_bit::init(true);
		}
#if defined (SPI_DOUBLE_SPEED)
		// SPI2X
		set_speed(speed);
		if (rate > rate_128)
			set_speed(speed_double);
#endif

		Control() =
			  MAKEBIT(SPE)
			| (order << DORD)
			| (mode  << MSTR)
			| (idle  << CPOL)
			| (edge  << CPHA)
			| (isr   << SPIE)
			| (((rate >> 1) & 1) << SPR1)
			| (((rate >> 0) & 1) << SPR0);
	}
#if defined (SPI_DOUBLE_SPEED)
	typedef IO::RegBit<SPI::port_ref<SPI::Status>, (IO::Bit)SPI2X> SPI2X_bit;
	static void set_speed(Speed sp) {
		if (sp == speed_double)
			SPI2X_bit::set();
		else 
			SPI2X_bit::clr();
	}
#endif
	static void reinit(Mode mode)
	{
		SPE_bit::set();
		MSTR_bit::set(mode);
	}
	static void stop()
	{
		SPE_bit::clr();
	}
	static void isr(Isr isr)
	{
		SPIE_bit::set(isr);
	}
	static uint8_t is_complete()
	{
		return SPIF_bit::get();
	}
	static void send(uint8_t data)
	{
		Data() = data;
	}
	static uint8_t recv()
	{
		return Data();
	}
	
	friend void SPI_STC_vect(void);
};

#endif

} // namespace AVRLIB

#endif//__AVRLIB_SPI_H__INCLUDED__
