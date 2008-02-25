#ifndef __AVRLIB_BUTTON_H__INCLUDED__
#define __AVRLIB_BUTTON_H__INCLUDED__

#ifndef __cplusplus
#error C++ compilation required
#endif

#include <avrlib/portio.h>
#include <avrlib/softimer.h>

namespace AVRLIB {

/**
 * Button: кнопка управления
 *
 * Отвечает за инициализацию входа, соединенного с кнопкой и детектирование
 * нажатия кнопки пользователем.
 */

template <IO::Port port, IO::Bit bit, bool edge>
class Button
{
	typedef Input<port, bit, edge> input;

public:
	Button(): state(false) { input::init(); }

	// Определение момента нажатия кнопки
	bool get_event()
	{
		ASM("; DEBUG: Button::get_event()");

		if (!state)
		{
			if (input::get()) // обнаружено нажатие
			{
				timer.reset();
				timer.set_ue(SOFTIMER_MAKETIME_MS(200));
				state = true;
			}
			return false;
		}
		else // фиксация нажатия
		{
			if (input::get()) // все еще нажата
			{
				if (timer.get_event()) // нажатие зафиксировано
				{
					state = false;
					return true;
				}
			}
			else // отпущена раньше времени
			{
				state = false;
			}
			return false;
		}
		// Возвращение признака нажатия кнопки
		//return (new_state != state) && (state = new_state);
	}

private:
	Softimer timer;
	// Управляющие переменные
	bool state; // сохраненное состояние активности входа
};

} // namespace AVRLIB

#endif//__AVRLIB_BUTTON_H__INCLUDED__
