#ifndef __AVRLIB_INTERVAL_H__INCLUDED__
#define __AVRLIB_INTERVAL_H__INCLUDED__

#ifndef __cplusplus
#error C++ compilation required
#endif

namespace AVRLIB {

/**
 * Interval: отслеживание интервалов
 */

template <typename Count, typename Value, Count get_count(), bool neg>
class Interval
{
public:
	typedef Count count_type;
	typedef Value value_type;

	Interval(): bound(0), enabled(false) {}

	void reset()
	{
		bound = get_count();
		enabled = false;
	}
	void set(value_type value)
	{
		neg ? bound -= value : bound += value;
		enabled = true;
	}
	bool get_event();

private:
	count_type bound;
	bool enabled;
};

template <typename Count, typename Value, Count get_count(), bool neg>
bool Interval<Count, Value, get_count, neg>::get_event()
{
	if (!enabled)
		return false;

	count_type excess = get_count() - bound;
	if (neg ? (excess > 0) : (excess < 0))
		return false;

	enabled = false;
	return true;
}

} // namespace AVRLIB

#endif//__AVRLIB_INTERVAL_H__INCLUDED__
