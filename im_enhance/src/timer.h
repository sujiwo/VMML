#include <boost/date_time/posix_time/posix_time.hpp>



namespace ice {

typedef boost::posix_time::ptime ptime;
typedef boost::posix_time::time_duration tduration;

inline ptime getCurrentTime ()
{ return boost::posix_time::microsec_clock::local_time(); }

inline double to_seconds (const tduration &td)
{ return (double(td.total_microseconds()) / 1e6); }

}	// namespace ice
