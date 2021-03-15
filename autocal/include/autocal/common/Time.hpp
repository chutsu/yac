#ifndef AUTOCAL_TIME_HPP
#define AUTOCAL_TIME_HPP

/*********************************************************************
 ** Pragmas
 *********************************************************************/

#ifdef _MSC_VER
// Okvistime has some magic interface that doesn't directly include
// its implementation, this just disables those warnings.
#pragma warning(disable : 4244)
#pragma warning(disable : 4661)
#endif

#ifdef _WIN32
#include <windows.h>
#endif

/*********************************************************************
 ** Headers
 *********************************************************************/

//#include <ros/platform.h>
#include <iostream>
#include <cmath>
//#include <ros/exception.h>
#include "Duration.hpp"
//#include "rostime_decl.h"

/*********************************************************************
 ** Cross Platform Headers
 *********************************************************************/

#ifdef WIN32
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif

/// \brief srl Main namespace of this package.
namespace autocal {

/*********************************************************************
 ** Exceptions
 *********************************************************************/

/**
 * @brief Thrown if windoze high perf. timestamping is unavailable.
 *
 * @sa getWallTime
 */
class NoHighPerformanceTimersException : std::runtime_error {
public:
  NoHighPerformanceTimersException()
      : std::runtime_error("This windows platform does not "
                           "support the high-performance timing api.") {}
};

/*********************************************************************
 ** Functions
 *********************************************************************/

void normalizeSecNSec(uint64_t &sec, uint64_t &nsec);
void normalizeSecNSec(uint32_t &sec, uint32_t &nsec);
void normalizeSecNSecUnsigned(int64_t &sec, int64_t &nsec);

/*********************************************************************
 ** Time Classes
 *********************************************************************/

/**
 * \brief Base class for Time implementations.  Provides storage, common
 * functions and operator overloads.
 * This should not need to be used directly.
 */
template <class T, class D>
class TimeBase {
public:
  uint32_t sec, nsec;

  TimeBase() : sec(0), nsec(0) {}
  TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec) {
    normalizeSecNSec(sec, nsec);
  }
  explicit TimeBase(double t) { fromSec(t); }
  ~TimeBase() {}
  D operator-(const T &rhs) const;
  T operator+(const D &rhs) const;
  T operator-(const D &rhs) const;
  T &operator+=(const D &rhs);
  T &operator-=(const D &rhs);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const {
    return !(*static_cast<const T *>(this) == rhs);
  }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;

  double toSec() const { return (double) sec + 1e-9 * (double) nsec; };
  T &fromSec(double t) {
    sec = (uint32_t) floor(t);
    nsec = (uint32_t) std::round((t - sec) * 1e9);
    return *static_cast<T *>(this);
  }

  uint64_t toNSec() const {
    return (uint64_t) sec * 1000000000ull + (uint64_t) nsec;
  }
  T &fromNSec(uint64_t t);

  inline bool isZero() const { return sec == 0 && nsec == 0; }
  inline bool is_zero() const { return isZero(); }
};

/**
 * \brief Time representation.  May either represent wall clock time or ROS
 * clock time.
 *
 * autocal::TimeBase provides most of its functionality.
 */
class Time : public TimeBase<Time, Duration> {
public:
  Time() : TimeBase<Time, Duration>() {}

  Time(uint32_t _sec, uint32_t _nsec) : TimeBase<Time, Duration>(_sec, _nsec) {}

  explicit Time(double t) { fromSec(t); }

  /**
   * \brief Retrieve the current time.  Returns the current wall clock time.
   */
  static Time now();
  /**
   * \brief Sleep until a specific time has been reached.
   */
  static bool sleepUntil(const Time &end);

  static void init();
  static void shutdown();
  static void setNow(const Time &new_now);
  static bool useSystemTime();
  static bool isSimTime();
  static bool isSystemTime();

  /**
   * \brief Returns whether or not the current time is valid.  Time is valid if
   * it is non-zero.
   */
  static bool isValid();
  /**
   * \brief Wait for time to become valid
   */
  static bool waitForValid();
  /**
   * \brief Wait for time to become valid, with timeout
   */
  static bool waitForValid(const autocal::WallDuration &timeout);
};

extern const Time TIME_MAX;
extern const Time TIME_MIN;

/**
 * \brief Time representation.  Always wall-clock time.
 *
 * autocal::TimeBase provides most of its functionality.
 */
class WallTime : public TimeBase<WallTime, WallDuration> {
public:
  WallTime() : TimeBase<WallTime, WallDuration>() {}

  WallTime(uint32_t _sec, uint32_t _nsec)
      : TimeBase<WallTime, WallDuration>(_sec, _nsec) {}

  explicit WallTime(double t) { fromSec(t); }

  /**
   * \brief Returns the current wall clock time.
   */
  static WallTime now();

  /**
   * \brief Sleep until a specific time has been reached.
   */
  static bool sleepUntil(const WallTime &end);

  static bool isSystemTime() { return true; }
};

std::ostream &operator<<(std::ostream &os, const Time &rhs);
std::ostream &operator<<(std::ostream &os, const WallTime &rhs);

} // namespace autocal

/*********************************************************************
 ** Headers
 *********************************************************************/

//#include <ros/platform.h>
#include <iostream>
#include <cmath>
//#include <ros/exception.h>

/*********************************************************************
 ** Cross Platform Headers
 *********************************************************************/

#ifdef WIN32
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif

/// \brief srl Main namespace of this package.
namespace autocal {

template <class T, class D>
T &TimeBase<T, D>::fromNSec(uint64_t t) {
  sec = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSec(sec, nsec);

  return *static_cast<T *>(this);
}

template <class T, class D>
D TimeBase<T, D>::operator-(const T &rhs) const {
  return D((int32_t) sec - (int32_t) rhs.sec,
           (int32_t) nsec - (int32_t) rhs.nsec); // carry handled in ctor
}

template <class T, class D>
T TimeBase<T, D>::operator-(const D &rhs) const {
  return *static_cast<const T *>(this) + (-rhs);
}

template <class T, class D>
T TimeBase<T, D>::operator+(const D &rhs) const {
  int64_t sec_sum = (int64_t) sec + (int64_t) rhs.sec;
  int64_t nsec_sum = (int64_t) nsec + (int64_t) rhs.nsec;

  // Throws an exception if we go out of 32-bit range
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);

  // now, it's safe to downcast back to uint32 bits
  return T((uint32_t) sec_sum, (uint32_t) nsec_sum);
}

template <class T, class D>
T &TimeBase<T, D>::operator+=(const D &rhs) {
  *this = *this + rhs;
  return *static_cast<T *>(this);
}

template <class T, class D>
T &TimeBase<T, D>::operator-=(const D &rhs) {
  *this += (-rhs);
  return *static_cast<T *>(this);
}

template <class T, class D>
bool TimeBase<T, D>::operator==(const T &rhs) const {
  return sec == rhs.sec && nsec == rhs.nsec;
}

template <class T, class D>
bool TimeBase<T, D>::operator<(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template <class T, class D>
bool TimeBase<T, D>::operator>(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template <class T, class D>
bool TimeBase<T, D>::operator<=(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template <class T, class D>
bool TimeBase<T, D>::operator>=(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

} // namespace autocal
#endif // AUTOCAL_TIME_HPP
