#ifndef AUTOCAL_CERES_COMMON_HPP
#define AUTOCAL_CERES_COMMON_HPP

#include <exception>
#include <stdexcept>
#include <sstream>
#include <typeinfo>

// #include "../param/ParameterBlock.hpp"
// #include "../error/ErrorInterface.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define AUTOCAL_DEFINE_EXCEPTION(exceptionName, exceptionParent)               \
  class exceptionName : public exceptionParent {                               \
  public:                                                                      \
    exceptionName(const char *message) : exceptionParent(message) {}           \
    exceptionName(std::string const &message) : exceptionParent(message) {}    \
    virtual ~exceptionName() throw() {}                                        \
  };

#define AUTOCAL_THROW(exceptionType, message)                                  \
  {                                                                            \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << message;                                    \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_THROW_SFP(exceptionType, SourceFilePos, message)               \
  {                                                                            \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << message;                                    \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       SourceFilePos,                                          \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_TRUE(exceptionType, condition, message)                 \
  if (!(condition)) {                                                          \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "assert(" << #condition                     \
                                << ") failed: " << message;                    \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_FALSE(exceptionType, condition, message)                \
  if ((condition)) {                                                           \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "assert( not " << #condition                \
                                << ") failed: " << message;                    \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_GE_LT(exceptionType,                                    \
                             value,                                            \
                             lowerBound,                                       \
                             upperBound,                                       \
                             message)                                          \
  if ((value) < (lowerBound) || (value) >= (upperBound)) {                     \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "assert(" << #lowerBound << " <= " << #value << " < "               \
        << #upperBound << ") failed [" << (lowerBound) << " <= " << (value)    \
        << " < " << (upperBound) << "]: " << message;                          \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_LT(exceptionType, value, upperBound, message)           \
  if ((value) >= (upperBound)) {                                               \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "assert(" << #value << " < " << #upperBound \
                                << ") failed [" << (value) << " < "            \
                                << (upperBound) << "]: " << message;           \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_GE(exceptionType, value, lowerBound, message)           \
  if ((value) < (lowerBound)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "assert(" << #value << " >= " << #lowerBound << ") failed ["        \
        << (value) << " >= " << (lowerBound) << "]: " << message;              \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_LE(exceptionType, value, upperBound, message)           \
  if ((value) > (upperBound)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "assert(" << #value << " <= " << #upperBound << ") failed ["        \
        << (value) << " <= " << (upperBound) << "]: " << message;              \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_GT(exceptionType, value, lowerBound, message)           \
  if ((value) <= (lowerBound)) {                                               \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "assert(" << #value << " > " << #lowerBound \
                                << ") failed [" << (value) << " > "            \
                                << (lowerBound) << "]: " << message;           \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_EQ(exceptionType, value, testValue, message)            \
  if ((value) != (testValue)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "assert(" << #value << " == " << #testValue \
                                << ") failed [" << (value)                     \
                                << " == " << (testValue) << "]: " << message;  \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_NE(exceptionType, value, testValue, message)            \
  if ((value) == (testValue)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "assert(" << #value << " != " << #testValue \
                                << ") failed [" << (value)                     \
                                << " != " << (testValue) << "]: " << message;  \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_NEAR(exceptionType,                                     \
                            value,                                             \
                            testValue,                                         \
                            abs_error,                                         \
                            message)                                           \
  if (!(fabs((testValue) - (value)) <= fabs(abs_error))) {                     \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "assert(" << #value << " == " << #testValue << ") failed ["         \
        << (value) << " == " << (testValue) << " ("                            \
        << fabs((testValue) - (value)) << " > " << fabs(abs_error)             \
        << ")]: " << message;                                                  \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#ifndef NDEBUG

#define AUTOCAL_THROW_DBG(exceptionType, message)                              \
  {                                                                            \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << message;                                    \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_TRUE_DBG(exceptionType, condition, message)             \
  if (!(condition)) {                                                          \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "debug assert(" << #condition               \
                                << ") failed: " << message;                    \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_FALSE_DBG(exceptionType, condition, message)            \
  if ((condition)) {                                                           \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "debug assert( not " << #condition          \
                                << ") failed: " << message;                    \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_DBG_RE(condition, message)                              \
  AUTOCAL_ASSERT_DBG(std::runtime_error, condition, message)

#define AUTOCAL_ASSERT_GE_LT_DBG(exceptionType,                                \
                                 value,                                        \
                                 lowerBound,                                   \
                                 upperBound,                                   \
                                 message)                                      \
  if ((value) < (lowerBound) || (value) >= (upperBound)) {                     \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "debug assert(" << #lowerBound << " <= " << #value << " < "         \
        << #upperBound << ") failed [" << (lowerBound) << " <= " << (value)    \
        << " < " << (upperBound) << "]: " << message;                          \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_LT_DBG(exceptionType, value, upperBound, message)       \
  if ((value) >= (upperBound)) {                                               \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "debug assert(" << #value << " < "          \
                                << #upperBound << ") failed [" << (value)      \
                                << " < " << (upperBound) << "]: " << message;  \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)       \
  if ((value) < (lowerBound)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "debug assert(" << #value << " >= " << #lowerBound << ") failed ["  \
        << (value) << " >= " << (lowerBound) << "]: " << message;              \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_LE_DBG(exceptionType, value, upperBound, message)       \
  if ((value) > (upperBound)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "debug assert(" << #value << " <= " << #upperBound << ") failed ["  \
        << (value) << " <= " << (upperBound) << "]: " << message;              \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)       \
  if ((value) <= (lowerBound)) {                                               \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream << "debug assert(" << #value << " > "          \
                                << #lowerBound << ") failed [" << (value)      \
                                << " > " << (lowerBound) << "]: " << message;  \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_EQ_DBG(exceptionType, value, testValue, message)        \
  if ((value) != (testValue)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "debug assert(" << #value << " == " << #testValue << ") failed ["   \
        << (value) << " == " << (testValue) << "]: " << message;               \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_NE_DBG(exceptionType, value, testValue, message)        \
  if ((value) == (testValue)) {                                                \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "debug assert(" << #value << " != " << #testValue << ") failed ["   \
        << (value) << " != " << (testValue) << "]: " << message;               \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_ASSERT_NEAR_DBG(exceptionType,                                 \
                                value,                                         \
                                testValue,                                     \
                                abs_error,                                     \
                                message)                                       \
  if (!(fabs((testValue) - (value)) <= fabs(abs_error))) {                     \
    std::stringstream autocal_assert_stringstream;                             \
    autocal_assert_stringstream                                                \
        << "debug assert(" << #value << " == " << #testValue << ") failed ["   \
        << (value) << " == " << (testValue) << " ("                            \
        << fabs((testValue) - (value)) << " > " << fabs(abs_error)             \
        << ")]: " << message;                                                  \
    autocal::detail::AUTOCAL_throw_exception<                                  \
        exceptionType>("[" #exceptionType "] ",                                \
                       __FUNCTION__,                                           \
                       __FILE__,                                               \
                       __LINE__,                                               \
                       autocal_assert_stringstream.str());                     \
  }

#define AUTOCAL_OUT(X) std::cout << #X << ": " << (X) << std::endl

#else

#define AUTOCAL_OUT(X)
#define AUTOCAL_THROW_DBG(exceptionType, message)
#define AUTOCAL_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define AUTOCAL_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define AUTOCAL_ASSERT_GE_LT_DBG(exceptionType,                                \
                                 value,                                        \
                                 lowerBound,                                   \
                                 upperBound,                                   \
                                 message)
#define AUTOCAL_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define AUTOCAL_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define AUTOCAL_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define AUTOCAL_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define AUTOCAL_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define AUTOCAL_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define AUTOCAL_ASSERT_NEAR_DBG(exceptionType,                                 \
                                value,                                         \
                                testValue,                                     \
                                abs_error,                                     \
                                message)
#endif

namespace autocal {

class source_file_pos {
public:
  std::string function;
  std::string file;
  int line;

  source_file_pos(std::string function, std::string file, int line)
      : function(function), file(file), line(line) {}

  operator std::string() { return toString(); }

  std::string toString() const {
    std::stringstream s;
    s << file << ":" << line << ": " << function << "()";
    return s.str();
  }
};

namespace detail {

template <typename AUTOCAL_EXCEPTION_T>
void AUTOCAL_throw_exception(std::string const &exceptionType,
                             autocal::source_file_pos sfp,
                             std::string const &message) {
  std::stringstream autocal_assert_stringstream;
  // I have no idea what broke doesn't work with the << operator. sleutenegger:
  // not just Windows, but in general...???
  autocal_assert_stringstream << exceptionType << sfp.toString() << " "
                              << message;
  throw(AUTOCAL_EXCEPTION_T(autocal_assert_stringstream.str()));
}

template <typename AUTOCAL_EXCEPTION_T>
void AUTOCAL_throw_exception(std::string const &exceptionType,
                             std::string const &function,
                             std::string const &file,
                             int line,
                             std::string const &message) {
  AUTOCAL_throw_exception<
      AUTOCAL_EXCEPTION_T>(exceptionType,
                           autocal::source_file_pos(function, file, line),
                           message);
}

} // namespace detail

template <typename AUTOCAL_EXCEPTION_T>
void autocal_assert_throw(bool assert_condition,
                          std::string message,
                          autocal::source_file_pos sfp) {
  if (!assert_condition) {
    detail::AUTOCAL_throw_exception<AUTOCAL_EXCEPTION_T>("", sfp, message);
  }
}

// bool check_jacobians(
//     ErrorInterface *error,
//     std::vector<ParameterBlock *> &param_blocks,
//     double rel_tol = 1e-4,
//     double delta = 1e-8);

} // namespace autocal
#endif // AUTOCAL_CERES_COMMON_HPP
