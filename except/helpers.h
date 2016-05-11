#ifndef EXCPET_HELPERS_H_
#define EXCPET_HELPERS_H_

#include "helpers.inc"
#include <system_error>

namespace lse {
namespace except {

struct TimeoutError : public std::exception {
  const char* what() const noexcept {
    return "TEXT_ERROR_TIMEOUT_ID";
  }
};

inline void ThrowSystemError(const std::string& what) {
  const std::runtime_error wrapper(what);
  if (!errno) throw wrapper;
  try {
    throw std::system_error(errno, std::system_category());
  } catch (const std::exception&) {
    std::throw_with_nested(wrapper);
  }
}

template<typename... T>
inline void ThrowNested(const T&... args) {
  detail::ThrowNestedImpl(args...);
}

template<typename T>
inline void UnwindNested(const std::exception& ex, T callback) {
  callback(ex);
  try {
    std::rethrow_if_nested(ex);
  } catch (const std::exception& ex) {
    UnwindNested(ex, callback);
  }
}

}  // namespace except
}  // namespace lse

#endif  // EXCEPT_HELPERS_H_
