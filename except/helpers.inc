#ifndef EXCPET_HELPERS_INC_
#define EXCPET_HELPERS_INC_

#include <stdexcept>

namespace detail {

inline void ThrowNestedImpl(const std::string& what) {
  throw std::runtime_error(what);
}

template<typename... T>
inline void ThrowNestedImpl(const std::string& what, const T&... tail) {
  try {
    ThrowNestedImpl(tail...);
  } catch (const std::exception& ex) {
    std::throw_with_nested(std::runtime_error(what));
  }
}

}  // namespace detail

#endif  // EXCEPT_HELPERS_INC_
