#ifndef PATTERNS_RAII_H_
#define PATTERNS_RAII_H_

#include "except/helpers.h"
#include <unistd.h>

namespace lse {
namespace patterns {

/**
 * @brief Manages file descriptor in a transparent way
 */
class RaiiFd {
 protected:
  int fd_;

 public:
  /**
   * @brief No default ctor available
   */
  RaiiFd() = delete;

  /**
   * @brief Implicitly converts file descriptor to RaiiFd
   *
   * @param fd Any file descriptor
   */
  inline RaiiFd(int fd) : fd_(fd) {
    if (fd_ < 0) except::ThrowSystemError("TEXT_BAD_FILE_DESCRIPTOR_ID");
  }

  /**
   * @brief Move constructor for RaiiFd
   *
   * @param op Disowning RaiiFd
   */
  inline RaiiFd(RaiiFd&& op) : fd_(op.fd_) {
    op.fd_ = -1;
  }

  /**
   * @brief Implicitly converts RaiiFd to file descriptor
   *
   * @return Managed file descriptor
   */
  inline operator int() const {return fd_;}

  /**
   * @brief Automatically closes managed file descriptor upon leaving code block
   */
  virtual ~RaiiFd() {
    if (fd_ >= 0) close(fd_);
  }
};

}  // namespace logging
}  // namespace lse

#endif  // PATTERNS_RAII_H_
