/**
 * @file result.hpp
 * @brief Minimal, non-throwing status type for Silk APIs.
 *
 * This header defines a simple `Result` class with an implicit boolean
 * conversion and an `ErrorCode` taxonomy. Use it to signal success/failure
 * without exceptions while still carrying concise, human-readable diagnostics
 * via the `detail` string.
 */
#pragma once

#include <string>
#include <string_view>

namespace silk {

/**
 * @enum ErrorCode
 * @brief Canonical error categories returned by Silk operations.
 *
 * These values are stable identifiers for failure modes across the API
 * surface. They are suitable for branching and telemetry. Prefer attaching
 * specifics (sizes, indices, preconditions) to `Result::detail` rather than
 * expanding this enum.
 */
enum class ErrorCode {
  Unknown,
  InvalidConfig,
  TooManyBody,
  InvalidHandle,
  InvalidMesh,
  IncorrectPinNum,
  IncorrectPositionNum,
  CholeskyDecompositionFail,
  NeedInitSolverFirst,
  IterativeSolveFail
};

/**
 * @class Result
 * @brief Non-throwing success/error value with human-friendly detail.
 *
 * Invariants:
 * - Success: `success == true`; `code` is ignored; `detail` is empty.
 * - Error:   `success == false`; `detail` may be empty but should describe
 *                                context when helpful.
 *
 * Usage:
 * @code
 * silk::Result r = do_work();
 * if (!r) {
 *   log_error(r.to_string());
 *   return r; // Propagate.
 * }
 * @endcode
 */
class Result {
 public:
  bool success = true;
  ErrorCode code = ErrorCode::Unknown;
  std::string detail;  ///< Optional diagnostic message for humans.

 public:
  /**
   * @brief Construct a success result.
   * @return A Result with `success == true`.
   */
  static Result ok();

  /**
   * @brief Construct an error result with a code and optional detail.
   * @param error_code Canonical error category.
   * @param detail Optional context (e.g., expected vs. actual sizes).
   * @return A Result with `success == false` and fields populated.
   */
  static Result error(ErrorCode error_code, std::string_view detail = {});

  /**
   * @brief Implicitly convert to `true` on success and `false` on error.
   */
  operator bool() const;

  /**
   * @brief Produce a human-readable summary for logging and diagnostics.
   * @return "OK" for success, or "<ErrorCode>. Reason: <detail>." for errors.
   *          The Reason part will be skipped if detail is empty.
   */
  std::string to_string() const;

 private:
  Result() = default;
};

}  // namespace silk
