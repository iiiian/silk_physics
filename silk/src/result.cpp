#include <spdlog/fmt/fmt.h>

#include <silk/result.hpp>

#include "compiler_builtin.hpp"

namespace silk {

std::string error_to_string(ErrorCode error) {
  switch (error) {
    case ErrorCode::Unknown: {
      return "Unknown";
    }
    case ErrorCode::InvalidConfig: {
      return "InvalidConfig";
    }
    case ErrorCode::TooManyBody: {
      return "TooManyBody";
    }
    case ErrorCode::InvalidHandle: {
      return "InvalidHandle";
    }
    case ErrorCode::InvalidMesh: {
      return "InvalidMesh";
    }
    case ErrorCode::IncorrectPinNum: {
      return "IncorrectPinNum";
    }
    case ErrorCode::IncorrectPositionNum: {
      return "IncorrectPositionNum";
    }
    case ErrorCode::CholeskyDecompositionFail: {
      return "CholeskyDecompositionFail";
    }
    case ErrorCode::NeedInitSolverFirst: {
      return "NeedInitSolverFirst";
    }
    case ErrorCode::IterativeSolveFail: {
      return "IterativeSolveFail";
    }
  }

  SILK_UNREACHABLE();
}

Result Result::ok() { return Result(); }

Result Result::error(ErrorCode error_code, std::string_view detail) {
  Result r;
  r.success = false;
  r.code = error_code;
  r.detail = detail;

  return r;
}

Result::operator bool() const { return success; }

std::string Result::to_string() const {
  if (success) {
    return "OK";
  }

  if (detail.empty()) {
    return error_to_string(code);
  }
  return fmt::format("{}. Reason: {}.", error_to_string(code), detail);
}

}  // namespace silk
