#pragma once

namespace silk {

enum class SymmetricStatus : int {
  NotSymmetric = 0,
  LowerTriangular = -1,
  UpperTriangular = 1
};

}
