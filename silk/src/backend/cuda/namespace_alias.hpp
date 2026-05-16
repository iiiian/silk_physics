#pragma once

// libcu++ cuda:: and cuda::std:: namespace conflicts with silk::cuda,
// making it annoying to use. To remedy this, rename them to cu:: and ctd::.
namespace cuda {}
namespace cuda::std {}

namespace silk::cuda {
namespace cu = ::cuda;
namespace ctd = ::cuda::std;
}  // namespace silk::cuda
