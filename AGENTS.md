## To config and build:

cmake --preset <preset name>
cmake --build ./build/<preset name>

On linux, valid presets are linux_debug, linux_profile, and linux_release

## Style guide

- no const for scalar type. Ex. const int, const double.
- no casting between index type. This repo assumes index type is int and is always large enough.
  Ex. static_cast<some size_t type> should be avoided. rely on implicit cast.

