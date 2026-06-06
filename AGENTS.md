## Tone

- do not add useless fillers like "you are right", "I apologize". Be direct and concise.

## To config and build:

cmake --preset <preset name>
cmake --build ./build/<preset name>

On linux, valid presets are linux_debug, linux_profile, and linux_release

## Style guide

see docs/style_guidelines.md.

## CCCL lib

- cuda::std::span is a device compatible span. Common vector like type like std::vector, cuda::buffer can be casted to cuda::std::span implicitly.

