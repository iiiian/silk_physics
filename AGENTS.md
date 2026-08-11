## Tone

- do not add useless fillers like "you are right", "I apologize". Be direct and concise.

## To config and build:

cmake --preset <preset name>
cmake --build ./build/<preset name>

On linux, valid presets are linux_debug, linux_profile, and linux_release. Ninja generator already uses all threads to build, do not pass -j and limit it manually. 

## Style guide

see docs/style_guidelines.md.

## CCCL lib

- cuda::std::span is a device compatible span. Common vector like type like std::vector, cuda::buffer can be casted to cuda::std::span implicitly.

## Error handling and checks

There are two kinds of error:

A. One that happens even in perfect program. Ex. invalid user input.
B. One caused by mistakes. Ex. invalid function parameter range due to mishandled edge cases.

For kind A, use `assert`. For kind B, use conditionaly logic plus `optional` or exception.

You own the whole stack, you should know exactly what kind of error might happens. DO NOT ADD USELESS DEFENSIVE CHECKS.

## Common bad C++ practice from LLM

- Over use `reserve` for std container. Incorrect usage of reserve increases allocation and increases code complexity. Use reserve only if size is known before hand and there's no frequent pushback later.
