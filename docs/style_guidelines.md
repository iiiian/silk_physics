## Types in PascalCase

Keep `class`, `struct`, and `enum` names in PascalCase.

```cpp
class AppleManager { /* ... */ };
enum class FruitKind { Sweet, Sour };
```

## Functions in lower_snake_case

Define free functions and methods with lower_snake_case identifiers.

```cpp
bool is_ripe(const Fruit& fruit);
```

## Variables in lower_snake_case

Use lower_snake_case for locals and parameters; append `_` to private data members to signal scope.

```cpp
int max_items;
```

## Constexpr Variables in SCREAMING_SNAKE

Use SCREAMING_SNAKE for `constexpr` variables.

```cpp
constexpr uint32_t FLAG_MASK;
```

## Suffix Non-Public Class Member Variables with `_`

```cpp
class Basket {
 private:
  int fruit_count_;

 public:
  int capacity;
};
```

## Size, Length & Count

Variables and struct and class members representing size, length or count should use the following suffixes:

- `_num`: The number of items in an array, vector or other container.
- `_count`: Accumulated, counted values (such as the number of items in a linked-list).
- `_size`: Size in bytes.
- `_len`: For strings (the length of the string without it's null byte, as used in `strlen`).

## `const` Strictness

Enforce `const` strictly everywhere except inside a function/class method.

```c++ 
int sum(const std::vector<int>& vec){
    
    // const is optional in function body
    int element_num = vec.size();
    
    // ...
}
```

No const for scalar types like double, int, etc.

## Always Use `{}`

Always use brackets in `if`, `for` construct.

```c++
// Never skip {}
if (true){
    // ...
}
```

## Index type

Use `int` as index type and always assume it's large enough. static_cast<some size_t type> should be avoided. rely on implicit cast.
