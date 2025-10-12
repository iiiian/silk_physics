Most of the style and formating guide is enforced by `clang-format` and the rest are documented in this file. Since I did not start to enforce these rules until later stage of development, some parts of the codebase are inconsistent. If you discover such case, please submit a PR.

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

## Always Use `{}`

Always use brackets in `if`, `for` construct.

```c++
// Never skip {}
if (true){
    // ...
}
```

## Class Declaration Layout

Explicitly write out `private`/`public` in each sections and follow below layout. 

```c++
class Basket {
  public:
    // anything that is not a method. i.e. class member, constexpr variable, using alias
    int private_var_;
  
  private:
    // anything that is not a method. i.e. class member, constexpr variable, using alias
    int public_var_;
    
  public:
    // private class method
    int count();
    
  private:
    // public class method
    void clear();
};
```



## Error Handling

Other than interfacing with external libraries, do not use exception. There are two reasons for this decision:

- The code interacts with many C-style numerical libraries, so avoiding exception style error handling make things consistent.
- Exception throwing is hidden from function signature. Which makes it difficult to reason about hidden code path.

Instead, return `bool`, `std::optional`, or `nullptr` to indicate failure. If detail information is valuable, use `spdlog` library to log it to `stdout`.  

```c++
std::optional<int> func_that_might_fail(); 
```

If a class constructor has a change to fail, use factory method instead.

```c++
class Foo{
  private:
  	// Move default ctor to private
    Foo() = default;
  
  public:
    // Factory function should be static and returns the optional of class type.
    // The name should be try_make_<class name>.
    static std::optional<Foo> try_make_foo();
}
```

## Document and Comments

Start public headers and major interfaces with `/** ... */` blocks that include `@brief`, `@param`, and `@return`.

```cpp
/**
 * @brief Count pieces of fruit in the basket.
 * @param basket Storage container to inspect.
 * @return Number of fruit items currently stored.
 */
int count_fruit(const Basket& basket);
```

Use short `/** ... */` summaries for non-trivial helpers and add inline `//` comments only when intent is unclear.

```cpp
/** Compute the freshness score for a fruit. */
int compute_freshness(const Fruit& fruit) {
  int age = fruit.days_on_shelf;
  // Older fruit loses one point per day.
  return std::max(0, 100 - age);
}
```

Tips: You can feed misc/llm_guidelines/cpp_comment.md to generate the initial comment/doc. Make sure to review it though.
