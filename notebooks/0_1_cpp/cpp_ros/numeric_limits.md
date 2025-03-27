以下是关于 C++ 中 `std::numeric_limits` 的 Markdown 笔记：

```markdown
# C++ `std::numeric_limits` 用法笔记

`std::numeric_limits` 是 C++ 标准库中提供的模板类，它用来获取不同数据类型的属性或限制。通过该类，可以查询某个类型的最小值、最大值、精度等信息。

## 1. 基本用法

`std::numeric_limits<T>::member` 可以用来获取类型 `T` 的各项限制或属性。常用的成员函数包括 `min()`, `max()`, `lowest()`, `epsilon()` 等。

```cpp
#include <iostream>
#include <limits>

int main() {
    std::cout << "int 最大值: " << std::numeric_limits<int>::max() << std::endl;
    std::cout << "double 最小值: " << std::numeric_limits<double>::min() << std::endl;
    return 0;
}
```

## 2. 常用成员函数

### 2.1 `min()`
返回类型 `T` 能表示的最小**正数**（对于浮点数）。对于整数类型，返回最小的负值。
```cpp
std::cout << std::numeric_limits<int>::min();      // -2147483648
std::cout << std::numeric_limits<double>::min();   // 2.22507e-308 (正的最小值)
```

### 2.2 `max()`
返回类型 `T` 能表示的最大值。
```cpp
std::cout << std::numeric_limits<int>::max();      // 2147483647
std::cout << std::numeric_limits<double>::max();   // 1.79769e+308
```

### 2.3 `lowest()`
返回类型 `T` 能表示的最小值，通常是负的最大绝对值（适用于浮点数）。
```cpp
std::cout << std::numeric_limits<float>::lowest();  // -3.40282e+38
```

### 2.4 `epsilon()`
返回浮点数类型的机器精度，即 1.0 和下一个可表示的浮点数之间的差值。
```cpp
std::cout << std::numeric_limits<double>::epsilon();  // 2.22045e-16
```

### 2.5 `infinity()`
返回表示正无穷大的值（仅适用于浮点数类型）。
```cpp
std::cout << std::numeric_limits<float>::infinity();  // inf
```

### 2.6 `is_signed`
判断类型是否为有符号类型（返回 `true` 或 `false`）。
```cpp
std::cout << std::numeric_limits<int>::is_signed;     // true
std::cout << std::numeric_limits<unsigned int>::is_signed; // false
```

### 2.7 `digits`
返回类型的有效位数（对于整数，表示二进制位数，对于浮点数，表示尾数的位数）。
```cpp
std::cout << std::numeric_limits<int>::digits;        // 31 (一个 int 类型有 32 位，其中 1 位符号位)
```

### 2.8 `has_infinity`
判断类型是否能表示无穷大（适用于浮点数类型）。
```cpp
std::cout << std::numeric_limits<float>::has_infinity;  // true
```

## 3. 常见的数据类型查询

```cpp
std::numeric_limits<int>::min();       // int 类型的最小值
std::numeric_limits<int>::max();       // int 类型的最大值
std::numeric_limits<float>::infinity();// float 类型的正无穷
std::numeric_limits<double>::epsilon();// double 类型的精度
```

### 示例：
```cpp
#include <iostream>
#include <limits>

int main() {
    std::cout << "int 最小值: " << std::numeric_limits<int>::min() << std::endl;
    std::cout << "int 最大值: " << std::numeric_limits<int>::max() << std::endl;
    std::cout << "float 正无穷: " << std::numeric_limits<float>::infinity() << std::endl;
    std::cout << "double 精度: " << std::numeric_limits<double>::epsilon() << std::endl;
    return 0;
}
```

## 4. 其他常用属性

- `is_integer`: 判断是否为整数类型。
- `is_exact`: 判断类型是否是精确表示的（整数类型为 `true`，浮点类型为 `false`）。
- `has_quiet_NaN`: 判断类型是否支持 `NaN` 值（仅适用于浮点数类型）。
- `has_denorm`: 判断是否支持非标准化浮点数。

## 5. 注意事项

1. 对于浮点数类型，`min()` 返回的是最小正数而不是最小负数。要获取负数的最小值，应该使用 `lowest()`。
2. 对于整数类型，`min()` 返回最小的负数。
3. `std::numeric_limits` 是一个模板类，使用时需要为其提供类型参数，例如 `std::numeric_limits<int>`、`std::numeric_limits<float>` 等。

## 6. 总结

`std::numeric_limits` 是一个非常有用的工具，特别是在处理不同数据类型的边界值时。通过它，我们可以轻松获取每种类型的最小值、最大值、无穷大和精度等信息，从而编写更加健壮的程序。
