# C++ `std::vector` 常用用法

`std::vector` 是 C++ 标准库中一个动态数组类模板，它可以根据需要动态调整大小，并提供了一系列常用的方法和操作。

## 1. 初始化

### 1.1 默认初始化
```cpp
std::vector<int> vec; // 空的 vector
```

### 1.2 指定大小初始化
```cpp
std::vector<int> vec(10); // 包含 10 个元素，默认初始化为 0
```

### 1.3 指定大小和初始值
```cpp
std::vector<int> vec(10, 5); // 包含 10 个元素，每个元素初始化为 5
```

### 1.4 用数组或其他容器初始化
```cpp
int arr[] = {1, 2, 3, 4};
std::vector<int> vec(arr, arr + 4); // 用数组初始化
```

## 2. 常用方法

### 2.1 `push_back()`
向 `vector` 的末尾添加元素。
```cpp
std::vector<int> vec;
vec.push_back(10); // 向 vec 中添加 10
```

### 2.2 `size()`
返回 `vector` 中元素的个数。
```cpp
int n = vec.size(); // 返回 vec 的大小
```

### 2.3 `empty()`
判断 `vector` 是否为空。
```cpp
if (vec.empty()) {
    // 如果 vec 为空，执行一些操作
}
```

### 2.4 `at()`
获取指定位置的元素，并且进行边界检查。
```cpp
int val = vec.at(2); // 获取 vec 中索引为 2 的元素
```

### 2.5 `operator[]`
获取指定位置的元素，不进行边界检查。
```cpp
int val = vec[2]; // 获取 vec 中索引为 2 的元素
```

### 2.6 `front()` 和 `back()`
获取 `vector` 的首元素和尾元素。
```cpp
int first = vec.front(); // 获取第一个元素
int last = vec.back();   // 获取最后一个元素
```

### 2.7 `insert()`
在指定位置插入元素。
```cpp
std::vector<int> vec = {1, 2, 3};
vec.insert(vec.begin() + 1, 10); // 在索引为 1 的位置插入 10
```

### 2.8 `erase()`
删除指定位置的元素。
```cpp
vec.erase(vec.begin() + 1); // 删除索引为 1 的元素
```

### 2.9 `clear()`
清空 `vector` 中的所有元素。
```cpp
vec.clear(); // 清空 vec
```

### 2.10 `resize()`
改变 `vector` 的大小。
```cpp
vec.resize(5); // 将 vec 的大小改变为 5
```

## 3. 迭代器

### 3.1 `begin()` 和 `end()`
返回 `vector` 的首迭代器和尾迭代器。
```cpp
for (auto it = vec.begin(); it != vec.end(); ++it) {
    std::cout << *it << " "; // 输出 vec 的所有元素
}
```

### 3.2 `rbegin()` 和 `rend()`
返回 `vector` 的逆向迭代器。
```cpp
for (auto it = vec.rbegin(); it != vec.rend(); ++it) {
    std::cout << *it << " "; // 逆序输出 vec 的所有元素
}
```

## 4. 容量相关

### 4.1 `capacity()`
返回当前 `vector` 能容纳的元素个数。
```cpp
size_t cap = vec.capacity(); // 获取 vec 的容量
```

### 4.2 `reserve()`
预先分配至少 `n` 个元素的存储空间，但不改变 `size`。
```cpp
vec.reserve(100); // 预分配空间以容纳 100 个元素
```

### 4.3 `shrink_to_fit()`
请求减少容量，使其与大小相等。
```cpp
vec.shrink_to_fit(); // 释放多余的容量
```

## 5. C++11 特性

### 5.1 列表初始化
可以用初始化列表来初始化 `vector`。
```cpp
std::vector<int> vec = {1, 2, 3, 4, 5};
```

### 5.2 `emplace_back()`
在 `vector` 末尾就地构造元素，避免拷贝或移动操作。
```cpp
vec.emplace_back(10); // 在末尾构造 10
```

## 6. 注意事项

1. 动态扩容：`vector` 是动态扩展的，但当容量不足时，扩展会导致重新分配内存，已有的元素会被拷贝到新的存储位置。
2. 访问越界：使用 `[]` 操作符访问元素时不进行边界检查，如果越界访问可能导致未定义行为。推荐使用 `at()` 方法，它会抛出异常来避免越界访问。
