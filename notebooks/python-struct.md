# 示例

## 使用大端顺序打包和解包三种不同大小的整数

    from struct import *
    pack(">bhl", 1, 2, 3)

    unpack('>bhl', b'\x01\x00\x02\x00\x00\x00\x03')

    calcsize('>bhl')

# Functions and Exceptions函数和异常 ¶

The module defines the following exception and functions:
该模块定义了以下异常和函数：

## exception struct.error 异常

Exception raised on various occasions; argument is a string describing what is wrong.

在不同场合提出异常； argument 是一个描述错误的字符串。

## struct.pack(format, v1, v2, ...)

Return a bytes object containing the values v1, v2, … packed according to the format string format. The arguments must match the values required by the format exactly.

返回一个字节对象，其中包含根据格式字符串格式打包的值 v1、v2、...。参数必须与格式所需的值完全匹配。

## struct.pack_into(format, buffer, offset, v1, v2, ...)

Pack the values v1, v2, … according to the format string format and write the packed bytes into the writable buffer buffer starting at position offset. Note that offset is a required argument.

根据格式字符串格式打包值 v1、v2、...，并将打包字节写入从位置 offset 开始的可写缓冲区 buffer。请注意，偏移量是必需的参数。

## struct.unpack(format, buffer)

Unpack from the buffer buffer (presumably packed by pack(format, ...)) according to the format string format. The result is a tuple even if it contains exactly one item. The buffer’s size in bytes must match the size required by the format, as reflected by calcsize().

根据格式字符串格式从缓冲区 buffer 中解包（大概由 pack(format, ...) 打包）。结果是一个元组，即使它只包含一项。缓冲区的大小（以字节为单位）必须与格式所需的大小相匹配，如 calcsize() 所反映的。

## struct.unpack_from(format, /, buffer, offset=0)

Unpack from buffer starting at position offset, according to the format string format. The result is a tuple even if it contains exactly one item. The buffer’s size in bytes, starting at position offset, must be at least the size required by the format, as reflected by calcsize().

根据格式字符串格式，从偏移位置开始从缓冲区解包。结果是一个元组，即使它只包含一项。缓冲区的大小（以字节为单位）从位置偏移量开始，必须至少是格式所需的大小，如 calcsize() 所反映的。

## struct.iter_unpack(format, buffer)

Iteratively unpack from the buffer buffer according to the format string format. This function returns an iterator which will read equally sized chunks from the buffer until all its contents have been consumed. The buffer’s size in bytes must be a multiple of the size required by the format, as reflected by calcsize().

根据格式字符串格式迭代地从缓冲区 buffer 中解包。该函数返回一个迭代器，它将从缓冲区读取相同大小的块，直到其所有内容都被消耗。缓冲区的大小（以字节为单位）必须是格式所需大小的倍数，如 calcsize() 所反映。

Each iteration yields a tuple as specified by the format string.

每次迭代都会生成一个由格式字符串指定的元组。

New in version 3.4. 3.4 版本中的新功能。

## struct.calcsize(format)

Return the size of the struct (and hence of the bytes object produced by pack(format, ...)) corresponding to the format string format.

返回与格式字符串格式相对应的结构体的大小（以及 pack(format, ...) 生成的字节对象的大小）
