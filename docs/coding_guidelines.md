# Modern C++ Coding Guidelines

We are using Modern C++11. Smart pointers, Lambdas, and C++11 multithreading primitives are your friend.

## Quick Note

The great thing about "standards" is that there are many to chose from: [ISO](https://isocpp.org/wiki/faq/coding-standards), [Sutter &amp; Stroustrup](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md), [ROS](http://wiki.ros.org/CppStyleGuide), [LINUX](https://www.kernel.org/doc/Documentation/process/coding-style.rst), [Google's](https://google.github.io/styleguide/cppguide.html), [Microsoft's](https://msdn.microsoft.com/en-us/library/888a6zcz.aspx), [CERN's](http://atlas-computing.web.cern.ch/atlas-computing/projects/qa/draft_guidelines.html), [GCC's](https://gcc.gnu.org/wiki/CppConventions), [ARM's](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0475c/CJAJAJCJ.html), [LLVM's](http://llvm.org/docs/CodingStandards.html) and probably 
thousands of others. Unfortunately most of these can't even agree on something as basic as how to name a class or a constant. This is probably due to the fact that these standards often carry lots of  legacy issues due to supporting existing code bases. The intention behind this document is to create guidance that remains as close to ISO, Sutter &amp; Stroustrup and ROS while resolving as many conflicts, disadvantages and inconsistencies as possible among them.


## clang-format

Formatting the syntax of C++ is normalized by the clang-format tool which has settings checked into
this project in the file `.clang-format`. These settings are set to match the formatting guidelines
listed below.  You can "format" a file using clang-format command line or by enabling Visual Studio
automatic-clang formatting either during every edit or when you save the file.  All files have been
formatted this way and the github workflow called `clang-format` will also ensure all pull requests
are correctly formatted so it should stay clean.  Obviously this does not include external code like
`Eigen` or `rpclib`.  

If you find a bug in clang-format you can disable clang formatting of a specific block of code by
using the following comments pair:

```c++
// clang-format off
...
// clang-format on
```

## Naming Conventions

Avoid using any sort of Hungarian notation on names and "_ptr" on pointers.

| **Code Element** | **Style** | **Comment** |
| --- | --- | --- |
| Namespace | under\_scored | Differentiate from class names |
| Class name | CamelCase | To differentiate from STL types which ISO recommends (do not use "C" or "T" prefixes) |
| Function name | camelCase | Lower case start is almost universal except for .Net world |
| Parameters/Locals | under\_scored | Vast majority of standards recommends this because \_ is more readable to C++ crowd (although not much to Java/.Net crowd) |
| Member variables | under\_scored\_with\_ | The prefix \_ is heavily discouraged as ISO has rules around reserving \_identifiers, so we recommend suffix instead |
| Enums and its members | CamelCase | Most except very old standards agree with this one |
| Globals | g\_under\_scored | You shouldn't have these in first place! |
| Constants | UPPER\_CASE | Very contentious and we just have to pick one here, unless if is a private constant in class or method, then use naming for Members or Locals |
| File names | Match case of class name in file | Lot of pro and cons either way but this removes inconsistency in auto generated code (important for ROS) |

## Header Files

Use a namespace qualified #ifdef to protect against multiple inclusion:

```
#ifndef msr_airsim_MyHeader_hpp
#define msr_airsim_MyHeader_hpp

//--your code

#endif
```

The reason we don't use #pragma once is because it's not supported if same header file exists at multiple places (which might be possible under ROS build system!).

## Bracketing

Inside function or method body place curly bracket on same line. 
Outside that the Namespace, Class and methods levels use separate line.
This is called [K&amp;R style](https://en.wikipedia.org/wiki/Indent_style#K.26R_style) and its variants are widely used in C++ vs other styles which are more popular in other languages. 
Notice that curlies are not required if you have single statement, but complex statements are easier to keep correct with the braces.

```
int main(int argc, char* argv[])
{
     while (x == y) {
        f0();
        if (cont()) {
            f1();
        } else {
            f2();
            f3();
        }
        if (x > 100)
            break;
    }
}
```

## Const and References

Religiously review all non-scalar parameters you declare to be candidate for const and references. If you are coming from languages such as C#/Java/Python,
the most often mistake you would make is to pass parameters by value instead of `const T&;` Especially most of the strings, vectors and maps you want to 
pass as `const T&;` (if they are readonly) or `T&` (if they are writable). Also add `const` suffix to methods as much as possible.

## Overriding
When overriding virtual method, use override suffix.


## Pointers

This is really about memory management.  A simulator has much performance critical code, so we try and avoid overloading the memory manager
with lots of calls to new/delete.  We also want to avoid too much copying of things on the stack, so we pass things by reference when ever possible.
But when the object really needs to live longer than the call stack you often need to allocate that object on
the heap, and so you have a pointer.  Now, if management of the lifetime of that object is going to be tricky we recommend using 
[C++ 11 smart pointers](https://cppstyle.wordpress.com/c11-smart-pointers/). 
But smart pointers do have a cost, so don’t use them blindly everywhere.  For private code 
where performance is paramount, raw pointers can be used.  Raw pointers are also often needed when interfacing with legacy systems
that only accept pointer types, for example, sockets API.  But we try to wrap those legacy interfaces as
much as possible and avoid that style of programming from leaking into the larger code base.  

Religiously check if you can use const everywhere, for example, `const float * const xP`. Avoid using prefix or suffix to indicate pointer types in variable names, i.e. use `my_obj` instead of `myobj_ptr` except in cases where it might make sense to differentiate variables better, for example, `int mynum = 5; int* mynum_ptr = mynum;`

## Null Checking

In Unreal C++ code, when checking if a pointer is null, it is preferable to use `IsValid(ptr)`. In addition to checking for a null pointer, this function will also return whether a UObject is properly initialized. This is useful in situations where a UObject is in the process of being garbage collected but still set to a non-null value.

## Indentation

The C++ code base uses four spaces for indentation (not tabs).

## Line Breaks

Files should be committed with Unix line breaks. When working on Windows, git can be configured to checkout files with Windows line breaks and automatically convert from Windows to Unix line breaks when committing by running the following command:

```
git config --global core.autocrlf true
```

When working on Linux, it is preferable to configure git to checkout files with Unix line breaks by running the following command:

```
git config --global core.autocrlf input
```

For more details on this setting, see [this documentation](https://docs.github.com/en/get-started/getting-started-with-git/configuring-git-to-handle-line-endings).

## This is Too Short, ye?

Yes, and it's on purpose because no one likes to read 200 page coding guidelines. The goal here is to cover only most significant things which are 
already not covered by [strict mode compilation in GCC](http://shitalshah.com/p/how-to-enable-and-use-gcc-strict-mode-compilation/) and Level 4 
warnings-as-errors in VC++. If you had like to know about how to write better code in C++, please see [GotW](https://herbsutter.com/gotw/) 
and [Effective Modern C++](http://shop.oreilly.com/product/0636920033707.do) book.
