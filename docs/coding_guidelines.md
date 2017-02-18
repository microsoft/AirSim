# Modern C++ Coding Guidelines

## Quick Note

The great thing about "standards" is that there are many to chose from: [ISO](https://isocpp.org/wiki/faq/coding-standards), [Sutter &amp; Stroustrup](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md), [ROS](http://wiki.ros.org/CppStyleGuide), [Boost](http://www.boost.org/doc/libs/1_34_0/more/lib_guide.htm), [LINUX](https://www.kernel.org/doc/Documentation/CodingStyle), [Google's](https://google.github.io/styleguide/cppguide.html), [Microsoft's](https://msdn.microsoft.com/en-us/library/888a6zcz.aspx), [CERN's](http://atlas-computing.web.cern.ch/atlas-computing/projects/qa/draft_guidelines.html), [GCC's](https://gcc.gnu.org/wiki/CppConventions), [ARM's](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0475c/CJAJAJCJ.html), [LLVM's](http://llvm.org/docs/CodingStandards.html) and probably other thousands. Unfortunately most of these can't even agree on something as basic as how to name a class or a constant. This is probably due to the fact that these standards often carry lot of their own legacy so their concern is also not to depart too much from their existing code bases. The intention behind this document is to create guidance that remains as close to ISO, Sutter &amp; Stroustrup, ROS and Boost while resolving as many conflicts, disadvantages and inconsistencies as possible among them with focus on developing open source infrastructure.

## Naming Conventions

| **Code Element** | **Style** | **Comment** |
| --- | --- | --- |
| Namespace | under\_scored | Differentiate from class names |
| Class name | CamelCase | To differentiate from STL types which ISO recommends |
| Function name | camelCase | Lower case start is almost universal except for .Net world |
| Parameters/Locals | under\_scored | Vast majority of standards recommends this because \_ is more readable to C++ crowd (although not much to Java/.Net crowd) |
| Member variables | under\_scored\_with\_ | The prefix \_ is heavily discouraged as ISO has rules around reserving \_identifiers |
| Enums and its members | CamelCase | Most except very old ones agree |
| Globals | g\_under\_scored | You shouldn't have these in first place! |
| Constants | UPPER\_CASE | Very contentious and we just have to pick one here |
| File names | Match case of class name in file | Lot of pro and cons either way but this removes inconsistency in auto generated code (important for ROS) |

## Header Files

Use,

```
#ifndef msr_airsim_MyHeader_hpp
#define msr_airsim_MyHeader_hpp

//--your code

#endif
```

The reason we don't use #pragma once is because it's not supported if same header file exists at multiple places (which might be possible under ROS build system!).

## Bracketing

Except for functions, place curly bracket on same line. This is called [K&amp;R style](https://en.wikipedia.org/wiki/Indent_style#K.26R_style) and its variants are widely used in C++ vs other styles which are more popular in other languages. Notice that curlies are not required if you have single statement.

```
int main(int argc, char* argv[])
{
     while (x == y) {
        f0();
         if (cont())
           f1();
         else {
            f2();
            f3();
        }
    }
}
```

## Const and References

Religiously review all non-scalar parameters you declare to be candidate for const and references. If you are coming from languages such as C#/Java/Python, the most often mistake you would make is to pass parameters by value instead of `const T&;` Especially most of the strings, vectors and maps you want to pass as `const T&;` (if they are readonly) or `T&` (if they are writable). Also add `const` suffix to methods as much as possible. When overriding virtual method, use override suffix.

## Pointers

There are very few reasons to use `new` or `delete` explicitly. Instead use smart pointers. However, this does not mean you should avoid using naked pointers! It's generally best not to pass around smart pointers in your interfaces. Instead design the code so that there is one clear owner of smart pointer while everyone else can just raw pointers. Sometime this might make things more complex and passing smart pointers around instead might be right thing to do. There is no silver bullet to this design criteria but prefer not having smart pointers in your public interfaces. Religiously check if you can use const everywhere, for example, `const float * const xP`. Avoid using prefix or suffix to indicate pointer types in variable names, i.e. use `my_obj` instead of `myobj_ptr` except in cases where it might make sense to differentiate variables better, for example, `int mynum = 5; int* mynum_ptr = mynum;`

## This is Too Short, ye?

Yes, and it's on purpose because no one likes to read 200 page coding guidelines. The goal here is to cover only most significant things. If you had like to know about how to write better code in C++, please see [GotW](https://herbsutter.com/gotw/) and [Effective Modern C++](http://shop.oreilly.com/product/0636920033707.do) book. All our code requires (strict mode compilation in GCC)[http://shitalshah.com/p/how-to-enable-and-use-gcc-strict-mode-compilation/] and  Level 4 warnings-as-errors in VC++.
