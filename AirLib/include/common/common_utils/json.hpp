/*
    __ _____ _____ _____
 __|  |   __|     |   | |  JSON for Modern C++
|  |  |__   |  |  | | | |  version 2.1.1
|_____|_____|_____|_|___|  https://github.com/nlohmann/json

Licensed under the MIT License <http://opensource.org/licenses/MIT>.
Copyright (c) 2013-2017 Niels Lohmann <http://nlohmann.me>.

Permission is hereby  granted, free of charge, to any  person obtaining a copy
of this software and associated  documentation files (the "Software"), to deal
in the Software  without restriction, including without  limitation the rights
to  use, copy,  modify, merge,  publish, distribute,  sublicense, and/or  sell
copies  of  the Software,  and  to  permit persons  to  whom  the Software  is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE  IS PROVIDED "AS  IS", WITHOUT WARRANTY  OF ANY KIND,  EXPRESS OR
IMPLIED,  INCLUDING BUT  NOT  LIMITED TO  THE  WARRANTIES OF  MERCHANTABILITY,
FITNESS FOR  A PARTICULAR PURPOSE AND  NONINFRINGEMENT. IN NO EVENT  SHALL THE
AUTHORS  OR COPYRIGHT  HOLDERS  BE  LIABLE FOR  ANY  CLAIM,  DAMAGES OR  OTHER
LIABILITY, WHETHER IN AN ACTION OF  CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE  OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef NLOHMANN_JSON_HPP
#define NLOHMANN_JSON_HPP
// clang-format off
#include <algorithm> // all_of, copy, fill, find, for_each, none_of, remove, reverse, transform
#include <array> // array
#include <cassert> // assert
#include <cctype> // isdigit
#include <ciso646> // and, not, or
#include <cmath> // isfinite, labs, ldexp, signbit
#include <cstddef> // nullptr_t, ptrdiff_t, size_t
#include <cstdint> // int64_t, uint64_t
#include <cstdlib> // abort, strtod, strtof, strtold, strtoul, strtoll, strtoull
#include <cstring> // strlen
#include <forward_list> // forward_list
#include <functional> // function, hash, less
#include <initializer_list> // initializer_list
#include <iomanip> // setw
#include <iostream> // istream, ostream
#include <iterator> // advance, begin, back_inserter, bidirectional_iterator_tag, distance, end, inserter, iterator, iterator_traits, next, random_access_iterator_tag, reverse_iterator
#include <limits> // numeric_limits
#include <locale> // locale
#include <map> // map
#include <memory> // addressof, allocator, allocator_traits, unique_ptr
#include <numeric> // accumulate
#include <sstream> // stringstream
#include <stdexcept> // domain_error, invalid_argument, out_of_range
#include <string> // getline, stoi, string, to_string
#include <type_traits> // add_pointer, conditional, decay, enable_if, false_type, integral_constant, is_arithmetic, is_base_of, is_const, is_constructible, is_convertible, is_default_constructible, is_enum, is_floating_point, is_integral, is_nothrow_move_assignable, is_nothrow_move_constructible, is_pointer, is_reference, is_same, is_scalar, is_signed, remove_const, remove_cv, remove_pointer, remove_reference, true_type, underlying_type
#include <utility> // declval, forward, make_pair, move, pair, swap
#include <vector> // vector

// exclude unsupported compilers
#if defined(__clang__)
    #if (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__) < 30400
        #error "unsupported Clang version - see https://github.com/nlohmann/json#supported-compilers"
    #endif
#elif defined(__GNUC__)
    #if (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__) < 40900
        #error "unsupported GCC version - see https://github.com/nlohmann/json#supported-compilers"
    #endif
#endif

// disable float-equal warnings on GCC/clang
#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

// disable documentation warnings on clang
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdocumentation"
#endif

// allow for portable deprecation warnings
#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
    #define JSON_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
    #define JSON_DEPRECATED __declspec(deprecated)
#else
    #define JSON_DEPRECATED
#endif

// allow to disable exceptions
#if not defined(JSON_NOEXCEPTION) || defined(__EXCEPTIONS)
    #define JSON_THROW(exception) throw exception
    #define JSON_TRY try
    #define JSON_CATCH(exception) catch(exception)
#else
    #define JSON_THROW(exception) std::abort()
    #define JSON_TRY if(true)
    #define JSON_CATCH(exception) if(false)
#endif

/*!
@brief namespace for Niels Lohmann
@see https://github.com/nlohmann
@since version 1.0.0
*/
namespace nlohmann
{

/*!
@brief unnamed namespace with internal helper functions

This namespace collects some functions that could not be defined inside the
@ref basic_json class.

@since version 2.1.0
*/
namespace detail
{
///////////////////////////
// JSON type enumeration //
///////////////////////////

/*!
@brief the JSON type enumeration

This enumeration collects the different JSON types. It is internally used to
distinguish the stored values, and the functions @ref basic_json::is_null(),
@ref basic_json::is_object(), @ref basic_json::is_array(),
@ref basic_json::is_string(), @ref basic_json::is_boolean(),
@ref basic_json::is_number() (with @ref basic_json::is_number_integer(),
@ref basic_json::is_number_unsigned(), and @ref basic_json::is_number_float()),
@ref basic_json::is_discarded(), @ref basic_json::is_primitive(), and
@ref basic_json::is_structured() rely on it.

@note There are three enumeration entries (number_integer, number_unsigned, and
number_float), because the library distinguishes these three types for numbers:
@ref basic_json::number_unsigned_t is used for unsigned integers,
@ref basic_json::number_integer_t is used for signed integers, and
@ref basic_json::number_float_t is used for floating-point numbers or to
approximate integers which do not fit in the limits of their respective type.

@sa @ref basic_json::basic_json(const value_t value_type) -- create a JSON
value with the default value for a given type

@since version 1.0.0
*/
enum class value_t : uint8_t
{
    null,            ///< null value
    object,          ///< object (unordered set of name/value pairs)
    array,           ///< array (ordered collection of values)
    string,          ///< string value
    boolean,         ///< boolean value
    number_integer,  ///< number value (signed integer)
    number_unsigned, ///< number value (unsigned integer)
    number_float,    ///< number value (floating-point)
    discarded        ///< discarded by the the parser callback function
};

/*!
@brief comparison operator for JSON types

Returns an ordering that is similar to Python:
- order: null < boolean < number < object < array < string
- furthermore, each type is not smaller than itself

@since version 1.0.0
*/
inline bool operator<(const value_t lhs, const value_t rhs) noexcept
{
    static constexpr std::array<uint8_t, 8> order = {{
            0, // null
            3, // object
            4, // array
            5, // string
            1, // boolean
            2, // integer
            2, // unsigned
            2, // float
        }
    };

    // discarded values are not comparable
    if (lhs == value_t::discarded or rhs == value_t::discarded)
    {
        return false;
    }

    return order[static_cast<std::size_t>(lhs)] <
           order[static_cast<std::size_t>(rhs)];
}


/////////////
// helpers //
/////////////

// alias templates to reduce boilerplate
template<bool B, typename T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template<typename T>
using uncvref_t = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

// taken from http://stackoverflow.com/a/26936864/266378
template<typename T>
using is_unscoped_enum =
    std::integral_constant<bool, std::is_convertible<T, int>::value and
    std::is_enum<T>::value>;

/*
Implementation of two C++17 constructs: conjunction, negation. This is needed
to avoid evaluating all the traits in a condition

For example: not std::is_same<void, T>::value and has_value_type<T>::value
will not compile when T = void (on MSVC at least). Whereas
conjunction<negation<std::is_same<void, T>>, has_value_type<T>>::value will
stop evaluating if negation<...>::value == false

Please note that those constructs must be used with caution, since symbols can
become very long quickly (which can slow down compilation and cause MSVC
internal compiler errors). Only use it when you have to (see example ahead).
*/
template<class...> struct conjunction : std::true_type {};
template<class B1> struct conjunction<B1> : B1 {};
template<class B1, class... Bn>
struct conjunction<B1, Bn...> : std::conditional<bool(B1::value), conjunction<Bn...>, B1>::type {};

template<class B> struct negation : std::integral_constant < bool, !B::value > {};

// dispatch utility (taken from ranges-v3)
template<unsigned N> struct priority_tag : priority_tag < N - 1 > {};
template<> struct priority_tag<0> {};


//////////////////
// constructors //
//////////////////

template<value_t> struct external_constructor;

template<>
struct external_constructor<value_t::boolean>
{
    template<typename BasicJsonType>
    static void construct(BasicJsonType& j, typename BasicJsonType::boolean_t b) noexcept
    {
        j.m_type = value_t::boolean;
        j.m_value = b;
        j.assert_invariant();
    }
};

template<>
struct external_constructor<value_t::string>
{
    template<typename BasicJsonType>
    static void construct(BasicJsonType& j, const typename BasicJsonType::string_t& s)
    {
        j.m_type = value_t::string;
        j.m_value = s;
        j.assert_invariant();
    }
};

template<>
struct external_constructor<value_t::number_float>
{
    template<typename BasicJsonType>
    static void construct(BasicJsonType& j, typename BasicJsonType::number_float_t val) noexcept
    {
        // replace infinity and NAN by null
        if (not std::isfinite(val))
        {
            j = BasicJsonType{};
        }
        else
        {
            j.m_type = value_t::number_float;
            j.m_value = val;
        }
        j.assert_invariant();
    }
};

template<>
struct external_constructor<value_t::number_unsigned>
{
    template<typename BasicJsonType>
    static void construct(BasicJsonType& j, typename BasicJsonType::number_unsigned_t val) noexcept
    {
        j.m_type = value_t::number_unsigned;
        j.m_value = val;
        j.assert_invariant();
    }
};

template<>
struct external_constructor<value_t::number_integer>
{
    template<typename BasicJsonType>
    static void construct(BasicJsonType& j, typename BasicJsonType::number_integer_t val) noexcept
    {
        j.m_type = value_t::number_integer;
        j.m_value = val;
        j.assert_invariant();
    }
};

template<>
struct external_constructor<value_t::array>
{
    template<typename BasicJsonType>
    static void construct(BasicJsonType& j, const typename BasicJsonType::array_t& arr)
    {
        j.m_type = value_t::array;
        j.m_value = arr;
        j.assert_invariant();
    }

    template<typename BasicJsonType, typename CompatibleArrayType,
             enable_if_t<not std::is_same<CompatibleArrayType,
                                          typename BasicJsonType::array_t>::value,
                         int> = 0>
    static void construct(BasicJsonType& j, const CompatibleArrayType& arr)
    {
        using std::begin;
        using std::end;
        j.m_type = value_t::array;
        j.m_value.array = j.template create<typename BasicJsonType::array_t>(begin(arr), end(arr));
        j.assert_invariant();
    }
};

template<>
struct external_constructor<value_t::object>
{
    template<typename BasicJsonType>
    static void construct(BasicJsonType& j, const typename BasicJsonType::object_t& obj)
    {
        j.m_type = value_t::object;
        j.m_value = obj;
        j.assert_invariant();
    }

    template<typename BasicJsonType, typename CompatibleObjectType,
             enable_if_t<not std::is_same<CompatibleObjectType,
                                          typename BasicJsonType::object_t>::value,
                         int> = 0>
    static void construct(BasicJsonType& j, const CompatibleObjectType& obj)
    {
        using std::begin;
        using std::end;

        j.m_type = value_t::object;
        j.m_value.object = j.template create<typename BasicJsonType::object_t>(begin(obj), end(obj));
        j.assert_invariant();
    }
};


////////////////////////
// has_/is_ functions //
////////////////////////

/*!
@brief Helper to determine whether there's a key_type for T.

This helper is used to tell associative containers apart from other containers
such as sequence containers. For instance, `std::map` passes the test as it
contains a `mapped_type`, whereas `std::vector` fails the test.

@sa http://stackoverflow.com/a/7728728/266378
@since version 1.0.0, overworked in version 2.0.6
*/
#define NLOHMANN_JSON_HAS_HELPER(type)                                        \
    template<typename T> struct has_##type {                                  \
    private:                                                                  \
        template<typename U, typename = typename U::type>                     \
        static int detect(U &&);                                              \
        static void detect(...);                                              \
    public:                                                                   \
        static constexpr bool value =                                         \
                std::is_integral<decltype(detect(std::declval<T>()))>::value; \
    }

NLOHMANN_JSON_HAS_HELPER(mapped_type);
NLOHMANN_JSON_HAS_HELPER(key_type);
NLOHMANN_JSON_HAS_HELPER(value_type);
NLOHMANN_JSON_HAS_HELPER(iterator);

#undef NLOHMANN_JSON_HAS_HELPER


template<bool B, class RealType, class CompatibleObjectType>
struct is_compatible_object_type_impl : std::false_type {};

template<class RealType, class CompatibleObjectType>
struct is_compatible_object_type_impl<true, RealType, CompatibleObjectType>
{
    static constexpr auto value =
        std::is_constructible<typename RealType::key_type,
        typename CompatibleObjectType::key_type>::value and
        std::is_constructible<typename RealType::mapped_type,
        typename CompatibleObjectType::mapped_type>::value;
};

template<class BasicJsonType, class CompatibleObjectType>
struct is_compatible_object_type
{
    static auto constexpr value = is_compatible_object_type_impl <
                                  conjunction<negation<std::is_same<void, CompatibleObjectType>>,
                                  has_mapped_type<CompatibleObjectType>,
                                  has_key_type<CompatibleObjectType>>::value,
                                  typename BasicJsonType::object_t, CompatibleObjectType >::value;
};

template<typename BasicJsonType, typename T>
struct is_basic_json_nested_type
{
    static auto constexpr value = std::is_same<T, typename BasicJsonType::iterator>::value or
                                  std::is_same<T, typename BasicJsonType::const_iterator>::value or
                                  std::is_same<T, typename BasicJsonType::reverse_iterator>::value or
                                  std::is_same<T, typename BasicJsonType::const_reverse_iterator>::value or
                                  std::is_same<T, typename BasicJsonType::json_pointer>::value;
};

template<class BasicJsonType, class CompatibleArrayType>
struct is_compatible_array_type
{
    static auto constexpr value =
        conjunction<negation<std::is_same<void, CompatibleArrayType>>,
        negation<is_compatible_object_type<
        BasicJsonType, CompatibleArrayType>>,
        negation<std::is_constructible<typename BasicJsonType::string_t,
        CompatibleArrayType>>,
        negation<is_basic_json_nested_type<BasicJsonType, CompatibleArrayType>>,
        has_value_type<CompatibleArrayType>,
        has_iterator<CompatibleArrayType>>::value;
};

template<bool, typename, typename>
struct is_compatible_integer_type_impl : std::false_type {};

template<typename RealIntegerType, typename CompatibleNumberIntegerType>
struct is_compatible_integer_type_impl<true, RealIntegerType, CompatibleNumberIntegerType>
{
    // is there an assert somewhere on overflows?
    using RealLimits = std::numeric_limits<RealIntegerType>;
    using CompatibleLimits = std::numeric_limits<CompatibleNumberIntegerType>;

    static constexpr auto value =
        std::is_constructible<RealIntegerType,
        CompatibleNumberIntegerType>::value and
        CompatibleLimits::is_integer and
        RealLimits::is_signed == CompatibleLimits::is_signed;
};

template<typename RealIntegerType, typename CompatibleNumberIntegerType>
struct is_compatible_integer_type
{
    static constexpr auto value =
        is_compatible_integer_type_impl <
        std::is_integral<CompatibleNumberIntegerType>::value and
        not std::is_same<bool, CompatibleNumberIntegerType>::value,
        RealIntegerType, CompatibleNumberIntegerType > ::value;
};


// trait checking if JSONSerializer<T>::from_json(json const&, udt&) exists
template<typename BasicJsonType, typename T>
struct has_from_json
{
  private:
    // also check the return type of from_json
    template<typename U, typename = enable_if_t<std::is_same<void, decltype(uncvref_t<U>::from_json(
                 std::declval<BasicJsonType>(), std::declval<T&>()))>::value>>
    static int detect(U&&);
    static void detect(...);

  public:
    static constexpr bool value = std::is_integral<decltype(
                                      detect(std::declval<typename BasicJsonType::template json_serializer<T, void>>()))>::value;
};

// This trait checks if JSONSerializer<T>::from_json(json const&) exists
// this overload is used for non-default-constructible user-defined-types
template<typename BasicJsonType, typename T>
struct has_non_default_from_json
{
  private:
    template <
        typename U,
        typename = enable_if_t<std::is_same<
                                   T, decltype(uncvref_t<U>::from_json(std::declval<BasicJsonType>()))>::value >>
    static int detect(U&&);
    static void detect(...);

  public:
    static constexpr bool value = std::is_integral<decltype(detect(
                                      std::declval<typename BasicJsonType::template json_serializer<T, void>>()))>::value;
};

// This trait checks if BasicJsonType::json_serializer<T>::to_json exists
template<typename BasicJsonType, typename T>
struct has_to_json
{
  private:
    template<typename U, typename = decltype(uncvref_t<U>::to_json(
                 std::declval<BasicJsonType&>(), std::declval<T>()))>
    static int detect(U&&);
    static void detect(...);

  public:
    static constexpr bool value = std::is_integral<decltype(detect(
                                      std::declval<typename BasicJsonType::template json_serializer<T, void>>()))>::value;
};


/////////////
// to_json //
/////////////

template<typename BasicJsonType, typename T, enable_if_t<
             std::is_same<T, typename BasicJsonType::boolean_t>::value, int> = 0>
void to_json(BasicJsonType& j, T b) noexcept
{
    external_constructor<value_t::boolean>::construct(j, b);
}

template<typename BasicJsonType, typename CompatibleString,
         enable_if_t<std::is_constructible<typename BasicJsonType::string_t,
                     CompatibleString>::value, int> = 0>
void to_json(BasicJsonType& j, const CompatibleString& s)
{
    external_constructor<value_t::string>::construct(j, s);
}

template<typename BasicJsonType, typename FloatType,
         enable_if_t<std::is_floating_point<FloatType>::value, int> = 0>
void to_json(BasicJsonType& j, FloatType val) noexcept
{
    external_constructor<value_t::number_float>::construct(j, static_cast<typename BasicJsonType::number_float_t>(val));
}

template <
    typename BasicJsonType, typename CompatibleNumberUnsignedType,
    enable_if_t<is_compatible_integer_type<typename BasicJsonType::number_unsigned_t,
                CompatibleNumberUnsignedType>::value, int> = 0 >
void to_json(BasicJsonType& j, CompatibleNumberUnsignedType val) noexcept
{
    external_constructor<value_t::number_unsigned>::construct(j, static_cast<typename BasicJsonType::number_unsigned_t>(val));
}

template <
    typename BasicJsonType, typename CompatibleNumberIntegerType,
    enable_if_t<is_compatible_integer_type<typename BasicJsonType::number_integer_t,
                CompatibleNumberIntegerType>::value, int> = 0 >
void to_json(BasicJsonType& j, CompatibleNumberIntegerType val) noexcept
{
    external_constructor<value_t::number_integer>::construct(j, static_cast<typename BasicJsonType::number_integer_t>(val));
}

template<typename BasicJsonType, typename UnscopedEnumType,
         enable_if_t<is_unscoped_enum<UnscopedEnumType>::value, int> = 0>
void to_json(BasicJsonType& j, UnscopedEnumType e) noexcept
{
    external_constructor<value_t::number_integer>::construct(j, e);
}

template <
    typename BasicJsonType, typename CompatibleArrayType,
    enable_if_t <
        is_compatible_array_type<BasicJsonType, CompatibleArrayType>::value or
        std::is_same<typename BasicJsonType::array_t, CompatibleArrayType>::value,
        int > = 0 >
void to_json(BasicJsonType& j, const  CompatibleArrayType& arr)
{
    external_constructor<value_t::array>::construct(j, arr);
}

template <
    typename BasicJsonType, typename CompatibleObjectType,
    enable_if_t<is_compatible_object_type<BasicJsonType, CompatibleObjectType>::value,
                int> = 0 >
void to_json(BasicJsonType& j, const  CompatibleObjectType& arr)
{
    external_constructor<value_t::object>::construct(j, arr);
}


///////////////
// from_json //
///////////////

// overloads for basic_json template parameters
template<typename BasicJsonType, typename ArithmeticType,
         enable_if_t<std::is_arithmetic<ArithmeticType>::value and
                     not std::is_same<ArithmeticType,
                                      typename BasicJsonType::boolean_t>::value,
                     int> = 0>
void get_arithmetic_value(const BasicJsonType& j, ArithmeticType& val)
{
    switch (static_cast<value_t>(j))
    {
        case value_t::number_unsigned:
        {
            val = static_cast<ArithmeticType>(
                      *j.template get_ptr<const typename BasicJsonType::number_unsigned_t*>());
            break;
        }
        case value_t::number_integer:
        {
            val = static_cast<ArithmeticType>(
                      *j.template get_ptr<const typename BasicJsonType::number_integer_t*>());
            break;
        }
        case value_t::number_float:
        {
            val = static_cast<ArithmeticType>(
                      *j.template get_ptr<const typename BasicJsonType::number_float_t*>());
            break;
        }
        default:
        {
            JSON_THROW(
                std::domain_error("type must be number, but is " + j.type_name()));
        }
    }
}

template<typename BasicJsonType>
void from_json(const BasicJsonType& j, typename BasicJsonType::boolean_t& b)
{
    if (not j.is_boolean())
    {
        JSON_THROW(std::domain_error("type must be boolean, but is " + j.type_name()));
    }
    b = *j.template get_ptr<const typename BasicJsonType::boolean_t*>();
}

template<typename BasicJsonType>
void from_json(const BasicJsonType& j, typename BasicJsonType::string_t& s)
{
    if (not j.is_string())
    {
        JSON_THROW(std::domain_error("type must be string, but is " + j.type_name()));
    }
    s = *j.template get_ptr<const typename BasicJsonType::string_t*>();
}

template<typename BasicJsonType>
void from_json(const BasicJsonType& j, typename BasicJsonType::number_float_t& val)
{
    get_arithmetic_value(j, val);
}

template<typename BasicJsonType>
void from_json(const BasicJsonType& j, typename BasicJsonType::number_unsigned_t& val)
{
    get_arithmetic_value(j, val);
}

template<typename BasicJsonType>
void from_json(const BasicJsonType& j, typename BasicJsonType::number_integer_t& val)
{
    get_arithmetic_value(j, val);
}

template<typename BasicJsonType, typename UnscopedEnumType,
         enable_if_t<is_unscoped_enum<UnscopedEnumType>::value, int> = 0>
void from_json(const BasicJsonType& j, UnscopedEnumType& e)
{
    typename std::underlying_type<UnscopedEnumType>::type val;
    get_arithmetic_value(j, val);
    e = static_cast<UnscopedEnumType>(val);
}

template<typename BasicJsonType>
void from_json(const BasicJsonType& j, typename BasicJsonType::array_t& arr)
{
    if (not j.is_array())
    {
        JSON_THROW(std::domain_error("type must be array, but is " + j.type_name()));
    }
    arr = *j.template get_ptr<const typename BasicJsonType::array_t*>();
}

// forward_list doesn't have an insert method
template<typename BasicJsonType, typename T, typename Allocator>
void from_json(const BasicJsonType& j, std::forward_list<T, Allocator>& l)
{
    // do not perform the check when user wants to retrieve jsons
    // (except when it's null.. ?)
    if (j.is_null())
    {
        JSON_THROW(std::domain_error("type must be array, but is " + j.type_name()));
    }
    if (not std::is_same<T, BasicJsonType>::value)
    {
        if (not j.is_array())
        {
            JSON_THROW(std::domain_error("type must be array, but is " + j.type_name()));
        }
    }
    for (auto it = j.rbegin(), end = j.rend(); it != end; ++it)
    {
        l.push_front(it->template get<T>());
    }
}

template<typename BasicJsonType, typename CompatibleArrayType>
void from_json_array_impl(const BasicJsonType& j, CompatibleArrayType& arr, priority_tag<0>)
{
    using std::begin;
    using std::end;

    std::transform(j.begin(), j.end(),
                   std::inserter(arr, end(arr)), [](const BasicJsonType & i)
    {
        // get<BasicJsonType>() returns *this, this won't call a from_json
        // method when value_type is BasicJsonType
        return i.template get<typename CompatibleArrayType::value_type>();
    });
}

template<typename BasicJsonType, typename CompatibleArrayType>
auto from_json_array_impl(const BasicJsonType& j, CompatibleArrayType& arr, priority_tag<1>)
-> decltype(
    arr.reserve(std::declval<typename CompatibleArrayType::size_type>()),
    void())
{
    using std::begin;
    using std::end;

    arr.reserve(j.size());
    std::transform(
        j.begin(), j.end(), std::inserter(arr, end(arr)), [](const BasicJsonType & i)
    {
        // get<BasicJsonType>() returns *this, this won't call a from_json
        // method when value_type is BasicJsonType
        return i.template get<typename CompatibleArrayType::value_type>();
    });
}

template<typename BasicJsonType, typename CompatibleArrayType,
         enable_if_t<is_compatible_array_type<BasicJsonType, CompatibleArrayType>::value and
                     not std::is_same<typename BasicJsonType::array_t, CompatibleArrayType>::value, int> = 0>
void from_json(const BasicJsonType& j, CompatibleArrayType& arr)
{
    if (j.is_null())
    {
        JSON_THROW(std::domain_error("type must be array, but is " + j.type_name()));
    }

    // when T == BasicJsonType, do not check if value_t is correct
    if (not std::is_same<typename CompatibleArrayType::value_type, BasicJsonType>::value)
    {
        if (not j.is_array())
        {
            JSON_THROW(std::domain_error("type must be array, but is " + j.type_name()));
        }
    }
    from_json_array_impl(j, arr, priority_tag<1> {});
}

template<typename BasicJsonType, typename CompatibleObjectType,
         enable_if_t<is_compatible_object_type<BasicJsonType, CompatibleObjectType>::value, int> = 0>
void from_json(const BasicJsonType& j, CompatibleObjectType& obj)
{
    if (not j.is_object())
    {
        JSON_THROW(std::domain_error("type must be object, but is " + j.type_name()));
    }

    auto inner_object = j.template get_ptr<const typename BasicJsonType::object_t*>();
    using std::begin;
    using std::end;
    // we could avoid the assignment, but this might require a for loop, which
    // might be less efficient than the container constructor for some
    // containers (would it?)
    obj = CompatibleObjectType(begin(*inner_object), end(*inner_object));
}

// overload for arithmetic types, not chosen for basic_json template arguments
// (BooleanType, etc..); note: Is it really necessary to provide explicit
// overloads for boolean_t etc. in case of a custom BooleanType which is not
// an arithmetic type?
template<typename BasicJsonType, typename ArithmeticType,
         enable_if_t <
             std::is_arithmetic<ArithmeticType>::value and
             not std::is_same<ArithmeticType, typename BasicJsonType::number_unsigned_t>::value and
             not std::is_same<ArithmeticType, typename BasicJsonType::number_integer_t>::value and
             not std::is_same<ArithmeticType, typename BasicJsonType::number_float_t>::value and
             not std::is_same<ArithmeticType, typename BasicJsonType::boolean_t>::value,
             int> = 0>
void from_json(const BasicJsonType& j, ArithmeticType& val)
{
    switch (static_cast<value_t>(j))
    {
        case value_t::number_unsigned:
        {
            val = static_cast<ArithmeticType>(*j.template get_ptr<const typename BasicJsonType::number_unsigned_t*>());
            break;
        }
        case value_t::number_integer:
        {
            val = static_cast<ArithmeticType>(*j.template get_ptr<const typename BasicJsonType::number_integer_t*>());
            break;
        }
        case value_t::number_float:
        {
            val = static_cast<ArithmeticType>(*j.template get_ptr<const typename BasicJsonType::number_float_t*>());
            break;
        }
        case value_t::boolean:
        {
            val = static_cast<ArithmeticType>(*j.template get_ptr<const typename BasicJsonType::boolean_t*>());
            break;
        }
        default:
        {
            JSON_THROW(std::domain_error("type must be number, but is " + j.type_name()));
        }
    }
}

struct to_json_fn
{
  private:
    template<typename BasicJsonType, typename T>
    auto call(BasicJsonType& j, T&& val, priority_tag<1>) const noexcept(noexcept(to_json(j, std::forward<T>(val))))
    -> decltype(to_json(j, std::forward<T>(val)), void())
    {
        return to_json(j, std::forward<T>(val));
    }

    template<typename BasicJsonType, typename T>
    void call(BasicJsonType&, T&&, priority_tag<0>) const noexcept
    {
        static_assert(sizeof(BasicJsonType) == 0,
                      "could not find to_json() method in T's namespace");
    }

  public:
    template<typename BasicJsonType, typename T>
    void operator()(BasicJsonType& j, T&& val) const
    noexcept(noexcept(std::declval<to_json_fn>().call(j, std::forward<T>(val), priority_tag<1> {})))
    {
        return call(j, std::forward<T>(val), priority_tag<1> {});
    }
};

struct from_json_fn
{
  private:
    template<typename BasicJsonType, typename T>
    auto call(const BasicJsonType& j, T& val, priority_tag<1>) const
    noexcept(noexcept(from_json(j, val)))
    -> decltype(from_json(j, val), void())
    {
        return from_json(j, val);
    }

    template<typename BasicJsonType, typename T>
    void call(const BasicJsonType&, T&, priority_tag<0>) const noexcept
    {
        static_assert(sizeof(BasicJsonType) == 0,
                      "could not find from_json() method in T's namespace");
    }

  public:
    template<typename BasicJsonType, typename T>
    void operator()(const BasicJsonType& j, T& val) const
    noexcept(noexcept(std::declval<from_json_fn>().call(j, val, priority_tag<1> {})))
    {
        return call(j, val, priority_tag<1> {});
    }
};

// taken from ranges-v3
template<typename T>
struct static_const
{
    static constexpr T value{};
};

template<typename T>
constexpr T static_const<T>::value;
} // namespace detail


/// namespace to hold default `to_json` / `from_json` functions
namespace
{
constexpr const auto& to_json = detail::static_const<detail::to_json_fn>::value;
constexpr const auto& from_json = detail::static_const<detail::from_json_fn>::value;
}


/*!
@brief default JSONSerializer template argument

This serializer ignores the template arguments and uses ADL
([argument-dependent lookup](http://en.cppreference.com/w/cpp/language/adl))
for serialization.
*/
template<typename = void, typename = void>
struct adl_serializer
{
    /*!
    @brief convert a JSON value to any value type

    This function is usually called by the `get()` function of the
    @ref basic_json class (either explicit or via conversion operators).

    @param[in] j         JSON value to read from
    @param[in,out] val  value to write to
    */
    template<typename BasicJsonType, typename ValueType>
    static void from_json(BasicJsonType&& j, ValueType& val) noexcept(
        noexcept(::nlohmann::from_json(std::forward<BasicJsonType>(j), val)))
    {
        ::nlohmann::from_json(std::forward<BasicJsonType>(j), val);
    }

    /*!
    @brief convert any value type to a JSON value

    This function is usually called by the constructors of the @ref basic_json
    class.

    @param[in,out] j  JSON value to write to
    @param[in] val     value to read from
    */
    template<typename BasicJsonType, typename ValueType>
    static void to_json(BasicJsonType& j, ValueType&& val) noexcept(
        noexcept(::nlohmann::to_json(j, std::forward<ValueType>(val))))
    {
        ::nlohmann::to_json(j, std::forward<ValueType>(val));
    }
};


/*!
@brief a class to store JSON values

@tparam ObjectType type for JSON objects (`std::map` by default; will be used
in @ref object_t)
@tparam ArrayType type for JSON arrays (`std::vector` by default; will be used
in @ref array_t)
@tparam StringType type for JSON strings and object keys (`std::string` by
default; will be used in @ref string_t)
@tparam BooleanType type for JSON booleans (`bool` by default; will be used
in @ref boolean_t)
@tparam NumberIntegerType type for JSON integer numbers (`int64_t` by
default; will be used in @ref number_integer_t)
@tparam NumberUnsignedType type for JSON unsigned integer numbers (@c
`uint64_t` by default; will be used in @ref number_unsigned_t)
@tparam NumberFloatType type for JSON floating-point numbers (`double` by
default; will be used in @ref number_float_t)
@tparam AllocatorType type of the allocator to use (`std::allocator` by
default)
@tparam JSONSerializer the serializer to resolve internal calls to `to_json()`
and `from_json()` (@ref adl_serializer by default)

@requirement The class satisfies the following concept requirements:
- Basic
 - [DefaultConstructible](http://en.cppreference.com/w/cpp/concept/DefaultConstructible):
   JSON values can be default constructed. The result will be a JSON null
   value.
 - [MoveConstructible](http://en.cppreference.com/w/cpp/concept/MoveConstructible):
   A JSON value can be constructed from an rvalue argument.
 - [CopyConstructible](http://en.cppreference.com/w/cpp/concept/CopyConstructible):
   A JSON value can be copy-constructed from an lvalue expression.
 - [MoveAssignable](http://en.cppreference.com/w/cpp/concept/MoveAssignable):
   A JSON value van be assigned from an rvalue argument.
 - [CopyAssignable](http://en.cppreference.com/w/cpp/concept/CopyAssignable):
   A JSON value can be copy-assigned from an lvalue expression.
 - [Destructible](http://en.cppreference.com/w/cpp/concept/Destructible):
   JSON values can be destructed.
- Layout
 - [StandardLayoutType](http://en.cppreference.com/w/cpp/concept/StandardLayoutType):
   JSON values have
   [standard layout](http://en.cppreference.com/w/cpp/language/data_members#Standard_layout):
   All non-static data members are private and standard layout types, the
   class has no virtual functions or (virtual) base classes.
- Library-wide
 - [EqualityComparable](http://en.cppreference.com/w/cpp/concept/EqualityComparable):
   JSON values can be compared with `==`, see @ref
   operator==(const_reference,const_reference).
 - [LessThanComparable](http://en.cppreference.com/w/cpp/concept/LessThanComparable):
   JSON values can be compared with `<`, see @ref
   operator<(const_reference,const_reference).
 - [Swappable](http://en.cppreference.com/w/cpp/concept/Swappable):
   Any JSON lvalue or rvalue of can be swapped with any lvalue or rvalue of
   other compatible types, using unqualified function call @ref swap().
 - [NullablePointer](http://en.cppreference.com/w/cpp/concept/NullablePointer):
   JSON values can be compared against `std::nullptr_t` objects which are used
   to model the `null` value.
- Container
 - [Container](http://en.cppreference.com/w/cpp/concept/Container):
   JSON values can be used like STL containers and provide iterator access.
 - [ReversibleContainer](http://en.cppreference.com/w/cpp/concept/ReversibleContainer);
   JSON values can be used like STL containers and provide reverse iterator
   access.

@invariant The member variables @a m_value and @a m_type have the following
relationship:
- If `m_type == value_t::object`, then `m_value.object != nullptr`.
- If `m_type == value_t::array`, then `m_value.array != nullptr`.
- If `m_type == value_t::string`, then `m_value.string != nullptr`.
The invariants are checked by member function assert_invariant().

@internal
@note ObjectType trick from http://stackoverflow.com/a/9860911
@endinternal

@see [RFC 7159: The JavaScript Object Notation (JSON) Data Interchange
Format](http://rfc7159.net/rfc7159)

@since version 1.0.0

@nosubgrouping
*/
template <
    template<typename U, typename V, typename... Args> class ObjectType = std::map,
    template<typename U, typename... Args> class ArrayType = std::vector,
    class StringType = std::string,
    class BooleanType = bool,
    class NumberIntegerType = std::int64_t,
    class NumberUnsignedType = std::uint64_t,
    class NumberFloatType = double,
    template<typename U> class AllocatorType = std::allocator,
    template<typename T, typename SFINAE = void> class JSONSerializer = adl_serializer
    >
class basic_json
{
  private:
    template<detail::value_t> friend struct detail::external_constructor;
    /// workaround type for MSVC
    using basic_json_t = basic_json<ObjectType, ArrayType, StringType,
          BooleanType, NumberIntegerType, NumberUnsignedType, NumberFloatType,
          AllocatorType, JSONSerializer>;

  public:
    using value_t = detail::value_t;
    // forward declarations
    template<typename U> class iter_impl;
    template<typename Base> class json_reverse_iterator;
    class json_pointer;
    template<typename T, typename SFINAE>
    using json_serializer = JSONSerializer<T, SFINAE>;

    /////////////////////
    // container types //
    /////////////////////

    /// @name container types
    /// The canonic container types to use @ref basic_json like any other STL
    /// container.
    /// @{

    /// the type of elements in a basic_json container
    using value_type = basic_json;

    /// the type of an element reference
    using reference = value_type&;
    /// the type of an element const reference
    using const_reference = const value_type&;

    /// a type to represent differences between iterators
    using difference_type = std::ptrdiff_t;
    /// a type to represent container sizes
    using size_type = std::size_t;

    /// the allocator type
    using allocator_type = AllocatorType<basic_json>;

    /// the type of an element pointer
    using pointer = typename std::allocator_traits<allocator_type>::pointer;
    /// the type of an element const pointer
    using const_pointer = typename std::allocator_traits<allocator_type>::const_pointer;

    /// an iterator for a basic_json container
    using iterator = iter_impl<basic_json>;
    /// a const iterator for a basic_json container
    using const_iterator = iter_impl<const basic_json>;
    /// a reverse iterator for a basic_json container
    using reverse_iterator = json_reverse_iterator<typename basic_json::iterator>;
    /// a const reverse iterator for a basic_json container
    using const_reverse_iterator = json_reverse_iterator<typename basic_json::const_iterator>;

    /// @}


    /*!
    @brief returns the allocator associated with the container
    */
    static allocator_type get_allocator()
    {
        return allocator_type();
    }

    /*!
    @brief returns version information on the library

    This function returns a JSON object with information about the library,
    including the version number and information on the platform and compiler.

    @return JSON object holding version information
    key         | description
    ----------- | ---------------
    `compiler`  | Information on the used compiler. It is an object with the following keys: `c++` (the used C++ standard), `family` (the compiler family; possible values are `clang`, `icc`, `gcc`, `ilecpp`, `msvc`, `pgcpp`, `sunpro`, and `unknown`), and `version` (the compiler version).
    `copyright` | The copyright line for the library as string.
    `name`      | The name of the library as string.
    `platform`  | The used platform as string. Possible values are `win32`, `linux`, `apple`, `unix`, and `unknown`.
    `url`       | The URL of the project as string.
    `version`   | The version of the library. It is an object with the following keys: `major`, `minor`, and `patch` as defined by [Semantic Versioning](http://semver.org), and `string` (the version string).

    @liveexample{The following code shows an example output of the `meta()`
    function.,meta}

    @complexity Constant.

    @since 2.1.0
    */
    static basic_json meta()
    {
        basic_json result;

        result["copyright"] = "(C) 2013-2017 Niels Lohmann";
        result["name"] = "JSON for Modern C++";
        result["url"] = "https://github.com/nlohmann/json";
        result["version"] =
        {
            {"string", "2.1.1"},
            {"major", 2},
            {"minor", 1},
            {"patch", 1}
        };

#ifdef _WIN32
        result["platform"] = "win32";
#elif defined __linux__
        result["platform"] = "linux";
#elif defined __APPLE__
        result["platform"] = "apple";
#elif defined __unix__
        result["platform"] = "unix";
#else
        result["platform"] = "unknown";
#endif

#if defined(__clang__)
        result["compiler"] = {{"family", "clang"}, {"version", __clang_version__}};
#elif defined(__ICC) || defined(__INTEL_COMPILER)
        result["compiler"] = {{"family", "icc"}, {"version", __INTEL_COMPILER}};
#elif defined(__GNUC__) || defined(__GNUG__)
        result["compiler"] = {{"family", "gcc"}, {"version", std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__) + "." + std::to_string(__GNUC_PATCHLEVEL__)}};
#elif defined(__HP_cc) || defined(__HP_aCC)
        result["compiler"] = "hp"
#elif defined(__IBMCPP__)
        result["compiler"] = {{"family", "ilecpp"}, {"version", __IBMCPP__}};
#elif defined(_MSC_VER)
        result["compiler"] = {{"family", "msvc"}, {"version", _MSC_VER}};
#elif defined(__PGI)
        result["compiler"] = {{"family", "pgcpp"}, {"version", __PGI}};
#elif defined(__SUNPRO_CC)
        result["compiler"] = {{"family", "sunpro"}, {"version", __SUNPRO_CC}};
#else
        result["compiler"] = {{"family", "unknown"}, {"version", "unknown"}};
#endif

#ifdef __cplusplus
        result["compiler"]["c++"] = std::to_string(__cplusplus);
#else
        result["compiler"]["c++"] = "unknown";
#endif
        return result;
    }


    ///////////////////////////
    // JSON value data types //
    ///////////////////////////

    /// @name JSON value data types
    /// The data types to store a JSON value. These types are derived from
    /// the template arguments passed to class @ref basic_json.
    /// @{

    /*!
    @brief a type for an object

    [RFC 7159](http://rfc7159.net/rfc7159) describes JSON objects as follows:
    > An object is an unordered collection of zero or more name/value pairs,
    > where a name is a string and a value is a string, number, boolean, null,
    > object, or array.

    To store objects in C++, a type is defined by the template parameters
    described below.

    @tparam ObjectType  the container to store objects (e.g., `std::map` or
    `std::unordered_map`)
    @tparam StringType the type of the keys or names (e.g., `std::string`).
    The comparison function `std::less<StringType>` is used to order elements
    inside the container.
    @tparam AllocatorType the allocator to use for objects (e.g.,
    `std::allocator`)

    #### Default type

    With the default values for @a ObjectType (`std::map`), @a StringType
    (`std::string`), and @a AllocatorType (`std::allocator`), the default
    value for @a object_t is:

    @code {.cpp}
    std::map<
      std::string, // key_type
      basic_json, // value_type
      std::less<std::string>, // key_compare
      std::allocator<std::pair<const std::string, basic_json>> // allocator_type
    >
    @endcode

    #### Behavior

    The choice of @a object_t influences the behavior of the JSON class. With
    the default type, objects have the following behavior:

    - When all names are unique, objects will be interoperable in the sense
      that all software implementations receiving that object will agree on
      the name-value mappings.
    - When the names within an object are not unique, later stored name/value
      pairs overwrite previously stored name/value pairs, leaving the used
      names unique. For instance, `{"key": 1}` and `{"key": 2, "key": 1}` will
      be treated as equal and both stored as `{"key": 1}`.
    - Internally, name/value pairs are stored in lexicographical order of the
      names. Objects will also be serialized (see @ref dump) in this order.
      For instance, `{"b": 1, "a": 2}` and `{"a": 2, "b": 1}` will be stored
      and serialized as `{"a": 2, "b": 1}`.
    - When comparing objects, the order of the name/value pairs is irrelevant.
      This makes objects interoperable in the sense that they will not be
      affected by these differences. For instance, `{"b": 1, "a": 2}` and
      `{"a": 2, "b": 1}` will be treated as equal.

    #### Limits

    [RFC 7159](http://rfc7159.net/rfc7159) specifies:
    > An implementation may set limits on the maximum depth of nesting.

    In this class, the object's limit of nesting is not constraint explicitly.
    However, a maximum depth of nesting may be introduced by the compiler or
    runtime environment. A theoretical limit can be queried by calling the
    @ref max_size function of a JSON object.

    #### Storage

    Objects are stored as pointers in a @ref basic_json type. That is, for any
    access to object values, a pointer of type `object_t*` must be
    dereferenced.

    @sa @ref array_t -- type for an array value

    @since version 1.0.0

    @note The order name/value pairs are added to the object is *not*
    preserved by the library. Therefore, iterating an object may return
    name/value pairs in a different order than they were originally stored. In
    fact, keys will be traversed in alphabetical order as `std::map` with
    `std::less` is used by default. Please note this behavior conforms to [RFC
    7159](http://rfc7159.net/rfc7159), because any order implements the
    specified "unordered" nature of JSON objects.
    */
    using object_t = ObjectType<StringType,
          basic_json,
          std::less<StringType>,
          AllocatorType<std::pair<const StringType,
          basic_json>>>;

    /*!
    @brief a type for an array

    [RFC 7159](http://rfc7159.net/rfc7159) describes JSON arrays as follows:
    > An array is an ordered sequence of zero or more values.

    To store objects in C++, a type is defined by the template parameters
    explained below.

    @tparam ArrayType  container type to store arrays (e.g., `std::vector` or
    `std::list`)
    @tparam AllocatorType allocator to use for arrays (e.g., `std::allocator`)

    #### Default type

    With the default values for @a ArrayType (`std::vector`) and @a
    AllocatorType (`std::allocator`), the default value for @a array_t is:

    @code {.cpp}
    std::vector<
      basic_json, // value_type
      std::allocator<basic_json> // allocator_type
    >
    @endcode

    #### Limits

    [RFC 7159](http://rfc7159.net/rfc7159) specifies:
    > An implementation may set limits on the maximum depth of nesting.

    In this class, the array's limit of nesting is not constraint explicitly.
    However, a maximum depth of nesting may be introduced by the compiler or
    runtime environment. A theoretical limit can be queried by calling the
    @ref max_size function of a JSON array.

    #### Storage

    Arrays are stored as pointers in a @ref basic_json type. That is, for any
    access to array values, a pointer of type `array_t*` must be dereferenced.

    @sa @ref object_t -- type for an object value

    @since version 1.0.0
    */
    using array_t = ArrayType<basic_json, AllocatorType<basic_json>>;

    /*!
    @brief a type for a string

    [RFC 7159](http://rfc7159.net/rfc7159) describes JSON strings as follows:
    > A string is a sequence of zero or more Unicode characters.

    To store objects in C++, a type is defined by the template parameter
    described below. Unicode values are split by the JSON class into
    byte-sized characters during deserialization.

    @tparam StringType  the container to store strings (e.g., `std::string`).
    Note this container is used for keys/names in objects, see @ref object_t.

    #### Default type

    With the default values for @a StringType (`std::string`), the default
    value for @a string_t is:

    @code {.cpp}
    std::string
    @endcode

    #### Encoding

    Strings are stored in UTF-8 encoding. Therefore, functions like
    `std::string::size()` or `std::string::length()` return the number of
    bytes in the string rather than the number of characters or glyphs.

    #### String comparison

    [RFC 7159](http://rfc7159.net/rfc7159) states:
    > Software implementations are typically required to test names of object
    > members for equality. Implementations that transform the textual
    > representation into sequences of Unicode code units and then perform the
    > comparison numerically, code unit by code unit, are interoperable in the
    > sense that implementations will agree in all cases on equality or
    > inequality of two strings. For example, implementations that compare
    > strings with escaped characters unconverted may incorrectly find that
    > `"a\\b"` and `"a\u005Cb"` are not equal.

    This implementation is interoperable as it does compare strings code unit
    by code unit.

    #### Storage

    String values are stored as pointers in a @ref basic_json type. That is,
    for any access to string values, a pointer of type `string_t*` must be
    dereferenced.

    @since version 1.0.0
    */
    using string_t = StringType;

    /*!
    @brief a type for a boolean

    [RFC 7159](http://rfc7159.net/rfc7159) implicitly describes a boolean as a
    type which differentiates the two literals `true` and `false`.

    To store objects in C++, a type is defined by the template parameter @a
    BooleanType which chooses the type to use.

    #### Default type

    With the default values for @a BooleanType (`bool`), the default value for
    @a boolean_t is:

    @code {.cpp}
    bool
    @endcode

    #### Storage

    Boolean values are stored directly inside a @ref basic_json type.

    @since version 1.0.0
    */
    using boolean_t = BooleanType;

    /*!
    @brief a type for a number (integer)

    [RFC 7159](http://rfc7159.net/rfc7159) describes numbers as follows:
    > The representation of numbers is similar to that used in most
    > programming languages. A number is represented in base 10 using decimal
    > digits. It contains an integer component that may be prefixed with an
    > optional minus sign, which may be followed by a fraction part and/or an
    > exponent part. Leading zeros are not allowed. (...) Numeric values that
    > cannot be represented in the grammar below (such as Infinity and NaN)
    > are not permitted.

    This description includes both integer and floating-point numbers.
    However, C++ allows more precise storage if it is known whether the number
    is a signed integer, an unsigned integer or a floating-point number.
    Therefore, three different types, @ref number_integer_t, @ref
    number_unsigned_t and @ref number_float_t are used.

    To store integer numbers in C++, a type is defined by the template
    parameter @a NumberIntegerType which chooses the type to use.

    #### Default type

    With the default values for @a NumberIntegerType (`int64_t`), the default
    value for @a number_integer_t is:

    @code {.cpp}
    int64_t
    @endcode

    #### Default behavior

    - The restrictions about leading zeros is not enforced in C++. Instead,
      leading zeros in integer literals lead to an interpretation as octal
      number. Internally, the value will be stored as decimal number. For
      instance, the C++ integer literal `010` will be serialized to `8`.
      During deserialization, leading zeros yield an error.
    - Not-a-number (NaN) values will be serialized to `null`.

    #### Limits

    [RFC 7159](http://rfc7159.net/rfc7159) specifies:
    > An implementation may set limits on the range and precision of numbers.

    When the default type is used, the maximal integer number that can be
    stored is `9223372036854775807` (INT64_MAX) and the minimal integer number
    that can be stored is `-9223372036854775808` (INT64_MIN). Integer numbers
    that are out of range will yield over/underflow when used in a
    constructor. During deserialization, too large or small integer numbers
    will be automatically be stored as @ref number_unsigned_t or @ref
    number_float_t.

    [RFC 7159](http://rfc7159.net/rfc7159) further states:
    > Note that when such software is used, numbers that are integers and are
    > in the range \f$[-2^{53}+1, 2^{53}-1]\f$ are interoperable in the sense
    > that implementations will agree exactly on their numeric values.

    As this range is a subrange of the exactly supported range [INT64_MIN,
    INT64_MAX], this class's integer type is interoperable.

    #### Storage

    Integer number values are stored directly inside a @ref basic_json type.

    @sa @ref number_float_t -- type for number values (floating-point)

    @sa @ref number_unsigned_t -- type for number values (unsigned integer)

    @since version 1.0.0
    */
    using number_integer_t = NumberIntegerType;

    /*!
    @brief a type for a number (unsigned)

    [RFC 7159](http://rfc7159.net/rfc7159) describes numbers as follows:
    > The representation of numbers is similar to that used in most
    > programming languages. A number is represented in base 10 using decimal
    > digits. It contains an integer component that may be prefixed with an
    > optional minus sign, which may be followed by a fraction part and/or an
    > exponent part. Leading zeros are not allowed. (...) Numeric values that
    > cannot be represented in the grammar below (such as Infinity and NaN)
    > are not permitted.

    This description includes both integer and floating-point numbers.
    However, C++ allows more precise storage if it is known whether the number
    is a signed integer, an unsigned integer or a floating-point number.
    Therefore, three different types, @ref number_integer_t, @ref
    number_unsigned_t and @ref number_float_t are used.

    To store unsigned integer numbers in C++, a type is defined by the
    template parameter @a NumberUnsignedType which chooses the type to use.

    #### Default type

    With the default values for @a NumberUnsignedType (`uint64_t`), the
    default value for @a number_unsigned_t is:

    @code {.cpp}
    uint64_t
    @endcode

    #### Default behavior

    - The restrictions about leading zeros is not enforced in C++. Instead,
      leading zeros in integer literals lead to an interpretation as octal
      number. Internally, the value will be stored as decimal number. For
      instance, the C++ integer literal `010` will be serialized to `8`.
      During deserialization, leading zeros yield an error.
    - Not-a-number (NaN) values will be serialized to `null`.

    #### Limits

    [RFC 7159](http://rfc7159.net/rfc7159) specifies:
    > An implementation may set limits on the range and precision of numbers.

    When the default type is used, the maximal integer number that can be
    stored is `18446744073709551615` (UINT64_MAX) and the minimal integer
    number that can be stored is `0`. Integer numbers that are out of range
    will yield over/underflow when used in a constructor. During
    deserialization, too large or small integer numbers will be automatically
    be stored as @ref number_integer_t or @ref number_float_t.

    [RFC 7159](http://rfc7159.net/rfc7159) further states:
    > Note that when such software is used, numbers that are integers and are
    > in the range \f$[-2^{53}+1, 2^{53}-1]\f$ are interoperable in the sense
    > that implementations will agree exactly on their numeric values.

    As this range is a subrange (when considered in conjunction with the
    number_integer_t type) of the exactly supported range [0, UINT64_MAX],
    this class's integer type is interoperable.

    #### Storage

    Integer number values are stored directly inside a @ref basic_json type.

    @sa @ref number_float_t -- type for number values (floating-point)
    @sa @ref number_integer_t -- type for number values (integer)

    @since version 2.0.0
    */
    using number_unsigned_t = NumberUnsignedType;

    /*!
    @brief a type for a number (floating-point)

    [RFC 7159](http://rfc7159.net/rfc7159) describes numbers as follows:
    > The representation of numbers is similar to that used in most
    > programming languages. A number is represented in base 10 using decimal
    > digits. It contains an integer component that may be prefixed with an
    > optional minus sign, which may be followed by a fraction part and/or an
    > exponent part. Leading zeros are not allowed. (...) Numeric values that
    > cannot be represented in the grammar below (such as Infinity and NaN)
    > are not permitted.

    This description includes both integer and floating-point numbers.
    However, C++ allows more precise storage if it is known whether the number
    is a signed integer, an unsigned integer or a floating-point number.
    Therefore, three different types, @ref number_integer_t, @ref
    number_unsigned_t and @ref number_float_t are used.

    To store floating-point numbers in C++, a type is defined by the template
    parameter @a NumberFloatType which chooses the type to use.

    #### Default type

    With the default values for @a NumberFloatType (`double`), the default
    value for @a number_float_t is:

    @code {.cpp}
    double
    @endcode

    #### Default behavior

    - The restrictions about leading zeros is not enforced in C++. Instead,
      leading zeros in floating-point literals will be ignored. Internally,
      the value will be stored as decimal number. For instance, the C++
      floating-point literal `01.2` will be serialized to `1.2`. During
      deserialization, leading zeros yield an error.
    - Not-a-number (NaN) values will be serialized to `null`.

    #### Limits

    [RFC 7159](http://rfc7159.net/rfc7159) states:
    > This specification allows implementations to set limits on the range and
    > precision of numbers accepted. Since software that implements IEEE
    > 754-2008 binary64 (double precision) numbers is generally available and
    > widely used, good interoperability can be achieved by implementations
    > that expect no more precision or range than these provide, in the sense
    > that implementations will approximate JSON numbers within the expected
    > precision.

    This implementation does exactly follow this approach, as it uses double
    precision floating-point numbers. Note values smaller than
    `-1.79769313486232e+308` and values greater than `1.79769313486232e+308`
    will be stored as NaN internally and be serialized to `null`.

    #### Storage

    Floating-point number values are stored directly inside a @ref basic_json
    type.

    @sa @ref number_integer_t -- type for number values (integer)

    @sa @ref number_unsigned_t -- type for number values (unsigned integer)

    @since version 1.0.0
    */
    using number_float_t = NumberFloatType;

    /// @}

  private:

    /// helper for exception-safe object creation
    template<typename T, typename... Args>
    static T* create(Args&& ... args)
    {
        AllocatorType<T> alloc;
        auto deleter = [&](T * object)
        {
            alloc.deallocate(object, 1);
        };
        std::unique_ptr<T, decltype(deleter)> object(alloc.allocate(1), deleter);
        alloc.construct(object.get(), std::forward<Args>(args)...);
        assert(object != nullptr);
        return object.release();
    }

    ////////////////////////
    // JSON value storage //
    ////////////////////////

    /*!
    @brief a JSON value

    The actual storage for a JSON value of the @ref basic_json class. This
    union combines the different storage types for the JSON value types
    defined in @ref value_t.

    JSON type | value_t type    | used type
    --------- | --------------- | ------------------------
    object    | object          | pointer to @ref object_t
    array     | array           | pointer to @ref array_t
    string    | string          | pointer to @ref string_t
    boolean   | boolean         | @ref boolean_t
    number    | number_integer  | @ref number_integer_t
    number    | number_unsigned | @ref number_unsigned_t
    number    | number_float    | @ref number_float_t
    null      | null            | *no value is stored*

    @note Variable-length types (objects, arrays, and strings) are stored as
    pointers. The size of the union should not exceed 64 bits if the default
    value types are used.

    @since version 1.0.0
    */
    union json_value
    {
        /// object (stored with pointer to save storage)
        object_t* object;
        /// array (stored with pointer to save storage)
        array_t* array;
        /// string (stored with pointer to save storage)
        string_t* string;
        /// boolean
        boolean_t boolean;
        /// number (integer)
        number_integer_t number_integer;
        /// number (unsigned integer)
        number_unsigned_t number_unsigned;
        /// number (floating-point)
        number_float_t number_float;

        /// default constructor (for null values)
        json_value() = default;
        /// constructor for booleans
        json_value(boolean_t v) noexcept : boolean(v) {}
        /// constructor for numbers (integer)
        json_value(number_integer_t v) noexcept : number_integer(v) {}
        /// constructor for numbers (unsigned)
        json_value(number_unsigned_t v) noexcept : number_unsigned(v) {}
        /// constructor for numbers (floating-point)
        json_value(number_float_t v) noexcept : number_float(v) {}
        /// constructor for empty values of a given type
        json_value(value_t t)
        {
            switch (t)
            {
                case value_t::object:
                {
                    object = create<object_t>();
                    break;
                }

                case value_t::array:
                {
                    array = create<array_t>();
                    break;
                }

                case value_t::string:
                {
                    string = create<string_t>("");
                    break;
                }

                case value_t::boolean:
                {
                    boolean = boolean_t(false);
                    break;
                }

                case value_t::number_integer:
                {
                    number_integer = number_integer_t(0);
                    break;
                }

                case value_t::number_unsigned:
                {
                    number_unsigned = number_unsigned_t(0);
                    break;
                }

                case value_t::number_float:
                {
                    number_float = number_float_t(0.0);
                    break;
                }

                case value_t::null:
                {
                    break;
                }

                default:
                {
                    if (t == value_t::null)
                    {
                        JSON_THROW(std::domain_error("961c151d2e87f2686a955a9be24d316f1362bf21 2.1.1")); // LCOV_EXCL_LINE
                    }
                    break;
                }
            }
        }

        /// constructor for strings
        json_value(const string_t& value)
        {
            string = create<string_t>(value);
        }

        /// constructor for objects
        json_value(const object_t& value)
        {
            object = create<object_t>(value);
        }

        /// constructor for arrays
        json_value(const array_t& value)
        {
            array = create<array_t>(value);
        }
    };

    /*!
    @brief checks the class invariants

    This function asserts the class invariants. It needs to be called at the
    end of every constructor to make sure that created objects respect the
    invariant. Furthermore, it has to be called each time the type of a JSON
    value is changed, because the invariant expresses a relationship between
    @a m_type and @a m_value.
    */
    void assert_invariant() const
    {
        assert(m_type != value_t::object or m_value.object != nullptr);
        assert(m_type != value_t::array or m_value.array != nullptr);
        assert(m_type != value_t::string or m_value.string != nullptr);
    }

  public:
    //////////////////////////
    // JSON parser callback //
    //////////////////////////

    /*!
    @brief JSON callback events

    This enumeration lists the parser events that can trigger calling a
    callback function of type @ref parser_callback_t during parsing.

    @image html callback_events.png "Example when certain parse events are triggered"

    @since version 1.0.0
    */
    enum class parse_event_t : uint8_t
    {
        /// the parser read `{` and started to process a JSON object
        object_start,
        /// the parser read `}` and finished processing a JSON object
        object_end,
        /// the parser read `[` and started to process a JSON array
        array_start,
        /// the parser read `]` and finished processing a JSON array
        array_end,
        /// the parser read a key of a value in an object
        key,
        /// the parser finished reading a JSON value
        value
    };

    /*!
    @brief per-element parser callback type

    With a parser callback function, the result of parsing a JSON text can be
    influenced. When passed to @ref parse(std::istream&, const
    parser_callback_t) or @ref parse(const CharT, const parser_callback_t),
    it is called on certain events (passed as @ref parse_event_t via parameter
    @a event) with a set recursion depth @a depth and context JSON value
    @a parsed. The return value of the callback function is a boolean
    indicating whether the element that emitted the callback shall be kept or
    not.

    We distinguish six scenarios (determined by the event type) in which the
    callback function can be called. The following table describes the values
    of the parameters @a depth, @a event, and @a parsed.

    parameter @a event | description | parameter @a depth | parameter @a parsed
    ------------------ | ----------- | ------------------ | -------------------
    parse_event_t::object_start | the parser read `{` and started to process a JSON object | depth of the parent of the JSON object | a JSON value with type discarded
    parse_event_t::key | the parser read a key of a value in an object | depth of the currently parsed JSON object | a JSON string containing the key
    parse_event_t::object_end | the parser read `}` and finished processing a JSON object | depth of the parent of the JSON object | the parsed JSON object
    parse_event_t::array_start | the parser read `[` and started to process a JSON array | depth of the parent of the JSON array | a JSON value with type discarded
    parse_event_t::array_end | the parser read `]` and finished processing a JSON array | depth of the parent of the JSON array | the parsed JSON array
    parse_event_t::value | the parser finished reading a JSON value | depth of the value | the parsed JSON value

    @image html callback_events.png "Example when certain parse events are triggered"

    Discarding a value (i.e., returning `false`) has different effects
    depending on the context in which function was called:

    - Discarded values in structured types are skipped. That is, the parser
      will behave as if the discarded value was never read.
    - In case a value outside a structured type is skipped, it is replaced
      with `null`. This case happens if the top-level element is skipped.

    @param[in] depth  the depth of the recursion during parsing

    @param[in] event  an event of type parse_event_t indicating the context in
    the callback function has been called

    @param[in,out] parsed  the current intermediate parse result; note that
    writing to this value has no effect for parse_event_t::key events

    @return Whether the JSON value which called the function during parsing
    should be kept (`true`) or not (`false`). In the latter case, it is either
    skipped completely or replaced by an empty discarded object.

    @sa @ref parse(std::istream&, parser_callback_t) or
    @ref parse(const CharT, const parser_callback_t) for examples

    @since version 1.0.0
    */
    using parser_callback_t = std::function<bool(int depth,
                              parse_event_t event,
                              basic_json& parsed)>;


    //////////////////
    // constructors //
    //////////////////

    /// @name constructors and destructors
    /// Constructors of class @ref basic_json, copy/move constructor, copy
    /// assignment, static functions creating objects, and the destructor.
    /// @{

    /*!
    @brief create an empty value with a given type

    Create an empty JSON value with a given type. The value will be default
    initialized with an empty value which depends on the type:

    Value type  | initial value
    ----------- | -------------
    null        | `null`
    boolean     | `false`
    string      | `""`
    number      | `0`
    object      | `{}`
    array       | `[]`

    @param[in] value_type  the type of the value to create

    @complexity Constant.

    @throw std::bad_alloc if allocation for object, array, or string value
    fails

    @liveexample{The following code shows the constructor for different @ref
    value_t values,basic_json__value_t}

    @since version 1.0.0
    */
    basic_json(const value_t value_type)
        : m_type(value_type), m_value(value_type)
    {
        assert_invariant();
    }

    /*!
    @brief create a null object

    Create a `null` JSON value. It either takes a null pointer as parameter
    (explicitly creating `null`) or no parameter (implicitly creating `null`).
    The passed null pointer itself is not read -- it is only used to choose
    the right constructor.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this constructor never throws
    exceptions.

    @liveexample{The following code shows the constructor with and without a
    null pointer parameter.,basic_json__nullptr_t}

    @since version 1.0.0
    */
    basic_json(std::nullptr_t = nullptr) noexcept
        : basic_json(value_t::null)
    {
        assert_invariant();
    }

    /*!
    @brief create a JSON value

    This is a "catch all" constructor for all compatible JSON types; that is,
    types for which a `to_json()` method exsits. The constructor forwards the
    parameter @a val to that method (to `json_serializer<U>::to_json` method
    with `U = uncvref_t<CompatibleType>`, to be exact).

    Template type @a CompatibleType includes, but is not limited to, the
    following types:
    - **arrays**: @ref array_t and all kinds of compatible containers such as
      `std::vector`, `std::deque`, `std::list`, `std::forward_list`,
      `std::array`, `std::set`, `std::unordered_set`, `std::multiset`, and
      `unordered_multiset` with a `value_type` from which a @ref basic_json
      value can be constructed.
    - **objects**: @ref object_t and all kinds of compatible associative
      containers such as `std::map`, `std::unordered_map`, `std::multimap`,
      and `std::unordered_multimap` with a `key_type` compatible to
      @ref string_t and a `value_type` from which a @ref basic_json value can
      be constructed.
    - **strings**: @ref string_t, string literals, and all compatible string
      containers can be used.
    - **numbers**: @ref number_integer_t, @ref number_unsigned_t,
      @ref number_float_t, and all convertible number types such as `int`,
      `size_t`, `int64_t`, `float` or `double` can be used.
    - **boolean**: @ref boolean_t / `bool` can be used.

    See the examples below.

    @tparam CompatibleType a type such that:
    - @a CompatibleType is not derived from `std::istream`,
    - @a CompatibleType is not @ref basic_json (to avoid hijacking copy/move
         constructors),
    - @a CompatibleType is not a @ref basic_json nested type (e.g.,
         @ref json_pointer, @ref iterator, etc ...)
    - @ref @ref json_serializer<U> has a
         `to_json(basic_json_t&, CompatibleType&&)` method

    @tparam U = `uncvref_t<CompatibleType>`

    @param[in] val the value to be forwarded

    @complexity Usually linear in the size of the passed @a val, also
                depending on the implementation of the called `to_json()`
                method.

    @throw what `json_serializer<U>::to_json()` throws

    @liveexample{The following code shows the constructor with several
    compatible types.,basic_json__CompatibleType}

    @since version 2.1.0
    */
    template<typename CompatibleType, typename U = detail::uncvref_t<CompatibleType>,
             detail::enable_if_t<not std::is_base_of<std::istream, U>::value and
                                 not std::is_same<U, basic_json_t>::value and
                                 not detail::is_basic_json_nested_type<
                                     basic_json_t, U>::value and
                                 detail::has_to_json<basic_json, U>::value,
                                 int> = 0>
    basic_json(CompatibleType && val) noexcept(noexcept(JSONSerializer<U>::to_json(
                std::declval<basic_json_t&>(), std::forward<CompatibleType>(val))))
    {
        JSONSerializer<U>::to_json(*this, std::forward<CompatibleType>(val));
        assert_invariant();
    }

    /*!
    @brief create a container (array or object) from an initializer list

    Creates a JSON value of type array or object from the passed initializer
    list @a init. In case @a type_deduction is `true` (default), the type of
    the JSON value to be created is deducted from the initializer list @a init
    according to the following rules:

    1. If the list is empty, an empty JSON object value `{}` is created.
    2. If the list consists of pairs whose first element is a string, a JSON
       object value is created where the first elements of the pairs are
       treated as keys and the second elements are as values.
    3. In all other cases, an array is created.

    The rules aim to create the best fit between a C++ initializer list and
    JSON values. The rationale is as follows:

    1. The empty initializer list is written as `{}` which is exactly an empty
       JSON object.
    2. C++ has now way of describing mapped types other than to list a list of
       pairs. As JSON requires that keys must be of type string, rule 2 is the
       weakest constraint one can pose on initializer lists to interpret them
       as an object.
    3. In all other cases, the initializer list could not be interpreted as
       JSON object type, so interpreting it as JSON array type is safe.

    With the rules described above, the following JSON values cannot be
    expressed by an initializer list:

    - the empty array (`[]`): use @ref array(std::initializer_list<basic_json>)
      with an empty initializer list in this case
    - arrays whose elements satisfy rule 2: use @ref
      array(std::initializer_list<basic_json>) with the same initializer list
      in this case

    @note When used without parentheses around an empty initializer list, @ref
    basic_json() is called instead of this function, yielding the JSON null
    value.

    @param[in] init  initializer list with JSON values

    @param[in] type_deduction internal parameter; when set to `true`, the type
    of the JSON value is deducted from the initializer list @a init; when set
    to `false`, the type provided via @a manual_type is forced. This mode is
    used by the functions @ref array(std::initializer_list<basic_json>) and
    @ref object(std::initializer_list<basic_json>).

    @param[in] manual_type internal parameter; when @a type_deduction is set
    to `false`, the created JSON value will use the provided type (only @ref
    value_t::array and @ref value_t::object are valid); when @a type_deduction
    is set to `true`, this parameter has no effect

    @throw std::domain_error if @a type_deduction is `false`, @a manual_type
    is `value_t::object`, but @a init contains an element which is not a pair
    whose first element is a string; example: `"cannot create object from
    initializer list"`

    @complexity Linear in the size of the initializer list @a init.

    @liveexample{The example below shows how JSON values are created from
    initializer lists.,basic_json__list_init_t}

    @sa @ref array(std::initializer_list<basic_json>) -- create a JSON array
    value from an initializer list
    @sa @ref object(std::initializer_list<basic_json>) -- create a JSON object
    value from an initializer list

    @since version 1.0.0
    */
    basic_json(std::initializer_list<basic_json> init,
               bool type_deduction = true,
               value_t manual_type = value_t::array)
    {
        // check if each element is an array with two elements whose first
        // element is a string
        bool is_an_object = std::all_of(init.begin(), init.end(),
                                        [](const basic_json & element)
        {
            return element.is_array() and element.size() == 2 and element[0].is_string();
        });

        // adjust type if type deduction is not wanted
        if (not type_deduction)
        {
            // if array is wanted, do not create an object though possible
            if (manual_type == value_t::array)
            {
                is_an_object = false;
            }

            // if object is wanted but impossible, throw an exception
            if (manual_type == value_t::object and not is_an_object)
            {
                JSON_THROW(std::domain_error("cannot create object from initializer list"));
            }
        }

        if (is_an_object)
        {
            // the initializer list is a list of pairs -> create object
            m_type = value_t::object;
            m_value = value_t::object;

            std::for_each(init.begin(), init.end(), [this](const basic_json & element)
            {
                m_value.object->emplace(*(element[0].m_value.string), element[1]);
            });
        }
        else
        {
            // the initializer list describes an array -> create array
            m_type = value_t::array;
            m_value.array = create<array_t>(init);
        }

        assert_invariant();
    }

    /*!
    @brief explicitly create an array from an initializer list

    Creates a JSON array value from a given initializer list. That is, given a
    list of values `a, b, c`, creates the JSON value `[a, b, c]`. If the
    initializer list is empty, the empty array `[]` is created.

    @note This function is only needed to express two edge cases that cannot
    be realized with the initializer list constructor (@ref
    basic_json(std::initializer_list<basic_json>, bool, value_t)). These cases
    are:
    1. creating an array whose elements are all pairs whose first element is a
    string -- in this case, the initializer list constructor would create an
    object, taking the first elements as keys
    2. creating an empty array -- passing the empty initializer list to the
    initializer list constructor yields an empty object

    @param[in] init  initializer list with JSON values to create an array from
    (optional)

    @return JSON array value

    @complexity Linear in the size of @a init.

    @liveexample{The following code shows an example for the `array`
    function.,array}

    @sa @ref basic_json(std::initializer_list<basic_json>, bool, value_t) --
    create a JSON value from an initializer list
    @sa @ref object(std::initializer_list<basic_json>) -- create a JSON object
    value from an initializer list

    @since version 1.0.0
    */
    static basic_json array(std::initializer_list<basic_json> init =
                                std::initializer_list<basic_json>())
    {
        return basic_json(init, false, value_t::array);
    }

    /*!
    @brief explicitly create an object from an initializer list

    Creates a JSON object value from a given initializer list. The initializer
    lists elements must be pairs, and their first elements must be strings. If
    the initializer list is empty, the empty object `{}` is created.

    @note This function is only added for symmetry reasons. In contrast to the
    related function @ref array(std::initializer_list<basic_json>), there are
    no cases which can only be expressed by this function. That is, any
    initializer list @a init can also be passed to the initializer list
    constructor @ref basic_json(std::initializer_list<basic_json>, bool,
    value_t).

    @param[in] init  initializer list to create an object from (optional)

    @return JSON object value

    @throw std::domain_error if @a init is not a pair whose first elements are
    strings; thrown by
    @ref basic_json(std::initializer_list<basic_json>, bool, value_t)

    @complexity Linear in the size of @a init.

    @liveexample{The following code shows an example for the `object`
    function.,object}

    @sa @ref basic_json(std::initializer_list<basic_json>, bool, value_t) --
    create a JSON value from an initializer list
    @sa @ref array(std::initializer_list<basic_json>) -- create a JSON array
    value from an initializer list

    @since version 1.0.0
    */
    static basic_json object(std::initializer_list<basic_json> init =
                                 std::initializer_list<basic_json>())
    {
        return basic_json(init, false, value_t::object);
    }

    /*!
    @brief construct an array with count copies of given value

    Constructs a JSON array value by creating @a cnt copies of a passed value.
    In case @a cnt is `0`, an empty array is created. As postcondition,
    `std::distance(begin(),end()) == cnt` holds.

    @param[in] cnt  the number of JSON copies of @a val to create
    @param[in] val  the JSON value to copy

    @complexity Linear in @a cnt.

    @liveexample{The following code shows examples for the @ref
    basic_json(size_type\, const basic_json&)
    constructor.,basic_json__size_type_basic_json}

    @since version 1.0.0
    */
    basic_json(size_type cnt, const basic_json& val)
        : m_type(value_t::array)
    {
        m_value.array = create<array_t>(cnt, val);
        assert_invariant();
    }

    /*!
    @brief construct a JSON container given an iterator range

    Constructs the JSON value with the contents of the range `[first, last)`.
    The semantics depends on the different types a JSON value can have:
    - In case of primitive types (number, boolean, or string), @a first must
      be `begin()` and @a last must be `end()`. In this case, the value is
      copied. Otherwise, std::out_of_range is thrown.
    - In case of structured types (array, object), the constructor behaves as
      similar versions for `std::vector`.
    - In case of a null type, std::domain_error is thrown.

    @tparam InputIT an input iterator type (@ref iterator or @ref
    const_iterator)

    @param[in] first begin of the range to copy from (included)
    @param[in] last end of the range to copy from (excluded)

    @pre Iterators @a first and @a last must be initialized. **This
         precondition is enforced with an assertion.**

    @throw std::domain_error if iterators are not compatible; that is, do not
    belong to the same JSON value; example: `"iterators are not compatible"`
    @throw std::out_of_range if iterators are for a primitive type (number,
    boolean, or string) where an out of range error can be detected easily;
    example: `"iterators out of range"`
    @throw std::bad_alloc if allocation for object, array, or string fails
    @throw std::domain_error if called with a null value; example: `"cannot
    use construct with iterators from null"`

    @complexity Linear in distance between @a first and @a last.

    @liveexample{The example below shows several ways to create JSON values by
    specifying a subrange with iterators.,basic_json__InputIt_InputIt}

    @since version 1.0.0
    */
    template<class InputIT, typename std::enable_if<
                 std::is_same<InputIT, typename basic_json_t::iterator>::value or
                 std::is_same<InputIT, typename basic_json_t::const_iterator>::value, int>::type = 0>
    basic_json(InputIT first, InputIT last)
    {
        assert(first.m_object != nullptr);
        assert(last.m_object != nullptr);

        // make sure iterator fits the current value
        if (first.m_object != last.m_object)
        {
            JSON_THROW(std::domain_error("iterators are not compatible"));
        }

        // copy type from first iterator
        m_type = first.m_object->m_type;

        // check if iterator range is complete for primitive values
        switch (m_type)
        {
            case value_t::boolean:
            case value_t::number_float:
            case value_t::number_integer:
            case value_t::number_unsigned:
            case value_t::string:
            {
                if (not first.m_it.primitive_iterator.is_begin() or not last.m_it.primitive_iterator.is_end())
                {
                    JSON_THROW(std::out_of_range("iterators out of range"));
                }
                break;
            }

            default:
            {
                break;
            }
        }

        switch (m_type)
        {
            case value_t::number_integer:
            {
                m_value.number_integer = first.m_object->m_value.number_integer;
                break;
            }

            case value_t::number_unsigned:
            {
                m_value.number_unsigned = first.m_object->m_value.number_unsigned;
                break;
            }

            case value_t::number_float:
            {
                m_value.number_float = first.m_object->m_value.number_float;
                break;
            }

            case value_t::boolean:
            {
                m_value.boolean = first.m_object->m_value.boolean;
                break;
            }

            case value_t::string:
            {
                m_value = *first.m_object->m_value.string;
                break;
            }

            case value_t::object:
            {
                m_value.object = create<object_t>(first.m_it.object_iterator,
                                                  last.m_it.object_iterator);
                break;
            }

            case value_t::array:
            {
                m_value.array = create<array_t>(first.m_it.array_iterator,
                                                last.m_it.array_iterator);
                break;
            }

            default:
            {
                JSON_THROW(std::domain_error("cannot use construct with iterators from " + first.m_object->type_name()));
            }
        }

        assert_invariant();
    }

    /*!
    @brief construct a JSON value given an input stream

    @param[in,out] i  stream to read a serialized JSON value from
    @param[in] cb a parser callback function of type @ref parser_callback_t
    which is used to control the deserialization by filtering unwanted values
    (optional)

    @complexity Linear in the length of the input. The parser is a predictive
    LL(1) parser. The complexity can be higher if the parser callback function
    @a cb has a super-linear complexity.

    @note A UTF-8 byte order mark is silently ignored.

    @deprecated This constructor is deprecated and will be removed in version
      3.0.0 to unify the interface of the library. Deserialization will be
      done by stream operators or by calling one of the `parse` functions,
      e.g. @ref parse(std::istream&, const parser_callback_t). That is, calls
      like `json j(i);` for an input stream @a i need to be replaced by
      `json j = json::parse(i);`. See the example below.

    @liveexample{The example below demonstrates constructing a JSON value from
    a `std::stringstream` with and without callback
    function.,basic_json__istream}

    @since version 2.0.0, deprecated in version 2.0.3, to be removed in
           version 3.0.0
    */
    JSON_DEPRECATED
    explicit basic_json(std::istream& i, const parser_callback_t cb = nullptr)
    {
        *this = parser(i, cb).parse();
        assert_invariant();
    }

    ///////////////////////////////////////
    // other constructors and destructor //
    ///////////////////////////////////////

    /*!
    @brief copy constructor

    Creates a copy of a given JSON value.

    @param[in] other  the JSON value to copy

    @complexity Linear in the size of @a other.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is linear.
    - As postcondition, it holds: `other == basic_json(other)`.

    @throw std::bad_alloc if allocation for object, array, or string fails.

    @liveexample{The following code shows an example for the copy
    constructor.,basic_json__basic_json}

    @since version 1.0.0
    */
    basic_json(const basic_json& other)
        : m_type(other.m_type)
    {
        // check of passed value is valid
        other.assert_invariant();

        switch (m_type)
        {
            case value_t::object:
            {
                m_value = *other.m_value.object;
                break;
            }

            case value_t::array:
            {
                m_value = *other.m_value.array;
                break;
            }

            case value_t::string:
            {
                m_value = *other.m_value.string;
                break;
            }

            case value_t::boolean:
            {
                m_value = other.m_value.boolean;
                break;
            }

            case value_t::number_integer:
            {
                m_value = other.m_value.number_integer;
                break;
            }

            case value_t::number_unsigned:
            {
                m_value = other.m_value.number_unsigned;
                break;
            }

            case value_t::number_float:
            {
                m_value = other.m_value.number_float;
                break;
            }

            default:
            {
                break;
            }
        }

        assert_invariant();
    }

    /*!
    @brief move constructor

    Move constructor. Constructs a JSON value with the contents of the given
    value @a other using move semantics. It "steals" the resources from @a
    other and leaves it as JSON null value.

    @param[in,out] other  value to move to this object

    @post @a other is a JSON null value

    @complexity Constant.

    @liveexample{The code below shows the move constructor explicitly called
    via std::move.,basic_json__moveconstructor}

    @since version 1.0.0
    */
    basic_json(basic_json&& other) noexcept
        : m_type(std::move(other.m_type)),
          m_value(std::move(other.m_value))
    {
        // check that passed value is valid
        other.assert_invariant();

        // invalidate payload
        other.m_type = value_t::null;
        other.m_value = {};

        assert_invariant();
    }

    /*!
    @brief copy assignment

    Copy assignment operator. Copies a JSON value via the "copy and swap"
    strategy: It is expressed in terms of the copy constructor, destructor,
    and the swap() member function.

    @param[in] other  value to copy from

    @complexity Linear.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is linear.

    @liveexample{The code below shows and example for the copy assignment. It
    creates a copy of value `a` which is then swapped with `b`. Finally\, the
    copy of `a` (which is the null value after the swap) is
    destroyed.,basic_json__copyassignment}

    @since version 1.0.0
    */
    reference& operator=(basic_json other) noexcept (
        std::is_nothrow_move_constructible<value_t>::value and
        std::is_nothrow_move_assignable<value_t>::value and
        std::is_nothrow_move_constructible<json_value>::value and
        std::is_nothrow_move_assignable<json_value>::value
    )
    {
        // check that passed value is valid
        other.assert_invariant();

        using std::swap;
        swap(m_type, other.m_type);
        swap(m_value, other.m_value);

        assert_invariant();
        return *this;
    }

    /*!
    @brief destructor

    Destroys the JSON value and frees all allocated memory.

    @complexity Linear.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is linear.
    - All stored elements are destroyed and all memory is freed.

    @since version 1.0.0
    */
    ~basic_json()
    {
        assert_invariant();

        switch (m_type)
        {
            case value_t::object:
            {
                AllocatorType<object_t> alloc;
                alloc.destroy(m_value.object);
                alloc.deallocate(m_value.object, 1);
                break;
            }

            case value_t::array:
            {
                AllocatorType<array_t> alloc;
                alloc.destroy(m_value.array);
                alloc.deallocate(m_value.array, 1);
                break;
            }

            case value_t::string:
            {
                AllocatorType<string_t> alloc;
                alloc.destroy(m_value.string);
                alloc.deallocate(m_value.string, 1);
                break;
            }

            default:
            {
                // all other types need no specific destructor
                break;
            }
        }
    }

    /// @}

  public:
    ///////////////////////
    // object inspection //
    ///////////////////////

    /// @name object inspection
    /// Functions to inspect the type of a JSON value.
    /// @{

    /*!
    @brief serialization

    Serialization function for JSON values. The function tries to mimic
    Python's `json.dumps()` function, and currently supports its @a indent
    parameter.

    @param[in] indent If indent is nonnegative, then array elements and object
    members will be pretty-printed with that indent level. An indent level of
    `0` will only insert newlines. `-1` (the default) selects the most compact
    representation.

    @return string containing the serialization of the JSON value

    @complexity Linear.

    @liveexample{The following example shows the effect of different @a indent
    parameters to the result of the serialization.,dump}

    @see https://docs.python.org/2/library/json.html#json.dump

    @since version 1.0.0
    */
    string_t dump(const int indent = -1) const
    {
        std::stringstream ss;

        if (indent >= 0)
        {
            dump(ss, true, static_cast<unsigned int>(indent));
        }
        else
        {
            dump(ss, false, 0);
        }

        return ss.str();
    }

    /*!
    @brief return the type of the JSON value (explicit)

    Return the type of the JSON value as a value from the @ref value_t
    enumeration.

    @return the type of the JSON value

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `type()` for all JSON
    types.,type}

    @since version 1.0.0
    */
    constexpr value_t type() const noexcept
    {
        return m_type;
    }

    /*!
    @brief return whether type is primitive

    This function returns true iff the JSON type is primitive (string, number,
    boolean, or null).

    @return `true` if type is primitive (string, number, boolean, or null),
    `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_primitive()` for all JSON
    types.,is_primitive}

    @sa @ref is_structured() -- returns whether JSON value is structured
    @sa @ref is_null() -- returns whether JSON value is `null`
    @sa @ref is_string() -- returns whether JSON value is a string
    @sa @ref is_boolean() -- returns whether JSON value is a boolean
    @sa @ref is_number() -- returns whether JSON value is a number

    @since version 1.0.0
    */
    constexpr bool is_primitive() const noexcept
    {
        return is_null() or is_string() or is_boolean() or is_number();
    }

    /*!
    @brief return whether type is structured

    This function returns true iff the JSON type is structured (array or
    object).

    @return `true` if type is structured (array or object), `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_structured()` for all JSON
    types.,is_structured}

    @sa @ref is_primitive() -- returns whether value is primitive
    @sa @ref is_array() -- returns whether value is an array
    @sa @ref is_object() -- returns whether value is an object

    @since version 1.0.0
    */
    constexpr bool is_structured() const noexcept
    {
        return is_array() or is_object();
    }

    /*!
    @brief return whether value is null

    This function returns true iff the JSON value is null.

    @return `true` if type is null, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_null()` for all JSON
    types.,is_null}

    @since version 1.0.0
    */
    constexpr bool is_null() const noexcept
    {
        return m_type == value_t::null;
    }

    /*!
    @brief return whether value is a boolean

    This function returns true iff the JSON value is a boolean.

    @return `true` if type is boolean, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_boolean()` for all JSON
    types.,is_boolean}

    @since version 1.0.0
    */
    constexpr bool is_boolean() const noexcept
    {
        return m_type == value_t::boolean;
    }

    /*!
    @brief return whether value is a number

    This function returns true iff the JSON value is a number. This includes
    both integer and floating-point values.

    @return `true` if type is number (regardless whether integer, unsigned
    integer or floating-type), `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_number()` for all JSON
    types.,is_number}

    @sa @ref is_number_integer() -- check if value is an integer or unsigned
    integer number
    @sa @ref is_number_unsigned() -- check if value is an unsigned integer
    number
    @sa @ref is_number_float() -- check if value is a floating-point number

    @since version 1.0.0
    */
    constexpr bool is_number() const noexcept
    {
        return is_number_integer() or is_number_float();
    }

    /*!
    @brief return whether value is an integer number

    This function returns true iff the JSON value is an integer or unsigned
    integer number. This excludes floating-point values.

    @return `true` if type is an integer or unsigned integer number, `false`
    otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_number_integer()` for all
    JSON types.,is_number_integer}

    @sa @ref is_number() -- check if value is a number
    @sa @ref is_number_unsigned() -- check if value is an unsigned integer
    number
    @sa @ref is_number_float() -- check if value is a floating-point number

    @since version 1.0.0
    */
    constexpr bool is_number_integer() const noexcept
    {
        return m_type == value_t::number_integer or m_type == value_t::number_unsigned;
    }

    /*!
    @brief return whether value is an unsigned integer number

    This function returns true iff the JSON value is an unsigned integer
    number. This excludes floating-point and (signed) integer values.

    @return `true` if type is an unsigned integer number, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_number_unsigned()` for all
    JSON types.,is_number_unsigned}

    @sa @ref is_number() -- check if value is a number
    @sa @ref is_number_integer() -- check if value is an integer or unsigned
    integer number
    @sa @ref is_number_float() -- check if value is a floating-point number

    @since version 2.0.0
    */
    constexpr bool is_number_unsigned() const noexcept
    {
        return m_type == value_t::number_unsigned;
    }

    /*!
    @brief return whether value is a floating-point number

    This function returns true iff the JSON value is a floating-point number.
    This excludes integer and unsigned integer values.

    @return `true` if type is a floating-point number, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_number_float()` for all
    JSON types.,is_number_float}

    @sa @ref is_number() -- check if value is number
    @sa @ref is_number_integer() -- check if value is an integer number
    @sa @ref is_number_unsigned() -- check if value is an unsigned integer
    number

    @since version 1.0.0
    */
    constexpr bool is_number_float() const noexcept
    {
        return m_type == value_t::number_float;
    }

    /*!
    @brief return whether value is an object

    This function returns true iff the JSON value is an object.

    @return `true` if type is object, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_object()` for all JSON
    types.,is_object}

    @since version 1.0.0
    */
    constexpr bool is_object() const noexcept
    {
        return m_type == value_t::object;
    }

    /*!
    @brief return whether value is an array

    This function returns true iff the JSON value is an array.

    @return `true` if type is array, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_array()` for all JSON
    types.,is_array}

    @since version 1.0.0
    */
    constexpr bool is_array() const noexcept
    {
        return m_type == value_t::array;
    }

    /*!
    @brief return whether value is a string

    This function returns true iff the JSON value is a string.

    @return `true` if type is string, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_string()` for all JSON
    types.,is_string}

    @since version 1.0.0
    */
    constexpr bool is_string() const noexcept
    {
        return m_type == value_t::string;
    }

    /*!
    @brief return whether value is discarded

    This function returns true iff the JSON value was discarded during parsing
    with a callback function (see @ref parser_callback_t).

    @note This function will always be `false` for JSON values after parsing.
    That is, discarded values can only occur during parsing, but will be
    removed when inside a structured value or replaced by null in other cases.

    @return `true` if type is discarded, `false` otherwise.

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies `is_discarded()` for all JSON
    types.,is_discarded}

    @since version 1.0.0
    */
    constexpr bool is_discarded() const noexcept
    {
        return m_type == value_t::discarded;
    }

    /*!
    @brief return the type of the JSON value (implicit)

    Implicitly return the type of the JSON value as a value from the @ref
    value_t enumeration.

    @return the type of the JSON value

    @complexity Constant.

    @exceptionsafety No-throw guarantee: this member function never throws
    exceptions.

    @liveexample{The following code exemplifies the @ref value_t operator for
    all JSON types.,operator__value_t}

    @since version 1.0.0
    */
    constexpr operator value_t() const noexcept
    {
        return m_type;
    }

    /// @}

  private:
    //////////////////
    // value access //
    //////////////////

    /// get a boolean (explicit)
    boolean_t get_impl(boolean_t* /*unused*/) const
    {
        if (is_boolean())
        {
            return m_value.boolean;
        }

        JSON_THROW(std::domain_error("type must be boolean, but is " + type_name()));
    }

    /// get a pointer to the value (object)
    object_t* get_impl_ptr(object_t* /*unused*/) noexcept
    {
        return is_object() ? m_value.object : nullptr;
    }

    /// get a pointer to the value (object)
    constexpr const object_t* get_impl_ptr(const object_t* /*unused*/) const noexcept
    {
        return is_object() ? m_value.object : nullptr;
    }

    /// get a pointer to the value (array)
    array_t* get_impl_ptr(array_t* /*unused*/) noexcept
    {
        return is_array() ? m_value.array : nullptr;
    }

    /// get a pointer to the value (array)
    constexpr const array_t* get_impl_ptr(const array_t* /*unused*/) const noexcept
    {
        return is_array() ? m_value.array : nullptr;
    }

    /// get a pointer to the value (string)
    string_t* get_impl_ptr(string_t* /*unused*/) noexcept
    {
        return is_string() ? m_value.string : nullptr;
    }

    /// get a pointer to the value (string)
    constexpr const string_t* get_impl_ptr(const string_t* /*unused*/) const noexcept
    {
        return is_string() ? m_value.string : nullptr;
    }

    /// get a pointer to the value (boolean)
    boolean_t* get_impl_ptr(boolean_t* /*unused*/) noexcept
    {
        return is_boolean() ? &m_value.boolean : nullptr;
    }

    /// get a pointer to the value (boolean)
    constexpr const boolean_t* get_impl_ptr(const boolean_t* /*unused*/) const noexcept
    {
        return is_boolean() ? &m_value.boolean : nullptr;
    }

    /// get a pointer to the value (integer number)
    number_integer_t* get_impl_ptr(number_integer_t* /*unused*/) noexcept
    {
        return is_number_integer() ? &m_value.number_integer : nullptr;
    }

    /// get a pointer to the value (integer number)
    constexpr const number_integer_t* get_impl_ptr(const number_integer_t* /*unused*/) const noexcept
    {
        return is_number_integer() ? &m_value.number_integer : nullptr;
    }

    /// get a pointer to the value (unsigned number)
    number_unsigned_t* get_impl_ptr(number_unsigned_t* /*unused*/) noexcept
    {
        return is_number_unsigned() ? &m_value.number_unsigned : nullptr;
    }

    /// get a pointer to the value (unsigned number)
    constexpr const number_unsigned_t* get_impl_ptr(const number_unsigned_t* /*unused*/) const noexcept
    {
        return is_number_unsigned() ? &m_value.number_unsigned : nullptr;
    }

    /// get a pointer to the value (floating-point number)
    number_float_t* get_impl_ptr(number_float_t* /*unused*/) noexcept
    {
        return is_number_float() ? &m_value.number_float : nullptr;
    }

    /// get a pointer to the value (floating-point number)
    constexpr const number_float_t* get_impl_ptr(const number_float_t* /*unused*/) const noexcept
    {
        return is_number_float() ? &m_value.number_float : nullptr;
    }

    /*!
    @brief helper function to implement get_ref()

    This funcion helps to implement get_ref() without code duplication for
    const and non-const overloads

    @tparam ThisType will be deduced as `basic_json` or `const basic_json`

    @throw std::domain_error if ReferenceType does not match underlying value
    type of the current JSON
    */
    template<typename ReferenceType, typename ThisType>
    static ReferenceType get_ref_impl(ThisType& obj)
    {
        // helper type
        using PointerType = typename std::add_pointer<ReferenceType>::type;

        // delegate the call to get_ptr<>()
        auto ptr = obj.template get_ptr<PointerType>();

        if (ptr != nullptr)
        {
            return *ptr;
        }

        JSON_THROW(std::domain_error("incompatible ReferenceType for get_ref, actual type is " +
                                     obj.type_name()));
    }

  public:
    /// @name value access
    /// Direct access to the stored value of a JSON value.
    /// @{

    /*!
    @brief get special-case overload

    This overloads avoids a lot of template boilerplate, it can be seen as the
    identity method

    @tparam BasicJsonType == @ref basic_json

    @return a copy of *this

    @complexity Constant.

    @since version 2.1.0
    */
    template <
        typename BasicJsonType,
        detail::enable_if_t<std::is_same<typename std::remove_const<BasicJsonType>::type,
                                         basic_json_t>::value,
                            int> = 0 >
    basic_json get() const
    {
        return *this;
    }

    /*!
    @brief get a value (explicit)

    Explicit type conversion between the JSON value and a compatible value
    which is [CopyConstructible](http://en.cppreference.com/w/cpp/concept/CopyConstructible)
    and [DefaultConstructible](http://en.cppreference.com/w/cpp/concept/DefaultConstructible).
    The value is converted by calling the @ref json_serializer<ValueType>
    `from_json()` method.

    The function is equivalent to executing
    @code {.cpp}
    ValueType ret;
    JSONSerializer<ValueType>::from_json(*this, ret);
    return ret;
    @endcode

    This overloads is chosen if:
    - @a ValueType is not @ref basic_json,
    - @ref json_serializer<ValueType> has a `from_json()` method of the form
      `void from_json(const @ref basic_json&, ValueType&)`, and
    - @ref json_serializer<ValueType> does not have a `from_json()` method of
      the form `ValueType from_json(const @ref basic_json&)`

    @tparam ValueTypeCV the provided value type
    @tparam ValueType the returned value type

    @return copy of the JSON value, converted to @a ValueType

    @throw what @ref json_serializer<ValueType> `from_json()` method throws

    @liveexample{The example below shows several conversions from JSON values
    to other types. There a few things to note: (1) Floating-point numbers can
    be converted to integers\, (2) A JSON array can be converted to a standard
    `std::vector<short>`\, (3) A JSON object can be converted to C++
    associative containers such as `std::unordered_map<std::string\,
    json>`.,get__ValueType_const}

    @since version 2.1.0
    */
    template <
        typename ValueTypeCV,
        typename ValueType = detail::uncvref_t<ValueTypeCV>,
        detail::enable_if_t <
            not std::is_same<basic_json_t, ValueType>::value and
            detail::has_from_json<basic_json_t, ValueType>::value and
            not detail::has_non_default_from_json<basic_json_t, ValueType>::value,
            int > = 0 >
    ValueType get() const noexcept(noexcept(
                                       JSONSerializer<ValueType>::from_json(std::declval<const basic_json_t&>(), std::declval<ValueType&>())))
    {
        // we cannot static_assert on ValueTypeCV being non-const, because
        // there is support for get<const basic_json_t>(), which is why we
        // still need the uncvref
        static_assert(not std::is_reference<ValueTypeCV>::value,
                      "get() cannot be used with reference types, you might want to use get_ref()");
        static_assert(std::is_default_constructible<ValueType>::value,
                      "types must be DefaultConstructible when used with get()");

        ValueType ret;
        JSONSerializer<ValueType>::from_json(*this, ret);
        return ret;
    }

    /*!
    @brief get a value (explicit); special case

    Explicit type conversion between the JSON value and a compatible value
    which is **not** [CopyConstructible](http://en.cppreference.com/w/cpp/concept/CopyConstructible)
    and **not** [DefaultConstructible](http://en.cppreference.com/w/cpp/concept/DefaultConstructible).
    The value is converted by calling the @ref json_serializer<ValueType>
    `from_json()` method.

    The function is equivalent to executing
    @code {.cpp}
    return JSONSerializer<ValueTypeCV>::from_json(*this);
    @endcode

    This overloads is chosen if:
    - @a ValueType is not @ref basic_json and
    - @ref json_serializer<ValueType> has a `from_json()` method of the form
      `ValueType from_json(const @ref basic_json&)`

    @note If @ref json_serializer<ValueType> has both overloads of
    `from_json()`, this one is chosen.

    @tparam ValueTypeCV the provided value type
    @tparam ValueType the returned value type

    @return copy of the JSON value, converted to @a ValueType

    @throw what @ref json_serializer<ValueType> `from_json()` method throws

    @since version 2.1.0
    */
    template <
        typename ValueTypeCV,
        typename ValueType = detail::uncvref_t<ValueTypeCV>,
        detail::enable_if_t<not std::is_same<basic_json_t, ValueType>::value and
                            detail::has_non_default_from_json<basic_json_t,
                                    ValueType>::value, int> = 0 >
    ValueType get() const noexcept(noexcept(
                                       JSONSerializer<ValueTypeCV>::from_json(std::declval<const basic_json_t&>())))
    {
        static_assert(not std::is_reference<ValueTypeCV>::value,
                      "get() cannot be used with reference types, you might want to use get_ref()");
        return JSONSerializer<ValueTypeCV>::from_json(*this);
    }

    /*!
    @brief get a pointer value (explicit)

    Explicit pointer access to the internally stored JSON value. No copies are
    made.

    @warning The pointer becomes invalid if the underlying JSON object
    changes.

    @tparam PointerType pointer type; must be a pointer to @ref array_t, @ref
    object_t, @ref string_t, @ref boolean_t, @ref number_integer_t,
    @ref number_unsigned_t, or @ref number_float_t.

    @return pointer to the internally stored JSON value if the requested
    pointer type @a PointerType fits to the JSON value; `nullptr` otherwise

    @complexity Constant.

    @liveexample{The example below shows how pointers to internal values of a
    JSON value can be requested. Note that no type conversions are made and a
    `nullptr` is returned if the value and the requested pointer type does not
    match.,get__PointerType}

    @sa @ref get_ptr() for explicit pointer-member access

    @since version 1.0.0
    */
    template<typename PointerType, typename std::enable_if<
                 std::is_pointer<PointerType>::value, int>::type = 0>
    PointerType get() noexcept
    {
        // delegate the call to get_ptr
        return get_ptr<PointerType>();
    }

    /*!
    @brief get a pointer value (explicit)
    @copydoc get()
    */
    template<typename PointerType, typename std::enable_if<
                 std::is_pointer<PointerType>::value, int>::type = 0>
    constexpr const PointerType get() const noexcept
    {
        // delegate the call to get_ptr
        return get_ptr<PointerType>();
    }

    /*!
    @brief get a pointer value (implicit)

    Implicit pointer access to the internally stored JSON value. No copies are
    made.

    @warning Writing data to the pointee of the result yields an undefined
    state.

    @tparam PointerType pointer type; must be a pointer to @ref array_t, @ref
    object_t, @ref string_t, @ref boolean_t, @ref number_integer_t,
    @ref number_unsigned_t, or @ref number_float_t. Enforced by a static
    assertion.

    @return pointer to the internally stored JSON value if the requested
    pointer type @a PointerType fits to the JSON value; `nullptr` otherwise

    @complexity Constant.

    @liveexample{The example below shows how pointers to internal values of a
    JSON value can be requested. Note that no type conversions are made and a
    `nullptr` is returned if the value and the requested pointer type does not
    match.,get_ptr}

    @since version 1.0.0
    */
    template<typename PointerType, typename std::enable_if<
                 std::is_pointer<PointerType>::value, int>::type = 0>
    PointerType get_ptr() noexcept
    {
        // get the type of the PointerType (remove pointer and const)
        using pointee_t = typename std::remove_const<typename
                          std::remove_pointer<typename
                          std::remove_const<PointerType>::type>::type>::type;
        // make sure the type matches the allowed types
        static_assert(
            std::is_same<object_t, pointee_t>::value
            or std::is_same<array_t, pointee_t>::value
            or std::is_same<string_t, pointee_t>::value
            or std::is_same<boolean_t, pointee_t>::value
            or std::is_same<number_integer_t, pointee_t>::value
            or std::is_same<number_unsigned_t, pointee_t>::value
            or std::is_same<number_float_t, pointee_t>::value
            , "incompatible pointer type");

        // delegate the call to get_impl_ptr<>()
        return get_impl_ptr(static_cast<PointerType>(nullptr));
    }

    /*!
    @brief get a pointer value (implicit)
    @copydoc get_ptr()
    */
    template<typename PointerType, typename std::enable_if<
                 std::is_pointer<PointerType>::value and
                 std::is_const<typename std::remove_pointer<PointerType>::type>::value, int>::type = 0>
    constexpr const PointerType get_ptr() const noexcept
    {
        // get the type of the PointerType (remove pointer and const)
        using pointee_t = typename std::remove_const<typename
                          std::remove_pointer<typename
                          std::remove_const<PointerType>::type>::type>::type;
        // make sure the type matches the allowed types
        static_assert(
            std::is_same<object_t, pointee_t>::value
            or std::is_same<array_t, pointee_t>::value
            or std::is_same<string_t, pointee_t>::value
            or std::is_same<boolean_t, pointee_t>::value
            or std::is_same<number_integer_t, pointee_t>::value
            or std::is_same<number_unsigned_t, pointee_t>::value
            or std::is_same<number_float_t, pointee_t>::value
            , "incompatible pointer type");

        // delegate the call to get_impl_ptr<>() const
        return get_impl_ptr(static_cast<const PointerType>(nullptr));
    }

    /*!
    @brief get a reference value (implicit)

    Implicit reference access to the internally stored JSON value. No copies
    are made.

    @warning Writing data to the referee of the result yields an undefined
    state.

    @tparam ReferenceType reference type; must be a reference to @ref array_t,
    @ref object_t, @ref string_t, @ref boolean_t, @ref number_integer_t, or
    @ref number_float_t. Enforced by static assertion.

    @return reference to the internally stored JSON value if the requested
    reference type @a ReferenceType fits to the JSON value; throws
    std::domain_error otherwise

    @throw std::domain_error in case passed type @a ReferenceType is
    incompatible with the stored JSON value

    @complexity Constant.

    @liveexample{The example shows several calls to `get_ref()`.,get_ref}

    @since version 1.1.0
    */
    template<typename ReferenceType, typename std::enable_if<
                 std::is_reference<ReferenceType>::value, int>::type = 0>
    ReferenceType get_ref()
    {
        // delegate call to get_ref_impl
        return get_ref_impl<ReferenceType>(*this);
    }

    /*!
    @brief get a reference value (implicit)
    @copydoc get_ref()
    */
    template<typename ReferenceType, typename std::enable_if<
                 std::is_reference<ReferenceType>::value and
                 std::is_const<typename std::remove_reference<ReferenceType>::type>::value, int>::type = 0>
    ReferenceType get_ref() const
    {
        // delegate call to get_ref_impl
        return get_ref_impl<ReferenceType>(*this);
    }

    /*!
    @brief get a value (implicit)

    Implicit type conversion between the JSON value and a compatible value.
    The call is realized by calling @ref get() const.

    @tparam ValueType non-pointer type compatible to the JSON value, for
    instance `int` for JSON integer numbers, `bool` for JSON booleans, or
    `std::vector` types for JSON arrays. The character type of @ref string_t
    as well as an initializer list of this type is excluded to avoid
    ambiguities as these types implicitly convert to `std::string`.

    @return copy of the JSON value, converted to type @a ValueType

    @throw std::domain_error in case passed type @a ValueType is incompatible
    to JSON, thrown by @ref get() const

    @complexity Linear in the size of the JSON value.

    @liveexample{The example below shows several conversions from JSON values
    to other types. There a few things to note: (1) Floating-point numbers can
    be converted to integers\, (2) A JSON array can be converted to a standard
    `std::vector<short>`\, (3) A JSON object can be converted to C++
    associative containers such as `std::unordered_map<std::string\,
    json>`.,operator__ValueType}

    @since version 1.0.0
    */
    template < typename ValueType, typename std::enable_if <
                   not std::is_pointer<ValueType>::value and
                   not std::is_same<ValueType, typename string_t::value_type>::value
#ifndef _MSC_VER  // fix for issue #167 operator<< ambiguity under VS2015
                   and not std::is_same<ValueType, std::initializer_list<typename string_t::value_type>>::value
#endif
                   , int >::type = 0 >
    operator ValueType() const
    {
        // delegate the call to get<>() const
        return get<ValueType>();
    }

    /// @}


    ////////////////////
    // element access //
    ////////////////////

    /// @name element access
    /// Access to the JSON value.
    /// @{

    /*!
    @brief access specified array element with bounds checking

    Returns a reference to the element at specified location @a idx, with
    bounds checking.

    @param[in] idx  index of the element to access

    @return reference to the element at index @a idx

    @throw std::domain_error if the JSON value is not an array; example:
    `"cannot use at() with string"`
    @throw std::out_of_range if the index @a idx is out of range of the array;
    that is, `idx >= size()`; example: `"array index 7 is out of range"`

    @complexity Constant.

    @liveexample{The example below shows how array elements can be read and
    written using `at()`.,at__size_type}

    @since version 1.0.0
    */
    reference at(size_type idx)
    {
        // at only works for arrays
        if (is_array())
        {
            JSON_TRY
            {
                return m_value.array->at(idx);
            }
            JSON_CATCH (std::out_of_range&)
            {
                // create better exception explanation
                JSON_THROW(std::out_of_range("array index " + std::to_string(idx) + " is out of range"));
            }
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use at() with " + type_name()));
        }
    }

    /*!
    @brief access specified array element with bounds checking

    Returns a const reference to the element at specified location @a idx,
    with bounds checking.

    @param[in] idx  index of the element to access

    @return const reference to the element at index @a idx

    @throw std::domain_error if the JSON value is not an array; example:
    `"cannot use at() with string"`
    @throw std::out_of_range if the index @a idx is out of range of the array;
    that is, `idx >= size()`; example: `"array index 7 is out of range"`

    @complexity Constant.

    @liveexample{The example below shows how array elements can be read using
    `at()`.,at__size_type_const}

    @since version 1.0.0
    */
    const_reference at(size_type idx) const
    {
        // at only works for arrays
        if (is_array())
        {
            JSON_TRY
            {
                return m_value.array->at(idx);
            }
            JSON_CATCH (std::out_of_range&)
            {
                // create better exception explanation
                JSON_THROW(std::out_of_range("array index " + std::to_string(idx) + " is out of range"));
            }
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use at() with " + type_name()));
        }
    }

    /*!
    @brief access specified object element with bounds checking

    Returns a reference to the element at with specified key @a key, with
    bounds checking.

    @param[in] key  key of the element to access

    @return reference to the element at key @a key

    @throw std::domain_error if the JSON value is not an object; example:
    `"cannot use at() with boolean"`
    @throw std::out_of_range if the key @a key is is not stored in the object;
    that is, `find(key) == end()`; example: `"key "the fast" not found"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read and
    written using `at()`.,at__object_t_key_type}

    @sa @ref operator[](const typename object_t::key_type&) for unchecked
    access by reference
    @sa @ref value() for access by value with a default value

    @since version 1.0.0
    */
    reference at(const typename object_t::key_type& key)
    {
        // at only works for objects
        if (is_object())
        {
            JSON_TRY
            {
                return m_value.object->at(key);
            }
            JSON_CATCH (std::out_of_range&)
            {
                // create better exception explanation
                JSON_THROW(std::out_of_range("key '" + key + "' not found"));
            }
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use at() with " + type_name()));
        }
    }

    /*!
    @brief access specified object element with bounds checking

    Returns a const reference to the element at with specified key @a key,
    with bounds checking.

    @param[in] key  key of the element to access

    @return const reference to the element at key @a key

    @throw std::domain_error if the JSON value is not an object; example:
    `"cannot use at() with boolean"`
    @throw std::out_of_range if the key @a key is is not stored in the object;
    that is, `find(key) == end()`; example: `"key "the fast" not found"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read using
    `at()`.,at__object_t_key_type_const}

    @sa @ref operator[](const typename object_t::key_type&) for unchecked
    access by reference
    @sa @ref value() for access by value with a default value

    @since version 1.0.0
    */
    const_reference at(const typename object_t::key_type& key) const
    {
        // at only works for objects
        if (is_object())
        {
            JSON_TRY
            {
                return m_value.object->at(key);
            }
            JSON_CATCH (std::out_of_range&)
            {
                // create better exception explanation
                JSON_THROW(std::out_of_range("key '" + key + "' not found"));
            }
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use at() with " + type_name()));
        }
    }

    /*!
    @brief access specified array element

    Returns a reference to the element at specified location @a idx.

    @note If @a idx is beyond the range of the array (i.e., `idx >= size()`),
    then the array is silently filled up with `null` values to make `idx` a
    valid reference to the last stored element.

    @param[in] idx  index of the element to access

    @return reference to the element at index @a idx

    @throw std::domain_error if JSON is not an array or null; example:
    `"cannot use operator[] with string"`

    @complexity Constant if @a idx is in the range of the array. Otherwise
    linear in `idx - size()`.

    @liveexample{The example below shows how array elements can be read and
    written using `[]` operator. Note the addition of `null`
    values.,operatorarray__size_type}

    @since version 1.0.0
    */
    reference operator[](size_type idx)
    {
        // implicitly convert null value to an empty array
        if (is_null())
        {
            m_type = value_t::array;
            m_value.array = create<array_t>();
            assert_invariant();
        }

        // operator[] only works for arrays
        if (is_array())
        {
            // fill up array with null values if given idx is outside range
            if (idx >= m_value.array->size())
            {
                m_value.array->insert(m_value.array->end(),
                                      idx - m_value.array->size() + 1,
                                      basic_json());
            }

            return m_value.array->operator[](idx);
        }

        JSON_THROW(std::domain_error("cannot use operator[] with " + type_name()));
    }

    /*!
    @brief access specified array element

    Returns a const reference to the element at specified location @a idx.

    @param[in] idx  index of the element to access

    @return const reference to the element at index @a idx

    @throw std::domain_error if JSON is not an array; example: `"cannot use
    operator[] with null"`

    @complexity Constant.

    @liveexample{The example below shows how array elements can be read using
    the `[]` operator.,operatorarray__size_type_const}

    @since version 1.0.0
    */
    const_reference operator[](size_type idx) const
    {
        // const operator[] only works for arrays
        if (is_array())
        {
            return m_value.array->operator[](idx);
        }

        JSON_THROW(std::domain_error("cannot use operator[] with " + type_name()));
    }

    /*!
    @brief access specified object element

    Returns a reference to the element at with specified key @a key.

    @note If @a key is not found in the object, then it is silently added to
    the object and filled with a `null` value to make `key` a valid reference.
    In case the value was `null` before, it is converted to an object.

    @param[in] key  key of the element to access

    @return reference to the element at key @a key

    @throw std::domain_error if JSON is not an object or null; example:
    `"cannot use operator[] with string"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read and
    written using the `[]` operator.,operatorarray__key_type}

    @sa @ref at(const typename object_t::key_type&) for access by reference
    with range checking
    @sa @ref value() for access by value with a default value

    @since version 1.0.0
    */
    reference operator[](const typename object_t::key_type& key)
    {
        // implicitly convert null value to an empty object
        if (is_null())
        {
            m_type = value_t::object;
            m_value.object = create<object_t>();
            assert_invariant();
        }

        // operator[] only works for objects
        if (is_object())
        {
            return m_value.object->operator[](key);
        }

        JSON_THROW(std::domain_error("cannot use operator[] with " + type_name()));
    }

    /*!
    @brief read-only access specified object element

    Returns a const reference to the element at with specified key @a key. No
    bounds checking is performed.

    @warning If the element with key @a key does not exist, the behavior is
    undefined.

    @param[in] key  key of the element to access

    @return const reference to the element at key @a key

    @pre The element with key @a key must exist. **This precondition is
         enforced with an assertion.**

    @throw std::domain_error if JSON is not an object; example: `"cannot use
    operator[] with null"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read using
    the `[]` operator.,operatorarray__key_type_const}

    @sa @ref at(const typename object_t::key_type&) for access by reference
    with range checking
    @sa @ref value() for access by value with a default value

    @since version 1.0.0
    */
    const_reference operator[](const typename object_t::key_type& key) const
    {
        // const operator[] only works for objects
        if (is_object())
        {
            assert(m_value.object->find(key) != m_value.object->end());
            return m_value.object->find(key)->second;
        }

        JSON_THROW(std::domain_error("cannot use operator[] with " + type_name()));
    }

    /*!
    @brief access specified object element

    Returns a reference to the element at with specified key @a key.

    @note If @a key is not found in the object, then it is silently added to
    the object and filled with a `null` value to make `key` a valid reference.
    In case the value was `null` before, it is converted to an object.

    @param[in] key  key of the element to access

    @return reference to the element at key @a key

    @throw std::domain_error if JSON is not an object or null; example:
    `"cannot use operator[] with string"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read and
    written using the `[]` operator.,operatorarray__key_type}

    @sa @ref at(const typename object_t::key_type&) for access by reference
    with range checking
    @sa @ref value() for access by value with a default value

    @since version 1.0.0
    */
    template<typename T, std::size_t n>
    reference operator[](T * (&key)[n])
    {
        return operator[](static_cast<const T>(key));
    }

    /*!
    @brief read-only access specified object element

    Returns a const reference to the element at with specified key @a key. No
    bounds checking is performed.

    @warning If the element with key @a key does not exist, the behavior is
    undefined.

    @note This function is required for compatibility reasons with Clang.

    @param[in] key  key of the element to access

    @return const reference to the element at key @a key

    @throw std::domain_error if JSON is not an object; example: `"cannot use
    operator[] with null"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read using
    the `[]` operator.,operatorarray__key_type_const}

    @sa @ref at(const typename object_t::key_type&) for access by reference
    with range checking
    @sa @ref value() for access by value with a default value

    @since version 1.0.0
    */
    template<typename T, std::size_t n>
    const_reference operator[](T * (&key)[n]) const
    {
        return operator[](static_cast<const T>(key));
    }

    /*!
    @brief access specified object element

    Returns a reference to the element at with specified key @a key.

    @note If @a key is not found in the object, then it is silently added to
    the object and filled with a `null` value to make `key` a valid reference.
    In case the value was `null` before, it is converted to an object.

    @param[in] key  key of the element to access

    @return reference to the element at key @a key

    @throw std::domain_error if JSON is not an object or null; example:
    `"cannot use operator[] with string"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read and
    written using the `[]` operator.,operatorarray__key_type}

    @sa @ref at(const typename object_t::key_type&) for access by reference
    with range checking
    @sa @ref value() for access by value with a default value

    @since version 1.1.0
    */
    template<typename T>
    reference operator[](T* key)
    {
        // implicitly convert null to object
        if (is_null())
        {
            m_type = value_t::object;
            m_value = value_t::object;
            assert_invariant();
        }

        // at only works for objects
        if (is_object())
        {
            return m_value.object->operator[](key);
        }

        JSON_THROW(std::domain_error("cannot use operator[] with " + type_name()));
    }

    /*!
    @brief read-only access specified object element

    Returns a const reference to the element at with specified key @a key. No
    bounds checking is performed.

    @warning If the element with key @a key does not exist, the behavior is
    undefined.

    @param[in] key  key of the element to access

    @return const reference to the element at key @a key

    @pre The element with key @a key must exist. **This precondition is
         enforced with an assertion.**

    @throw std::domain_error if JSON is not an object; example: `"cannot use
    operator[] with null"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be read using
    the `[]` operator.,operatorarray__key_type_const}

    @sa @ref at(const typename object_t::key_type&) for access by reference
    with range checking
    @sa @ref value() for access by value with a default value

    @since version 1.1.0
    */
    template<typename T>
    const_reference operator[](T* key) const
    {
        // at only works for objects
        if (is_object())
        {
            assert(m_value.object->find(key) != m_value.object->end());
            return m_value.object->find(key)->second;
        }

        JSON_THROW(std::domain_error("cannot use operator[] with " + type_name()));
    }

    /*!
    @brief access specified object element with default value

    Returns either a copy of an object's element at the specified key @a key
    or a given default value if no element with key @a key exists.

    The function is basically equivalent to executing
    @code {.cpp}
    try {
        return at(key);
    } catch(std::out_of_range) {
        return default_value;
    }
    @endcode

    @note Unlike @ref at(const typename object_t::key_type&), this function
    does not throw if the given key @a key was not found.

    @note Unlike @ref operator[](const typename object_t::key_type& key), this
    function does not implicitly add an element to the position defined by @a
    key. This function is furthermore also applicable to const objects.

    @param[in] key  key of the element to access
    @param[in] default_value  the value to return if @a key is not found

    @tparam ValueType type compatible to JSON values, for instance `int` for
    JSON integer numbers, `bool` for JSON booleans, or `std::vector` types for
    JSON arrays. Note the type of the expected value at @a key and the default
    value @a default_value must be compatible.

    @return copy of the element at key @a key or @a default_value if @a key
    is not found

    @throw std::domain_error if JSON is not an object; example: `"cannot use
    value() with null"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be queried
    with a default value.,basic_json__value}

    @sa @ref at(const typename object_t::key_type&) for access by reference
    with range checking
    @sa @ref operator[](const typename object_t::key_type&) for unchecked
    access by reference

    @since version 1.0.0
    */
    template<class ValueType, typename std::enable_if<
                 std::is_convertible<basic_json_t, ValueType>::value, int>::type = 0>
    ValueType value(const typename object_t::key_type& key, ValueType default_value) const
    {
        // at only works for objects
        if (is_object())
        {
            // if key is found, return value and given default value otherwise
            const auto it = find(key);
            if (it != end())
            {
                return *it;
            }

            return default_value;
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use value() with " + type_name()));
        }
    }

    /*!
    @brief overload for a default value of type const char*
    @copydoc basic_json::value(const typename object_t::key_type&, ValueType) const
    */
    string_t value(const typename object_t::key_type& key, const char* default_value) const
    {
        return value(key, string_t(default_value));
    }

    /*!
    @brief access specified object element via JSON Pointer with default value

    Returns either a copy of an object's element at the specified key @a key
    or a given default value if no element with key @a key exists.

    The function is basically equivalent to executing
    @code {.cpp}
    try {
        return at(ptr);
    } catch(std::out_of_range) {
        return default_value;
    }
    @endcode

    @note Unlike @ref at(const json_pointer&), this function does not throw
    if the given key @a key was not found.

    @param[in] ptr  a JSON pointer to the element to access
    @param[in] default_value  the value to return if @a ptr found no value

    @tparam ValueType type compatible to JSON values, for instance `int` for
    JSON integer numbers, `bool` for JSON booleans, or `std::vector` types for
    JSON arrays. Note the type of the expected value at @a key and the default
    value @a default_value must be compatible.

    @return copy of the element at key @a key or @a default_value if @a key
    is not found

    @throw std::domain_error if JSON is not an object; example: `"cannot use
    value() with null"`

    @complexity Logarithmic in the size of the container.

    @liveexample{The example below shows how object elements can be queried
    with a default value.,basic_json__value_ptr}

    @sa @ref operator[](const json_pointer&) for unchecked access by reference

    @since version 2.0.2
    */
    template<class ValueType, typename std::enable_if<
                 std::is_convertible<basic_json_t, ValueType>::value, int>::type = 0>
    ValueType value(const json_pointer& ptr, ValueType default_value) const
    {
        // at only works for objects
        if (is_object())
        {
            // if pointer resolves a value, return it or use default value
            JSON_TRY
            {
                return ptr.get_checked(this);
            }
            JSON_CATCH (std::out_of_range&)
            {
                return default_value;
            }
        }

        JSON_THROW(std::domain_error("cannot use value() with " + type_name()));
    }

    /*!
    @brief overload for a default value of type const char*
    @copydoc basic_json::value(const json_pointer&, ValueType) const
    */
    string_t value(const json_pointer& ptr, const char* default_value) const
    {
        return value(ptr, string_t(default_value));
    }

    /*!
    @brief access the first element

    Returns a reference to the first element in the container. For a JSON
    container `c`, the expression `c.front()` is equivalent to `*c.begin()`.

    @return In case of a structured type (array or object), a reference to the
    first element is returned. In case of number, string, or boolean values, a
    reference to the value is returned.

    @complexity Constant.

    @pre The JSON value must not be `null` (would throw `std::out_of_range`)
    or an empty array or object (undefined behavior, **guarded by
    assertions**).
    @post The JSON value remains unchanged.

    @throw std::out_of_range when called on `null` value

    @liveexample{The following code shows an example for `front()`.,front}

    @sa @ref back() -- access the last element

    @since version 1.0.0
    */
    reference front()
    {
        return *begin();
    }

    /*!
    @copydoc basic_json::front()
    */
    const_reference front() const
    {
        return *cbegin();
    }

    /*!
    @brief access the last element

    Returns a reference to the last element in the container. For a JSON
    container `c`, the expression `c.back()` is equivalent to
    @code {.cpp}
    auto tmp = c.end();
    --tmp;
    return *tmp;
    @endcode

    @return In case of a structured type (array or object), a reference to the
    last element is returned. In case of number, string, or boolean values, a
    reference to the value is returned.

    @complexity Constant.

    @pre The JSON value must not be `null` (would throw `std::out_of_range`)
    or an empty array or object (undefined behavior, **guarded by
    assertions**).
    @post The JSON value remains unchanged.

    @throw std::out_of_range when called on `null` value.

    @liveexample{The following code shows an example for `back()`.,back}

    @sa @ref front() -- access the first element

    @since version 1.0.0
    */
    reference back()
    {
        auto tmp = end();
        --tmp;
        return *tmp;
    }

    /*!
    @copydoc basic_json::back()
    */
    const_reference back() const
    {
        auto tmp = cend();
        --tmp;
        return *tmp;
    }

    /*!
    @brief remove element given an iterator

    Removes the element specified by iterator @a pos. The iterator @a pos must
    be valid and dereferenceable. Thus the `end()` iterator (which is valid,
    but is not dereferenceable) cannot be used as a value for @a pos.

    If called on a primitive type other than `null`, the resulting JSON value
    will be `null`.

    @param[in] pos iterator to the element to remove
    @return Iterator following the last removed element. If the iterator @a
    pos refers to the last element, the `end()` iterator is returned.

    @tparam IteratorType an @ref iterator or @ref const_iterator

    @post Invalidates iterators and references at or after the point of the
    erase, including the `end()` iterator.

    @throw std::domain_error if called on a `null` value; example: `"cannot
    use erase() with null"`
    @throw std::domain_error if called on an iterator which does not belong to
    the current JSON value; example: `"iterator does not fit current value"`
    @throw std::out_of_range if called on a primitive type with invalid
    iterator (i.e., any iterator which is not `begin()`); example: `"iterator
    out of range"`

    @complexity The complexity depends on the type:
    - objects: amortized constant
    - arrays: linear in distance between @a pos and the end of the container
    - strings: linear in the length of the string
    - other types: constant

    @liveexample{The example shows the result of `erase()` for different JSON
    types.,erase__IteratorType}

    @sa @ref erase(IteratorType, IteratorType) -- removes the elements in
    the given range
    @sa @ref erase(const typename object_t::key_type&) -- removes the element
    from an object at the given key
    @sa @ref erase(const size_type) -- removes the element from an array at
    the given index

    @since version 1.0.0
    */
    template<class IteratorType, typename std::enable_if<
                 std::is_same<IteratorType, typename basic_json_t::iterator>::value or
                 std::is_same<IteratorType, typename basic_json_t::const_iterator>::value, int>::type
             = 0>
    IteratorType erase(IteratorType pos)
    {
        // make sure iterator fits the current value
        if (this != pos.m_object)
        {
            JSON_THROW(std::domain_error("iterator does not fit current value"));
        }

        IteratorType result = end();

        switch (m_type)
        {
            case value_t::boolean:
            case value_t::number_float:
            case value_t::number_integer:
            case value_t::number_unsigned:
            case value_t::string:
            {
                if (not pos.m_it.primitive_iterator.is_begin())
                {
                    JSON_THROW(std::out_of_range("iterator out of range"));
                }

                if (is_string())
                {
                    AllocatorType<string_t> alloc;
                    alloc.destroy(m_value.string);
                    alloc.deallocate(m_value.string, 1);
                    m_value.string = nullptr;
                }

                m_type = value_t::null;
                assert_invariant();
                break;
            }

            case value_t::object:
            {
                result.m_it.object_iterator = m_value.object->erase(pos.m_it.object_iterator);
                break;
            }

            case value_t::array:
            {
                result.m_it.array_iterator = m_value.array->erase(pos.m_it.array_iterator);
                break;
            }

            default:
            {
                JSON_THROW(std::domain_error("cannot use erase() with " + type_name()));
            }
        }

        return result;
    }

    /*!
    @brief remove elements given an iterator range

    Removes the element specified by the range `[first; last)`. The iterator
    @a first does not need to be dereferenceable if `first == last`: erasing
    an empty range is a no-op.

    If called on a primitive type other than `null`, the resulting JSON value
    will be `null`.

    @param[in] first iterator to the beginning of the range to remove
    @param[in] last iterator past the end of the range to remove
    @return Iterator following the last removed element. If the iterator @a
    second refers to the last element, the `end()` iterator is returned.

    @tparam IteratorType an @ref iterator or @ref const_iterator

    @post Invalidates iterators and references at or after the point of the
    erase, including the `end()` iterator.

    @throw std::domain_error if called on a `null` value; example: `"cannot
    use erase() with null"`
    @throw std::domain_error if called on iterators which does not belong to
    the current JSON value; example: `"iterators do not fit current value"`
    @throw std::out_of_range if called on a primitive type with invalid
    iterators (i.e., if `first != begin()` and `last != end()`); example:
    `"iterators out of range"`

    @complexity The complexity depends on the type:
    - objects: `log(size()) + std::distance(first, last)`
    - arrays: linear in the distance between @a first and @a last, plus linear
      in the distance between @a last and end of the container
    - strings: linear in the length of the string
    - other types: constant

    @liveexample{The example shows the result of `erase()` for different JSON
    types.,erase__IteratorType_IteratorType}

    @sa @ref erase(IteratorType) -- removes the element at a given position
    @sa @ref erase(const typename object_t::key_type&) -- removes the element
    from an object at the given key
    @sa @ref erase(const size_type) -- removes the element from an array at
    the given index

    @since version 1.0.0
    */
    template<class IteratorType, typename std::enable_if<
                 std::is_same<IteratorType, typename basic_json_t::iterator>::value or
                 std::is_same<IteratorType, typename basic_json_t::const_iterator>::value, int>::type
             = 0>
    IteratorType erase(IteratorType first, IteratorType last)
    {
        // make sure iterator fits the current value
        if (this != first.m_object or this != last.m_object)
        {
            JSON_THROW(std::domain_error("iterators do not fit current value"));
        }

        IteratorType result = end();

        switch (m_type)
        {
            case value_t::boolean:
            case value_t::number_float:
            case value_t::number_integer:
            case value_t::number_unsigned:
            case value_t::string:
            {
                if (not first.m_it.primitive_iterator.is_begin() or not last.m_it.primitive_iterator.is_end())
                {
                    JSON_THROW(std::out_of_range("iterators out of range"));
                }

                if (is_string())
                {
                    AllocatorType<string_t> alloc;
                    alloc.destroy(m_value.string);
                    alloc.deallocate(m_value.string, 1);
                    m_value.string = nullptr;
                }

                m_type = value_t::null;
                assert_invariant();
                break;
            }

            case value_t::object:
            {
                result.m_it.object_iterator = m_value.object->erase(first.m_it.object_iterator,
                                              last.m_it.object_iterator);
                break;
            }

            case value_t::array:
            {
                result.m_it.array_iterator = m_value.array->erase(first.m_it.array_iterator,
                                             last.m_it.array_iterator);
                break;
            }

            default:
            {
                JSON_THROW(std::domain_error("cannot use erase() with " + type_name()));
            }
        }

        return result;
    }

    /*!
    @brief remove element from a JSON object given a key

    Removes elements from a JSON object with the key value @a key.

    @param[in] key value of the elements to remove

    @return Number of elements removed. If @a ObjectType is the default
    `std::map` type, the return value will always be `0` (@a key was not
    found) or `1` (@a key was found).

    @post References and iterators to the erased elements are invalidated.
    Other references and iterators are not affected.

    @throw std::domain_error when called on a type other than JSON object;
    example: `"cannot use erase() with null"`

    @complexity `log(size()) + count(key)`

    @liveexample{The example shows the effect of `erase()`.,erase__key_type}

    @sa @ref erase(IteratorType) -- removes the element at a given position
    @sa @ref erase(IteratorType, IteratorType) -- removes the elements in
    the given range
    @sa @ref erase(const size_type) -- removes the element from an array at
    the given index

    @since version 1.0.0
    */
    size_type erase(const typename object_t::key_type& key)
    {
        // this erase only works for objects
        if (is_object())
        {
            return m_value.object->erase(key);
        }

        JSON_THROW(std::domain_error("cannot use erase() with " + type_name()));
    }

    /*!
    @brief remove element from a JSON array given an index

    Removes element from a JSON array at the index @a idx.

    @param[in] idx index of the element to remove

    @throw std::domain_error when called on a type other than JSON array;
    example: `"cannot use erase() with null"`
    @throw std::out_of_range when `idx >= size()`; example: `"array index 17
    is out of range"`

    @complexity Linear in distance between @a idx and the end of the container.

    @liveexample{The example shows the effect of `erase()`.,erase__size_type}

    @sa @ref erase(IteratorType) -- removes the element at a given position
    @sa @ref erase(IteratorType, IteratorType) -- removes the elements in
    the given range
    @sa @ref erase(const typename object_t::key_type&) -- removes the element
    from an object at the given key

    @since version 1.0.0
    */
    void erase(const size_type idx)
    {
        // this erase only works for arrays
        if (is_array())
        {
            if (idx >= size())
            {
                JSON_THROW(std::out_of_range("array index " + std::to_string(idx) + " is out of range"));
            }

            m_value.array->erase(m_value.array->begin() + static_cast<difference_type>(idx));
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use erase() with " + type_name()));
        }
    }

    /// @}


    ////////////
    // lookup //
    ////////////

    /// @name lookup
    /// @{

    /*!
    @brief find an element in a JSON object

    Finds an element in a JSON object with key equivalent to @a key. If the
    element is not found or the JSON value is not an object, end() is
    returned.

    @note This method always returns @ref end() when executed on a JSON type
          that is not an object.

    @param[in] key key value of the element to search for

    @return Iterator to an element with key equivalent to @a key. If no such
    element is found or the JSON value is not an object, past-the-end (see
    @ref end()) iterator is returned.

    @complexity Logarithmic in the size of the JSON object.

    @liveexample{The example shows how `find()` is used.,find__key_type}

    @since version 1.0.0
    */
    iterator find(typename object_t::key_type key)
    {
        auto result = end();

        if (is_object())
        {
            result.m_it.object_iterator = m_value.object->find(key);
        }

        return result;
    }

    /*!
    @brief find an element in a JSON object
    @copydoc find(typename object_t::key_type)
    */
    const_iterator find(typename object_t::key_type key) const
    {
        auto result = cend();

        if (is_object())
        {
            result.m_it.object_iterator = m_value.object->find(key);
        }

        return result;
    }

    /*!
    @brief returns the number of occurrences of a key in a JSON object

    Returns the number of elements with key @a key. If ObjectType is the
    default `std::map` type, the return value will always be `0` (@a key was
    not found) or `1` (@a key was found).

    @note This method always returns `0` when executed on a JSON type that is
          not an object.

    @param[in] key key value of the element to count

    @return Number of elements with key @a key. If the JSON value is not an
    object, the return value will be `0`.

    @complexity Logarithmic in the size of the JSON object.

    @liveexample{The example shows how `count()` is used.,count}

    @since version 1.0.0
    */
    size_type count(typename object_t::key_type key) const
    {
        // return 0 for all nonobject types
        return is_object() ? m_value.object->count(key) : 0;
    }

    /// @}


    ///////////////
    // iterators //
    ///////////////

    /// @name iterators
    /// @{

    /*!
    @brief returns an iterator to the first element

    Returns an iterator to the first element.

    @image html range-begin-end.svg "Illustration from cppreference.com"

    @return iterator to the first element

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is constant.

    @liveexample{The following code shows an example for `begin()`.,begin}

    @sa @ref cbegin() -- returns a const iterator to the beginning
    @sa @ref end() -- returns an iterator to the end
    @sa @ref cend() -- returns a const iterator to the end

    @since version 1.0.0
    */
    iterator begin() noexcept
    {
        iterator result(this);
        result.set_begin();
        return result;
    }

    /*!
    @copydoc basic_json::cbegin()
    */
    const_iterator begin() const noexcept
    {
        return cbegin();
    }

    /*!
    @brief returns a const iterator to the first element

    Returns a const iterator to the first element.

    @image html range-begin-end.svg "Illustration from cppreference.com"

    @return const iterator to the first element

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is constant.
    - Has the semantics of `const_cast<const basic_json&>(*this).begin()`.

    @liveexample{The following code shows an example for `cbegin()`.,cbegin}

    @sa @ref begin() -- returns an iterator to the beginning
    @sa @ref end() -- returns an iterator to the end
    @sa @ref cend() -- returns a const iterator to the end

    @since version 1.0.0
    */
    const_iterator cbegin() const noexcept
    {
        const_iterator result(this);
        result.set_begin();
        return result;
    }

    /*!
    @brief returns an iterator to one past the last element

    Returns an iterator to one past the last element.

    @image html range-begin-end.svg "Illustration from cppreference.com"

    @return iterator one past the last element

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is constant.

    @liveexample{The following code shows an example for `end()`.,end}

    @sa @ref cend() -- returns a const iterator to the end
    @sa @ref begin() -- returns an iterator to the beginning
    @sa @ref cbegin() -- returns a const iterator to the beginning

    @since version 1.0.0
    */
    iterator end() noexcept
    {
        iterator result(this);
        result.set_end();
        return result;
    }

    /*!
    @copydoc basic_json::cend()
    */
    const_iterator end() const noexcept
    {
        return cend();
    }

    /*!
    @brief returns a const iterator to one past the last element

    Returns a const iterator to one past the last element.

    @image html range-begin-end.svg "Illustration from cppreference.com"

    @return const iterator one past the last element

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is constant.
    - Has the semantics of `const_cast<const basic_json&>(*this).end()`.

    @liveexample{The following code shows an example for `cend()`.,cend}

    @sa @ref end() -- returns an iterator to the end
    @sa @ref begin() -- returns an iterator to the beginning
    @sa @ref cbegin() -- returns a const iterator to the beginning

    @since version 1.0.0
    */
    const_iterator cend() const noexcept
    {
        const_iterator result(this);
        result.set_end();
        return result;
    }

    /*!
    @brief returns an iterator to the reverse-beginning

    Returns an iterator to the reverse-beginning; that is, the last element.

    @image html range-rbegin-rend.svg "Illustration from cppreference.com"

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [ReversibleContainer](http://en.cppreference.com/w/cpp/concept/ReversibleContainer)
    requirements:
    - The complexity is constant.
    - Has the semantics of `reverse_iterator(end())`.

    @liveexample{The following code shows an example for `rbegin()`.,rbegin}

    @sa @ref crbegin() -- returns a const reverse iterator to the beginning
    @sa @ref rend() -- returns a reverse iterator to the end
    @sa @ref crend() -- returns a const reverse iterator to the end

    @since version 1.0.0
    */
    reverse_iterator rbegin() noexcept
    {
        return reverse_iterator(end());
    }

    /*!
    @copydoc basic_json::crbegin()
    */
    const_reverse_iterator rbegin() const noexcept
    {
        return crbegin();
    }

    /*!
    @brief returns an iterator to the reverse-end

    Returns an iterator to the reverse-end; that is, one before the first
    element.

    @image html range-rbegin-rend.svg "Illustration from cppreference.com"

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [ReversibleContainer](http://en.cppreference.com/w/cpp/concept/ReversibleContainer)
    requirements:
    - The complexity is constant.
    - Has the semantics of `reverse_iterator(begin())`.

    @liveexample{The following code shows an example for `rend()`.,rend}

    @sa @ref crend() -- returns a const reverse iterator to the end
    @sa @ref rbegin() -- returns a reverse iterator to the beginning
    @sa @ref crbegin() -- returns a const reverse iterator to the beginning

    @since version 1.0.0
    */
    reverse_iterator rend() noexcept
    {
        return reverse_iterator(begin());
    }

    /*!
    @copydoc basic_json::crend()
    */
    const_reverse_iterator rend() const noexcept
    {
        return crend();
    }

    /*!
    @brief returns a const reverse iterator to the last element

    Returns a const iterator to the reverse-beginning; that is, the last
    element.

    @image html range-rbegin-rend.svg "Illustration from cppreference.com"

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [ReversibleContainer](http://en.cppreference.com/w/cpp/concept/ReversibleContainer)
    requirements:
    - The complexity is constant.
    - Has the semantics of `const_cast<const basic_json&>(*this).rbegin()`.

    @liveexample{The following code shows an example for `crbegin()`.,crbegin}

    @sa @ref rbegin() -- returns a reverse iterator to the beginning
    @sa @ref rend() -- returns a reverse iterator to the end
    @sa @ref crend() -- returns a const reverse iterator to the end

    @since version 1.0.0
    */
    const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator(cend());
    }

    /*!
    @brief returns a const reverse iterator to one before the first

    Returns a const reverse iterator to the reverse-end; that is, one before
    the first element.

    @image html range-rbegin-rend.svg "Illustration from cppreference.com"

    @complexity Constant.

    @requirement This function helps `basic_json` satisfying the
    [ReversibleContainer](http://en.cppreference.com/w/cpp/concept/ReversibleContainer)
    requirements:
    - The complexity is constant.
    - Has the semantics of `const_cast<const basic_json&>(*this).rend()`.

    @liveexample{The following code shows an example for `crend()`.,crend}

    @sa @ref rend() -- returns a reverse iterator to the end
    @sa @ref rbegin() -- returns a reverse iterator to the beginning
    @sa @ref crbegin() -- returns a const reverse iterator to the beginning

    @since version 1.0.0
    */
    const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator(cbegin());
    }

  private:
    // forward declaration
    template<typename IteratorType> class iteration_proxy;

  public:
    /*!
    @brief wrapper to access iterator member functions in range-based for

    This function allows to access @ref iterator::key() and @ref
    iterator::value() during range-based for loops. In these loops, a
    reference to the JSON values is returned, so there is no access to the
    underlying iterator.

    @note The name of this function is not yet final and may change in the
    future.
    */
    static iteration_proxy<iterator> iterator_wrapper(reference cont)
    {
        return iteration_proxy<iterator>(cont);
    }

    /*!
    @copydoc iterator_wrapper(reference)
    */
    static iteration_proxy<const_iterator> iterator_wrapper(const_reference cont)
    {
        return iteration_proxy<const_iterator>(cont);
    }

    /// @}


    //////////////
    // capacity //
    //////////////

    /// @name capacity
    /// @{

    /*!
    @brief checks whether the container is empty

    Checks if a JSON value has no elements.

    @return The return value depends on the different types and is
            defined as follows:
            Value type  | return value
            ----------- | -------------
            null        | `true`
            boolean     | `false`
            string      | `false`
            number      | `false`
            object      | result of function `object_t::empty()`
            array       | result of function `array_t::empty()`

    @note This function does not return whether a string stored as JSON value
    is empty - it returns whether the JSON container itself is empty which is
    false in the case of a string.

    @complexity Constant, as long as @ref array_t and @ref object_t satisfy
    the Container concept; that is, their `empty()` functions have constant
    complexity.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is constant.
    - Has the semantics of `begin() == end()`.

    @liveexample{The following code uses `empty()` to check if a JSON
    object contains any elements.,empty}

    @sa @ref size() -- returns the number of elements

    @since version 1.0.0
    */
    bool empty() const noexcept
    {
        switch (m_type)
        {
            case value_t::null:
            {
                // null values are empty
                return true;
            }

            case value_t::array:
            {
                // delegate call to array_t::empty()
                return m_value.array->empty();
            }

            case value_t::object:
            {
                // delegate call to object_t::empty()
                return m_value.object->empty();
            }

            default:
            {
                // all other types are nonempty
                return false;
            }
        }
    }

    /*!
    @brief returns the number of elements

    Returns the number of elements in a JSON value.

    @return The return value depends on the different types and is
            defined as follows:
            Value type  | return value
            ----------- | -------------
            null        | `0`
            boolean     | `1`
            string      | `1`
            number      | `1`
            object      | result of function object_t::size()
            array       | result of function array_t::size()

    @note This function does not return the length of a string stored as JSON
    value - it returns the number of elements in the JSON value which is 1 in
    the case of a string.

    @complexity Constant, as long as @ref array_t and @ref object_t satisfy
    the Container concept; that is, their size() functions have constant
    complexity.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is constant.
    - Has the semantics of `std::distance(begin(), end())`.

    @liveexample{The following code calls `size()` on the different value
    types.,size}

    @sa @ref empty() -- checks whether the container is empty
    @sa @ref max_size() -- returns the maximal number of elements

    @since version 1.0.0
    */
    size_type size() const noexcept
    {
        switch (m_type)
        {
            case value_t::null:
            {
                // null values are empty
                return 0;
            }

            case value_t::array:
            {
                // delegate call to array_t::size()
                return m_value.array->size();
            }

            case value_t::object:
            {
                // delegate call to object_t::size()
                return m_value.object->size();
            }

            default:
            {
                // all other types have size 1
                return 1;
            }
        }
    }

    /*!
    @brief returns the maximum possible number of elements

    Returns the maximum number of elements a JSON value is able to hold due to
    system or library implementation limitations, i.e. `std::distance(begin(),
    end())` for the JSON value.

    @return The return value depends on the different types and is
            defined as follows:
            Value type  | return value
            ----------- | -------------
            null        | `0` (same as `size()`)
            boolean     | `1` (same as `size()`)
            string      | `1` (same as `size()`)
            number      | `1` (same as `size()`)
            object      | result of function `object_t::max_size()`
            array       | result of function `array_t::max_size()`

    @complexity Constant, as long as @ref array_t and @ref object_t satisfy
    the Container concept; that is, their `max_size()` functions have constant
    complexity.

    @requirement This function helps `basic_json` satisfying the
    [Container](http://en.cppreference.com/w/cpp/concept/Container)
    requirements:
    - The complexity is constant.
    - Has the semantics of returning `b.size()` where `b` is the largest
      possible JSON value.

    @liveexample{The following code calls `max_size()` on the different value
    types. Note the output is implementation specific.,max_size}

    @sa @ref size() -- returns the number of elements

    @since version 1.0.0
    */
    size_type max_size() const noexcept
    {
        switch (m_type)
        {
            case value_t::array:
            {
                // delegate call to array_t::max_size()
                return m_value.array->max_size();
            }

            case value_t::object:
            {
                // delegate call to object_t::max_size()
                return m_value.object->max_size();
            }

            default:
            {
                // all other types have max_size() == size()
                return size();
            }
        }
    }

    /// @}


    ///////////////
    // modifiers //
    ///////////////

    /// @name modifiers
    /// @{

    /*!
    @brief clears the contents

    Clears the content of a JSON value and resets it to the default value as
    if @ref basic_json(value_t) would have been called:

    Value type  | initial value
    ----------- | -------------
    null        | `null`
    boolean     | `false`
    string      | `""`
    number      | `0`
    object      | `{}`
    array       | `[]`

    @complexity Linear in the size of the JSON value.

    @liveexample{The example below shows the effect of `clear()` to different
    JSON types.,clear}

    @since version 1.0.0
    */
    void clear() noexcept
    {
        switch (m_type)
        {
            case value_t::number_integer:
            {
                m_value.number_integer = 0;
                break;
            }

            case value_t::number_unsigned:
            {
                m_value.number_unsigned = 0;
                break;
            }

            case value_t::number_float:
            {
                m_value.number_float = 0.0;
                break;
            }

            case value_t::boolean:
            {
                m_value.boolean = false;
                break;
            }

            case value_t::string:
            {
                m_value.string->clear();
                break;
            }

            case value_t::array:
            {
                m_value.array->clear();
                break;
            }

            case value_t::object:
            {
                m_value.object->clear();
                break;
            }

            default:
            {
                break;
            }
        }
    }

    /*!
    @brief add an object to an array

    Appends the given element @a val to the end of the JSON value. If the
    function is called on a JSON null value, an empty array is created before
    appending @a val.

    @param[in] val the value to add to the JSON array

    @throw std::domain_error when called on a type other than JSON array or
    null; example: `"cannot use push_back() with number"`

    @complexity Amortized constant.

    @liveexample{The example shows how `push_back()` and `+=` can be used to
    add elements to a JSON array. Note how the `null` value was silently
    converted to a JSON array.,push_back}

    @since version 1.0.0
    */
    void push_back(basic_json&& val)
    {
        // push_back only works for null objects or arrays
        if (not(is_null() or is_array()))
        {
            JSON_THROW(std::domain_error("cannot use push_back() with " + type_name()));
        }

        // transform null object into an array
        if (is_null())
        {
            m_type = value_t::array;
            m_value = value_t::array;
            assert_invariant();
        }

        // add element to array (move semantics)
        m_value.array->push_back(std::move(val));
        // invalidate object
        val.m_type = value_t::null;
    }

    /*!
    @brief add an object to an array
    @copydoc push_back(basic_json&&)
    */
    reference operator+=(basic_json&& val)
    {
        push_back(std::move(val));
        return *this;
    }

    /*!
    @brief add an object to an array
    @copydoc push_back(basic_json&&)
    */
    void push_back(const basic_json& val)
    {
        // push_back only works for null objects or arrays
        if (not(is_null() or is_array()))
        {
            JSON_THROW(std::domain_error("cannot use push_back() with " + type_name()));
        }

        // transform null object into an array
        if (is_null())
        {
            m_type = value_t::array;
            m_value = value_t::array;
            assert_invariant();
        }

        // add element to array
        m_value.array->push_back(val);
    }

    /*!
    @brief add an object to an array
    @copydoc push_back(basic_json&&)
    */
    reference operator+=(const basic_json& val)
    {
        push_back(val);
        return *this;
    }

    /*!
    @brief add an object to an object

    Inserts the given element @a val to the JSON object. If the function is
    called on a JSON null value, an empty object is created before inserting
    @a val.

    @param[in] val the value to add to the JSON object

    @throw std::domain_error when called on a type other than JSON object or
    null; example: `"cannot use push_back() with number"`

    @complexity Logarithmic in the size of the container, O(log(`size()`)).

    @liveexample{The example shows how `push_back()` and `+=` can be used to
    add elements to a JSON object. Note how the `null` value was silently
    converted to a JSON object.,push_back__object_t__value}

    @since version 1.0.0
    */
    void push_back(const typename object_t::value_type& val)
    {
        // push_back only works for null objects or objects
        if (not(is_null() or is_object()))
        {
            JSON_THROW(std::domain_error("cannot use push_back() with " + type_name()));
        }

        // transform null object into an object
        if (is_null())
        {
            m_type = value_t::object;
            m_value = value_t::object;
            assert_invariant();
        }

        // add element to array
        m_value.object->insert(val);
    }

    /*!
    @brief add an object to an object
    @copydoc push_back(const typename object_t::value_type&)
    */
    reference operator+=(const typename object_t::value_type& val)
    {
        push_back(val);
        return *this;
    }

    /*!
    @brief add an object to an object

    This function allows to use `push_back` with an initializer list. In case

    1. the current value is an object,
    2. the initializer list @a init contains only two elements, and
    3. the first element of @a init is a string,

    @a init is converted into an object element and added using
    @ref push_back(const typename object_t::value_type&). Otherwise, @a init
    is converted to a JSON value and added using @ref push_back(basic_json&&).

    @param init  an initializer list

    @complexity Linear in the size of the initializer list @a init.

    @note This function is required to resolve an ambiguous overload error,
          because pairs like `{"key", "value"}` can be both interpreted as
          `object_t::value_type` or `std::initializer_list<basic_json>`, see
          https://github.com/nlohmann/json/issues/235 for more information.

    @liveexample{The example shows how initializer lists are treated as
    objects when possible.,push_back__initializer_list}
    */
    void push_back(std::initializer_list<basic_json> init)
    {
        if (is_object() and init.size() == 2 and init.begin()->is_string())
        {
            const string_t key = *init.begin();
            push_back(typename object_t::value_type(key, *(init.begin() + 1)));
        }
        else
        {
            push_back(basic_json(init));
        }
    }

    /*!
    @brief add an object to an object
    @copydoc push_back(std::initializer_list<basic_json>)
    */
    reference operator+=(std::initializer_list<basic_json> init)
    {
        push_back(init);
        return *this;
    }

    /*!
    @brief add an object to an array

    Creates a JSON value from the passed parameters @a args to the end of the
    JSON value. If the function is called on a JSON null value, an empty array
    is created before appending the value created from @a args.

    @param[in] args arguments to forward to a constructor of @ref basic_json
    @tparam Args compatible types to create a @ref basic_json object

    @throw std::domain_error when called on a type other than JSON array or
    null; example: `"cannot use emplace_back() with number"`

    @complexity Amortized constant.

    @liveexample{The example shows how `push_back()` can be used to add
    elements to a JSON array. Note how the `null` value was silently converted
    to a JSON array.,emplace_back}

    @since version 2.0.8
    */
    template<class... Args>
    void emplace_back(Args&& ... args)
    {
        // emplace_back only works for null objects or arrays
        if (not(is_null() or is_array()))
        {
            JSON_THROW(std::domain_error("cannot use emplace_back() with " + type_name()));
        }

        // transform null object into an array
        if (is_null())
        {
            m_type = value_t::array;
            m_value = value_t::array;
            assert_invariant();
        }

        // add element to array (perfect forwarding)
        m_value.array->emplace_back(std::forward<Args>(args)...);
    }

    /*!
    @brief add an object to an object if key does not exist

    Inserts a new element into a JSON object constructed in-place with the
    given @a args if there is no element with the key in the container. If the
    function is called on a JSON null value, an empty object is created before
    appending the value created from @a args.

    @param[in] args arguments to forward to a constructor of @ref basic_json
    @tparam Args compatible types to create a @ref basic_json object

    @return a pair consisting of an iterator to the inserted element, or the
            already-existing element if no insertion happened, and a bool
            denoting whether the insertion took place.

    @throw std::domain_error when called on a type other than JSON object or
    null; example: `"cannot use emplace() with number"`

    @complexity Logarithmic in the size of the container, O(log(`size()`)).

    @liveexample{The example shows how `emplace()` can be used to add elements
    to a JSON object. Note how the `null` value was silently converted to a
    JSON object. Further note how no value is added if there was already one
    value stored with the same key.,emplace}

    @since version 2.0.8
    */
    template<class... Args>
    std::pair<iterator, bool> emplace(Args&& ... args)
    {
        // emplace only works for null objects or arrays
        if (not(is_null() or is_object()))
        {
            JSON_THROW(std::domain_error("cannot use emplace() with " + type_name()));
        }

        // transform null object into an object
        if (is_null())
        {
            m_type = value_t::object;
            m_value = value_t::object;
            assert_invariant();
        }

        // add element to array (perfect forwarding)
        auto res = m_value.object->emplace(std::forward<Args>(args)...);
        // create result iterator and set iterator to the result of emplace
        auto it = begin();
        it.m_it.object_iterator = res.first;

        // return pair of iterator and boolean
        return {it, res.second};
    }

    /*!
    @brief inserts element

    Inserts element @a val before iterator @a pos.

    @param[in] pos iterator before which the content will be inserted; may be
    the end() iterator
    @param[in] val element to insert
    @return iterator pointing to the inserted @a val.

    @throw std::domain_error if called on JSON values other than arrays;
    example: `"cannot use insert() with string"`
    @throw std::domain_error if @a pos is not an iterator of *this; example:
    `"iterator does not fit current value"`

    @complexity Constant plus linear in the distance between @a pos and end of
    the container.

    @liveexample{The example shows how `insert()` is used.,insert}

    @since version 1.0.0
    */
    iterator insert(const_iterator pos, const basic_json& val)
    {
        // insert only works for arrays
        if (is_array())
        {
            // check if iterator pos fits to this JSON value
            if (pos.m_object != this)
            {
                JSON_THROW(std::domain_error("iterator does not fit current value"));
            }

            // insert to array and return iterator
            iterator result(this);
            result.m_it.array_iterator = m_value.array->insert(pos.m_it.array_iterator, val);
            return result;
        }

        JSON_THROW(std::domain_error("cannot use insert() with " + type_name()));
    }

    /*!
    @brief inserts element
    @copydoc insert(const_iterator, const basic_json&)
    */
    iterator insert(const_iterator pos, basic_json&& val)
    {
        return insert(pos, val);
    }

    /*!
    @brief inserts elements

    Inserts @a cnt copies of @a val before iterator @a pos.

    @param[in] pos iterator before which the content will be inserted; may be
    the end() iterator
    @param[in] cnt number of copies of @a val to insert
    @param[in] val element to insert
    @return iterator pointing to the first element inserted, or @a pos if
    `cnt==0`

    @throw std::domain_error if called on JSON values other than arrays;
    example: `"cannot use insert() with string"`
    @throw std::domain_error if @a pos is not an iterator of *this; example:
    `"iterator does not fit current value"`

    @complexity Linear in @a cnt plus linear in the distance between @a pos
    and end of the container.

    @liveexample{The example shows how `insert()` is used.,insert__count}

    @since version 1.0.0
    */
    iterator insert(const_iterator pos, size_type cnt, const basic_json& val)
    {
        // insert only works for arrays
        if (is_array())
        {
            // check if iterator pos fits to this JSON value
            if (pos.m_object != this)
            {
                JSON_THROW(std::domain_error("iterator does not fit current value"));
            }

            // insert to array and return iterator
            iterator result(this);
            result.m_it.array_iterator = m_value.array->insert(pos.m_it.array_iterator, cnt, val);
            return result;
        }

        JSON_THROW(std::domain_error("cannot use insert() with " + type_name()));
    }

    /*!
    @brief inserts elements

    Inserts elements from range `[first, last)` before iterator @a pos.

    @param[in] pos iterator before which the content will be inserted; may be
    the end() iterator
    @param[in] first begin of the range of elements to insert
    @param[in] last end of the range of elements to insert

    @throw std::domain_error if called on JSON values other than arrays;
    example: `"cannot use insert() with string"`
    @throw std::domain_error if @a pos is not an iterator of *this; example:
    `"iterator does not fit current value"`
    @throw std::domain_error if @a first and @a last do not belong to the same
    JSON value; example: `"iterators do not fit"`
    @throw std::domain_error if @a first or @a last are iterators into
    container for which insert is called; example: `"passed iterators may not
    belong to container"`

    @return iterator pointing to the first element inserted, or @a pos if
    `first==last`

    @complexity Linear in `std::distance(first, last)` plus linear in the
    distance between @a pos and end of the container.

    @liveexample{The example shows how `insert()` is used.,insert__range}

    @since version 1.0.0
    */
    iterator insert(const_iterator pos, const_iterator first, const_iterator last)
    {
        // insert only works for arrays
        if (not is_array())
        {
            JSON_THROW(std::domain_error("cannot use insert() with " + type_name()));
        }

        // check if iterator pos fits to this JSON value
        if (pos.m_object != this)
        {
            JSON_THROW(std::domain_error("iterator does not fit current value"));
        }

        // check if range iterators belong to the same JSON object
        if (first.m_object != last.m_object)
        {
            JSON_THROW(std::domain_error("iterators do not fit"));
        }

        if (first.m_object == this or last.m_object == this)
        {
            JSON_THROW(std::domain_error("passed iterators may not belong to container"));
        }

        // insert to array and return iterator
        iterator result(this);
        result.m_it.array_iterator = m_value.array->insert(
                                         pos.m_it.array_iterator,
                                         first.m_it.array_iterator,
                                         last.m_it.array_iterator);
        return result;
    }

    /*!
    @brief inserts elements

    Inserts elements from initializer list @a ilist before iterator @a pos.

    @param[in] pos iterator before which the content will be inserted; may be
    the end() iterator
    @param[in] ilist initializer list to insert the values from

    @throw std::domain_error if called on JSON values other than arrays;
    example: `"cannot use insert() with string"`
    @throw std::domain_error if @a pos is not an iterator of *this; example:
    `"iterator does not fit current value"`

    @return iterator pointing to the first element inserted, or @a pos if
    `ilist` is empty

    @complexity Linear in `ilist.size()` plus linear in the distance between
    @a pos and end of the container.

    @liveexample{The example shows how `insert()` is used.,insert__ilist}

    @since version 1.0.0
    */
    iterator insert(const_iterator pos, std::initializer_list<basic_json> ilist)
    {
        // insert only works for arrays
        if (not is_array())
        {
            JSON_THROW(std::domain_error("cannot use insert() with " + type_name()));
        }

        // check if iterator pos fits to this JSON value
        if (pos.m_object != this)
        {
            JSON_THROW(std::domain_error("iterator does not fit current value"));
        }

        // insert to array and return iterator
        iterator result(this);
        result.m_it.array_iterator = m_value.array->insert(pos.m_it.array_iterator, ilist);
        return result;
    }

    /*!
    @brief exchanges the values

    Exchanges the contents of the JSON value with those of @a other. Does not
    invoke any move, copy, or swap operations on individual elements. All
    iterators and references remain valid. The past-the-end iterator is
    invalidated.

    @param[in,out] other JSON value to exchange the contents with

    @complexity Constant.

    @liveexample{The example below shows how JSON values can be swapped with
    `swap()`.,swap__reference}

    @since version 1.0.0
    */
    void swap(reference other) noexcept (
        std::is_nothrow_move_constructible<value_t>::value and
        std::is_nothrow_move_assignable<value_t>::value and
        std::is_nothrow_move_constructible<json_value>::value and
        std::is_nothrow_move_assignable<json_value>::value
    )
    {
        std::swap(m_type, other.m_type);
        std::swap(m_value, other.m_value);
        assert_invariant();
    }

    /*!
    @brief exchanges the values

    Exchanges the contents of a JSON array with those of @a other. Does not
    invoke any move, copy, or swap operations on individual elements. All
    iterators and references remain valid. The past-the-end iterator is
    invalidated.

    @param[in,out] other array to exchange the contents with

    @throw std::domain_error when JSON value is not an array; example:
    `"cannot use swap() with string"`

    @complexity Constant.

    @liveexample{The example below shows how arrays can be swapped with
    `swap()`.,swap__array_t}

    @since version 1.0.0
    */
    void swap(array_t& other)
    {
        // swap only works for arrays
        if (is_array())
        {
            std::swap(*(m_value.array), other);
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use swap() with " + type_name()));
        }
    }

    /*!
    @brief exchanges the values

    Exchanges the contents of a JSON object with those of @a other. Does not
    invoke any move, copy, or swap operations on individual elements. All
    iterators and references remain valid. The past-the-end iterator is
    invalidated.

    @param[in,out] other object to exchange the contents with

    @throw std::domain_error when JSON value is not an object; example:
    `"cannot use swap() with string"`

    @complexity Constant.

    @liveexample{The example below shows how objects can be swapped with
    `swap()`.,swap__object_t}

    @since version 1.0.0
    */
    void swap(object_t& other)
    {
        // swap only works for objects
        if (is_object())
        {
            std::swap(*(m_value.object), other);
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use swap() with " + type_name()));
        }
    }

    /*!
    @brief exchanges the values

    Exchanges the contents of a JSON string with those of @a other. Does not
    invoke any move, copy, or swap operations on individual elements. All
    iterators and references remain valid. The past-the-end iterator is
    invalidated.

    @param[in,out] other string to exchange the contents with

    @throw std::domain_error when JSON value is not a string; example: `"cannot
    use swap() with boolean"`

    @complexity Constant.

    @liveexample{The example below shows how strings can be swapped with
    `swap()`.,swap__string_t}

    @since version 1.0.0
    */
    void swap(string_t& other)
    {
        // swap only works for strings
        if (is_string())
        {
            std::swap(*(m_value.string), other);
        }
        else
        {
            JSON_THROW(std::domain_error("cannot use swap() with " + type_name()));
        }
    }

    /// @}

  public:
    //////////////////////////////////////////
    // lexicographical comparison operators //
    //////////////////////////////////////////

    /// @name lexicographical comparison operators
    /// @{

    /*!
    @brief comparison: equal

    Compares two JSON values for equality according to the following rules:
    - Two JSON values are equal if (1) they are from the same type and (2)
      their stored values are the same.
    - Integer and floating-point numbers are automatically converted before
      comparison. Floating-point numbers are compared indirectly: two
      floating-point numbers `f1` and `f2` are considered equal if neither
      `f1 > f2` nor `f2 > f1` holds.
    - Two JSON null values are equal.

    @param[in] lhs  first JSON value to consider
    @param[in] rhs  second JSON value to consider
    @return whether the values @a lhs and @a rhs are equal

    @complexity Linear.

    @liveexample{The example demonstrates comparing several JSON
    types.,operator__equal}

    @since version 1.0.0
    */
    friend bool operator==(const_reference lhs, const_reference rhs) noexcept
    {
        const auto lhs_type = lhs.type();
        const auto rhs_type = rhs.type();

        if (lhs_type == rhs_type)
        {
            switch (lhs_type)
            {
                case value_t::array:
                {
                    return *lhs.m_value.array == *rhs.m_value.array;
                }
                case value_t::object:
                {
                    return *lhs.m_value.object == *rhs.m_value.object;
                }
                case value_t::null:
                {
                    return true;
                }
                case value_t::string:
                {
                    return *lhs.m_value.string == *rhs.m_value.string;
                }
                case value_t::boolean:
                {
                    return lhs.m_value.boolean == rhs.m_value.boolean;
                }
                case value_t::number_integer:
                {
                    return lhs.m_value.number_integer == rhs.m_value.number_integer;
                }
                case value_t::number_unsigned:
                {
                    return lhs.m_value.number_unsigned == rhs.m_value.number_unsigned;
                }
                case value_t::number_float:
                {
                    return lhs.m_value.number_float == rhs.m_value.number_float;
                }
                default:
                {
                    return false;
                }
            }
        }
        else if (lhs_type == value_t::number_integer and rhs_type == value_t::number_float)
        {
            return static_cast<number_float_t>(lhs.m_value.number_integer) == rhs.m_value.number_float;
        }
        else if (lhs_type == value_t::number_float and rhs_type == value_t::number_integer)
        {
            return lhs.m_value.number_float == static_cast<number_float_t>(rhs.m_value.number_integer);
        }
        else if (lhs_type == value_t::number_unsigned and rhs_type == value_t::number_float)
        {
            return static_cast<number_float_t>(lhs.m_value.number_unsigned) == rhs.m_value.number_float;
        }
        else if (lhs_type == value_t::number_float and rhs_type == value_t::number_unsigned)
        {
            return lhs.m_value.number_float == static_cast<number_float_t>(rhs.m_value.number_unsigned);
        }
        else if (lhs_type == value_t::number_unsigned and rhs_type == value_t::number_integer)
        {
            return static_cast<number_integer_t>(lhs.m_value.number_unsigned) == rhs.m_value.number_integer;
        }
        else if (lhs_type == value_t::number_integer and rhs_type == value_t::number_unsigned)
        {
            return lhs.m_value.number_integer == static_cast<number_integer_t>(rhs.m_value.number_unsigned);
        }

        return false;
    }

    /*!
    @brief comparison: equal
    @copydoc operator==(const_reference, const_reference)
    */
    template<typename ScalarType, typename std::enable_if<
                 std::is_scalar<ScalarType>::value, int>::type = 0>
    friend bool operator==(const_reference lhs, const ScalarType rhs) noexcept
    {
        return (lhs == basic_json(rhs));
    }

    /*!
    @brief comparison: equal
    @copydoc operator==(const_reference, const_reference)
    */
    template<typename ScalarType, typename std::enable_if<
                 std::is_scalar<ScalarType>::value, int>::type = 0>
    friend bool operator==(const ScalarType lhs, const_reference rhs) noexcept
    {
        return (basic_json(lhs) == rhs);
    }

    /*!
    @brief comparison: not equal

    Compares two JSON values for inequality by calculating `not (lhs == rhs)`.

    @param[in] lhs  first JSON value to consider
    @param[in] rhs  second JSON value to consider
    @return whether the values @a lhs and @a rhs are not equal

    @complexity Linear.

    @liveexample{The example demonstrates comparing several JSON
    types.,operator__notequal}

    @since version 1.0.0
    */
    friend bool operator!=(const_reference lhs, const_reference rhs) noexcept
    {
        return not (lhs == rhs);
    }

    /*!
    @brief comparison: not equal
    @copydoc operator!=(const_reference, const_reference)
    */
    template<typename ScalarType, typename std::enable_if<
                 std::is_scalar<ScalarType>::value, int>::type = 0>
    friend bool operator!=(const_reference lhs, const ScalarType rhs) noexcept
    {
        return (lhs != basic_json(rhs));
    }

    /*!
    @brief comparison: not equal
    @copydoc operator!=(const_reference, const_reference)
    */
    template<typename ScalarType, typename std::enable_if<
                 std::is_scalar<ScalarType>::value, int>::type = 0>
    friend bool operator!=(const ScalarType lhs, const_reference rhs) noexcept
    {
        return (basic_json(lhs) != rhs);
    }

    /*!
    @brief comparison: less than

    Compares whether one JSON value @a lhs is less than another JSON value @a
    rhs according to the following rules:
    - If @a lhs and @a rhs have the same type, the values are compared using
      the default `<` operator.
    - Integer and floating-point numbers are automatically converted before
      comparison
    - In case @a lhs and @a rhs have different types, the values are ignored
      and the order of the types is considered, see
      @ref operator<(const value_t, const value_t).

    @param[in] lhs  first JSON value to consider
    @param[in] rhs  second JSON value to consider
    @return whether @a lhs is less than @a rhs

    @complexity Linear.

    @liveexample{The example demonstrates comparing several JSON
    types.,operator__less}

    @since version 1.0.0
    */
    friend bool operator<(const_reference lhs, const_reference rhs) noexcept
    {
        const auto lhs_type = lhs.type();
        const auto rhs_type = rhs.type();

        if (lhs_type == rhs_type)
        {
            switch (lhs_type)
            {
                case value_t::array:
                {
                    return *lhs.m_value.array < *rhs.m_value.array;
                }
                case value_t::object:
                {
                    return *lhs.m_value.object < *rhs.m_value.object;
                }
                case value_t::null:
                {
                    return false;
                }
                case value_t::string:
                {
                    return *lhs.m_value.string < *rhs.m_value.string;
                }
                case value_t::boolean:
                {
                    return lhs.m_value.boolean < rhs.m_value.boolean;
                }
                case value_t::number_integer:
                {
                    return lhs.m_value.number_integer < rhs.m_value.number_integer;
                }
                case value_t::number_unsigned:
                {
                    return lhs.m_value.number_unsigned < rhs.m_value.number_unsigned;
                }
                case value_t::number_float:
                {
                    return lhs.m_value.number_float < rhs.m_value.number_float;
                }
                default:
                {
                    return false;
                }
            }
        }
        else if (lhs_type == value_t::number_integer and rhs_type == value_t::number_float)
        {
            return static_cast<number_float_t>(lhs.m_value.number_integer) < rhs.m_value.number_float;
        }
        else if (lhs_type == value_t::number_float and rhs_type == value_t::number_integer)
        {
            return lhs.m_value.number_float < static_cast<number_float_t>(rhs.m_value.number_integer);
        }
        else if (lhs_type == value_t::number_unsigned and rhs_type == value_t::number_float)
        {
            return static_cast<number_float_t>(lhs.m_value.number_unsigned) < rhs.m_value.number_float;
        }
        else if (lhs_type == value_t::number_float and rhs_type == value_t::number_unsigned)
        {
            return lhs.m_value.number_float < static_cast<number_float_t>(rhs.m_value.number_unsigned);
        }
        else if (lhs_type == value_t::number_integer and rhs_type == value_t::number_unsigned)
        {
            return lhs.m_value.number_integer < static_cast<number_integer_t>(rhs.m_value.number_unsigned);
        }
        else if (lhs_type == value_t::number_unsigned and rhs_type == value_t::number_integer)
        {
            return static_cast<number_integer_t>(lhs.m_value.number_unsigned) < rhs.m_value.number_integer;
        }

        // We only reach this line if we cannot compare values. In that case,
        // we compare types. Note we have to call the operator explicitly,
        // because MSVC has problems otherwise.
        return operator<(lhs_type, rhs_type);
    }

    /*!
    @brief comparison: less than or equal

    Compares whether one JSON value @a lhs is less than or equal to another
    JSON value by calculating `not (rhs < lhs)`.

    @param[in] lhs  first JSON value to consider
    @param[in] rhs  second JSON value to consider
    @return whether @a lhs is less than or equal to @a rhs

    @complexity Linear.

    @liveexample{The example demonstrates comparing several JSON
    types.,operator__greater}

    @since version 1.0.0
    */
    friend bool operator<=(const_reference lhs, const_reference rhs) noexcept
    {
        return not (rhs < lhs);
    }

    /*!
    @brief comparison: greater than

    Compares whether one JSON value @a lhs is greater than another
    JSON value by calculating `not (lhs <= rhs)`.

    @param[in] lhs  first JSON value to consider
    @param[in] rhs  second JSON value to consider
    @return whether @a lhs is greater than to @a rhs

    @complexity Linear.

    @liveexample{The example demonstrates comparing several JSON
    types.,operator__lessequal}

    @since version 1.0.0
    */
    friend bool operator>(const_reference lhs, const_reference rhs) noexcept
    {
        return not (lhs <= rhs);
    }

    /*!
    @brief comparison: greater than or equal

    Compares whether one JSON value @a lhs is greater than or equal to another
    JSON value by calculating `not (lhs < rhs)`.

    @param[in] lhs  first JSON value to consider
    @param[in] rhs  second JSON value to consider
    @return whether @a lhs is greater than or equal to @a rhs

    @complexity Linear.

    @liveexample{The example demonstrates comparing several JSON
    types.,operator__greaterequal}

    @since version 1.0.0
    */
    friend bool operator>=(const_reference lhs, const_reference rhs) noexcept
    {
        return not (lhs < rhs);
    }

    /// @}


    ///////////////////
    // serialization //
    ///////////////////

    /// @name serialization
    /// @{

    /*!
    @brief serialize to stream

    Serialize the given JSON value @a j to the output stream @a o. The JSON
    value will be serialized using the @ref dump member function. The
    indentation of the output can be controlled with the member variable
    `width` of the output stream @a o. For instance, using the manipulator
    `std::setw(4)` on @a o sets the indentation level to `4` and the
    serialization result is the same as calling `dump(4)`.

    @param[in,out] o  stream to serialize to
    @param[in] j  JSON value to serialize

    @return the stream @a o

    @complexity Linear.

    @liveexample{The example below shows the serialization with different
    parameters to `width` to adjust the indentation level.,operator_serialize}

    @since version 1.0.0
    */
    friend std::ostream& operator<<(std::ostream& o, const basic_json& j)
    {
        // read width member and use it as indentation parameter if nonzero
        const bool pretty_print = (o.width() > 0);
        const auto indentation = (pretty_print ? o.width() : 0);

        // reset width to 0 for subsequent calls to this stream
        o.width(0);

        // do the actual serialization
        j.dump(o, pretty_print, static_cast<unsigned int>(indentation));

        return o;
    }

    /*!
    @brief serialize to stream
    @copydoc operator<<(std::ostream&, const basic_json&)
    */
    friend std::ostream& operator>>(const basic_json& j, std::ostream& o)
    {
        return o << j;
    }

    /// @}


    /////////////////////
    // deserialization //
    /////////////////////

    /// @name deserialization
    /// @{

    /*!
    @brief deserialize from an array

    This function reads from an array of 1-byte values.

    @pre Each element of the container has a size of 1 byte. Violating this
    precondition yields undefined behavior. **This precondition is enforced
    with a static assertion.**

    @param[in] array  array to read from
    @param[in] cb  a parser callback function of type @ref parser_callback_t
    which is used to control the deserialization by filtering unwanted values
    (optional)

    @return result of the deserialization

    @complexity Linear in the length of the input. The parser is a predictive
    LL(1) parser. The complexity can be higher if the parser callback function
    @a cb has a super-linear complexity.

    @note A UTF-8 byte order mark is silently ignored.

    @liveexample{The example below demonstrates the `parse()` function reading
    from an array.,parse__array__parser_callback_t}

    @since version 2.0.3
    */
    template<class T, std::size_t N>
    static basic_json parse(T (&array)[N],
                            const parser_callback_t cb = nullptr)
    {
        // delegate the call to the iterator-range parse overload
        return parse(std::begin(array), std::end(array), cb);
    }

    /*!
    @brief deserialize from string literal

    @tparam CharT character/literal type with size of 1 byte
    @param[in] s  string literal to read a serialized JSON value from
    @param[in] cb a parser callback function of type @ref parser_callback_t
    which is used to control the deserialization by filtering unwanted values
    (optional)

    @return result of the deserialization

    @complexity Linear in the length of the input. The parser is a predictive
    LL(1) parser. The complexity can be higher if the parser callback function
    @a cb has a super-linear complexity.

    @note A UTF-8 byte order mark is silently ignored.
    @note String containers like `std::string` or @ref string_t can be parsed
          with @ref parse(const ContiguousContainer&, const parser_callback_t)

    @liveexample{The example below demonstrates the `parse()` function with
    and without callback function.,parse__string__parser_callback_t}

    @sa @ref parse(std::istream&, const parser_callback_t) for a version that
    reads from an input stream

    @since version 1.0.0 (originally for @ref string_t)
    */
    template<typename CharT, typename std::enable_if<
                 std::is_pointer<CharT>::value and
                 std::is_integral<typename std::remove_pointer<CharT>::type>::value and
                 sizeof(typename std::remove_pointer<CharT>::type) == 1, int>::type = 0>
    static basic_json parse(const CharT s,
                            const parser_callback_t cb = nullptr)
    {
        return parser(reinterpret_cast<const char*>(s), cb).parse();
    }

    /*!
    @brief deserialize from stream

    @param[in,out] i  stream to read a serialized JSON value from
    @param[in] cb a parser callback function of type @ref parser_callback_t
    which is used to control the deserialization by filtering unwanted values
    (optional)

    @return result of the deserialization

    @complexity Linear in the length of the input. The parser is a predictive
    LL(1) parser. The complexity can be higher if the parser callback function
    @a cb has a super-linear complexity.

    @note A UTF-8 byte order mark is silently ignored.

    @liveexample{The example below demonstrates the `parse()` function with
    and without callback function.,parse__istream__parser_callback_t}

    @sa @ref parse(const CharT, const parser_callback_t) for a version
    that reads from a string

    @since version 1.0.0
    */
    static basic_json parse(std::istream& i,
                            const parser_callback_t cb = nullptr)
    {
        return parser(i, cb).parse();
    }

    /*!
    @copydoc parse(std::istream&, const parser_callback_t)
    */
    static basic_json parse(std::istream&& i,
                            const parser_callback_t cb = nullptr)
    {
        return parser(i, cb).parse();
    }

    /*!
    @brief deserialize from an iterator range with contiguous storage

    This function reads from an iterator range of a container with contiguous
    storage of 1-byte values. Compatible container types include
    `std::vector`, `std::string`, `std::array`, `std::valarray`, and
    `std::initializer_list`. Furthermore, C-style arrays can be used with
    `std::begin()`/`std::end()`. User-defined containers can be used as long
    as they implement random-access iterators and a contiguous storage.

    @pre The iterator range is contiguous. Violating this precondition yields
    undefined behavior. **This precondition is enforced with an assertion.**
    @pre Each element in the range has a size of 1 byte. Violating this
    precondition yields undefined behavior. **This precondition is enforced
    with a static assertion.**

    @warning There is no way to enforce all preconditions at compile-time. If
             the function is called with noncompliant iterators and with
             assertions switched off, the behavior is undefined and will most
             likely yield segmentation violation.

    @tparam IteratorType iterator of container with contiguous storage
    @param[in] first  begin of the range to parse (included)
    @param[in] last  end of the range to parse (excluded)
    @param[in] cb  a parser callback function of type @ref parser_callback_t
    which is used to control the deserialization by filtering unwanted values
    (optional)

    @return result of the deserialization

    @complexity Linear in the length of the input. The parser is a predictive
    LL(1) parser. The complexity can be higher if the parser callback function
    @a cb has a super-linear complexity.

    @note A UTF-8 byte order mark is silently ignored.

    @liveexample{The example below demonstrates the `parse()` function reading
    from an iterator range.,parse__iteratortype__parser_callback_t}

    @since version 2.0.3
    */
    template<class IteratorType, typename std::enable_if<
                 std::is_base_of<
                     std::random_access_iterator_tag,
                     typename std::iterator_traits<IteratorType>::iterator_category>::value, int>::type = 0>
    static basic_json parse(IteratorType first, IteratorType last,
                            const parser_callback_t cb = nullptr)
    {
        // assertion to check that the iterator range is indeed contiguous,
        // see http://stackoverflow.com/a/35008842/266378 for more discussion
        assert(std::accumulate(first, last, std::pair<bool, int>(true, 0),
                               [&first](std::pair<bool, int> res, decltype(*first) val)
        {
            res.first &= (val == *(std::next(std::addressof(*first), res.second++)));
            return res;
        }).first);

        // assertion to check that each element is 1 byte long
        static_assert(sizeof(typename std::iterator_traits<IteratorType>::value_type) == 1,
                      "each element in the iterator range must have the size of 1 byte");

        // if iterator range is empty, create a parser with an empty string
        // to generate "unexpected EOF" error message
        if (std::distance(first, last) <= 0)
        {
            return parser("").parse();
        }

        return parser(first, last, cb).parse();
    }

    /*!
    @brief deserialize from a container with contiguous storage

    This function reads from a container with contiguous storage of 1-byte
    values. Compatible container types include `std::vector`, `std::string`,
    `std::array`, and `std::initializer_list`. User-defined containers can be
    used as long as they implement random-access iterators and a contiguous
    storage.

    @pre The container storage is contiguous. Violating this precondition
    yields undefined behavior. **This precondition is enforced with an
    assertion.**
    @pre Each element of the container has a size of 1 byte. Violating this
    precondition yields undefined behavior. **This precondition is enforced
    with a static assertion.**

    @warning There is no way to enforce all preconditions at compile-time. If
             the function is called with a noncompliant container and with
             assertions switched off, the behavior is undefined and will most
             likely yield segmentation violation.

    @tparam ContiguousContainer container type with contiguous storage
    @param[in] c  container to read from
    @param[in] cb  a parser callback function of type @ref parser_callback_t
    which is used to control the deserialization by filtering unwanted values
    (optional)

    @return result of the deserialization

    @complexity Linear in the length of the input. The parser is a predictive
    LL(1) parser. The complexity can be higher if the parser callback function
    @a cb has a super-linear complexity.

    @note A UTF-8 byte order mark is silently ignored.

    @liveexample{The example below demonstrates the `parse()` function reading
    from a contiguous container.,parse__contiguouscontainer__parser_callback_t}

    @since version 2.0.3
    */
    template<class ContiguousContainer, typename std::enable_if<
                 not std::is_pointer<ContiguousContainer>::value and
                 std::is_base_of<
                     std::random_access_iterator_tag,
                     typename std::iterator_traits<decltype(std::begin(std::declval<ContiguousContainer const>()))>::iterator_category>::value
                 , int>::type = 0>
    static basic_json parse(const ContiguousContainer& c,
                            const parser_callback_t cb = nullptr)
    {
        // delegate the call to the iterator-range parse overload
        return parse(std::begin(c), std::end(c), cb);
    }

    /*!
    @brief deserialize from stream

    Deserializes an input stream to a JSON value.

    @param[in,out] i  input stream to read a serialized JSON value from
    @param[in,out] j  JSON value to write the deserialized input to

    @throw std::invalid_argument in case of parse errors

    @complexity Linear in the length of the input. The parser is a predictive
    LL(1) parser.

    @note A UTF-8 byte order mark is silently ignored.

    @liveexample{The example below shows how a JSON value is constructed by
    reading a serialization from a stream.,operator_deserialize}

    @sa parse(std::istream&, const parser_callback_t) for a variant with a
    parser callback function to filter values while parsing

    @since version 1.0.0
    */
    friend std::istream& operator<<(basic_json& j, std::istream& i)
    {
        j = parser(i).parse();
        return i;
    }

    /*!
    @brief deserialize from stream
    @copydoc operator<<(basic_json&, std::istream&)
    */
    friend std::istream& operator>>(std::istream& i, basic_json& j)
    {
        j = parser(i).parse();
        return i;
    }

    /// @}

    //////////////////////////////////////////
    // binary serialization/deserialization //
    //////////////////////////////////////////

    /// @name binary serialization/deserialization support
    /// @{

  private:
    /*!
    @note Some code in the switch cases has been copied, because otherwise
          copilers would complain about implicit fallthrough and there is no
          portable attribute to mute such warnings.
    */
    template<typename T>
    static void add_to_vector(std::vector<uint8_t>& vec, size_t bytes, const T number)
    {
        assert(bytes == 1 or bytes == 2 or bytes == 4 or bytes == 8);

        switch (bytes)
        {
            case 8:
            {
                vec.push_back(static_cast<uint8_t>((static_cast<uint64_t>(number) >> 070) & 0xff));
                vec.push_back(static_cast<uint8_t>((static_cast<uint64_t>(number) >> 060) & 0xff));
                vec.push_back(static_cast<uint8_t>((static_cast<uint64_t>(number) >> 050) & 0xff));
                vec.push_back(static_cast<uint8_t>((static_cast<uint64_t>(number) >> 040) & 0xff));
                vec.push_back(static_cast<uint8_t>((number >> 030) & 0xff));
                vec.push_back(static_cast<uint8_t>((number >> 020) & 0xff));
                vec.push_back(static_cast<uint8_t>((number >> 010) & 0xff));
                vec.push_back(static_cast<uint8_t>(number & 0xff));
                break;
            }

            case 4:
            {
                vec.push_back(static_cast<uint8_t>((number >> 030) & 0xff));
                vec.push_back(static_cast<uint8_t>((number >> 020) & 0xff));
                vec.push_back(static_cast<uint8_t>((number >> 010) & 0xff));
                vec.push_back(static_cast<uint8_t>(number & 0xff));
                break;
            }

            case 2:
            {
                vec.push_back(static_cast<uint8_t>((number >> 010) & 0xff));
                vec.push_back(static_cast<uint8_t>(number & 0xff));
                break;
            }

            case 1:
            {
                vec.push_back(static_cast<uint8_t>(number & 0xff));
                break;
            }
        }
    }

    /*!
    @brief take sufficient bytes from a vector to fill an integer variable

    In the context of binary serialization formats, we need to read several
    bytes from a byte vector and combine them to multi-byte integral data
    types.

    @param[in] vec  byte vector to read from
    @param[in] current_index  the position in the vector after which to read

    @return the next sizeof(T) bytes from @a vec, in reverse order as T

    @tparam T the integral return type

    @throw std::out_of_range if there are less than sizeof(T)+1 bytes in the
           vector @a vec to read

    In the for loop, the bytes from the vector are copied in reverse order into
    the return value. In the figures below, let sizeof(T)=4 and `i` be the loop
    variable.

    Precondition:

    vec:   |   |   | a | b | c | d |      T: |   |   |   |   |
                 ^               ^             ^                ^
           current_index         i            ptr        sizeof(T)

    Postcondition:

    vec:   |   |   | a | b | c | d |      T: | d | c | b | a |
                 ^   ^                                     ^
                 |   i                                    ptr
           current_index

    @sa Code adapted from <http://stackoverflow.com/a/41031865/266378>.
    */
    template<typename T>
    static T get_from_vector(const std::vector<uint8_t>& vec, const size_t current_index)
    {
        if (current_index + sizeof(T) + 1 > vec.size())
        {
            JSON_THROW(std::out_of_range("cannot read " + std::to_string(sizeof(T)) + " bytes from vector"));
        }

        T result;
        auto* ptr = reinterpret_cast<uint8_t*>(&result);
        for (size_t i = 0; i < sizeof(T); ++i)
        {
            *ptr++ = vec[current_index + sizeof(T) - i];
        }
        return result;
    }

    /*!
    @brief create a MessagePack serialization of a given JSON value

    This is a straightforward implementation of the MessagePack specification.

    @param[in] j  JSON value to serialize
    @param[in,out] v  byte vector to write the serialization to

    @sa https://github.com/msgpack/msgpack/blob/master/spec.md
    */
    static void to_msgpack_internal(const basic_json& j, std::vector<uint8_t>& v)
    {
        switch (j.type())
        {
            case value_t::null:
            {
                // nil
                v.push_back(0xc0);
                break;
            }

            case value_t::boolean:
            {
                // true and false
                v.push_back(j.m_value.boolean ? 0xc3 : 0xc2);
                break;
            }

            case value_t::number_integer:
            {
                if (j.m_value.number_integer >= 0)
                {
                    // MessagePack does not differentiate between positive
                    // signed integers and unsigned integers. Therefore, we
                    // used the code from the value_t::number_unsigned case
                    // here.
                    if (j.m_value.number_unsigned < 128)
                    {
                        // positive fixnum
                        add_to_vector(v, 1, j.m_value.number_unsigned);
                    }
                    else if (j.m_value.number_unsigned <= std::numeric_limits<uint8_t>::max())
                    {
                        // uint 8
                        v.push_back(0xcc);
                        add_to_vector(v, 1, j.m_value.number_unsigned);
                    }
                    else if (j.m_value.number_unsigned <= std::numeric_limits<uint16_t>::max())
                    {
                        // uint 16
                        v.push_back(0xcd);
                        add_to_vector(v, 2, j.m_value.number_unsigned);
                    }
                    else if (j.m_value.number_unsigned <= std::numeric_limits<uint32_t>::max())
                    {
                        // uint 32
                        v.push_back(0xce);
                        add_to_vector(v, 4, j.m_value.number_unsigned);
                    }
                    else if (j.m_value.number_unsigned <= std::numeric_limits<uint64_t>::max())
                    {
                        // uint 64
                        v.push_back(0xcf);
                        add_to_vector(v, 8, j.m_value.number_unsigned);
                    }
                }
                else
                {
                    if (j.m_value.number_integer >= -32)
                    {
                        // negative fixnum
                        add_to_vector(v, 1, j.m_value.number_integer);
                    }
                    else if (j.m_value.number_integer >= std::numeric_limits<int8_t>::min() and j.m_value.number_integer <= std::numeric_limits<int8_t>::max())
                    {
                        // int 8
                        v.push_back(0xd0);
                        add_to_vector(v, 1, j.m_value.number_integer);
                    }
                    else if (j.m_value.number_integer >= std::numeric_limits<int16_t>::min() and j.m_value.number_integer <= std::numeric_limits<int16_t>::max())
                    {
                        // int 16
                        v.push_back(0xd1);
                        add_to_vector(v, 2, j.m_value.number_integer);
                    }
                    else if (j.m_value.number_integer >= std::numeric_limits<int32_t>::min() and j.m_value.number_integer <= std::numeric_limits<int32_t>::max())
                    {
                        // int 32
                        v.push_back(0xd2);
                        add_to_vector(v, 4, j.m_value.number_integer);
                    }
                    else if (j.m_value.number_integer >= std::numeric_limits<int64_t>::min() and j.m_value.number_integer <= std::numeric_limits<int64_t>::max())
                    {
                        // int 64
                        v.push_back(0xd3);
                        add_to_vector(v, 8, j.m_value.number_integer);
                    }
                }
                break;
            }

            case value_t::number_unsigned:
            {
                if (j.m_value.number_unsigned < 128)
                {
                    // positive fixnum
                    add_to_vector(v, 1, j.m_value.number_unsigned);
                }
                else if (j.m_value.number_unsigned <= std::numeric_limits<uint8_t>::max())
                {
                    // uint 8
                    v.push_back(0xcc);
                    add_to_vector(v, 1, j.m_value.number_unsigned);
                }
                else if (j.m_value.number_unsigned <= std::numeric_limits<uint16_t>::max())
                {
                    // uint 16
                    v.push_back(0xcd);
                    add_to_vector(v, 2, j.m_value.number_unsigned);
                }
                else if (j.m_value.number_unsigned <= std::numeric_limits<uint32_t>::max())
                {
                    // uint 32
                    v.push_back(0xce);
                    add_to_vector(v, 4, j.m_value.number_unsigned);
                }
                else if (j.m_value.number_unsigned <= std::numeric_limits<uint64_t>::max())
                {
                    // uint 64
                    v.push_back(0xcf);
                    add_to_vector(v, 8, j.m_value.number_unsigned);
                }
                break;
            }

            case value_t::number_float:
            {
                // float 64
                v.push_back(0xcb);
                const auto* helper = reinterpret_cast<const uint8_t*>(&(j.m_value.number_float));
                for (size_t i = 0; i < 8; ++i)
                {
                    v.push_back(helper[7 - i]);
                }
                break;
            }

            case value_t::string:
            {
                const auto N = j.m_value.string->size();
                if (N <= 31)
                {
                    // fixstr
                    v.push_back(static_cast<uint8_t>(0xa0 | N));
                }
                else if (N <= 255)
                {
                    // str 8
                    v.push_back(0xd9);
                    add_to_vector(v, 1, N);
                }
                else if (N <= 65535)
                {
                    // str 16
                    v.push_back(0xda);
                    add_to_vector(v, 2, N);
                }
                else if (N <= 4294967295)
                {
                    // str 32
                    v.push_back(0xdb);
                    add_to_vector(v, 4, N);
                }

                // append string
                std::copy(j.m_value.string->begin(), j.m_value.string->end(),
                          std::back_inserter(v));
                break;
            }

            case value_t::array:
            {
                const auto N = j.m_value.array->size();
                if (N <= 15)
                {
                    // fixarray
                    v.push_back(static_cast<uint8_t>(0x90 | N));
                }
                else if (N <= 0xffff)
                {
                    // array 16
                    v.push_back(0xdc);
                    add_to_vector(v, 2, N);
                }
                else if (N <= 0xffffffff)
                {
                    // array 32
                    v.push_back(0xdd);
                    add_to_vector(v, 4, N);
                }

                // append each element
                for (const auto& el : *j.m_value.array)
                {
                    to_msgpack_internal(el, v);
                }
                break;
            }

            case value_t::object:
            {
                const auto N = j.m_value.object->size();
                if (N <= 15)
                {
                    // fixmap
                    v.push_back(static_cast<uint8_t>(0x80 | (N & 0xf)));
                }
                else if (N <= 65535)
                {
                    // map 16
                    v.push_back(0xde);
                    add_to_vector(v, 2, N);
                }
                else if (N <= 4294967295)
                {
                    // map 32
                    v.push_back(0xdf);
                    add_to_vector(v, 4, N);
                }

                // append each element
                for (const auto& el : *j.m_value.object)
                {
                    to_msgpack_internal(el.first, v);
                    to_msgpack_internal(el.second, v);
                }
                break;
            }

            default:
            {
                break;
            }
        }
    }

    /*!
    @brief create a CBOR serialization of a given JSON value

    This is a straightforward implementation of the CBOR specification.

    @param[in] j  JSON value to serialize
    @param[in,out] v  byte vector to write the serialization to

    @sa https://tools.ietf.org/html/rfc7049
    */
    static void to_cbor_internal(const basic_json& j, std::vector<uint8_t>& v)
    {
        switch (j.type())
        {
            case value_t::null:
            {
                v.push_back(0xf6);
                break;
            }

            case value_t::boolean:
            {
                v.push_back(j.m_value.boolean ? 0xf5 : 0xf4);
                break;
            }

            case value_t::number_integer:
            {
                if (j.m_value.number_integer >= 0)
                {
                    // CBOR does not differentiate between positive signed
                    // integers and unsigned integers. Therefore, we used the
                    // code from the value_t::number_unsigned case here.
                    if (j.m_value.number_integer <= 0x17)
                    {
                        add_to_vector(v, 1, j.m_value.number_integer);
                    }
                    else if (j.m_value.number_integer <= std::numeric_limits<uint8_t>::max())
                    {
                        v.push_back(0x18);
                        // one-byte uint8_t
                        add_to_vector(v, 1, j.m_value.number_integer);
                    }
                    else if (j.m_value.number_integer <= std::numeric_limits<uint16_t>::max())
                    {
                        v.push_back(0x19);
                        // two-byte uint16_t
                        add_to_vector(v, 2, j.m_value.number_integer);
                    }
                    else if (j.m_value.number_integer <= std::numeric_limits<uint32_t>::max())
                    {
                        v.push_back(0x1a);
                        // four-byte uint32_t
                        add_to_vector(v, 4, j.m_value.number_integer);
                    }
                    else
                    {
                        v.push_back(0x1b);
                        // eight-byte uint64_t
                        add_to_vector(v, 8, j.m_value.number_integer);
                    }
                }
                else
                {
                    // The conversions below encode the sign in the first
                    // byte, and the value is converted to a positive number.
                    const auto positive_number = -1 - j.m_value.number_integer;
                    if (j.m_value.number_integer >= -24)
                    {
                        v.push_back(static_cast<uint8_t>(0x20 + positive_number));
                    }
                    else if (positive_number <= std::numeric_limits<uint8_t>::max())
                    {
                        // int 8
                        v.push_back(0x38);
                        add_to_vector(v, 1, positive_number);
                    }
                    else if (positive_number <= std::numeric_limits<uint16_t>::max())
                    {
                        // int 16
                        v.push_back(0x39);
                        add_to_vector(v, 2, positive_number);
                    }
                    else if (positive_number <= std::numeric_limits<uint32_t>::max())
                    {
                        // int 32
                        v.push_back(0x3a);
                        add_to_vector(v, 4, positive_number);
                    }
                    else
                    {
                        // int 64
                        v.push_back(0x3b);
                        add_to_vector(v, 8, positive_number);
                    }
                }
                break;
            }

            case value_t::number_unsigned:
            {
                if (j.m_value.number_unsigned <= 0x17)
                {
                    v.push_back(static_cast<uint8_t>(j.m_value.number_unsigned));
                }
                else if (j.m_value.number_unsigned <= 0xff)
                {
                    v.push_back(0x18);
                    // one-byte uint8_t
                    add_to_vector(v, 1, j.m_value.number_unsigned);
                }
                else if (j.m_value.number_unsigned <= 0xffff)
                {
                    v.push_back(0x19);
                    // two-byte uint16_t
                    add_to_vector(v, 2, j.m_value.number_unsigned);
                }
                else if (j.m_value.number_unsigned <= 0xffffffff)
                {
                    v.push_back(0x1a);
                    // four-byte uint32_t
                    add_to_vector(v, 4, j.m_value.number_unsigned);
                }
                else if (j.m_value.number_unsigned <= 0xffffffffffffffff)
                {
                    v.push_back(0x1b);
                    // eight-byte uint64_t
                    add_to_vector(v, 8, j.m_value.number_unsigned);
                }
                break;
            }

            case value_t::number_float:
            {
                // Double-Precision Float
                v.push_back(0xfb);
                const auto* helper = reinterpret_cast<const uint8_t*>(&(j.m_value.number_float));
                for (size_t i = 0; i < 8; ++i)
                {
                    v.push_back(helper[7 - i]);
                }
                break;
            }

            case value_t::string:
            {
                const auto N = j.m_value.string->size();
                if (N <= 0x17)
                {
                    v.push_back(0x60 + static_cast<uint8_t>(N));  // 1 byte for string + size
                }
                else if (N <= 0xff)
                {
                    v.push_back(0x78);  // one-byte uint8_t for N
                    add_to_vector(v, 1, N);
                }
                else if (N <= 0xffff)
                {
                    v.push_back(0x79);  // two-byte uint16_t for N
                    add_to_vector(v, 2, N);
                }
                else if (N <= 0xffffffff)
                {
                    v.push_back(0x7a); // four-byte uint32_t for N
                    add_to_vector(v, 4, N);
                }
                // LCOV_EXCL_START
                else if (N <= 0xffffffffffffffff)
                {
                    v.push_back(0x7b);  // eight-byte uint64_t for N
                    add_to_vector(v, 8, N);
                }
                // LCOV_EXCL_STOP

                // append string
                std::copy(j.m_value.string->begin(), j.m_value.string->end(),
                          std::back_inserter(v));
                break;
            }

            case value_t::array:
            {
                const auto N = j.m_value.array->size();
                if (N <= 0x17)
                {
                    v.push_back(0x80 + static_cast<uint8_t>(N));  // 1 byte for array + size
                }
                else if (N <= 0xff)
                {
                    v.push_back(0x98);  // one-byte uint8_t for N
                    add_to_vector(v, 1, N);
                }
                else if (N <= 0xffff)
                {
                    v.push_back(0x99);  // two-byte uint16_t for N
                    add_to_vector(v, 2, N);
                }
                else if (N <= 0xffffffff)
                {
                    v.push_back(0x9a);  // four-byte uint32_t for N
                    add_to_vector(v, 4, N);
                }
                // LCOV_EXCL_START
                else if (N <= 0xffffffffffffffff)
                {
                    v.push_back(0x9b);  // eight-byte uint64_t for N
                    add_to_vector(v, 8, N);
                }
                // LCOV_EXCL_STOP

                // append each element
                for (const auto& el : *j.m_value.array)
                {
                    to_cbor_internal(el, v);
                }
                break;
            }

            case value_t::object:
            {
                const auto N = j.m_value.object->size();
                if (N <= 0x17)
                {
                    v.push_back(0xa0 + static_cast<uint8_t>(N));  // 1 byte for object + size
                }
                else if (N <= 0xff)
                {
                    v.push_back(0xb8);
                    add_to_vector(v, 1, N);  // one-byte uint8_t for N
                }
                else if (N <= 0xffff)
                {
                    v.push_back(0xb9);
                    add_to_vector(v, 2, N);  // two-byte uint16_t for N
                }
                else if (N <= 0xffffffff)
                {
                    v.push_back(0xba);
                    add_to_vector(v, 4, N);  // four-byte uint32_t for N
                }
                // LCOV_EXCL_START
                else if (N <= 0xffffffffffffffff)
                {
                    v.push_back(0xbb);
                    add_to_vector(v, 8, N);  // eight-byte uint64_t for N
                }
                // LCOV_EXCL_STOP

                // append each element
                for (const auto& el : *j.m_value.object)
                {
                    to_cbor_internal(el.first, v);
                    to_cbor_internal(el.second, v);
                }
                break;
            }

            default:
            {
                break;
            }
        }
    }


    /*
    @brief checks if given lengths do not exceed the size of a given vector

    To secure the access to the byte vector during CBOR/MessagePack
    deserialization, bytes are copied from the vector into buffers. This
    function checks if the number of bytes to copy (@a len) does not exceed
    the size @s size of the vector. Additionally, an @a offset is given from
    where to start reading the bytes.

    This function checks whether reading the bytes is safe; that is, offset is
    a valid index in the vector, offset+len

    @param[in] size    size of the byte vector
    @param[in] len     number of bytes to read
    @param[in] offset  offset where to start reading

    vec:  x x x x x X X X X X
          ^         ^         ^
          0         offset    len

    @throws out_of_range if `len > v.size()`
    */
    static void check_length(const size_t size, const size_t len, const size_t offset)
    {
        // simple case: requested length is greater than the vector's length
        if (len > size or offset > size)
        {
            JSON_THROW(std::out_of_range("len out of range"));
        }

        // second case: adding offset would result in overflow
        if ((size > (std::numeric_limits<size_t>::max() - offset)))
        {
            JSON_THROW(std::out_of_range("len+offset out of range"));
        }

        // last case: reading past the end of the vector
        if (len + offset > size)
        {
            JSON_THROW(std::out_of_range("len+offset out of range"));
        }
    }

    /*!
    @brief create a JSON value from a given MessagePack vector

    @param[in] v  MessagePack serialization
    @param[in] idx  byte index to start reading from @a v

    @return deserialized JSON value

    @throw std::invalid_argument if unsupported features from MessagePack were
    used in the given vector @a v or if the input is not valid MessagePack
    @throw std::out_of_range if the given vector ends prematurely

    @sa https://github.com/msgpack/msgpack/blob/master/spec.md
    */
    static basic_json from_msgpack_internal(const std::vector<uint8_t>& v, size_t& idx)
    {
        // make sure reading 1 byte is safe
        check_length(v.size(), 1, idx);

        // store and increment index
        const size_t current_idx = idx++;

        if (v[current_idx] <= 0xbf)
        {
            if (v[current_idx] <= 0x7f) // positive fixint
            {
                return v[current_idx];
            }
            if (v[current_idx] <= 0x8f) // fixmap
            {
                basic_json result = value_t::object;
                const size_t len = v[current_idx] & 0x0f;
                for (size_t i = 0; i < len; ++i)
                {
                    std::string key = from_msgpack_internal(v, idx);
                    result[key] = from_msgpack_internal(v, idx);
                }
                return result;
            }
            else if (v[current_idx] <= 0x9f) // fixarray
            {
                basic_json result = value_t::array;
                const size_t len = v[current_idx] & 0x0f;
                for (size_t i = 0; i < len; ++i)
                {
                    result.push_back(from_msgpack_internal(v, idx));
                }
                return result;
            }
            else // fixstr
            {
                const size_t len = v[current_idx] & 0x1f;
                const size_t offset = current_idx + 1;
                idx += len; // skip content bytes
                check_length(v.size(), len, offset);
                return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
            }
        }
        else if (v[current_idx] >= 0xe0) // negative fixint
        {
            return static_cast<int8_t>(v[current_idx]);
        }
        else
        {
            switch (v[current_idx])
            {
                case 0xc0: // nil
                {
                    return value_t::null;
                }

                case 0xc2: // false
                {
                    return false;
                }

                case 0xc3: // true
                {
                    return true;
                }

                case 0xca: // float 32
                {
                    // copy bytes in reverse order into the double variable
                    float res;
                    for (size_t byte = 0; byte < sizeof(float); ++byte)
                    {
                        reinterpret_cast<uint8_t*>(&res)[sizeof(float) - byte - 1] = v.at(current_idx + 1 + byte);
                    }
                    idx += sizeof(float); // skip content bytes
                    return res;
                }

                case 0xcb: // float 64
                {
                    // copy bytes in reverse order into the double variable
                    double res;
                    for (size_t byte = 0; byte < sizeof(double); ++byte)
                    {
                        reinterpret_cast<uint8_t*>(&res)[sizeof(double) - byte - 1] = v.at(current_idx + 1 + byte);
                    }
                    idx += sizeof(double); // skip content bytes
                    return res;
                }

                case 0xcc: // uint 8
                {
                    idx += 1; // skip content byte
                    return get_from_vector<uint8_t>(v, current_idx);
                }

                case 0xcd: // uint 16
                {
                    idx += 2; // skip 2 content bytes
                    return get_from_vector<uint16_t>(v, current_idx);
                }

                case 0xce: // uint 32
                {
                    idx += 4; // skip 4 content bytes
                    return get_from_vector<uint32_t>(v, current_idx);
                }

                case 0xcf: // uint 64
                {
                    idx += 8; // skip 8 content bytes
                    return get_from_vector<uint64_t>(v, current_idx);
                }

                case 0xd0: // int 8
                {
                    idx += 1; // skip content byte
                    return get_from_vector<int8_t>(v, current_idx);
                }

                case 0xd1: // int 16
                {
                    idx += 2; // skip 2 content bytes
                    return get_from_vector<int16_t>(v, current_idx);
                }

                case 0xd2: // int 32
                {
                    idx += 4; // skip 4 content bytes
                    return get_from_vector<int32_t>(v, current_idx);
                }

                case 0xd3: // int 64
                {
                    idx += 8; // skip 8 content bytes
                    return get_from_vector<int64_t>(v, current_idx);
                }

                case 0xd9: // str 8
                {
                    const auto len = static_cast<size_t>(get_from_vector<uint8_t>(v, current_idx));
                    const size_t offset = current_idx + 2;
                    idx += len + 1; // skip size byte + content bytes
                    check_length(v.size(), len, offset);
                    return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
                }

                case 0xda: // str 16
                {
                    const auto len = static_cast<size_t>(get_from_vector<uint16_t>(v, current_idx));
                    const size_t offset = current_idx + 3;
                    idx += len + 2; // skip 2 size bytes + content bytes
                    check_length(v.size(), len, offset);
                    return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
                }

                case 0xdb: // str 32
                {
                    const auto len = static_cast<size_t>(get_from_vector<uint32_t>(v, current_idx));
                    const size_t offset = current_idx + 5;
                    idx += len + 4; // skip 4 size bytes + content bytes
                    check_length(v.size(), len, offset);
                    return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
                }

                case 0xdc: // array 16
                {
                    basic_json result = value_t::array;
                    const auto len = static_cast<size_t>(get_from_vector<uint16_t>(v, current_idx));
                    idx += 2; // skip 2 size bytes
                    for (size_t i = 0; i < len; ++i)
                    {
                        result.push_back(from_msgpack_internal(v, idx));
                    }
                    return result;
                }

                case 0xdd: // array 32
                {
                    basic_json result = value_t::array;
                    const auto len = static_cast<size_t>(get_from_vector<uint32_t>(v, current_idx));
                    idx += 4; // skip 4 size bytes
                    for (size_t i = 0; i < len; ++i)
                    {
                        result.push_back(from_msgpack_internal(v, idx));
                    }
                    return result;
                }

                case 0xde: // map 16
                {
                    basic_json result = value_t::object;
                    const auto len = static_cast<size_t>(get_from_vector<uint16_t>(v, current_idx));
                    idx += 2; // skip 2 size bytes
                    for (size_t i = 0; i < len; ++i)
                    {
                        std::string key = from_msgpack_internal(v, idx);
                        result[key] = from_msgpack_internal(v, idx);
                    }
                    return result;
                }

                case 0xdf: // map 32
                {
                    basic_json result = value_t::object;
                    const auto len = static_cast<size_t>(get_from_vector<uint32_t>(v, current_idx));
                    idx += 4; // skip 4 size bytes
                    for (size_t i = 0; i < len; ++i)
                    {
                        std::string key = from_msgpack_internal(v, idx);
                        result[key] = from_msgpack_internal(v, idx);
                    }
                    return result;
                }

                default:
                {
                    JSON_THROW(std::invalid_argument("error parsing a msgpack @ " + std::to_string(current_idx) + ": " + std::to_string(static_cast<int>(v[current_idx]))));
                }
            }
        }
    }

    /*!
    @brief create a JSON value from a given CBOR vector

    @param[in] v  CBOR serialization
    @param[in] idx  byte index to start reading from @a v

    @return deserialized JSON value

    @throw std::invalid_argument if unsupported features from CBOR were used in
    the given vector @a v or if the input is not valid CBOR
    @throw std::out_of_range if the given vector ends prematurely

    @sa https://tools.ietf.org/html/rfc7049
    */
    static basic_json from_cbor_internal(const std::vector<uint8_t>& v, size_t& idx)
    {
        // store and increment index
        const size_t current_idx = idx++;

        switch (v.at(current_idx))
        {
            // Integer 0x00..0x17 (0..23)
            case 0x00:
            case 0x01:
            case 0x02:
            case 0x03:
            case 0x04:
            case 0x05:
            case 0x06:
            case 0x07:
            case 0x08:
            case 0x09:
            case 0x0a:
            case 0x0b:
            case 0x0c:
            case 0x0d:
            case 0x0e:
            case 0x0f:
            case 0x10:
            case 0x11:
            case 0x12:
            case 0x13:
            case 0x14:
            case 0x15:
            case 0x16:
            case 0x17:
            {
                return v[current_idx];
            }

            case 0x18: // Unsigned integer (one-byte uint8_t follows)
            {
                idx += 1; // skip content byte
                return get_from_vector<uint8_t>(v, current_idx);
            }

            case 0x19: // Unsigned integer (two-byte uint16_t follows)
            {
                idx += 2; // skip 2 content bytes
                return get_from_vector<uint16_t>(v, current_idx);
            }

            case 0x1a: // Unsigned integer (four-byte uint32_t follows)
            {
                idx += 4; // skip 4 content bytes
                return get_from_vector<uint32_t>(v, current_idx);
            }

            case 0x1b: // Unsigned integer (eight-byte uint64_t follows)
            {
                idx += 8; // skip 8 content bytes
                return get_from_vector<uint64_t>(v, current_idx);
            }

            // Negative integer -1-0x00..-1-0x17 (-1..-24)
            case 0x20:
            case 0x21:
            case 0x22:
            case 0x23:
            case 0x24:
            case 0x25:
            case 0x26:
            case 0x27:
            case 0x28:
            case 0x29:
            case 0x2a:
            case 0x2b:
            case 0x2c:
            case 0x2d:
            case 0x2e:
            case 0x2f:
            case 0x30:
            case 0x31:
            case 0x32:
            case 0x33:
            case 0x34:
            case 0x35:
            case 0x36:
            case 0x37:
            {
                return static_cast<int8_t>(0x20 - 1 - v[current_idx]);
            }

            case 0x38: // Negative integer (one-byte uint8_t follows)
            {
                idx += 1; // skip content byte
                // must be uint8_t !
                return static_cast<number_integer_t>(-1) - get_from_vector<uint8_t>(v, current_idx);
            }

            case 0x39: // Negative integer -1-n (two-byte uint16_t follows)
            {
                idx += 2; // skip 2 content bytes
                return static_cast<number_integer_t>(-1) - get_from_vector<uint16_t>(v, current_idx);
            }

            case 0x3a: // Negative integer -1-n (four-byte uint32_t follows)
            {
                idx += 4; // skip 4 content bytes
                return static_cast<number_integer_t>(-1) - get_from_vector<uint32_t>(v, current_idx);
            }

            case 0x3b: // Negative integer -1-n (eight-byte uint64_t follows)
            {
                idx += 8; // skip 8 content bytes
                return static_cast<number_integer_t>(-1) - static_cast<number_integer_t>(get_from_vector<uint64_t>(v, current_idx));
            }

            // UTF-8 string (0x00..0x17 bytes follow)
            case 0x60:
            case 0x61:
            case 0x62:
            case 0x63:
            case 0x64:
            case 0x65:
            case 0x66:
            case 0x67:
            case 0x68:
            case 0x69:
            case 0x6a:
            case 0x6b:
            case 0x6c:
            case 0x6d:
            case 0x6e:
            case 0x6f:
            case 0x70:
            case 0x71:
            case 0x72:
            case 0x73:
            case 0x74:
            case 0x75:
            case 0x76:
            case 0x77:
            {
                const auto len = static_cast<size_t>(v[current_idx] - 0x60);
                const size_t offset = current_idx + 1;
                idx += len; // skip content bytes
                check_length(v.size(), len, offset);
                return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
            }

            case 0x78: // UTF-8 string (one-byte uint8_t for n follows)
            {
                const auto len = static_cast<size_t>(get_from_vector<uint8_t>(v, current_idx));
                const size_t offset = current_idx + 2;
                idx += len + 1; // skip size byte + content bytes
                check_length(v.size(), len, offset);
                return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
            }

            case 0x79: // UTF-8 string (two-byte uint16_t for n follow)
            {
                const auto len = static_cast<size_t>(get_from_vector<uint16_t>(v, current_idx));
                const size_t offset = current_idx + 3;
                idx += len + 2; // skip 2 size bytes + content bytes
                check_length(v.size(), len, offset);
                return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
            }

            case 0x7a: // UTF-8 string (four-byte uint32_t for n follow)
            {
                const auto len = static_cast<size_t>(get_from_vector<uint32_t>(v, current_idx));
                const size_t offset = current_idx + 5;
                idx += len + 4; // skip 4 size bytes + content bytes
                check_length(v.size(), len, offset);
                return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
            }

            case 0x7b: // UTF-8 string (eight-byte uint64_t for n follow)
            {
                const auto len = static_cast<size_t>(get_from_vector<uint64_t>(v, current_idx));
                const size_t offset = current_idx + 9;
                idx += len + 8; // skip 8 size bytes + content bytes
                check_length(v.size(), len, offset);
                return std::string(reinterpret_cast<const char*>(v.data()) + offset, len);
            }

            case 0x7f: // UTF-8 string (indefinite length)
            {
                std::string result;
                while (v.at(idx) != 0xff)
                {
                    string_t s = from_cbor_internal(v, idx);
                    result += s;
                }
                // skip break byte (0xFF)
                idx += 1;
                return result;
            }

            // array (0x00..0x17 data items follow)
            case 0x80:
            case 0x81:
            case 0x82:
            case 0x83:
            case 0x84:
            case 0x85:
            case 0x86:
            case 0x87:
            case 0x88:
            case 0x89:
            case 0x8a:
            case 0x8b:
            case 0x8c:
            case 0x8d:
            case 0x8e:
            case 0x8f:
            case 0x90:
            case 0x91:
            case 0x92:
            case 0x93:
            case 0x94:
            case 0x95:
            case 0x96:
            case 0x97:
            {
                basic_json result = value_t::array;
                const auto len = static_cast<size_t>(v[current_idx] - 0x80);
                for (size_t i = 0; i < len; ++i)
                {
                    result.push_back(from_cbor_internal(v, idx));
                }
                return result;
            }

            case 0x98: // array (one-byte uint8_t for n follows)
            {
                basic_json result = value_t::array;
                const auto len = static_cast<size_t>(get_from_vector<uint8_t>(v, current_idx));
                idx += 1; // skip 1 size byte
                for (size_t i = 0; i < len; ++i)
                {
                    result.push_back(from_cbor_internal(v, idx));
                }
                return result;
            }

            case 0x99: // array (two-byte uint16_t for n follow)
            {
                basic_json result = value_t::array;
                const auto len = static_cast<size_t>(get_from_vector<uint16_t>(v, current_idx));
                idx += 2; // skip 4 size bytes
                for (size_t i = 0; i < len; ++i)
                {
                    result.push_back(from_cbor_internal(v, idx));
                }
                return result;
            }

            case 0x9a: // array (four-byte uint32_t for n follow)
            {
                basic_json result = value_t::array;
                const auto len = static_cast<size_t>(get_from_vector<uint32_t>(v, current_idx));
                idx += 4; // skip 4 size bytes
                for (size_t i = 0; i < len; ++i)
                {
                    result.push_back(from_cbor_internal(v, idx));
                }
                return result;
            }

            case 0x9b: // array (eight-byte uint64_t for n follow)
            {
                basic_json result = value_t::array;
                const auto len = static_cast<size_t>(get_from_vector<uint64_t>(v, current_idx));
                idx += 8; // skip 8 size bytes
                for (size_t i = 0; i < len; ++i)
                {
                    result.push_back(from_cbor_internal(v, idx));
                }
                return result;
            }

            case 0x9f: // array (indefinite length)
            {
                basic_json result = value_t::array;
                while (v.at(idx) != 0xff)
                {
                    result.push_back(from_cbor_internal(v, idx));
                }
                // skip break byte (0xFF)
                idx += 1;
                return result;
            }

            // map (0x00..0x17 pairs of data items follow)
            case 0xa0:
            case 0xa1:
            case 0xa2:
            case 0xa3:
            case 0xa4:
            case 0xa5:
            case 0xa6:
            case 0xa7:
            case 0xa8:
            case 0xa9:
            case 0xaa:
            case 0xab:
            case 0xac:
            case 0xad:
            case 0xae:
            case 0xaf:
            case 0xb0:
            case 0xb1:
            case 0xb2:
            case 0xb3:
            case 0xb4:
            case 0xb5:
            case 0xb6:
            case 0xb7:
            {
                basic_json result = value_t::object;
                const auto len = static_cast<size_t>(v[current_idx] - 0xa0);
                for (size_t i = 0; i < len; ++i)
                {
                    std::string key = from_cbor_internal(v, idx);
                    result[key] = from_cbor_internal(v, idx);
                }
                return result;
            }

            case 0xb8: // map (one-byte uint8_t for n follows)
            {
                basic_json result = value_t::object;
                const auto len = static_cast<size_t>(get_from_vector<uint8_t>(v, current_idx));
                idx += 1; // skip 1 size byte
                for (size_t i = 0; i < len; ++i)
                {
                    std::string key = from_cbor_internal(v, idx);
                    result[key] = from_cbor_internal(v, idx);
                }
                return result;
            }

            case 0xb9: // map (two-byte uint16_t for n follow)
            {
                basic_json result = value_t::object;
                const auto len = static_cast<size_t>(get_from_vector<uint16_t>(v, current_idx));
                idx += 2; // skip 2 size bytes
                for (size_t i = 0; i < len; ++i)
                {
                    std::string key = from_cbor_internal(v, idx);
                    result[key] = from_cbor_internal(v, idx);
                }
                return result;
            }

            case 0xba: // map (four-byte uint32_t for n follow)
            {
                basic_json result = value_t::object;
                const auto len = static_cast<size_t>(get_from_vector<uint32_t>(v, current_idx));
                idx += 4; // skip 4 size bytes
                for (size_t i = 0; i < len; ++i)
                {
                    std::string key = from_cbor_internal(v, idx);
                    result[key] = from_cbor_internal(v, idx);
                }
                return result;
            }

            case 0xbb: // map (eight-byte uint64_t for n follow)
            {
                basic_json result = value_t::object;
                const auto len = static_cast<size_t>(get_from_vector<uint64_t>(v, current_idx));
                idx += 8; // skip 8 size bytes
                for (size_t i = 0; i < len; ++i)
                {
                    std::string key = from_cbor_internal(v, idx);
                    result[key] = from_cbor_internal(v, idx);
                }
                return result;
            }

            case 0xbf: // map (indefinite length)
            {
                basic_json result = value_t::object;
                while (v.at(idx) != 0xff)
                {
                    std::string key = from_cbor_internal(v, idx);
                    result[key] = from_cbor_internal(v, idx);
                }
                // skip break byte (0xFF)
                idx += 1;
                return result;
            }

            case 0xf4: // false
            {
                return false;
            }

            case 0xf5: // true
            {
                return true;
            }

            case 0xf6: // null
            {
                return value_t::null;
            }

            case 0xf9: // Half-Precision Float (two-byte IEEE 754)
            {
                idx += 2; // skip two content bytes

                // code from RFC 7049, Appendix D, Figure 3:
                // As half-precision floating-point numbers were only added to
                // IEEE 754 in 2008, today's programming platforms often still
                // only have limited support for them. It is very easy to
                // include at least decoding support for them even without such
                // support. An example of a small decoder for half-precision
                // floating-point numbers in the C language is shown in Fig. 3.
                const int half = (v.at(current_idx + 1) << 8) + v.at(current_idx + 2);
                const int exp = (half >> 10) & 0x1f;
                const int mant = half & 0x3ff;
                double val;
                if (exp == 0)
                {
                    val = std::ldexp(mant, -24);
                }
                else if (exp != 31)
                {
                    val = std::ldexp(mant + 1024, exp - 25);
                }
                else
                {
                    val = mant == 0
                          ? std::numeric_limits<double>::infinity()
                          : std::numeric_limits<double>::quiet_NaN();
                }
                return (half & 0x8000) != 0 ? -val : val;
            }

            case 0xfa: // Single-Precision Float (four-byte IEEE 754)
            {
                // copy bytes in reverse order into the float variable
                float res;
                for (size_t byte = 0; byte < sizeof(float); ++byte)
                {
                    reinterpret_cast<uint8_t*>(&res)[sizeof(float) - byte - 1] = v.at(current_idx + 1 + byte);
                }
                idx += sizeof(float); // skip content bytes
                return res;
            }

            case 0xfb: // Double-Precision Float (eight-byte IEEE 754)
            {
                // copy bytes in reverse order into the double variable
                double res;
                for (size_t byte = 0; byte < sizeof(double); ++byte)
                {
                    reinterpret_cast<uint8_t*>(&res)[sizeof(double) - byte - 1] = v.at(current_idx + 1 + byte);
                }
                idx += sizeof(double); // skip content bytes
                return res;
            }

            default: // anything else (0xFF is handled inside the other types)
            {
                JSON_THROW(std::invalid_argument("error parsing a CBOR @ " + std::to_string(current_idx) + ": " + std::to_string(static_cast<int>(v[current_idx]))));
            }
        }
    }

  public:
    /*!
    @brief create a MessagePack serialization of a given JSON value

    Serializes a given JSON value @a j to a byte vector using the MessagePack
    serialization format. MessagePack is a binary serialization format which
    aims to be more compact than JSON itself, yet more efficient to parse.

    @param[in] j  JSON value to serialize
    @return MessagePack serialization as byte vector

    @complexity Linear in the size of the JSON value @a j.

    @liveexample{The example shows the serialization of a JSON value to a byte
    vector in MessagePack format.,to_msgpack}

    @sa http://msgpack.org
    @sa @ref from_msgpack(const std::vector<uint8_t>&, const size_t) for the
        analogous deserialization
    @sa @ref to_cbor(const basic_json& for the related CBOR format

    @since version 2.0.9
    */
    static std::vector<uint8_t> to_msgpack(const basic_json& j)
    {
        std::vector<uint8_t> result;
        to_msgpack_internal(j, result);
        return result;
    }

    /*!
    @brief create a JSON value from a byte vector in MessagePack format

    Deserializes a given byte vector @a v to a JSON value using the MessagePack
    serialization format.

    @param[in] v  a byte vector in MessagePack format
    @param[in] start_index the index to start reading from @a v (0 by default)
    @return deserialized JSON value

    @throw std::invalid_argument if unsupported features from MessagePack were
    used in the given vector @a v or if the input is not valid MessagePack
    @throw std::out_of_range if the given vector ends prematurely

    @complexity Linear in the size of the byte vector @a v.

    @liveexample{The example shows the deserialization of a byte vector in
    MessagePack format to a JSON value.,from_msgpack}

    @sa http://msgpack.org
    @sa @ref to_msgpack(const basic_json&) for the analogous serialization
    @sa @ref from_cbor(const std::vector<uint8_t>&, const size_t) for the
        related CBOR format

    @since version 2.0.9, parameter @a start_index since 2.1.1
    */
    static basic_json from_msgpack(const std::vector<uint8_t>& v,
                                   const size_t start_index = 0)
    {
        size_t i = start_index;
        return from_msgpack_internal(v, i);
    }

    /*!
    @brief create a MessagePack serialization of a given JSON value

    Serializes a given JSON value @a j to a byte vector using the CBOR (Concise
    Binary Object Representation) serialization format. CBOR is a binary
    serialization format which aims to be more compact than JSON itself, yet
    more efficient to parse.

    @param[in] j  JSON value to serialize
    @return MessagePack serialization as byte vector

    @complexity Linear in the size of the JSON value @a j.

    @liveexample{The example shows the serialization of a JSON value to a byte
    vector in CBOR format.,to_cbor}

    @sa http://cbor.io
    @sa @ref from_cbor(const std::vector<uint8_t>&, const size_t) for the
        analogous deserialization
    @sa @ref to_msgpack(const basic_json& for the related MessagePack format

    @since version 2.0.9
    */
    static std::vector<uint8_t> to_cbor(const basic_json& j)
    {
        std::vector<uint8_t> result;
        to_cbor_internal(j, result);
        return result;
    }

    /*!
    @brief create a JSON value from a byte vector in CBOR format

    Deserializes a given byte vector @a v to a JSON value using the CBOR
    (Concise Binary Object Representation) serialization format.

    @param[in] v  a byte vector in CBOR format
    @param[in] start_index the index to start reading from @a v (0 by default)
    @return deserialized JSON value

    @throw std::invalid_argument if unsupported features from CBOR were used in
    the given vector @a v or if the input is not valid MessagePack
    @throw std::out_of_range if the given vector ends prematurely

    @complexity Linear in the size of the byte vector @a v.

    @liveexample{The example shows the deserialization of a byte vector in CBOR
    format to a JSON value.,from_cbor}

    @sa http://cbor.io
    @sa @ref to_cbor(const basic_json&) for the analogous serialization
    @sa @ref from_msgpack(const std::vector<uint8_t>&, const size_t) for the
        related MessagePack format

    @since version 2.0.9, parameter @a start_index since 2.1.1
    */
    static basic_json from_cbor(const std::vector<uint8_t>& v,
                                const size_t start_index = 0)
    {
        size_t i = start_index;
        return from_cbor_internal(v, i);
    }

    /// @}

    ///////////////////////////
    // convenience functions //
    ///////////////////////////

    /*!
    @brief return the type as string

    Returns the type name as string to be used in error messages - usually to
    indicate that a function was called on a wrong JSON type.

    @return basically a string representation of a the @a m_type member

    @complexity Constant.

    @liveexample{The following code exemplifies `type_name()` for all JSON
    types.,type_name}

    @since version 1.0.0, public since 2.1.0
    */
    std::string type_name() const
    {
        {
            switch (m_type)
            {
                case value_t::null:
                    return "null";
                case value_t::object:
                    return "object";
                case value_t::array:
                    return "array";
                case value_t::string:
                    return "string";
                case value_t::boolean:
                    return "boolean";
                case value_t::discarded:
                    return "discarded";
                default:
                    return "number";
            }
        }
    }

  private:
    /*!
    @brief calculates the extra space to escape a JSON string

    @param[in] s  the string to escape
    @return the number of characters required to escape string @a s

    @complexity Linear in the length of string @a s.
    */
    static std::size_t extra_space(const string_t& s) noexcept
    {
        return std::accumulate(s.begin(), s.end(), size_t{},
                               [](size_t res, typename string_t::value_type c)
        {
            switch (c)
            {
                case '"':
                case '\\':
                case '\b':
                case '\f':
                case '\n':
                case '\r':
                case '\t':
                {
                    // from c (1 byte) to \x (2 bytes)
                    return res + 1;
                }

                default:
                {
                    if (c >= 0x00 and c <= 0x1f)
                    {
                        // from c (1 byte) to \uxxxx (6 bytes)
                        return res + 5;
                    }

                    return res;
                }
            }
        });
    }

    /*!
    @brief escape a string

    Escape a string by replacing certain special characters by a sequence of
    an escape character (backslash) and another character and other control
    characters by a sequence of "\u" followed by a four-digit hex
    representation.

    @param[in] s  the string to escape
    @return  the escaped string

    @complexity Linear in the length of string @a s.
    */
    static string_t escape_string(const string_t& s)
    {
        const auto space = extra_space(s);
        if (space == 0)
        {
            return s;
        }

        // create a result string of necessary size
        string_t result(s.size() + space, '\\');
        std::size_t pos = 0;

        for (const auto& c : s)
        {
            switch (c)
            {
                // quotation mark (0x22)
                case '"':
                {
                    result[pos + 1] = '"';
                    pos += 2;
                    break;
                }

                // reverse solidus (0x5c)
                case '\\':
                {
                    // nothing to change
                    pos += 2;
                    break;
                }

                // backspace (0x08)
                case '\b':
                {
                    result[pos + 1] = 'b';
                    pos += 2;
                    break;
                }

                // formfeed (0x0c)
                case '\f':
                {
                    result[pos + 1] = 'f';
                    pos += 2;
                    break;
                }

                // newline (0x0a)
                case '\n':
                {
                    result[pos + 1] = 'n';
                    pos += 2;
                    break;
                }

                // carriage return (0x0d)
                case '\r':
                {
                    result[pos + 1] = 'r';
                    pos += 2;
                    break;
                }

                // horizontal tab (0x09)
                case '\t':
                {
                    result[pos + 1] = 't';
                    pos += 2;
                    break;
                }

                default:
                {
                    if (c >= 0x00 and c <= 0x1f)
                    {
                        // convert a number 0..15 to its hex representation
                        // (0..f)
                        static const char hexify[16] =
                        {
                            '0', '1', '2', '3', '4', '5', '6', '7',
                            '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
                        };

                        // print character c as \uxxxx
                        for (const char m :
                    { 'u', '0', '0', hexify[c >> 4], hexify[c & 0x0f]
                        })
                        {
                            result[++pos] = m;
                        }

                        ++pos;
                    }
                    else
                    {
                        // all other characters are added as-is
                        result[pos++] = c;
                    }
                    break;
                }
            }
        }

        return result;
    }


    /*!
    @brief locale-independent serialization for built-in arithmetic types
    */
    struct numtostr
    {
      public:
        template<typename NumberType>
        numtostr(NumberType value)
        {
            x_write(value, std::is_integral<NumberType>());
        }

        const char* c_str() const
        {
            return m_buf.data();
        }

      private:
        /// a (hopefully) large enough character buffer
        std::array < char, 64 > m_buf{{}};

        template<typename NumberType>
        void x_write(NumberType x, /*is_integral=*/std::true_type)
        {
            // special case for "0"
            if (x == 0)
            {
                m_buf[0] = '0';
                return;
            }

            const bool is_negative = x < 0;
            size_t i = 0;

            // spare 1 byte for '\0'
            while (x != 0 and i < m_buf.size() - 1)
            {
                const auto digit = std::labs(static_cast<long>(x % 10));
                m_buf[i++] = static_cast<char>('0' + digit);
                x /= 10;
            }

            // make sure the number has been processed completely
            assert(x == 0);

            if (is_negative)
            {
                // make sure there is capacity for the '-'
                assert(i < m_buf.size() - 2);
                m_buf[i++] = '-';
            }

            std::reverse(m_buf.begin(), m_buf.begin() + i);
        }

        template<typename NumberType>
        void x_write(NumberType x, /*is_integral=*/std::false_type)
        {
            // special case for 0.0 and -0.0
            if (x == 0)
            {
                size_t i = 0;
                if (std::signbit(x))
                {
                    m_buf[i++] = '-';
                }
                m_buf[i++] = '0';
                m_buf[i++] = '.';
                m_buf[i] = '0';
                return;
            }

            // get number of digits for a text -> float -> text round-trip
            static constexpr auto d = std::numeric_limits<NumberType>::digits10;

            // the actual conversion
            const auto written_bytes = snprintf(m_buf.data(), m_buf.size(), "%.*g", d, x);

            // negative value indicates an error
            assert(written_bytes > 0);
            // check if buffer was large enough
            assert(static_cast<size_t>(written_bytes) < m_buf.size());

            // read information from locale
            const auto loc = localeconv();
            assert(loc != nullptr);
            const char thousands_sep = !loc->thousands_sep ? '\0'
                                       : loc->thousands_sep[0];

            const char decimal_point = !loc->decimal_point ? '\0'
                                       : loc->decimal_point[0];

            // erase thousands separator
            if (thousands_sep != '\0')
            {
                const auto end = std::remove(m_buf.begin(), m_buf.begin() + written_bytes, thousands_sep);
                std::fill(end, m_buf.end(), '\0');
            }

            // convert decimal point to '.'
            if (decimal_point != '\0' and decimal_point != '.')
            {
                for (auto& c : m_buf)
                {
                    if (c == decimal_point)
                    {
                        c = '.';
                        break;
                    }
                }
            }

            // determine if need to append ".0"
            size_t i = 0;
            bool value_is_int_like = true;
            for (i = 0; i < m_buf.size(); ++i)
            {
                // break when end of number is reached
                if (m_buf[i] == '\0')
                {
                    break;
                }

                // check if we find non-int character
                value_is_int_like = value_is_int_like and m_buf[i] != '.' and
                                    m_buf[i] != 'e' and m_buf[i] != 'E';
            }

            if (value_is_int_like)
            {
                // there must be 2 bytes left for ".0"
                assert((i + 2) < m_buf.size());
                // we write to the end of the number
                assert(m_buf[i] == '\0');
                assert(m_buf[i - 1] != '\0');

                // add ".0"
                m_buf[i] = '.';
                m_buf[i + 1] = '0';

                // the resulting string is properly terminated
                assert(m_buf[i + 2] == '\0');
            }
        }
    };


    /*!
    @brief internal implementation of the serialization function

    This function is called by the public member function dump and organizes
    the serialization internally. The indentation level is propagated as
    additional parameter. In case of arrays and objects, the function is
    called recursively. Note that

    - strings and object keys are escaped using `escape_string()`
    - integer numbers are converted implicitly via `operator<<`
    - floating-point numbers are converted to a string using `"%g"` format

    @param[out] o              stream to write to
    @param[in] pretty_print    whether the output shall be pretty-printed
    @param[in] indent_step     the indent level
    @param[in] current_indent  the current indent level (only used internally)
    */
    void dump(std::ostream& o,
              const bool pretty_print,
              const unsigned int indent_step,
              const unsigned int current_indent = 0) const
    {
        // variable to hold indentation for recursive calls
        unsigned int new_indent = current_indent;

        switch (m_type)
        {
            case value_t::object:
            {
                if (m_value.object->empty())
                {
                    o << "{}";
                    return;
                }

                o << "{";

                // increase indentation
                if (pretty_print)
                {
                    new_indent += indent_step;
                    o << "\n";
                }

                for (auto i = m_value.object->cbegin(); i != m_value.object->cend(); ++i)
                {
                    if (i != m_value.object->cbegin())
                    {
                        o << (pretty_print ? ",\n" : ",");
                    }
                    o << string_t(new_indent, ' ') << "\""
                      << escape_string(i->first) << "\":"
                      << (pretty_print ? " " : "");
                    i->second.dump(o, pretty_print, indent_step, new_indent);
                }

                // decrease indentation
                if (pretty_print)
                {
                    new_indent -= indent_step;
                    o << "\n";
                }

                o << string_t(new_indent, ' ') + "}";
                return;
            }

            case value_t::array:
            {
                if (m_value.array->empty())
                {
                    o << "[]";
                    return;
                }

                o << "[";

                // increase indentation
                if (pretty_print)
                {
                    new_indent += indent_step;
                    o << "\n";
                }

                for (auto i = m_value.array->cbegin(); i != m_value.array->cend(); ++i)
                {
                    if (i != m_value.array->cbegin())
                    {
                        o << (pretty_print ? ",\n" : ",");
                    }
                    o << string_t(new_indent, ' ');
                    i->dump(o, pretty_print, indent_step, new_indent);
                }

                // decrease indentation
                if (pretty_print)
                {
                    new_indent -= indent_step;
                    o << "\n";
                }

                o << string_t(new_indent, ' ') << "]";
                return;
            }

            case value_t::string:
            {
                o << string_t("\"") << escape_string(*m_value.string) << "\"";
                return;
            }

            case value_t::boolean:
            {
                o << (m_value.boolean ? "true" : "false");
                return;
            }

            case value_t::number_integer:
            {
                o << numtostr(m_value.number_integer).c_str();
                return;
            }

            case value_t::number_unsigned:
            {
                o << numtostr(m_value.number_unsigned).c_str();
                return;
            }

            case value_t::number_float:
            {
                o << numtostr(m_value.number_float).c_str();
                return;
            }

            case value_t::discarded:
            {
                o << "<discarded>";
                return;
            }

            case value_t::null:
            {
                o << "null";
                return;
            }
        }
    }

  private:
    //////////////////////
    // member variables //
    //////////////////////

    /// the type of the current element
    value_t m_type = value_t::null;

    /// the value of the current element
    json_value m_value = {};


  private:
    ///////////////
    // iterators //
    ///////////////

    /*!
    @brief an iterator for primitive JSON types

    This class models an iterator for primitive JSON types (boolean, number,
    string). It's only purpose is to allow the iterator/const_iterator classes
    to "iterate" over primitive values. Internally, the iterator is modeled by
    a `difference_type` variable. Value begin_value (`0`) models the begin,
    end_value (`1`) models past the end.
    */
    class primitive_iterator_t
    {
      public:

        difference_type get_value() const noexcept
        {
            return m_it;
        }
        /// set iterator to a defined beginning
        void set_begin() noexcept
        {
            m_it = begin_value;
        }

        /// set iterator to a defined past the end
        void set_end() noexcept
        {
            m_it = end_value;
        }

        /// return whether the iterator can be dereferenced
        constexpr bool is_begin() const noexcept
        {
            return (m_it == begin_value);
        }

        /// return whether the iterator is at end
        constexpr bool is_end() const noexcept
        {
            return (m_it == end_value);
        }

        friend constexpr bool operator==(primitive_iterator_t lhs, primitive_iterator_t rhs) noexcept
        {
            return lhs.m_it == rhs.m_it;
        }

        friend constexpr bool operator!=(primitive_iterator_t lhs, primitive_iterator_t rhs) noexcept
        {
            return !(lhs == rhs);
        }

        friend constexpr bool operator<(primitive_iterator_t lhs, primitive_iterator_t rhs) noexcept
        {
            return lhs.m_it < rhs.m_it;
        }

        friend constexpr bool operator<=(primitive_iterator_t lhs, primitive_iterator_t rhs) noexcept
        {
            return lhs.m_it <= rhs.m_it;
        }

        friend constexpr bool operator>(primitive_iterator_t lhs, primitive_iterator_t rhs) noexcept
        {
            return lhs.m_it > rhs.m_it;
        }

        friend constexpr bool operator>=(primitive_iterator_t lhs, primitive_iterator_t rhs) noexcept
        {
            return lhs.m_it >= rhs.m_it;
        }

        primitive_iterator_t operator+(difference_type i)
        {
            auto result = *this;
            result += i;
            return result;
        }

        friend constexpr difference_type operator-(primitive_iterator_t lhs, primitive_iterator_t rhs) noexcept
        {
            return lhs.m_it - rhs.m_it;
        }

        friend std::ostream& operator<<(std::ostream& os, primitive_iterator_t it)
        {
            return os << it.m_it;
        }

        primitive_iterator_t& operator++()
        {
            ++m_it;
            return *this;
        }

        primitive_iterator_t operator++(int)
        {
            auto result = *this;
            m_it++;
            return result;
        }

        primitive_iterator_t& operator--()
        {
            --m_it;
            return *this;
        }

        primitive_iterator_t operator--(int)
        {
            auto result = *this;
            m_it--;
            return result;
        }

        primitive_iterator_t& operator+=(difference_type n)
        {
            m_it += n;
            return *this;
        }

        primitive_iterator_t& operator-=(difference_type n)
        {
            m_it -= n;
            return *this;
        }

      private:
        static constexpr difference_type begin_value = 0;
        static constexpr difference_type end_value = begin_value + 1;

        /// iterator as signed integer type
        difference_type m_it = std::numeric_limits<std::ptrdiff_t>::denorm_min();
    };

    /*!
    @brief an iterator value

    @note This structure could easily be a union, but MSVC currently does not
    allow unions members with complex constructors, see
    https://github.com/nlohmann/json/pull/105.
    */
    struct internal_iterator
    {
        /// iterator for JSON objects
        typename object_t::iterator object_iterator;
        /// iterator for JSON arrays
        typename array_t::iterator array_iterator;
        /// generic iterator for all other types
        primitive_iterator_t primitive_iterator;

        /// create an uninitialized internal_iterator
        internal_iterator() noexcept
            : object_iterator(), array_iterator(), primitive_iterator()
        {}
    };

    /// proxy class for the iterator_wrapper functions
    template<typename IteratorType>
    class iteration_proxy
    {
      private:
        /// helper class for iteration
        class iteration_proxy_internal
        {
          private:
            /// the iterator
            IteratorType anchor;
            /// an index for arrays (used to create key names)
            size_t array_index = 0;

          public:
            explicit iteration_proxy_internal(IteratorType it) noexcept
                : anchor(it)
            {}

            /// dereference operator (needed for range-based for)
            iteration_proxy_internal& operator*()
            {
                return *this;
            }

            /// increment operator (needed for range-based for)
            iteration_proxy_internal& operator++()
            {
                ++anchor;
                ++array_index;

                return *this;
            }

            /// inequality operator (needed for range-based for)
            bool operator!= (const iteration_proxy_internal& o) const
            {
                return anchor != o.anchor;
            }

            /// return key of the iterator
            typename basic_json::string_t key() const
            {
                assert(anchor.m_object != nullptr);

                switch (anchor.m_object->type())
                {
                    // use integer array index as key
                    case value_t::array:
                    {
                        return std::to_string(array_index);
                    }

                    // use key from the object
                    case value_t::object:
                    {
                        return anchor.key();
                    }

                    // use an empty key for all primitive types
                    default:
                    {
                        return "";
                    }
                }
            }

            /// return value of the iterator
            typename IteratorType::reference value() const
            {
                return anchor.value();
            }
        };

        /// the container to iterate
        typename IteratorType::reference container;

      public:
        /// construct iteration proxy from a container
        explicit iteration_proxy(typename IteratorType::reference cont)
            : container(cont)
        {}

        /// return iterator begin (needed for range-based for)
        iteration_proxy_internal begin() noexcept
        {
            return iteration_proxy_internal(container.begin());
        }

        /// return iterator end (needed for range-based for)
        iteration_proxy_internal end() noexcept
        {
            return iteration_proxy_internal(container.end());
        }
    };

  public:
    /*!
    @brief a template for a random access iterator for the @ref basic_json class

    This class implements a both iterators (iterator and const_iterator) for the
    @ref basic_json class.

    @note An iterator is called *initialized* when a pointer to a JSON value
          has been set (e.g., by a constructor or a copy assignment). If the
          iterator is default-constructed, it is *uninitialized* and most
          methods are undefined. **The library uses assertions to detect calls
          on uninitialized iterators.**

    @requirement The class satisfies the following concept requirements:
    - [RandomAccessIterator](http://en.cppreference.com/w/cpp/concept/RandomAccessIterator):
      The iterator that can be moved to point (forward and backward) to any
      element in constant time.

    @since version 1.0.0, simplified in version 2.0.9
    */
    template<typename U>
    class iter_impl : public std::iterator<std::random_access_iterator_tag, U>
    {
        /// allow basic_json to access private members
        friend class basic_json;

        // make sure U is basic_json or const basic_json
        static_assert(std::is_same<U, basic_json>::value
                      or std::is_same<U, const basic_json>::value,
                      "iter_impl only accepts (const) basic_json");

      public:
        /// the type of the values when the iterator is dereferenced
        using value_type = typename basic_json::value_type;
        /// a type to represent differences between iterators
        using difference_type = typename basic_json::difference_type;
        /// defines a pointer to the type iterated over (value_type)
        using pointer = typename std::conditional<std::is_const<U>::value,
              typename basic_json::const_pointer,
              typename basic_json::pointer>::type;
        /// defines a reference to the type iterated over (value_type)
        using reference = typename std::conditional<std::is_const<U>::value,
              typename basic_json::const_reference,
              typename basic_json::reference>::type;
        /// the category of the iterator
        using iterator_category = std::bidirectional_iterator_tag;

        /// default constructor
        iter_impl() = default;

        /*!
        @brief constructor for a given JSON instance
        @param[in] object  pointer to a JSON object for this iterator
        @pre object != nullptr
        @post The iterator is initialized; i.e. `m_object != nullptr`.
        */
        explicit iter_impl(pointer object) noexcept
            : m_object(object)
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    m_it.object_iterator = typename object_t::iterator();
                    break;
                }

                case basic_json::value_t::array:
                {
                    m_it.array_iterator = typename array_t::iterator();
                    break;
                }

                default:
                {
                    m_it.primitive_iterator = primitive_iterator_t();
                    break;
                }
            }
        }

        /*
        Use operator `const_iterator` instead of `const_iterator(const iterator&
        other) noexcept` to avoid two class definitions for @ref iterator and
        @ref const_iterator.

        This function is only called if this class is an @ref iterator. If this
        class is a @ref const_iterator this function is not called.
        */
        operator const_iterator() const
        {
            const_iterator ret;

            if (m_object)
            {
                ret.m_object = m_object;
                ret.m_it = m_it;
            }

            return ret;
        }

        /*!
        @brief copy constructor
        @param[in] other  iterator to copy from
        @note It is not checked whether @a other is initialized.
        */
        iter_impl(const iter_impl& other) noexcept
            : m_object(other.m_object), m_it(other.m_it)
        {}

        /*!
        @brief copy assignment
        @param[in,out] other  iterator to copy from
        @note It is not checked whether @a other is initialized.
        */
        iter_impl& operator=(iter_impl other) noexcept(
            std::is_nothrow_move_constructible<pointer>::value and
            std::is_nothrow_move_assignable<pointer>::value and
            std::is_nothrow_move_constructible<internal_iterator>::value and
            std::is_nothrow_move_assignable<internal_iterator>::value
        )
        {
            std::swap(m_object, other.m_object);
            std::swap(m_it, other.m_it);
            return *this;
        }

      private:
        /*!
        @brief set the iterator to the first value
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        void set_begin() noexcept
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    m_it.object_iterator = m_object->m_value.object->begin();
                    break;
                }

                case basic_json::value_t::array:
                {
                    m_it.array_iterator = m_object->m_value.array->begin();
                    break;
                }

                case basic_json::value_t::null:
                {
                    // set to end so begin()==end() is true: null is empty
                    m_it.primitive_iterator.set_end();
                    break;
                }

                default:
                {
                    m_it.primitive_iterator.set_begin();
                    break;
                }
            }
        }

        /*!
        @brief set the iterator past the last value
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        void set_end() noexcept
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    m_it.object_iterator = m_object->m_value.object->end();
                    break;
                }

                case basic_json::value_t::array:
                {
                    m_it.array_iterator = m_object->m_value.array->end();
                    break;
                }

                default:
                {
                    m_it.primitive_iterator.set_end();
                    break;
                }
            }
        }

      public:
        /*!
        @brief return a reference to the value pointed to by the iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        reference operator*() const
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    assert(m_it.object_iterator != m_object->m_value.object->end());
                    return m_it.object_iterator->second;
                }

                case basic_json::value_t::array:
                {
                    assert(m_it.array_iterator != m_object->m_value.array->end());
                    return *m_it.array_iterator;
                }

                case basic_json::value_t::null:
                {
                    JSON_THROW(std::out_of_range("cannot get value"));
                }

                default:
                {
                    if (m_it.primitive_iterator.is_begin())
                    {
                        return *m_object;
                    }

                    JSON_THROW(std::out_of_range("cannot get value"));
                }
            }
        }

        /*!
        @brief dereference the iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        pointer operator->() const
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    assert(m_it.object_iterator != m_object->m_value.object->end());
                    return &(m_it.object_iterator->second);
                }

                case basic_json::value_t::array:
                {
                    assert(m_it.array_iterator != m_object->m_value.array->end());
                    return &*m_it.array_iterator;
                }

                default:
                {
                    if (m_it.primitive_iterator.is_begin())
                    {
                        return m_object;
                    }

                    JSON_THROW(std::out_of_range("cannot get value"));
                }
            }
        }

        /*!
        @brief post-increment (it++)
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl operator++(int)
        {
            auto result = *this;
            ++(*this);
            return result;
        }

        /*!
        @brief pre-increment (++it)
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl& operator++()
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    std::advance(m_it.object_iterator, 1);
                    break;
                }

                case basic_json::value_t::array:
                {
                    std::advance(m_it.array_iterator, 1);
                    break;
                }

                default:
                {
                    ++m_it.primitive_iterator;
                    break;
                }
            }

            return *this;
        }

        /*!
        @brief post-decrement (it--)
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl operator--(int)
        {
            auto result = *this;
            --(*this);
            return result;
        }

        /*!
        @brief pre-decrement (--it)
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl& operator--()
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    std::advance(m_it.object_iterator, -1);
                    break;
                }

                case basic_json::value_t::array:
                {
                    std::advance(m_it.array_iterator, -1);
                    break;
                }

                default:
                {
                    --m_it.primitive_iterator;
                    break;
                }
            }

            return *this;
        }

        /*!
        @brief  comparison: equal
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        bool operator==(const iter_impl& other) const
        {
            // if objects are not the same, the comparison is undefined
            if (m_object != other.m_object)
            {
                JSON_THROW(std::domain_error("cannot compare iterators of different containers"));
            }

            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    return (m_it.object_iterator == other.m_it.object_iterator);
                }

                case basic_json::value_t::array:
                {
                    return (m_it.array_iterator == other.m_it.array_iterator);
                }

                default:
                {
                    return (m_it.primitive_iterator == other.m_it.primitive_iterator);
                }
            }
        }

        /*!
        @brief  comparison: not equal
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        bool operator!=(const iter_impl& other) const
        {
            return not operator==(other);
        }

        /*!
        @brief  comparison: smaller
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        bool operator<(const iter_impl& other) const
        {
            // if objects are not the same, the comparison is undefined
            if (m_object != other.m_object)
            {
                JSON_THROW(std::domain_error("cannot compare iterators of different containers"));
            }

            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    JSON_THROW(std::domain_error("cannot compare order of object iterators"));
                }

                case basic_json::value_t::array:
                {
                    return (m_it.array_iterator < other.m_it.array_iterator);
                }

                default:
                {
                    return (m_it.primitive_iterator < other.m_it.primitive_iterator);
                }
            }
        }

        /*!
        @brief  comparison: less than or equal
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        bool operator<=(const iter_impl& other) const
        {
            return not other.operator < (*this);
        }

        /*!
        @brief  comparison: greater than
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        bool operator>(const iter_impl& other) const
        {
            return not operator<=(other);
        }

        /*!
        @brief  comparison: greater than or equal
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        bool operator>=(const iter_impl& other) const
        {
            return not operator<(other);
        }

        /*!
        @brief  add to iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl& operator+=(difference_type i)
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    JSON_THROW(std::domain_error("cannot use offsets with object iterators"));
                }

                case basic_json::value_t::array:
                {
                    std::advance(m_it.array_iterator, i);
                    break;
                }

                default:
                {
                    m_it.primitive_iterator += i;
                    break;
                }
            }

            return *this;
        }

        /*!
        @brief  subtract from iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl& operator-=(difference_type i)
        {
            return operator+=(-i);
        }

        /*!
        @brief  add to iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl operator+(difference_type i)
        {
            auto result = *this;
            result += i;
            return result;
        }

        /*!
        @brief  subtract from iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        iter_impl operator-(difference_type i)
        {
            auto result = *this;
            result -= i;
            return result;
        }

        /*!
        @brief  return difference
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        difference_type operator-(const iter_impl& other) const
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    JSON_THROW(std::domain_error("cannot use offsets with object iterators"));
                }

                case basic_json::value_t::array:
                {
                    return m_it.array_iterator - other.m_it.array_iterator;
                }

                default:
                {
                    return m_it.primitive_iterator - other.m_it.primitive_iterator;
                }
            }
        }

        /*!
        @brief  access to successor
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        reference operator[](difference_type n) const
        {
            assert(m_object != nullptr);

            switch (m_object->m_type)
            {
                case basic_json::value_t::object:
                {
                    JSON_THROW(std::domain_error("cannot use operator[] for object iterators"));
                }

                case basic_json::value_t::array:
                {
                    return *std::next(m_it.array_iterator, n);
                }

                case basic_json::value_t::null:
                {
                    JSON_THROW(std::out_of_range("cannot get value"));
                }

                default:
                {
                    if (m_it.primitive_iterator.get_value() == -n)
                    {
                        return *m_object;
                    }

                    JSON_THROW(std::out_of_range("cannot get value"));
                }
            }
        }

        /*!
        @brief  return the key of an object iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        typename object_t::key_type key() const
        {
            assert(m_object != nullptr);

            if (m_object->is_object())
            {
                return m_it.object_iterator->first;
            }

            JSON_THROW(std::domain_error("cannot use key() for non-object iterators"));
        }

        /*!
        @brief  return the value of an iterator
        @pre The iterator is initialized; i.e. `m_object != nullptr`.
        */
        reference value() const
        {
            return operator*();
        }

      private:
        /// associated JSON instance
        pointer m_object = nullptr;
        /// the actual iterator of the associated instance
        internal_iterator m_it = internal_iterator();
    };

    /*!
    @brief a template for a reverse iterator class

    @tparam Base the base iterator type to reverse. Valid types are @ref
    iterator (to create @ref reverse_iterator) and @ref const_iterator (to
    create @ref const_reverse_iterator).

    @requirement The class satisfies the following concept requirements:
    - [RandomAccessIterator](http://en.cppreference.com/w/cpp/concept/RandomAccessIterator):
      The iterator that can be moved to point (forward and backward) to any
      element in constant time.
    - [OutputIterator](http://en.cppreference.com/w/cpp/concept/OutputIterator):
      It is possible to write to the pointed-to element (only if @a Base is
      @ref iterator).

    @since version 1.0.0
    */
    template<typename Base>
    class json_reverse_iterator : public std::reverse_iterator<Base>
    {
      public:
        /// shortcut to the reverse iterator adaptor
        using base_iterator = std::reverse_iterator<Base>;
        /// the reference type for the pointed-to element
        using reference = typename Base::reference;

        /// create reverse iterator from iterator
        json_reverse_iterator(const typename base_iterator::iterator_type& it) noexcept
            : base_iterator(it)
        {}

        /// create reverse iterator from base class
        json_reverse_iterator(const base_iterator& it) noexcept
            : base_iterator(it)
        {}

        /// post-increment (it++)
        json_reverse_iterator operator++(int)
        {
            return base_iterator::operator++(1);
        }

        /// pre-increment (++it)
        json_reverse_iterator& operator++()
        {
            base_iterator::operator++();
            return *this;
        }

        /// post-decrement (it--)
        json_reverse_iterator operator--(int)
        {
            return base_iterator::operator--(1);
        }

        /// pre-decrement (--it)
        json_reverse_iterator& operator--()
        {
            base_iterator::operator--();
            return *this;
        }

        /// add to iterator
        json_reverse_iterator& operator+=(difference_type i)
        {
            base_iterator::operator+=(i);
            return *this;
        }

        /// add to iterator
        json_reverse_iterator operator+(difference_type i) const
        {
            auto result = *this;
            result += i;
            return result;
        }

        /// subtract from iterator
        json_reverse_iterator operator-(difference_type i) const
        {
            auto result = *this;
            result -= i;
            return result;
        }

        /// return difference
        difference_type operator-(const json_reverse_iterator& other) const
        {
            return this->base() - other.base();
        }

        /// access to successor
        reference operator[](difference_type n) const
        {
            return *(this->operator+(n));
        }

        /// return the key of an object iterator
        typename object_t::key_type key() const
        {
            auto it = --this->base();
            return it.key();
        }

        /// return the value of an iterator
        reference value() const
        {
            auto it = --this->base();
            return it.operator * ();
        }
    };


  private:
    //////////////////////
    // lexer and parser //
    //////////////////////

    /*!
    @brief lexical analysis

    This class organizes the lexical analysis during JSON deserialization. The
    core of it is a scanner generated by [re2c](http://re2c.org) that
    processes a buffer and recognizes tokens according to RFC 7159.
    */
    class lexer
    {
      public:
        /// token types for the parser
        enum class token_type
        {
            uninitialized,   ///< indicating the scanner is uninitialized
            literal_true,    ///< the `true` literal
            literal_false,   ///< the `false` literal
            literal_null,    ///< the `null` literal
            value_string,    ///< a string -- use get_string() for actual value
            value_unsigned,  ///< an unsigned integer -- use get_number() for actual value
            value_integer,   ///< a signed integer -- use get_number() for actual value
            value_float,     ///< an floating point number -- use get_number() for actual value
            begin_array,     ///< the character for array begin `[`
            begin_object,    ///< the character for object begin `{`
            end_array,       ///< the character for array end `]`
            end_object,      ///< the character for object end `}`
            name_separator,  ///< the name separator `:`
            value_separator, ///< the value separator `,`
            parse_error,     ///< indicating a parse error
            end_of_input     ///< indicating the end of the input buffer
        };

        /// the char type to use in the lexer
        using lexer_char_t = unsigned char;

        /// a lexer from a buffer with given length
        lexer(const lexer_char_t* buff, const size_t len) noexcept
            : m_content(buff)
        {
            assert(m_content != nullptr);
            m_start = m_cursor = m_content;
            m_limit = m_content + len;
        }

        /// a lexer from an input stream
        explicit lexer(std::istream& s)
            : m_stream(&s), m_line_buffer()
        {
            // immediately abort if stream is erroneous
            if (s.fail())
            {
                JSON_THROW(std::invalid_argument("stream error"));
            }

            // fill buffer
            fill_line_buffer();

            // skip UTF-8 byte-order mark
            if (m_line_buffer.size() >= 3 and m_line_buffer.substr(0, 3) == "\xEF\xBB\xBF")
            {
                m_line_buffer[0] = ' ';
                m_line_buffer[1] = ' ';
                m_line_buffer[2] = ' ';
            }
        }

        // switch off unwanted functions (due to pointer members)
        lexer() = delete;
        lexer(const lexer&) = delete;
        lexer operator=(const lexer&) = delete;

        /*!
        @brief create a string from one or two Unicode code points

        There are two cases: (1) @a codepoint1 is in the Basic Multilingual
        Plane (U+0000 through U+FFFF) and @a codepoint2 is 0, or (2)
        @a codepoint1 and @a codepoint2 are a UTF-16 surrogate pair to
        represent a code point above U+FFFF.

        @param[in] codepoint1  the code point (can be high surrogate)
        @param[in] codepoint2  the code point (can be low surrogate or 0)

        @return string representation of the code point; the length of the
        result string is between 1 and 4 characters.

        @throw std::out_of_range if code point is > 0x10ffff; example: `"code
        points above 0x10FFFF are invalid"`
        @throw std::invalid_argument if the low surrogate is invalid; example:
        `""missing or wrong low surrogate""`

        @complexity Constant.

        @see <http://en.wikipedia.org/wiki/UTF-8#Sample_code>
        */
        static string_t to_unicode(const std::size_t codepoint1,
                                   const std::size_t codepoint2 = 0)
        {
            // calculate the code point from the given code points
            std::size_t codepoint = codepoint1;

            // check if codepoint1 is a high surrogate
            if (codepoint1 >= 0xD800 and codepoint1 <= 0xDBFF)
            {
                // check if codepoint2 is a low surrogate
                if (codepoint2 >= 0xDC00 and codepoint2 <= 0xDFFF)
                {
                    codepoint =
                        // high surrogate occupies the most significant 22 bits
                        (codepoint1 << 10)
                        // low surrogate occupies the least significant 15 bits
                        + codepoint2
                        // there is still the 0xD800, 0xDC00 and 0x10000 noise
                        // in the result so we have to subtract with:
                        // (0xD800 << 10) + DC00 - 0x10000 = 0x35FDC00
                        - 0x35FDC00;
                }
                else
                {
                    JSON_THROW(std::invalid_argument("missing or wrong low surrogate"));
                }
            }

            string_t result;

            if (codepoint < 0x80)
            {
                // 1-byte characters: 0xxxxxxx (ASCII)
                result.append(1, static_cast<typename string_t::value_type>(codepoint));
            }
            else if (codepoint <= 0x7ff)
            {
                // 2-byte characters: 110xxxxx 10xxxxxx
                result.append(1, static_cast<typename string_t::value_type>(0xC0 | ((codepoint >> 6) & 0x1F)));
                result.append(1, static_cast<typename string_t::value_type>(0x80 | (codepoint & 0x3F)));
            }
            else if (codepoint <= 0xffff)
            {
                // 3-byte characters: 1110xxxx 10xxxxxx 10xxxxxx
                result.append(1, static_cast<typename string_t::value_type>(0xE0 | ((codepoint >> 12) & 0x0F)));
                result.append(1, static_cast<typename string_t::value_type>(0x80 | ((codepoint >> 6) & 0x3F)));
                result.append(1, static_cast<typename string_t::value_type>(0x80 | (codepoint & 0x3F)));
            }
            else if (codepoint <= 0x10ffff)
            {
                // 4-byte characters: 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
                result.append(1, static_cast<typename string_t::value_type>(0xF0 | ((codepoint >> 18) & 0x07)));
                result.append(1, static_cast<typename string_t::value_type>(0x80 | ((codepoint >> 12) & 0x3F)));
                result.append(1, static_cast<typename string_t::value_type>(0x80 | ((codepoint >> 6) & 0x3F)));
                result.append(1, static_cast<typename string_t::value_type>(0x80 | (codepoint & 0x3F)));
            }
            else
            {
                JSON_THROW(std::out_of_range("code points above 0x10FFFF are invalid"));
            }

            return result;
        }

        /// return name of values of type token_type (only used for errors)
        static std::string token_type_name(const token_type t)
        {
            switch (t)
            {
                case token_type::uninitialized:
                    return "<uninitialized>";
                case token_type::literal_true:
                    return "true literal";
                case token_type::literal_false:
                    return "false literal";
                case token_type::literal_null:
                    return "null literal";
                case token_type::value_string:
                    return "string literal";
                case lexer::token_type::value_unsigned:
                case lexer::token_type::value_integer:
                case lexer::token_type::value_float:
                    return "number literal";
                case token_type::begin_array:
                    return "'['";
                case token_type::begin_object:
                    return "'{'";
                case token_type::end_array:
                    return "']'";
                case token_type::end_object:
                    return "'}'";
                case token_type::name_separator:
                    return "':'";
                case token_type::value_separator:
                    return "','";
                case token_type::parse_error:
                    return "<parse error>";
                case token_type::end_of_input:
                    return "end of input";
                default:
                {
                    // catch non-enum values
                    return "unknown token"; // LCOV_EXCL_LINE
                }
            }
        }

        /*!
        This function implements a scanner for JSON. It is specified using
        regular expressions that try to follow RFC 7159 as close as possible.
        These regular expressions are then translated into a minimized
        deterministic finite automaton (DFA) by the tool
        [re2c](http://re2c.org). As a result, the translated code for this
        function consists of a large block of code with `goto` jumps.

        @return the class of the next token read from the buffer

        @complexity Linear in the length of the input.\n

        Proposition: The loop below will always terminate for finite input.\n

        Proof (by contradiction): Assume a finite input. To loop forever, the
        loop must never hit code with a `break` statement. The only code
        snippets without a `break` statement are the continue statements for
        whitespace and byte-order-marks. To loop forever, the input must be an
        infinite sequence of whitespace or byte-order-marks. This contradicts
        the assumption of finite input, q.e.d.
        */
        token_type scan()
        {
            while (true)
            {
                // pointer for backtracking information
                m_marker = nullptr;

                // remember the begin of the token
                m_start = m_cursor;
                assert(m_start != nullptr);


                {
                    lexer_char_t yych;
                    unsigned int yyaccept = 0;
                    static const unsigned char yybm[] =
                    {
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,  32,  32,   0,   0,  32,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        160, 128,   0, 128, 128, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        192, 192, 192, 192, 192, 192, 192, 192,
                        192, 192, 128, 128, 128, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        128, 128, 128, 128,   0, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        128, 128, 128, 128, 128, 128, 128, 128,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,   0,   0,
                    };
                    if ((m_limit - m_cursor) < 5)
                    {
                        fill_line_buffer(5);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yybm[0 + yych] & 32)
                    {
                        goto basic_json_parser_6;
                    }
                    if (yych <= '[')
                    {
                        if (yych <= '-')
                        {
                            if (yych <= '"')
                            {
                                if (yych <= 0x00)
                                {
                                    goto basic_json_parser_2;
                                }
                                if (yych <= '!')
                                {
                                    goto basic_json_parser_4;
                                }
                                goto basic_json_parser_9;
                            }
                            else
                            {
                                if (yych <= '+')
                                {
                                    goto basic_json_parser_4;
                                }
                                if (yych <= ',')
                                {
                                    goto basic_json_parser_10;
                                }
                                goto basic_json_parser_12;
                            }
                        }
                        else
                        {
                            if (yych <= '9')
                            {
                                if (yych <= '/')
                                {
                                    goto basic_json_parser_4;
                                }
                                if (yych <= '0')
                                {
                                    goto basic_json_parser_13;
                                }
                                goto basic_json_parser_15;
                            }
                            else
                            {
                                if (yych <= ':')
                                {
                                    goto basic_json_parser_17;
                                }
                                if (yych <= 'Z')
                                {
                                    goto basic_json_parser_4;
                                }
                                goto basic_json_parser_19;
                            }
                        }
                    }
                    else
                    {
                        if (yych <= 'n')
                        {
                            if (yych <= 'e')
                            {
                                if (yych == ']')
                                {
                                    goto basic_json_parser_21;
                                }
                                goto basic_json_parser_4;
                            }
                            else
                            {
                                if (yych <= 'f')
                                {
                                    goto basic_json_parser_23;
                                }
                                if (yych <= 'm')
                                {
                                    goto basic_json_parser_4;
                                }
                                goto basic_json_parser_24;
                            }
                        }
                        else
                        {
                            if (yych <= 'z')
                            {
                                if (yych == 't')
                                {
                                    goto basic_json_parser_25;
                                }
                                goto basic_json_parser_4;
                            }
                            else
                            {
                                if (yych <= '{')
                                {
                                    goto basic_json_parser_26;
                                }
                                if (yych == '}')
                                {
                                    goto basic_json_parser_28;
                                }
                                goto basic_json_parser_4;
                            }
                        }
                    }
basic_json_parser_2:
                    ++m_cursor;
                    {
                        last_token_type = token_type::end_of_input;
                        break;
                    }
basic_json_parser_4:
                    ++m_cursor;
basic_json_parser_5:
                    {
                        last_token_type = token_type::parse_error;
                        break;
                    }
basic_json_parser_6:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yybm[0 + yych] & 32)
                    {
                        goto basic_json_parser_6;
                    }
                    {
                        continue;
                    }
basic_json_parser_9:
                    yyaccept = 0;
                    yych = *(m_marker = ++m_cursor);
                    if (yych <= 0x1F)
                    {
                        goto basic_json_parser_5;
                    }
                    if (yych <= 0x7F)
                    {
                        goto basic_json_parser_31;
                    }
                    if (yych <= 0xC1)
                    {
                        goto basic_json_parser_5;
                    }
                    if (yych <= 0xF4)
                    {
                        goto basic_json_parser_31;
                    }
                    goto basic_json_parser_5;
basic_json_parser_10:
                    ++m_cursor;
                    {
                        last_token_type = token_type::value_separator;
                        break;
                    }
basic_json_parser_12:
                    yych = *++m_cursor;
                    if (yych <= '/')
                    {
                        goto basic_json_parser_5;
                    }
                    if (yych <= '0')
                    {
                        goto basic_json_parser_43;
                    }
                    if (yych <= '9')
                    {
                        goto basic_json_parser_45;
                    }
                    goto basic_json_parser_5;
basic_json_parser_13:
                    yyaccept = 1;
                    yych = *(m_marker = ++m_cursor);
                    if (yych <= '9')
                    {
                        if (yych == '.')
                        {
                            goto basic_json_parser_47;
                        }
                        if (yych >= '0')
                        {
                            goto basic_json_parser_48;
                        }
                    }
                    else
                    {
                        if (yych <= 'E')
                        {
                            if (yych >= 'E')
                            {
                                goto basic_json_parser_51;
                            }
                        }
                        else
                        {
                            if (yych == 'e')
                            {
                                goto basic_json_parser_51;
                            }
                        }
                    }
basic_json_parser_14:
                    {
                        last_token_type = token_type::value_unsigned;
                        break;
                    }
basic_json_parser_15:
                    yyaccept = 1;
                    m_marker = ++m_cursor;
                    if ((m_limit - m_cursor) < 3)
                    {
                        fill_line_buffer(3);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yybm[0 + yych] & 64)
                    {
                        goto basic_json_parser_15;
                    }
                    if (yych <= 'D')
                    {
                        if (yych == '.')
                        {
                            goto basic_json_parser_47;
                        }
                        goto basic_json_parser_14;
                    }
                    else
                    {
                        if (yych <= 'E')
                        {
                            goto basic_json_parser_51;
                        }
                        if (yych == 'e')
                        {
                            goto basic_json_parser_51;
                        }
                        goto basic_json_parser_14;
                    }
basic_json_parser_17:
                    ++m_cursor;
                    {
                        last_token_type = token_type::name_separator;
                        break;
                    }
basic_json_parser_19:
                    ++m_cursor;
                    {
                        last_token_type = token_type::begin_array;
                        break;
                    }
basic_json_parser_21:
                    ++m_cursor;
                    {
                        last_token_type = token_type::end_array;
                        break;
                    }
basic_json_parser_23:
                    yyaccept = 0;
                    yych = *(m_marker = ++m_cursor);
                    if (yych == 'a')
                    {
                        goto basic_json_parser_52;
                    }
                    goto basic_json_parser_5;
basic_json_parser_24:
                    yyaccept = 0;
                    yych = *(m_marker = ++m_cursor);
                    if (yych == 'u')
                    {
                        goto basic_json_parser_53;
                    }
                    goto basic_json_parser_5;
basic_json_parser_25:
                    yyaccept = 0;
                    yych = *(m_marker = ++m_cursor);
                    if (yych == 'r')
                    {
                        goto basic_json_parser_54;
                    }
                    goto basic_json_parser_5;
basic_json_parser_26:
                    ++m_cursor;
                    {
                        last_token_type = token_type::begin_object;
                        break;
                    }
basic_json_parser_28:
                    ++m_cursor;
                    {
                        last_token_type = token_type::end_object;
                        break;
                    }
basic_json_parser_30:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
basic_json_parser_31:
                    if (yybm[0 + yych] & 128)
                    {
                        goto basic_json_parser_30;
                    }
                    if (yych <= 0xE0)
                    {
                        if (yych <= '\\')
                        {
                            if (yych <= 0x1F)
                            {
                                goto basic_json_parser_32;
                            }
                            if (yych <= '"')
                            {
                                goto basic_json_parser_33;
                            }
                            goto basic_json_parser_35;
                        }
                        else
                        {
                            if (yych <= 0xC1)
                            {
                                goto basic_json_parser_32;
                            }
                            if (yych <= 0xDF)
                            {
                                goto basic_json_parser_36;
                            }
                            goto basic_json_parser_37;
                        }
                    }
                    else
                    {
                        if (yych <= 0xEF)
                        {
                            if (yych == 0xED)
                            {
                                goto basic_json_parser_39;
                            }
                            goto basic_json_parser_38;
                        }
                        else
                        {
                            if (yych <= 0xF0)
                            {
                                goto basic_json_parser_40;
                            }
                            if (yych <= 0xF3)
                            {
                                goto basic_json_parser_41;
                            }
                            if (yych <= 0xF4)
                            {
                                goto basic_json_parser_42;
                            }
                        }
                    }
basic_json_parser_32:
                    m_cursor = m_marker;
                    if (yyaccept <= 1)
                    {
                        if (yyaccept == 0)
                        {
                            goto basic_json_parser_5;
                        }
                        else
                        {
                            goto basic_json_parser_14;
                        }
                    }
                    else
                    {
                        if (yyaccept == 2)
                        {
                            goto basic_json_parser_44;
                        }
                        else
                        {
                            goto basic_json_parser_58;
                        }
                    }
basic_json_parser_33:
                    ++m_cursor;
                    {
                        last_token_type = token_type::value_string;
                        break;
                    }
basic_json_parser_35:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 'e')
                    {
                        if (yych <= '/')
                        {
                            if (yych == '"')
                            {
                                goto basic_json_parser_30;
                            }
                            if (yych <= '.')
                            {
                                goto basic_json_parser_32;
                            }
                            goto basic_json_parser_30;
                        }
                        else
                        {
                            if (yych <= '\\')
                            {
                                if (yych <= '[')
                                {
                                    goto basic_json_parser_32;
                                }
                                goto basic_json_parser_30;
                            }
                            else
                            {
                                if (yych == 'b')
                                {
                                    goto basic_json_parser_30;
                                }
                                goto basic_json_parser_32;
                            }
                        }
                    }
                    else
                    {
                        if (yych <= 'q')
                        {
                            if (yych <= 'f')
                            {
                                goto basic_json_parser_30;
                            }
                            if (yych == 'n')
                            {
                                goto basic_json_parser_30;
                            }
                            goto basic_json_parser_32;
                        }
                        else
                        {
                            if (yych <= 's')
                            {
                                if (yych <= 'r')
                                {
                                    goto basic_json_parser_30;
                                }
                                goto basic_json_parser_32;
                            }
                            else
                            {
                                if (yych <= 't')
                                {
                                    goto basic_json_parser_30;
                                }
                                if (yych <= 'u')
                                {
                                    goto basic_json_parser_55;
                                }
                                goto basic_json_parser_32;
                            }
                        }
                    }
basic_json_parser_36:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 0x7F)
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= 0xBF)
                    {
                        goto basic_json_parser_30;
                    }
                    goto basic_json_parser_32;
basic_json_parser_37:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 0x9F)
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= 0xBF)
                    {
                        goto basic_json_parser_36;
                    }
                    goto basic_json_parser_32;
basic_json_parser_38:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 0x7F)
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= 0xBF)
                    {
                        goto basic_json_parser_36;
                    }
                    goto basic_json_parser_32;
basic_json_parser_39:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 0x7F)
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= 0x9F)
                    {
                        goto basic_json_parser_36;
                    }
                    goto basic_json_parser_32;
basic_json_parser_40:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 0x8F)
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= 0xBF)
                    {
                        goto basic_json_parser_38;
                    }
                    goto basic_json_parser_32;
basic_json_parser_41:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 0x7F)
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= 0xBF)
                    {
                        goto basic_json_parser_38;
                    }
                    goto basic_json_parser_32;
basic_json_parser_42:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 0x7F)
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= 0x8F)
                    {
                        goto basic_json_parser_38;
                    }
                    goto basic_json_parser_32;
basic_json_parser_43:
                    yyaccept = 2;
                    yych = *(m_marker = ++m_cursor);
                    if (yych <= '9')
                    {
                        if (yych == '.')
                        {
                            goto basic_json_parser_47;
                        }
                        if (yych >= '0')
                        {
                            goto basic_json_parser_48;
                        }
                    }
                    else
                    {
                        if (yych <= 'E')
                        {
                            if (yych >= 'E')
                            {
                                goto basic_json_parser_51;
                            }
                        }
                        else
                        {
                            if (yych == 'e')
                            {
                                goto basic_json_parser_51;
                            }
                        }
                    }
basic_json_parser_44:
                    {
                        last_token_type = token_type::value_integer;
                        break;
                    }
basic_json_parser_45:
                    yyaccept = 2;
                    m_marker = ++m_cursor;
                    if ((m_limit - m_cursor) < 3)
                    {
                        fill_line_buffer(3);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= '9')
                    {
                        if (yych == '.')
                        {
                            goto basic_json_parser_47;
                        }
                        if (yych <= '/')
                        {
                            goto basic_json_parser_44;
                        }
                        goto basic_json_parser_45;
                    }
                    else
                    {
                        if (yych <= 'E')
                        {
                            if (yych <= 'D')
                            {
                                goto basic_json_parser_44;
                            }
                            goto basic_json_parser_51;
                        }
                        else
                        {
                            if (yych == 'e')
                            {
                                goto basic_json_parser_51;
                            }
                            goto basic_json_parser_44;
                        }
                    }
basic_json_parser_47:
                    yych = *++m_cursor;
                    if (yych <= '/')
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych <= '9')
                    {
                        goto basic_json_parser_56;
                    }
                    goto basic_json_parser_32;
basic_json_parser_48:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= '/')
                    {
                        goto basic_json_parser_50;
                    }
                    if (yych <= '9')
                    {
                        goto basic_json_parser_48;
                    }
basic_json_parser_50:
                    {
                        last_token_type = token_type::parse_error;
                        break;
                    }
basic_json_parser_51:
                    yych = *++m_cursor;
                    if (yych <= ',')
                    {
                        if (yych == '+')
                        {
                            goto basic_json_parser_59;
                        }
                        goto basic_json_parser_32;
                    }
                    else
                    {
                        if (yych <= '-')
                        {
                            goto basic_json_parser_59;
                        }
                        if (yych <= '/')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= '9')
                        {
                            goto basic_json_parser_60;
                        }
                        goto basic_json_parser_32;
                    }
basic_json_parser_52:
                    yych = *++m_cursor;
                    if (yych == 'l')
                    {
                        goto basic_json_parser_62;
                    }
                    goto basic_json_parser_32;
basic_json_parser_53:
                    yych = *++m_cursor;
                    if (yych == 'l')
                    {
                        goto basic_json_parser_63;
                    }
                    goto basic_json_parser_32;
basic_json_parser_54:
                    yych = *++m_cursor;
                    if (yych == 'u')
                    {
                        goto basic_json_parser_64;
                    }
                    goto basic_json_parser_32;
basic_json_parser_55:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= '@')
                    {
                        if (yych <= '/')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= '9')
                        {
                            goto basic_json_parser_65;
                        }
                        goto basic_json_parser_32;
                    }
                    else
                    {
                        if (yych <= 'F')
                        {
                            goto basic_json_parser_65;
                        }
                        if (yych <= '`')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= 'f')
                        {
                            goto basic_json_parser_65;
                        }
                        goto basic_json_parser_32;
                    }
basic_json_parser_56:
                    yyaccept = 3;
                    m_marker = ++m_cursor;
                    if ((m_limit - m_cursor) < 3)
                    {
                        fill_line_buffer(3);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= 'D')
                    {
                        if (yych <= '/')
                        {
                            goto basic_json_parser_58;
                        }
                        if (yych <= '9')
                        {
                            goto basic_json_parser_56;
                        }
                    }
                    else
                    {
                        if (yych <= 'E')
                        {
                            goto basic_json_parser_51;
                        }
                        if (yych == 'e')
                        {
                            goto basic_json_parser_51;
                        }
                    }
basic_json_parser_58:
                    {
                        last_token_type = token_type::value_float;
                        break;
                    }
basic_json_parser_59:
                    yych = *++m_cursor;
                    if (yych <= '/')
                    {
                        goto basic_json_parser_32;
                    }
                    if (yych >= ':')
                    {
                        goto basic_json_parser_32;
                    }
basic_json_parser_60:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= '/')
                    {
                        goto basic_json_parser_58;
                    }
                    if (yych <= '9')
                    {
                        goto basic_json_parser_60;
                    }
                    goto basic_json_parser_58;
basic_json_parser_62:
                    yych = *++m_cursor;
                    if (yych == 's')
                    {
                        goto basic_json_parser_66;
                    }
                    goto basic_json_parser_32;
basic_json_parser_63:
                    yych = *++m_cursor;
                    if (yych == 'l')
                    {
                        goto basic_json_parser_67;
                    }
                    goto basic_json_parser_32;
basic_json_parser_64:
                    yych = *++m_cursor;
                    if (yych == 'e')
                    {
                        goto basic_json_parser_69;
                    }
                    goto basic_json_parser_32;
basic_json_parser_65:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= '@')
                    {
                        if (yych <= '/')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= '9')
                        {
                            goto basic_json_parser_71;
                        }
                        goto basic_json_parser_32;
                    }
                    else
                    {
                        if (yych <= 'F')
                        {
                            goto basic_json_parser_71;
                        }
                        if (yych <= '`')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= 'f')
                        {
                            goto basic_json_parser_71;
                        }
                        goto basic_json_parser_32;
                    }
basic_json_parser_66:
                    yych = *++m_cursor;
                    if (yych == 'e')
                    {
                        goto basic_json_parser_72;
                    }
                    goto basic_json_parser_32;
basic_json_parser_67:
                    ++m_cursor;
                    {
                        last_token_type = token_type::literal_null;
                        break;
                    }
basic_json_parser_69:
                    ++m_cursor;
                    {
                        last_token_type = token_type::literal_true;
                        break;
                    }
basic_json_parser_71:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= '@')
                    {
                        if (yych <= '/')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= '9')
                        {
                            goto basic_json_parser_74;
                        }
                        goto basic_json_parser_32;
                    }
                    else
                    {
                        if (yych <= 'F')
                        {
                            goto basic_json_parser_74;
                        }
                        if (yych <= '`')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= 'f')
                        {
                            goto basic_json_parser_74;
                        }
                        goto basic_json_parser_32;
                    }
basic_json_parser_72:
                    ++m_cursor;
                    {
                        last_token_type = token_type::literal_false;
                        break;
                    }
basic_json_parser_74:
                    ++m_cursor;
                    if (m_limit <= m_cursor)
                    {
                        fill_line_buffer(1);    // LCOV_EXCL_LINE
                    }
                    yych = *m_cursor;
                    if (yych <= '@')
                    {
                        if (yych <= '/')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= '9')
                        {
                            goto basic_json_parser_30;
                        }
                        goto basic_json_parser_32;
                    }
                    else
                    {
                        if (yych <= 'F')
                        {
                            goto basic_json_parser_30;
                        }
                        if (yych <= '`')
                        {
                            goto basic_json_parser_32;
                        }
                        if (yych <= 'f')
                        {
                            goto basic_json_parser_30;
                        }
                        goto basic_json_parser_32;
                    }
                }

            }

            return last_token_type;
        }

        /*!
        @brief append data from the stream to the line buffer

        This function is called by the scan() function when the end of the
        buffer (`m_limit`) is reached and the `m_cursor` pointer cannot be
        incremented without leaving the limits of the line buffer. Note re2c
        decides when to call this function.

        If the lexer reads from contiguous storage, there is no trailing null
        byte. Therefore, this function must make sure to add these padding
        null bytes.

        If the lexer reads from an input stream, this function reads the next
        line of the input.

        @pre
            p p p p p p u u u u u x . . . . . .
            ^           ^       ^   ^
            m_content   m_start |   m_limit
                                m_cursor

        @post
            u u u u u x x x x x x x . . . . . .
            ^       ^               ^
            |       m_cursor        m_limit
            m_start
            m_content
        */
        void fill_line_buffer(size_t n = 0)
        {
            // if line buffer is used, m_content points to its data
            assert(m_line_buffer.empty()
                   or m_content == reinterpret_cast<const lexer_char_t*>(m_line_buffer.data()));

            // if line buffer is used, m_limit is set past the end of its data
            assert(m_line_buffer.empty()
                   or m_limit == m_content + m_line_buffer.size());

            // pointer relationships
            assert(m_content <= m_start);
            assert(m_start <= m_cursor);
            assert(m_cursor <= m_limit);
            assert(m_marker == nullptr or m_marker  <= m_limit);

            // number of processed characters (p)
            const auto num_processed_chars = static_cast<size_t>(m_start - m_content);
            // offset for m_marker wrt. to m_start
            const auto offset_marker = (m_marker == nullptr) ? 0 : m_marker - m_start;
            // number of unprocessed characters (u)
            const auto offset_cursor = m_cursor - m_start;

            // no stream is used or end of file is reached
            if (m_stream == nullptr or m_stream->eof())
            {
                // m_start may or may not be pointing into m_line_buffer at
                // this point. We trust the standard library to do the right
                // thing. See http://stackoverflow.com/q/28142011/266378
                m_line_buffer.assign(m_start, m_limit);

                // append n characters to make sure that there is sufficient
                // space between m_cursor and m_limit
                m_line_buffer.append(1, '\x00');
                if (n > 0)
                {
                    m_line_buffer.append(n - 1, '\x01');
                }
            }
            else
            {
                // delete processed characters from line buffer
                m_line_buffer.erase(0, num_processed_chars);
                // read next line from input stream
                m_line_buffer_tmp.clear();
                std::getline(*m_stream, m_line_buffer_tmp, '\n');

                // add line with newline symbol to the line buffer
                m_line_buffer += m_line_buffer_tmp;
                m_line_buffer.push_back('\n');
            }

            // set pointers
            m_content = reinterpret_cast<const lexer_char_t*>(m_line_buffer.data());
            assert(m_content != nullptr);
            m_start  = m_content;
            m_marker = m_start + offset_marker;
            m_cursor = m_start + offset_cursor;
            m_limit  = m_start + m_line_buffer.size();
        }

        /// return string representation of last read token
        string_t get_token_string() const
        {
            assert(m_start != nullptr);
            return string_t(reinterpret_cast<typename string_t::const_pointer>(m_start),
                            static_cast<size_t>(m_cursor - m_start));
        }

        /*!
        @brief return string value for string tokens

        The function iterates the characters between the opening and closing
        quotes of the string value. The complete string is the range
        [m_start,m_cursor). Consequently, we iterate from m_start+1 to
        m_cursor-1.

        We differentiate two cases:

        1. Escaped characters. In this case, a new character is constructed
           according to the nature of the escape. Some escapes create new
           characters (e.g., `"\\n"` is replaced by `"\n"`), some are copied
           as is (e.g., `"\\\\"`). Furthermore, Unicode escapes of the shape
           `"\\uxxxx"` need special care. In this case, to_unicode takes care
           of the construction of the values.
        2. Unescaped characters are copied as is.

        @pre `m_cursor - m_start >= 2`, meaning the length of the last token
        is at least 2 bytes which is trivially true for any string (which
        consists of at least two quotes).

            " c1 c2 c3 ... "
            ^                ^
            m_start          m_cursor

        @complexity Linear in the length of the string.\n

        Lemma: The loop body will always terminate.\n

        Proof (by contradiction): Assume the loop body does not terminate. As
        the loop body does not contain another loop, one of the called
        functions must never return. The called functions are `std::strtoul`
        and to_unicode. Neither function can loop forever, so the loop body
        will never loop forever which contradicts the assumption that the loop
        body does not terminate, q.e.d.\n

        Lemma: The loop condition for the for loop is eventually false.\n

        Proof (by contradiction): Assume the loop does not terminate. Due to
        the above lemma, this can only be due to a tautological loop
        condition; that is, the loop condition i < m_cursor - 1 must always be
        true. Let x be the change of i for any loop iteration. Then
        m_start + 1 + x < m_cursor - 1 must hold to loop indefinitely. This
        can be rephrased to m_cursor - m_start - 2 > x. With the
        precondition, we x <= 0, meaning that the loop condition holds
        indefinitely if i is always decreased. However, observe that the value
        of i is strictly increasing with each iteration, as it is incremented
        by 1 in the iteration expression and never decremented inside the loop
        body. Hence, the loop condition will eventually be false which
        contradicts the assumption that the loop condition is a tautology,
        q.e.d.

        @return string value of current token without opening and closing
        quotes
        @throw std::out_of_range if to_unicode fails
        */
        string_t get_string() const
        {
            assert(m_cursor - m_start >= 2);

            string_t result;
            result.reserve(static_cast<size_t>(m_cursor - m_start - 2));

            // iterate the result between the quotes
            for (const lexer_char_t* i = m_start + 1; i < m_cursor - 1; ++i)
            {
                // find next escape character
                auto e = std::find(i, m_cursor - 1, '\\');
                if (e != i)
                {
                    // see https://github.com/nlohmann/json/issues/365#issuecomment-262874705
                    for (auto k = i; k < e; k++)
                    {
                        result.push_back(static_cast<typename string_t::value_type>(*k));
                    }
                    i = e - 1; // -1 because of ++i
                }
                else
                {
                    // processing escaped character
                    // read next character
                    ++i;

                    switch (*i)
                    {
                        // the default escapes
                        case 't':
                        {
                            result += "\t";
                            break;
                        }
                        case 'b':
                        {
                            result += "\b";
                            break;
                        }
                        case 'f':
                        {
                            result += "\f";
                            break;
                        }
                        case 'n':
                        {
                            result += "\n";
                            break;
                        }
                        case 'r':
                        {
                            result += "\r";
                            break;
                        }
                        case '\\':
                        {
                            result += "\\";
                            break;
                        }
                        case '/':
                        {
                            result += "/";
                            break;
                        }
                        case '"':
                        {
                            result += "\"";
                            break;
                        }

                        // unicode
                        case 'u':
                        {
                            // get code xxxx from uxxxx
                            auto codepoint = std::strtoul(std::string(reinterpret_cast<typename string_t::const_pointer>(i + 1),
                                                          4).c_str(), nullptr, 16);

                            // check if codepoint is a high surrogate
                            if (codepoint >= 0xD800 and codepoint <= 0xDBFF)
                            {
                                // make sure there is a subsequent unicode
                                if ((i + 6 >= m_limit) or * (i + 5) != '\\' or * (i + 6) != 'u')
                                {
                                    JSON_THROW(std::invalid_argument("missing low surrogate"));
                                }

                                // get code yyyy from uxxxx\uyyyy
                                auto codepoint2 = std::strtoul(std::string(reinterpret_cast<typename string_t::const_pointer>
                                                               (i + 7), 4).c_str(), nullptr, 16);
                                result += to_unicode(codepoint, codepoint2);
                                // skip the next 10 characters (xxxx\uyyyy)
                                i += 10;
                            }
                            else if (codepoint >= 0xDC00 and codepoint <= 0xDFFF)
                            {
                                // we found a lone low surrogate
                                JSON_THROW(std::invalid_argument("missing high surrogate"));
                            }
                            else
                            {
                                // add unicode character(s)
                                result += to_unicode(codepoint);
                                // skip the next four characters (xxxx)
                                i += 4;
                            }
                            break;
                        }
                    }
                }
            }

            return result;
        }


        /*!
        @brief parse string into a built-in arithmetic type as if the current
               locale is POSIX.

        @note in floating-point case strtod may parse past the token's end -
              this is not an error

        @note any leading blanks are not handled
        */
        struct strtonum
        {
          public:
            strtonum(const char* start, const char* end)
                : m_start(start), m_end(end)
            {}

            /*!
            @return true iff parsed successfully as number of type T

            @param[in,out] val shall contain parsed value, or undefined value
            if could not parse
            */
            template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
            bool to(T& val) const
            {
                return parse(val, std::is_integral<T>());
            }

          private:
            const char* const m_start = nullptr;
            const char* const m_end = nullptr;

            // floating-point conversion

            // overloaded wrappers for strtod/strtof/strtold
            // that will be called from parse<floating_point_t>
            static void strtof(float& f, const char* str, char** endptr)
            {
                f = std::strtof(str, endptr);
            }

            static void strtof(double& f, const char* str, char** endptr)
            {
                f = std::strtod(str, endptr);
            }

            static void strtof(long double& f, const char* str, char** endptr)
            {
                f = std::strtold(str, endptr);
            }

            template<typename T>
            bool parse(T& value, /*is_integral=*/std::false_type) const
            {
                // replace decimal separator with locale-specific version,
                // when necessary; data will point to either the original
                // string, or buf, or tempstr containing the fixed string.
                std::string tempstr;
                std::array<char, 64> buf;
                const size_t len = static_cast<size_t>(m_end - m_start);

                // lexer will reject empty numbers
                assert(len > 0);

                // since dealing with strtod family of functions, we're
                // getting the decimal point char from the C locale facilities
                // instead of C++'s numpunct facet of the current std::locale
                const auto loc = localeconv();
                assert(loc != nullptr);
                const char decimal_point_char = (loc->decimal_point == nullptr) ? '.' : loc->decimal_point[0];

                const char* data = m_start;

                if (decimal_point_char != '.')
                {
                    const size_t ds_pos = static_cast<size_t>(std::find(m_start, m_end, '.') - m_start);

                    if (ds_pos != len)
                    {
                        // copy the data into the local buffer or tempstr, if
                        // buffer is too small; replace decimal separator, and
                        // update data to point to the modified bytes
                        if ((len + 1) < buf.size())
                        {
                            std::copy(m_start, m_end, buf.begin());
                            buf[len] = 0;
                            buf[ds_pos] = decimal_point_char;
                            data = buf.data();
                        }
                        else
                        {
                            tempstr.assign(m_start, m_end);
                            tempstr[ds_pos] = decimal_point_char;
                            data = tempstr.c_str();
                        }
                    }
                }

                char* endptr = nullptr;
                value = 0;
                // this calls appropriate overload depending on T
                strtof(value, data, &endptr);

                // parsing was successful iff strtof parsed exactly the number
                // of characters determined by the lexer (len)
                const bool ok = (endptr == (data + len));

                if (ok and (value == static_cast<T>(0.0)) and (*data == '-'))
                {
                    // some implementations forget to negate the zero
                    value = -0.0;
                }

                return ok;
            }

            // integral conversion

            signed long long parse_integral(char** endptr, /*is_signed*/std::true_type) const
            {
                return std::strtoll(m_start, endptr, 10);
            }

            unsigned long long parse_integral(char** endptr, /*is_signed*/std::false_type) const
            {
                return std::strtoull(m_start, endptr, 10);
            }

            template<typename T>
            bool parse(T& value, /*is_integral=*/std::true_type) const
            {
                char* endptr = nullptr;
                errno = 0; // these are thread-local
                const auto x = parse_integral(&endptr, std::is_signed<T>());

                // called right overload?
                static_assert(std::is_signed<T>() == std::is_signed<decltype(x)>(), "");

                value = static_cast<T>(x);

                return (x == static_cast<decltype(x)>(value)) // x fits into destination T
                       and (x < 0) == (value < 0)             // preserved sign
                       //and ((x != 0) or is_integral())        // strto[u]ll did nto fail
                       and (errno == 0)                       // strto[u]ll did not overflow
                       and (m_start < m_end)                  // token was not empty
                       and (endptr == m_end);                 // parsed entire token exactly
            }
        };

        /*!
        @brief return number value for number tokens

        This function translates the last token into the most appropriate
        number type (either integer, unsigned integer or floating point),
        which is passed back to the caller via the result parameter.

        integral numbers that don't fit into the the range of the respective
        type are parsed as number_float_t

        floating-point values do not satisfy std::isfinite predicate
        are converted to value_t::null

        throws if the entire string [m_start .. m_cursor) cannot be
        interpreted as a number

        @param[out] result  @ref basic_json object to receive the number.
        @param[in]  token   the type of the number token
        */
        bool get_number(basic_json& result, const token_type token) const
        {
            assert(m_start != nullptr);
            assert(m_start < m_cursor);
            assert((token == token_type::value_unsigned) or
                   (token == token_type::value_integer) or
                   (token == token_type::value_float));

            strtonum num_converter(reinterpret_cast<const char*>(m_start),
                                   reinterpret_cast<const char*>(m_cursor));

            switch (token)
            {
                case lexer::token_type::value_unsigned:
                {
                    number_unsigned_t val;
                    if (num_converter.to(val))
                    {
                        // parsing successful
                        result.m_type = value_t::number_unsigned;
                        result.m_value = val;
                        return true;
                    }
                    break;
                }

                case lexer::token_type::value_integer:
                {
                    number_integer_t val;
                    if (num_converter.to(val))
                    {
                        // parsing successful
                        result.m_type = value_t::number_integer;
                        result.m_value = val;
                        return true;
                    }
                    break;
                }

                default:
                {
                    break;
                }
            }

            // parse float (either explicitly or because a previous conversion
            // failed)
            number_float_t val;
            if (num_converter.to(val))
            {
                // parsing successful
                result.m_type = value_t::number_float;
                result.m_value = val;

                // replace infinity and NAN by null
                if (not std::isfinite(result.m_value.number_float))
                {
                    result.m_type  = value_t::null;
                    result.m_value = basic_json::json_value();
                }

                return true;
            }

            // couldn't parse number in any format
            return false;
        }

      private:
        /// optional input stream
        std::istream* m_stream = nullptr;
        /// line buffer buffer for m_stream
        string_t m_line_buffer {};
        /// used for filling m_line_buffer
        string_t m_line_buffer_tmp {};
        /// the buffer pointer
        const lexer_char_t* m_content = nullptr;
        /// pointer to the beginning of the current symbol
        const lexer_char_t* m_start = nullptr;
        /// pointer for backtracking information
        const lexer_char_t* m_marker = nullptr;
        /// pointer to the current symbol
        const lexer_char_t* m_cursor = nullptr;
        /// pointer to the end of the buffer
        const lexer_char_t* m_limit = nullptr;
        /// the last token type
        token_type last_token_type = token_type::end_of_input;
    };

    /*!
    @brief syntax analysis

    This class implements a recursive decent parser.
    */
    class parser
    {
      public:
        /// a parser reading from a string literal
        parser(const char* buff, const parser_callback_t cb = nullptr)
            : callback(cb),
              m_lexer(reinterpret_cast<const typename lexer::lexer_char_t*>(buff), std::strlen(buff))
        {}

        /// a parser reading from an input stream
        parser(std::istream& is, const parser_callback_t cb = nullptr)
            : callback(cb), m_lexer(is)
        {}

        /// a parser reading from an iterator range with contiguous storage
        template<class IteratorType, typename std::enable_if<
                     std::is_same<typename std::iterator_traits<IteratorType>::iterator_category, std::random_access_iterator_tag>::value
                     , int>::type
                 = 0>
        parser(IteratorType first, IteratorType last, const parser_callback_t cb = nullptr)
            : callback(cb),
              m_lexer(reinterpret_cast<const typename lexer::lexer_char_t*>(&(*first)),
                      static_cast<size_t>(std::distance(first, last)))
        {}

        /// public parser interface
        basic_json parse()
        {
            // read first token
            get_token();

            basic_json result = parse_internal(true);
            result.assert_invariant();

            expect(lexer::token_type::end_of_input);

            // return parser result and replace it with null in case the
            // top-level value was discarded by the callback function
            return result.is_discarded() ? basic_json() : std::move(result);
        }

      private:
        /// the actual parser
        basic_json parse_internal(bool keep)
        {
            auto result = basic_json(value_t::discarded);

            switch (last_token)
            {
                case lexer::token_type::begin_object:
                {
                    if (keep and (not callback
                                  or ((keep = callback(depth++, parse_event_t::object_start, result)) != 0)))
                    {
                        // explicitly set result to object to cope with {}
                        result.m_type = value_t::object;
                        result.m_value = value_t::object;
                    }

                    // read next token
                    get_token();

                    // closing } -> we are done
                    if (last_token == lexer::token_type::end_object)
                    {
                        get_token();
                        if (keep and callback and not callback(--depth, parse_event_t::object_end, result))
                        {
                            result = basic_json(value_t::discarded);
                        }
                        return result;
                    }

                    // no comma is expected here
                    unexpect(lexer::token_type::value_separator);

                    // otherwise: parse key-value pairs
                    do
                    {
                        // ugly, but could be fixed with loop reorganization
                        if (last_token == lexer::token_type::value_separator)
                        {
                            get_token();
                        }

                        // store key
                        expect(lexer::token_type::value_string);
                        const auto key = m_lexer.get_string();

                        bool keep_tag = false;
                        if (keep)
                        {
                            if (callback)
                            {
                                basic_json k(key);
                                keep_tag = callback(depth, parse_event_t::key, k);
                            }
                            else
                            {
                                keep_tag = true;
                            }
                        }

                        // parse separator (:)
                        get_token();
                        expect(lexer::token_type::name_separator);

                        // parse and add value
                        get_token();
                        auto value = parse_internal(keep);
                        if (keep and keep_tag and not value.is_discarded())
                        {
                            result[key] = std::move(value);
                        }
                    }
                    while (last_token == lexer::token_type::value_separator);

                    // closing }
                    expect(lexer::token_type::end_object);
                    get_token();
                    if (keep and callback and not callback(--depth, parse_event_t::object_end, result))
                    {
                        result = basic_json(value_t::discarded);
                    }

                    return result;
                }

                case lexer::token_type::begin_array:
                {
                    if (keep and (not callback
                                  or ((keep = callback(depth++, parse_event_t::array_start, result)) != 0)))
                    {
                        // explicitly set result to object to cope with []
                        result.m_type = value_t::array;
                        result.m_value = value_t::array;
                    }

                    // read next token
                    get_token();

                    // closing ] -> we are done
                    if (last_token == lexer::token_type::end_array)
                    {
                        get_token();
                        if (callback and not callback(--depth, parse_event_t::array_end, result))
                        {
                            result = basic_json(value_t::discarded);
                        }
                        return result;
                    }

                    // no comma is expected here
                    unexpect(lexer::token_type::value_separator);

                    // otherwise: parse values
                    do
                    {
                        // ugly, but could be fixed with loop reorganization
                        if (last_token == lexer::token_type::value_separator)
                        {
                            get_token();
                        }

                        // parse value
                        auto value = parse_internal(keep);
                        if (keep and not value.is_discarded())
                        {
                            result.push_back(std::move(value));
                        }
                    }
                    while (last_token == lexer::token_type::value_separator);

                    // closing ]
                    expect(lexer::token_type::end_array);
                    get_token();
                    if (keep and callback and not callback(--depth, parse_event_t::array_end, result))
                    {
                        result = basic_json(value_t::discarded);
                    }

                    return result;
                }

                case lexer::token_type::literal_null:
                {
                    get_token();
                    result.m_type = value_t::null;
                    break;
                }

                case lexer::token_type::value_string:
                {
                    const auto s = m_lexer.get_string();
                    get_token();
                    result = basic_json(s);
                    break;
                }

                case lexer::token_type::literal_true:
                {
                    get_token();
                    result.m_type = value_t::boolean;
                    result.m_value = true;
                    break;
                }

                case lexer::token_type::literal_false:
                {
                    get_token();
                    result.m_type = value_t::boolean;
                    result.m_value = false;
                    break;
                }

                case lexer::token_type::value_unsigned:
                case lexer::token_type::value_integer:
                case lexer::token_type::value_float:
                {
                    m_lexer.get_number(result, last_token);
                    get_token();
                    break;
                }

                default:
                {
                    // the last token was unexpected
                    unexpect(last_token);
                }
            }

            if (keep and callback and not callback(depth, parse_event_t::value, result))
            {
                result = basic_json(value_t::discarded);
            }
            return result;
        }

        /// get next token from lexer
        typename lexer::token_type get_token()
        {
            last_token = m_lexer.scan();
            return last_token;
        }

        void expect(typename lexer::token_type t) const
        {
            if (t != last_token)
            {
                std::string error_msg = "parse error - unexpected ";
                error_msg += (last_token == lexer::token_type::parse_error ? ("'" +  m_lexer.get_token_string() +
                              "'") :
                              lexer::token_type_name(last_token));
                error_msg += "; expected " + lexer::token_type_name(t);
                JSON_THROW(std::invalid_argument(error_msg));
            }
        }

        void unexpect(typename lexer::token_type t) const
        {
            if (t == last_token)
            {
                std::string error_msg = "parse error - unexpected ";
                error_msg += (last_token == lexer::token_type::parse_error ? ("'" +  m_lexer.get_token_string() +
                              "'") :
                              lexer::token_type_name(last_token));
                JSON_THROW(std::invalid_argument(error_msg));
            }
        }

      private:
        /// current level of recursion
        int depth = 0;
        /// callback function
        const parser_callback_t callback = nullptr;
        /// the type of the last read token
        typename lexer::token_type last_token = lexer::token_type::uninitialized;
        /// the lexer
        lexer m_lexer;
    };

  public:
    /*!
    @brief JSON Pointer

    A JSON pointer defines a string syntax for identifying a specific value
    within a JSON document. It can be used with functions `at` and
    `operator[]`. Furthermore, JSON pointers are the base for JSON patches.

    @sa [RFC 6901](https://tools.ietf.org/html/rfc6901)

    @since version 2.0.0
    */
    class json_pointer
    {
        /// allow basic_json to access private members
        friend class basic_json;

      public:
        /*!
        @brief create JSON pointer

        Create a JSON pointer according to the syntax described in
        [Section 3 of RFC6901](https://tools.ietf.org/html/rfc6901#section-3).

        @param[in] s  string representing the JSON pointer; if omitted, the
                      empty string is assumed which references the whole JSON
                      value

        @throw std::domain_error if reference token is nonempty and does not
        begin with a slash (`/`); example: `"JSON pointer must be empty or
        begin with /"`
        @throw std::domain_error if a tilde (`~`) is not followed by `0`
        (representing `~`) or `1` (representing `/`); example: `"escape error:
        ~ must be followed with 0 or 1"`

        @liveexample{The example shows the construction several valid JSON
        pointers as well as the exceptional behavior.,json_pointer}

        @since version 2.0.0
        */
        explicit json_pointer(const std::string& s = "")
            : reference_tokens(split(s))
        {}

        /*!
        @brief return a string representation of the JSON pointer

        @invariant For each JSON pointer `ptr`, it holds:
        @code {.cpp}
        ptr == json_pointer(ptr.to_string());
        @endcode

        @return a string representation of the JSON pointer

        @liveexample{The example shows the result of `to_string`.,
        json_pointer__to_string}

        @since version 2.0.0
        */
        std::string to_string() const noexcept
        {
            return std::accumulate(reference_tokens.begin(),
                                   reference_tokens.end(), std::string{},
                                   [](const std::string & a, const std::string & b)
            {
                return a + "/" + escape(b);
            });
        }

        /// @copydoc to_string()
        operator std::string() const
        {
            return to_string();
        }

      private:
        /// remove and return last reference pointer
        std::string pop_back()
        {
            if (is_root())
            {
                JSON_THROW(std::domain_error("JSON pointer has no parent"));
            }

            auto last = reference_tokens.back();
            reference_tokens.pop_back();
            return last;
        }

        /// return whether pointer points to the root document
        bool is_root() const
        {
            return reference_tokens.empty();
        }

        json_pointer top() const
        {
            if (is_root())
            {
                JSON_THROW(std::domain_error("JSON pointer has no parent"));
            }

            json_pointer result = *this;
            result.reference_tokens = {reference_tokens[0]};
            return result;
        }

        /*!
        @brief create and return a reference to the pointed to value

        @complexity Linear in the number of reference tokens.
        */
        reference get_and_create(reference j) const
        {
            pointer result = &j;

            // in case no reference tokens exist, return a reference to the
            // JSON value j which will be overwritten by a primitive value
            for (const auto& reference_token : reference_tokens)
            {
                switch (result->m_type)
                {
                    case value_t::null:
                    {
                        if (reference_token == "0")
                        {
                            // start a new array if reference token is 0
                            result = &result->operator[](0);
                        }
                        else
                        {
                            // start a new object otherwise
                            result = &result->operator[](reference_token);
                        }
                        break;
                    }

                    case value_t::object:
                    {
                        // create an entry in the object
                        result = &result->operator[](reference_token);
                        break;
                    }

                    case value_t::array:
                    {
                        // create an entry in the array
                        result = &result->operator[](static_cast<size_type>(std::stoi(reference_token)));
                        break;
                    }

                    /*
                    The following code is only reached if there exists a
                    reference token _and_ the current value is primitive. In
                    this case, we have an error situation, because primitive
                    values may only occur as single value; that is, with an
                    empty list of reference tokens.
                    */
                    default:
                    {
                        JSON_THROW(std::domain_error("invalid value to unflatten"));
                    }
                }
            }

            return *result;
        }

        /*!
        @brief return a reference to the pointed to value

        @note This version does not throw if a value is not present, but tries
        to create nested values instead. For instance, calling this function
        with pointer `"/this/that"` on a null value is equivalent to calling
        `operator[]("this").operator[]("that")` on that value, effectively
        changing the null value to an object.

        @param[in] ptr  a JSON value

        @return reference to the JSON value pointed to by the JSON pointer

        @complexity Linear in the length of the JSON pointer.

        @throw std::out_of_range      if the JSON pointer can not be resolved
        @throw std::domain_error      if an array index begins with '0'
        @throw std::invalid_argument  if an array index was not a number
        */
        reference get_unchecked(pointer ptr) const
        {
            for (const auto& reference_token : reference_tokens)
            {
                // convert null values to arrays or objects before continuing
                if (ptr->m_type == value_t::null)
                {
                    // check if reference token is a number
                    const bool nums = std::all_of(reference_token.begin(),
                                                  reference_token.end(),
                                                  [](const char x)
                    {
                        return std::isdigit(x);
                    });

                    // change value to array for numbers or "-" or to object
                    // otherwise
                    if (nums or reference_token == "-")
                    {
                        *ptr = value_t::array;
                    }
                    else
                    {
                        *ptr = value_t::object;
                    }
                }

                switch (ptr->m_type)
                {
                    case value_t::object:
                    {
                        // use unchecked object access
                        ptr = &ptr->operator[](reference_token);
                        break;
                    }

                    case value_t::array:
                    {
                        // error condition (cf. RFC 6901, Sect. 4)
                        if (reference_token.size() > 1 and reference_token[0] == '0')
                        {
                            JSON_THROW(std::domain_error("array index must not begin with '0'"));
                        }

                        if (reference_token == "-")
                        {
                            // explicitly treat "-" as index beyond the end
                            ptr = &ptr->operator[](ptr->m_value.array->size());
                        }
                        else
                        {
                            // convert array index to number; unchecked access
                            ptr = &ptr->operator[](static_cast<size_type>(std::stoi(reference_token)));
                        }
                        break;
                    }

                    default:
                    {
                        JSON_THROW(std::out_of_range("unresolved reference token '" + reference_token + "'"));
                    }
                }
            }

            return *ptr;
        }

        reference get_checked(pointer ptr) const
        {
            for (const auto& reference_token : reference_tokens)
            {
                switch (ptr->m_type)
                {
                    case value_t::object:
                    {
                        // note: at performs range check
                        ptr = &ptr->at(reference_token);
                        break;
                    }

                    case value_t::array:
                    {
                        if (reference_token == "-")
                        {
                            // "-" always fails the range check
                            JSON_THROW(std::out_of_range("array index '-' (" +
                                                         std::to_string(ptr->m_value.array->size()) +
                                                         ") is out of range"));
                        }

                        // error condition (cf. RFC 6901, Sect. 4)
                        if (reference_token.size() > 1 and reference_token[0] == '0')
                        {
                            JSON_THROW(std::domain_error("array index must not begin with '0'"));
                        }

                        // note: at performs range check
                        ptr = &ptr->at(static_cast<size_type>(std::stoi(reference_token)));
                        break;
                    }

                    default:
                    {
                        JSON_THROW(std::out_of_range("unresolved reference token '" + reference_token + "'"));
                    }
                }
            }

            return *ptr;
        }

        /*!
        @brief return a const reference to the pointed to value

        @param[in] ptr  a JSON value

        @return const reference to the JSON value pointed to by the JSON
                pointer
        */
        const_reference get_unchecked(const_pointer ptr) const
        {
            for (const auto& reference_token : reference_tokens)
            {
                switch (ptr->m_type)
                {
                    case value_t::object:
                    {
                        // use unchecked object access
                        ptr = &ptr->operator[](reference_token);
                        break;
                    }

                    case value_t::array:
                    {
                        if (reference_token == "-")
                        {
                            // "-" cannot be used for const access
                            JSON_THROW(std::out_of_range("array index '-' (" +
                                                         std::to_string(ptr->m_value.array->size()) +
                                                         ") is out of range"));
                        }

                        // error condition (cf. RFC 6901, Sect. 4)
                        if (reference_token.size() > 1 and reference_token[0] == '0')
                        {
                            JSON_THROW(std::domain_error("array index must not begin with '0'"));
                        }

                        // use unchecked array access
                        ptr = &ptr->operator[](static_cast<size_type>(std::stoi(reference_token)));
                        break;
                    }

                    default:
                    {
                        JSON_THROW(std::out_of_range("unresolved reference token '" + reference_token + "'"));
                    }
                }
            }

            return *ptr;
        }

        const_reference get_checked(const_pointer ptr) const
        {
            for (const auto& reference_token : reference_tokens)
            {
                switch (ptr->m_type)
                {
                    case value_t::object:
                    {
                        // note: at performs range check
                        ptr = &ptr->at(reference_token);
                        break;
                    }

                    case value_t::array:
                    {
                        if (reference_token == "-")
                        {
                            // "-" always fails the range check
                            JSON_THROW(std::out_of_range("array index '-' (" +
                                                         std::to_string(ptr->m_value.array->size()) +
                                                         ") is out of range"));
                        }

                        // error condition (cf. RFC 6901, Sect. 4)
                        if (reference_token.size() > 1 and reference_token[0] == '0')
                        {
                            JSON_THROW(std::domain_error("array index must not begin with '0'"));
                        }

                        // note: at performs range check
                        ptr = &ptr->at(static_cast<size_type>(std::stoi(reference_token)));
                        break;
                    }

                    default:
                    {
                        JSON_THROW(std::out_of_range("unresolved reference token '" + reference_token + "'"));
                    }
                }
            }

            return *ptr;
        }

        /// split the string input to reference tokens
        static std::vector<std::string> split(const std::string& reference_string)
        {
            std::vector<std::string> result;

            // special case: empty reference string -> no reference tokens
            if (reference_string.empty())
            {
                return result;
            }

            // check if nonempty reference string begins with slash
            if (reference_string[0] != '/')
            {
                JSON_THROW(std::domain_error("JSON pointer must be empty or begin with '/'"));
            }

            // extract the reference tokens:
            // - slash: position of the last read slash (or end of string)
            // - start: position after the previous slash
            for (
                // search for the first slash after the first character
                size_t slash = reference_string.find_first_of('/', 1),
                // set the beginning of the first reference token
                start = 1;
                // we can stop if start == string::npos+1 = 0
                start != 0;
                // set the beginning of the next reference token
                // (will eventually be 0 if slash == std::string::npos)
                start = slash + 1,
                // find next slash
                slash = reference_string.find_first_of('/', start))
            {
                // use the text between the beginning of the reference token
                // (start) and the last slash (slash).
                auto reference_token = reference_string.substr(start, slash - start);

                // check reference tokens are properly escaped
                for (size_t pos = reference_token.find_first_of('~');
                        pos != std::string::npos;
                        pos = reference_token.find_first_of('~', pos + 1))
                {
                    assert(reference_token[pos] == '~');

                    // ~ must be followed by 0 or 1
                    if (pos == reference_token.size() - 1 or
                            (reference_token[pos + 1] != '0' and
                             reference_token[pos + 1] != '1'))
                    {
                        JSON_THROW(std::domain_error("escape error: '~' must be followed with '0' or '1'"));
                    }
                }

                // finally, store the reference token
                unescape(reference_token);
                result.push_back(reference_token);
            }

            return result;
        }

      private:
        /*!
        @brief replace all occurrences of a substring by another string

        @param[in,out] s  the string to manipulate; changed so that all
                          occurrences of @a f are replaced with @a t
        @param[in]     f  the substring to replace with @a t
        @param[in]     t  the string to replace @a f

        @pre The search string @a f must not be empty.

        @since version 2.0.0
        */
        static void replace_substring(std::string& s,
                                      const std::string& f,
                                      const std::string& t)
        {
            assert(not f.empty());

            for (
                size_t pos = s.find(f);         // find first occurrence of f
                pos != std::string::npos;       // make sure f was found
                s.replace(pos, f.size(), t),    // replace with t
                pos = s.find(f, pos + t.size()) // find next occurrence of f
            );
        }

        /// escape tilde and slash
        static std::string escape(std::string s)
        {
            // escape "~"" to "~0" and "/" to "~1"
            replace_substring(s, "~", "~0");
            replace_substring(s, "/", "~1");
            return s;
        }

        /// unescape tilde and slash
        static void unescape(std::string& s)
        {
            // first transform any occurrence of the sequence '~1' to '/'
            replace_substring(s, "~1", "/");
            // then transform any occurrence of the sequence '~0' to '~'
            replace_substring(s, "~0", "~");
        }

        /*!
        @param[in] reference_string  the reference string to the current value
        @param[in] value             the value to consider
        @param[in,out] result        the result object to insert values to

        @note Empty objects or arrays are flattened to `null`.
        */
        static void flatten(const std::string& reference_string,
                            const basic_json& value,
                            basic_json& result)
        {
            switch (value.m_type)
            {
                case value_t::array:
                {
                    if (value.m_value.array->empty())
                    {
                        // flatten empty array as null
                        result[reference_string] = nullptr;
                    }
                    else
                    {
                        // iterate array and use index as reference string
                        for (size_t i = 0; i < value.m_value.array->size(); ++i)
                        {
                            flatten(reference_string + "/" + std::to_string(i),
                                    value.m_value.array->operator[](i), result);
                        }
                    }
                    break;
                }

                case value_t::object:
                {
                    if (value.m_value.object->empty())
                    {
                        // flatten empty object as null
                        result[reference_string] = nullptr;
                    }
                    else
                    {
                        // iterate object and use keys as reference string
                        for (const auto& element : *value.m_value.object)
                        {
                            flatten(reference_string + "/" + escape(element.first),
                                    element.second, result);
                        }
                    }
                    break;
                }

                default:
                {
                    // add primitive value with its reference string
                    result[reference_string] = value;
                    break;
                }
            }
        }

        /*!
        @param[in] value  flattened JSON

        @return unflattened JSON
        */
        static basic_json unflatten(const basic_json& value)
        {
            if (not value.is_object())
            {
                JSON_THROW(std::domain_error("only objects can be unflattened"));
            }

            basic_json result;

            // iterate the JSON object values
            for (const auto& element : *value.m_value.object)
            {
                if (not element.second.is_primitive())
                {
                    JSON_THROW(std::domain_error("values in object must be primitive"));
                }

                // assign value to reference pointed to by JSON pointer; Note
                // that if the JSON pointer is "" (i.e., points to the whole
                // value), function get_and_create returns a reference to
                // result itself. An assignment will then create a primitive
                // value.
                json_pointer(element.first).get_and_create(result) = element.second;
            }

            return result;
        }

      private:
        friend bool operator==(json_pointer const& lhs,
                               json_pointer const& rhs) noexcept
        {
            return lhs.reference_tokens == rhs.reference_tokens;
        }

        friend bool operator!=(json_pointer const& lhs,
                               json_pointer const& rhs) noexcept
        {
            return !(lhs == rhs);
        }

        /// the reference tokens
        std::vector<std::string> reference_tokens {};
    };

    //////////////////////////
    // JSON Pointer support //
    //////////////////////////

    /// @name JSON Pointer functions
    /// @{

    /*!
    @brief access specified element via JSON Pointer

    Uses a JSON pointer to retrieve a reference to the respective JSON value.
    No bound checking is performed. Similar to @ref operator[](const typename
    object_t::key_type&), `null` values are created in arrays and objects if
    necessary.

    In particular:
    - If the JSON pointer points to an object key that does not exist, it
      is created an filled with a `null` value before a reference to it
      is returned.
    - If the JSON pointer points to an array index that does not exist, it
      is created an filled with a `null` value before a reference to it
      is returned. All indices between the current maximum and the given
      index are also filled with `null`.
    - The special value `-` is treated as a synonym for the index past the
      end.

    @param[in] ptr  a JSON pointer

    @return reference to the element pointed to by @a ptr

    @complexity Constant.

    @throw std::out_of_range      if the JSON pointer can not be resolved
    @throw std::domain_error      if an array index begins with '0'
    @throw std::invalid_argument  if an array index was not a number

    @liveexample{The behavior is shown in the example.,operatorjson_pointer}

    @since version 2.0.0
    */
    reference operator[](const json_pointer& ptr)
    {
        return ptr.get_unchecked(this);
    }

    /*!
    @brief access specified element via JSON Pointer

    Uses a JSON pointer to retrieve a reference to the respective JSON value.
    No bound checking is performed. The function does not change the JSON
    value; no `null` values are created. In particular, the the special value
    `-` yields an exception.

    @param[in] ptr  JSON pointer to the desired element

    @return const reference to the element pointed to by @a ptr

    @complexity Constant.

    @throw std::out_of_range      if the JSON pointer can not be resolved
    @throw std::domain_error      if an array index begins with '0'
    @throw std::invalid_argument  if an array index was not a number

    @liveexample{The behavior is shown in the example.,operatorjson_pointer_const}

    @since version 2.0.0
    */
    const_reference operator[](const json_pointer& ptr) const
    {
        return ptr.get_unchecked(this);
    }

    /*!
    @brief access specified element via JSON Pointer

    Returns a reference to the element at with specified JSON pointer @a ptr,
    with bounds checking.

    @param[in] ptr  JSON pointer to the desired element

    @return reference to the element pointed to by @a ptr

    @complexity Constant.

    @throw std::out_of_range      if the JSON pointer can not be resolved
    @throw std::domain_error      if an array index begins with '0'
    @throw std::invalid_argument  if an array index was not a number

    @liveexample{The behavior is shown in the example.,at_json_pointer}

    @since version 2.0.0
    */
    reference at(const json_pointer& ptr)
    {
        return ptr.get_checked(this);
    }

    /*!
    @brief access specified element via JSON Pointer

    Returns a const reference to the element at with specified JSON pointer @a
    ptr, with bounds checking.

    @param[in] ptr  JSON pointer to the desired element

    @return reference to the element pointed to by @a ptr

    @complexity Constant.

    @throw std::out_of_range      if the JSON pointer can not be resolved
    @throw std::domain_error      if an array index begins with '0'
    @throw std::invalid_argument  if an array index was not a number

    @liveexample{The behavior is shown in the example.,at_json_pointer_const}

    @since version 2.0.0
    */
    const_reference at(const json_pointer& ptr) const
    {
        return ptr.get_checked(this);
    }

    /*!
    @brief return flattened JSON value

    The function creates a JSON object whose keys are JSON pointers (see [RFC
    6901](https://tools.ietf.org/html/rfc6901)) and whose values are all
    primitive. The original JSON value can be restored using the @ref
    unflatten() function.

    @return an object that maps JSON pointers to primitive values

    @note Empty objects and arrays are flattened to `null` and will not be
          reconstructed correctly by the @ref unflatten() function.

    @complexity Linear in the size the JSON value.

    @liveexample{The following code shows how a JSON object is flattened to an
    object whose keys consist of JSON pointers.,flatten}

    @sa @ref unflatten() for the reverse function

    @since version 2.0.0
    */
    basic_json flatten() const
    {
        basic_json result(value_t::object);
        json_pointer::flatten("", *this, result);
        return result;
    }

    /*!
    @brief unflatten a previously flattened JSON value

    The function restores the arbitrary nesting of a JSON value that has been
    flattened before using the @ref flatten() function. The JSON value must
    meet certain constraints:
    1. The value must be an object.
    2. The keys must be JSON pointers (see
       [RFC 6901](https://tools.ietf.org/html/rfc6901))
    3. The mapped values must be primitive JSON types.

    @return the original JSON from a flattened version

    @note Empty objects and arrays are flattened by @ref flatten() to `null`
          values and can not unflattened to their original type. Apart from
          this example, for a JSON value `j`, the following is always true:
          `j == j.flatten().unflatten()`.

    @complexity Linear in the size the JSON value.

    @liveexample{The following code shows how a flattened JSON object is
    unflattened into the original nested JSON object.,unflatten}

    @sa @ref flatten() for the reverse function

    @since version 2.0.0
    */
    basic_json unflatten() const
    {
        return json_pointer::unflatten(*this);
    }

    /// @}

    //////////////////////////
    // JSON Patch functions //
    //////////////////////////

    /// @name JSON Patch functions
    /// @{

    /*!
    @brief applies a JSON patch

    [JSON Patch](http://jsonpatch.com) defines a JSON document structure for
    expressing a sequence of operations to apply to a JSON) document. With
    this function, a JSON Patch is applied to the current JSON value by
    executing all operations from the patch.

    @param[in] json_patch  JSON patch document
    @return patched document

    @note The application of a patch is atomic: Either all operations succeed
          and the patched document is returned or an exception is thrown. In
          any case, the original value is not changed: the patch is applied
          to a copy of the value.

    @throw std::out_of_range if a JSON pointer inside the patch could not
    be resolved successfully in the current JSON value; example: `"key baz
    not found"`
    @throw invalid_argument if the JSON patch is malformed (e.g., mandatory
    attributes are missing); example: `"operation add must have member path"`

    @complexity Linear in the size of the JSON value and the length of the
    JSON patch. As usually only a fraction of the JSON value is affected by
    the patch, the complexity can usually be neglected.

    @liveexample{The following code shows how a JSON patch is applied to a
    value.,patch}

    @sa @ref diff -- create a JSON patch by comparing two JSON values

    @sa [RFC 6902 (JSON Patch)](https://tools.ietf.org/html/rfc6902)
    @sa [RFC 6901 (JSON Pointer)](https://tools.ietf.org/html/rfc6901)

    @since version 2.0.0
    */
    basic_json patch(const basic_json& json_patch) const
    {
        // make a working copy to apply the patch to
        basic_json result = *this;

        // the valid JSON Patch operations
        enum class patch_operations {add, remove, replace, move, copy, test, invalid};

        const auto get_op = [](const std::string op)
        {
            if (op == "add")
            {
                return patch_operations::add;
            }
            if (op == "remove")
            {
                return patch_operations::remove;
            }
            if (op == "replace")
            {
                return patch_operations::replace;
            }
            if (op == "move")
            {
                return patch_operations::move;
            }
            if (op == "copy")
            {
                return patch_operations::copy;
            }
            if (op == "test")
            {
                return patch_operations::test;
            }

            return patch_operations::invalid;
        };

        // wrapper for "add" operation; add value at ptr
        const auto operation_add = [&result](json_pointer & ptr, basic_json val)
        {
            // adding to the root of the target document means replacing it
            if (ptr.is_root())
            {
                result = val;
            }
            else
            {
                // make sure the top element of the pointer exists
                json_pointer top_pointer = ptr.top();
                if (top_pointer != ptr)
                {
                    result.at(top_pointer);
                }

                // get reference to parent of JSON pointer ptr
                const auto last_path = ptr.pop_back();
                basic_json& parent = result[ptr];

                switch (parent.m_type)
                {
                    case value_t::null:
                    case value_t::object:
                    {
                        // use operator[] to add value
                        parent[last_path] = val;
                        break;
                    }

                    case value_t::array:
                    {
                        if (last_path == "-")
                        {
                            // special case: append to back
                            parent.push_back(val);
                        }
                        else
                        {
                            const auto idx = std::stoi(last_path);
                            if (static_cast<size_type>(idx) > parent.size())
                            {
                                // avoid undefined behavior
                                JSON_THROW(std::out_of_range("array index " + std::to_string(idx) + " is out of range"));
                            }
                            else
                            {
                                // default case: insert add offset
                                parent.insert(parent.begin() + static_cast<difference_type>(idx), val);
                            }
                        }
                        break;
                    }

                    default:
                    {
                        // if there exists a parent it cannot be primitive
                        assert(false);  // LCOV_EXCL_LINE
                    }
                }
            }
        };

        // wrapper for "remove" operation; remove value at ptr
        const auto operation_remove = [&result](json_pointer & ptr)
        {
            // get reference to parent of JSON pointer ptr
            const auto last_path = ptr.pop_back();
            basic_json& parent = result.at(ptr);

            // remove child
            if (parent.is_object())
            {
                // perform range check
                auto it = parent.find(last_path);
                if (it != parent.end())
                {
                    parent.erase(it);
                }
                else
                {
                    JSON_THROW(std::out_of_range("key '" + last_path + "' not found"));
                }
            }
            else if (parent.is_array())
            {
                // note erase performs range check
                parent.erase(static_cast<size_type>(std::stoi(last_path)));
            }
        };

        // type check
        if (not json_patch.is_array())
        {
            // a JSON patch must be an array of objects
            JSON_THROW(std::invalid_argument("JSON patch must be an array of objects"));
        }

        // iterate and apply the operations
        for (const auto& val : json_patch)
        {
            // wrapper to get a value for an operation
            const auto get_value = [&val](const std::string & op,
                                          const std::string & member,
                                          bool string_type) -> basic_json&
            {
                // find value
                auto it = val.m_value.object->find(member);

                // context-sensitive error message
                const auto error_msg = (op == "op") ? "operation" : "operation '" + op + "'";

                // check if desired value is present
                if (it == val.m_value.object->end())
                {
                    JSON_THROW(std::invalid_argument(error_msg + " must have member '" + member + "'"));
                }

                // check if result is of type string
                if (string_type and not it->second.is_string())
                {
                    JSON_THROW(std::invalid_argument(error_msg + " must have string member '" + member + "'"));
                }

                // no error: return value
                return it->second;
            };

            // type check
            if (not val.is_object())
            {
                JSON_THROW(std::invalid_argument("JSON patch must be an array of objects"));
            }

            // collect mandatory members
            const std::string op = get_value("op", "op", true);
            const std::string path = get_value(op, "path", true);
            json_pointer ptr(path);

            switch (get_op(op))
            {
                case patch_operations::add:
                {
                    operation_add(ptr, get_value("add", "value", false));
                    break;
                }

                case patch_operations::remove:
                {
                    operation_remove(ptr);
                    break;
                }

                case patch_operations::replace:
                {
                    // the "path" location must exist - use at()
                    result.at(ptr) = get_value("replace", "value", false);
                    break;
                }

                case patch_operations::move:
                {
                    const std::string from_path = get_value("move", "from", true);
                    json_pointer from_ptr(from_path);

                    // the "from" location must exist - use at()
                    basic_json v = result.at(from_ptr);

                    // The move operation is functionally identical to a
                    // "remove" operation on the "from" location, followed
                    // immediately by an "add" operation at the target
                    // location with the value that was just removed.
                    operation_remove(from_ptr);
                    operation_add(ptr, v);
                    break;
                }

                case patch_operations::copy:
                {
                    const std::string from_path = get_value("copy", "from", true);;
                    const json_pointer from_ptr(from_path);

                    // the "from" location must exist - use at()
                    result[ptr] = result.at(from_ptr);
                    break;
                }

                case patch_operations::test:
                {
                    bool success = false;
                    JSON_TRY
                    {
                        // check if "value" matches the one at "path"
                        // the "path" location must exist - use at()
                        success = (result.at(ptr) == get_value("test", "value", false));
                    }
                    JSON_CATCH (std::out_of_range&)
                    {
                        // ignore out of range errors: success remains false
                    }

                    // throw an exception if test fails
                    if (not success)
                    {
                        JSON_THROW(std::domain_error("unsuccessful: " + val.dump()));
                    }

                    break;
                }

                case patch_operations::invalid:
                {
                    // op must be "add", "remove", "replace", "move", "copy", or
                    // "test"
                    JSON_THROW(std::invalid_argument("operation value '" + op + "' is invalid"));
                }
            }
        }

        return result;
    }

    /*!
    @brief creates a diff as a JSON patch

    Creates a [JSON Patch](http://jsonpatch.com) so that value @a source can
    be changed into the value @a target by calling @ref patch function.

    @invariant For two JSON values @a source and @a target, the following code
    yields always `true`:
    @code {.cpp}
    source.patch(diff(source, target)) == target;
    @endcode

    @note Currently, only `remove`, `add`, and `replace` operations are
          generated.

    @param[in] source  JSON value to compare from
    @param[in] target  JSON value to compare against
    @param[in] path    helper value to create JSON pointers

    @return a JSON patch to convert the @a source to @a target

    @complexity Linear in the lengths of @a source and @a target.

    @liveexample{The following code shows how a JSON patch is created as a
    diff for two JSON values.,diff}

    @sa @ref patch -- apply a JSON patch

    @sa [RFC 6902 (JSON Patch)](https://tools.ietf.org/html/rfc6902)

    @since version 2.0.0
    */
    static basic_json diff(const basic_json& source,
                           const basic_json& target,
                           const std::string& path = "")
    {
        // the patch
        basic_json result(value_t::array);

        // if the values are the same, return empty patch
        if (source == target)
        {
            return result;
        }

        if (source.type() != target.type())
        {
            // different types: replace value
            result.push_back(
            {
                {"op", "replace"},
                {"path", path},
                {"value", target}
            });
        }
        else
        {
            switch (source.type())
            {
                case value_t::array:
                {
                    // first pass: traverse common elements
                    size_t i = 0;
                    while (i < source.size() and i < target.size())
                    {
                        // recursive call to compare array values at index i
                        auto temp_diff = diff(source[i], target[i], path + "/" + std::to_string(i));
                        result.insert(result.end(), temp_diff.begin(), temp_diff.end());
                        ++i;
                    }

                    // i now reached the end of at least one array
                    // in a second pass, traverse the remaining elements

                    // remove my remaining elements
                    const auto end_index = static_cast<difference_type>(result.size());
                    while (i < source.size())
                    {
                        // add operations in reverse order to avoid invalid
                        // indices
                        result.insert(result.begin() + end_index, object(
                        {
                            {"op", "remove"},
                            {"path", path + "/" + std::to_string(i)}
                        }));
                        ++i;
                    }

                    // add other remaining elements
                    while (i < target.size())
                    {
                        result.push_back(
                        {
                            {"op", "add"},
                            {"path", path + "/" + std::to_string(i)},
                            {"value", target[i]}
                        });
                        ++i;
                    }

                    break;
                }

                case value_t::object:
                {
                    // first pass: traverse this object's elements
                    for (auto it = source.begin(); it != source.end(); ++it)
                    {
                        // escape the key name to be used in a JSON patch
                        const auto key = json_pointer::escape(it.key());

                        if (target.find(it.key()) != target.end())
                        {
                            // recursive call to compare object values at key it
                            auto temp_diff = diff(it.value(), target[it.key()], path + "/" + key);
                            result.insert(result.end(), temp_diff.begin(), temp_diff.end());
                        }
                        else
                        {
                            // found a key that is not in o -> remove it
                            result.push_back(object(
                            {
                                {"op", "remove"},
                                {"path", path + "/" + key}
                            }));
                        }
                    }

                    // second pass: traverse other object's elements
                    for (auto it = target.begin(); it != target.end(); ++it)
                    {
                        if (source.find(it.key()) == source.end())
                        {
                            // found a key that is not in this -> add it
                            const auto key = json_pointer::escape(it.key());
                            result.push_back(
                            {
                                {"op", "add"},
                                {"path", path + "/" + key},
                                {"value", it.value()}
                            });
                        }
                    }

                    break;
                }

                default:
                {
                    // both primitive type: replace value
                    result.push_back(
                    {
                        {"op", "replace"},
                        {"path", path},
                        {"value", target}
                    });
                    break;
                }
            }
        }

        return result;
    }

    /// @}
};

/////////////
// presets //
/////////////

/*!
@brief default JSON class

This type is the default specialization of the @ref basic_json class which
uses the standard template types.

@since version 1.0.0
*/
using json = basic_json<>;
} // namespace nlohmann


///////////////////////
// nonmember support //
///////////////////////

// specialization of std::swap, and std::hash
namespace std
{
/*!
@brief exchanges the values of two JSON objects

@since version 1.0.0
*/
template<>
inline void swap(nlohmann::json& j1,
                 nlohmann::json& j2) noexcept(
                     is_nothrow_move_constructible<nlohmann::json>::value and
                     is_nothrow_move_assignable<nlohmann::json>::value
                 )
{
    j1.swap(j2);
}

/// hash value for JSON objects
template<>
struct hash<nlohmann::json>
{
    /*!
    @brief return a hash value for a JSON object

    @since version 1.0.0
    */
    std::size_t operator()(const nlohmann::json& j) const
    {
        // a naive hashing via the string representation
        const auto& h = hash<nlohmann::json::string_t>();
        return h(j.dump());
    }
};
} // namespace std

/*!
@brief user-defined string literal for JSON values

This operator implements a user-defined string literal for JSON objects. It
can be used by adding `"_json"` to a string literal and returns a JSON object
if no parse error occurred.

@param[in] s  a string representation of a JSON object
@param[in] n  the length of string @a s
@return a JSON object

@since version 1.0.0
*/
inline nlohmann::json operator "" _json(const char* s, std::size_t n)
{
    return nlohmann::json::parse(s, s + n);
}

/*!
@brief user-defined string literal for JSON pointer

This operator implements a user-defined string literal for JSON Pointers. It
can be used by adding `"_json_pointer"` to a string literal and returns a JSON pointer
object if no parse error occurred.

@param[in] s  a string representation of a JSON Pointer
@param[in] n  the length of string @a s
@return a JSON pointer object

@since version 2.0.0
*/
inline nlohmann::json::json_pointer operator "" _json_pointer(const char* s, std::size_t n)
{
    return nlohmann::json::json_pointer(std::string(s, n));
}

// restore GCC/clang diagnostic settings
#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
    #pragma GCC diagnostic pop
#endif

// clean up
#undef JSON_CATCH
#undef JSON_DEPRECATED
#undef JSON_THROW
#undef JSON_TRY
// clang-format on
#endif
