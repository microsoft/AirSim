# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# if !defined(MSGPACK_PP_LOCAL_LIMITS)
#    error MSGPACK_PP_ERROR:  local iteration boundaries are not defined
# elif !defined(MSGPACK_PP_LOCAL_MACRO)
#    error MSGPACK_PP_ERROR:  local iteration target macro is not defined
# else
#    if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#        define MSGPACK_PP_LOCAL_S MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_LOCAL_LIMITS)
#        define MSGPACK_PP_LOCAL_F MSGPACK_PP_TUPLE_ELEM(2, 1, MSGPACK_PP_LOCAL_LIMITS)
#    else
#        define MSGPACK_PP_VALUE MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_LOCAL_LIMITS)
#        include <rpc/msgpack/preprocessor/iteration/detail/start.hpp>
#        define MSGPACK_PP_VALUE MSGPACK_PP_TUPLE_ELEM(2, 1, MSGPACK_PP_LOCAL_LIMITS)
#        include <rpc/msgpack/preprocessor/iteration/detail/finish.hpp>
#        define MSGPACK_PP_LOCAL_S MSGPACK_PP_LOCAL_SE()
#        define MSGPACK_PP_LOCAL_F MSGPACK_PP_LOCAL_FE()
#    endif
# endif
#
# if (MSGPACK_PP_LOCAL_S) > (MSGPACK_PP_LOCAL_F)
#    include <rpc/msgpack/preprocessor/iteration/detail/rlocal.hpp>
# else
#    if MSGPACK_PP_LOCAL_C(0)
        MSGPACK_PP_LOCAL_MACRO(0)
#    endif
#    if MSGPACK_PP_LOCAL_C(1)
        MSGPACK_PP_LOCAL_MACRO(1)
#    endif
#    if MSGPACK_PP_LOCAL_C(2)
        MSGPACK_PP_LOCAL_MACRO(2)
#    endif
#    if MSGPACK_PP_LOCAL_C(3)
        MSGPACK_PP_LOCAL_MACRO(3)
#    endif
#    if MSGPACK_PP_LOCAL_C(4)
        MSGPACK_PP_LOCAL_MACRO(4)
#    endif
#    if MSGPACK_PP_LOCAL_C(5)
        MSGPACK_PP_LOCAL_MACRO(5)
#    endif
#    if MSGPACK_PP_LOCAL_C(6)
        MSGPACK_PP_LOCAL_MACRO(6)
#    endif
#    if MSGPACK_PP_LOCAL_C(7)
        MSGPACK_PP_LOCAL_MACRO(7)
#    endif
#    if MSGPACK_PP_LOCAL_C(8)
        MSGPACK_PP_LOCAL_MACRO(8)
#    endif
#    if MSGPACK_PP_LOCAL_C(9)
        MSGPACK_PP_LOCAL_MACRO(9)
#    endif
#    if MSGPACK_PP_LOCAL_C(10)
        MSGPACK_PP_LOCAL_MACRO(10)
#    endif
#    if MSGPACK_PP_LOCAL_C(11)
        MSGPACK_PP_LOCAL_MACRO(11)
#    endif
#    if MSGPACK_PP_LOCAL_C(12)
        MSGPACK_PP_LOCAL_MACRO(12)
#    endif
#    if MSGPACK_PP_LOCAL_C(13)
        MSGPACK_PP_LOCAL_MACRO(13)
#    endif
#    if MSGPACK_PP_LOCAL_C(14)
        MSGPACK_PP_LOCAL_MACRO(14)
#    endif
#    if MSGPACK_PP_LOCAL_C(15)
        MSGPACK_PP_LOCAL_MACRO(15)
#    endif
#    if MSGPACK_PP_LOCAL_C(16)
        MSGPACK_PP_LOCAL_MACRO(16)
#    endif
#    if MSGPACK_PP_LOCAL_C(17)
        MSGPACK_PP_LOCAL_MACRO(17)
#    endif
#    if MSGPACK_PP_LOCAL_C(18)
        MSGPACK_PP_LOCAL_MACRO(18)
#    endif
#    if MSGPACK_PP_LOCAL_C(19)
        MSGPACK_PP_LOCAL_MACRO(19)
#    endif
#    if MSGPACK_PP_LOCAL_C(20)
        MSGPACK_PP_LOCAL_MACRO(20)
#    endif
#    if MSGPACK_PP_LOCAL_C(21)
        MSGPACK_PP_LOCAL_MACRO(21)
#    endif
#    if MSGPACK_PP_LOCAL_C(22)
        MSGPACK_PP_LOCAL_MACRO(22)
#    endif
#    if MSGPACK_PP_LOCAL_C(23)
        MSGPACK_PP_LOCAL_MACRO(23)
#    endif
#    if MSGPACK_PP_LOCAL_C(24)
        MSGPACK_PP_LOCAL_MACRO(24)
#    endif
#    if MSGPACK_PP_LOCAL_C(25)
        MSGPACK_PP_LOCAL_MACRO(25)
#    endif
#    if MSGPACK_PP_LOCAL_C(26)
        MSGPACK_PP_LOCAL_MACRO(26)
#    endif
#    if MSGPACK_PP_LOCAL_C(27)
        MSGPACK_PP_LOCAL_MACRO(27)
#    endif
#    if MSGPACK_PP_LOCAL_C(28)
        MSGPACK_PP_LOCAL_MACRO(28)
#    endif
#    if MSGPACK_PP_LOCAL_C(29)
        MSGPACK_PP_LOCAL_MACRO(29)
#    endif
#    if MSGPACK_PP_LOCAL_C(30)
        MSGPACK_PP_LOCAL_MACRO(30)
#    endif
#    if MSGPACK_PP_LOCAL_C(31)
        MSGPACK_PP_LOCAL_MACRO(31)
#    endif
#    if MSGPACK_PP_LOCAL_C(32)
        MSGPACK_PP_LOCAL_MACRO(32)
#    endif
#    if MSGPACK_PP_LOCAL_C(33)
        MSGPACK_PP_LOCAL_MACRO(33)
#    endif
#    if MSGPACK_PP_LOCAL_C(34)
        MSGPACK_PP_LOCAL_MACRO(34)
#    endif
#    if MSGPACK_PP_LOCAL_C(35)
        MSGPACK_PP_LOCAL_MACRO(35)
#    endif
#    if MSGPACK_PP_LOCAL_C(36)
        MSGPACK_PP_LOCAL_MACRO(36)
#    endif
#    if MSGPACK_PP_LOCAL_C(37)
        MSGPACK_PP_LOCAL_MACRO(37)
#    endif
#    if MSGPACK_PP_LOCAL_C(38)
        MSGPACK_PP_LOCAL_MACRO(38)
#    endif
#    if MSGPACK_PP_LOCAL_C(39)
        MSGPACK_PP_LOCAL_MACRO(39)
#    endif
#    if MSGPACK_PP_LOCAL_C(40)
        MSGPACK_PP_LOCAL_MACRO(40)
#    endif
#    if MSGPACK_PP_LOCAL_C(41)
        MSGPACK_PP_LOCAL_MACRO(41)
#    endif
#    if MSGPACK_PP_LOCAL_C(42)
        MSGPACK_PP_LOCAL_MACRO(42)
#    endif
#    if MSGPACK_PP_LOCAL_C(43)
        MSGPACK_PP_LOCAL_MACRO(43)
#    endif
#    if MSGPACK_PP_LOCAL_C(44)
        MSGPACK_PP_LOCAL_MACRO(44)
#    endif
#    if MSGPACK_PP_LOCAL_C(45)
        MSGPACK_PP_LOCAL_MACRO(45)
#    endif
#    if MSGPACK_PP_LOCAL_C(46)
        MSGPACK_PP_LOCAL_MACRO(46)
#    endif
#    if MSGPACK_PP_LOCAL_C(47)
        MSGPACK_PP_LOCAL_MACRO(47)
#    endif
#    if MSGPACK_PP_LOCAL_C(48)
        MSGPACK_PP_LOCAL_MACRO(48)
#    endif
#    if MSGPACK_PP_LOCAL_C(49)
        MSGPACK_PP_LOCAL_MACRO(49)
#    endif
#    if MSGPACK_PP_LOCAL_C(50)
        MSGPACK_PP_LOCAL_MACRO(50)
#    endif
#    if MSGPACK_PP_LOCAL_C(51)
        MSGPACK_PP_LOCAL_MACRO(51)
#    endif
#    if MSGPACK_PP_LOCAL_C(52)
        MSGPACK_PP_LOCAL_MACRO(52)
#    endif
#    if MSGPACK_PP_LOCAL_C(53)
        MSGPACK_PP_LOCAL_MACRO(53)
#    endif
#    if MSGPACK_PP_LOCAL_C(54)
        MSGPACK_PP_LOCAL_MACRO(54)
#    endif
#    if MSGPACK_PP_LOCAL_C(55)
        MSGPACK_PP_LOCAL_MACRO(55)
#    endif
#    if MSGPACK_PP_LOCAL_C(56)
        MSGPACK_PP_LOCAL_MACRO(56)
#    endif
#    if MSGPACK_PP_LOCAL_C(57)
        MSGPACK_PP_LOCAL_MACRO(57)
#    endif
#    if MSGPACK_PP_LOCAL_C(58)
        MSGPACK_PP_LOCAL_MACRO(58)
#    endif
#    if MSGPACK_PP_LOCAL_C(59)
        MSGPACK_PP_LOCAL_MACRO(59)
#    endif
#    if MSGPACK_PP_LOCAL_C(60)
        MSGPACK_PP_LOCAL_MACRO(60)
#    endif
#    if MSGPACK_PP_LOCAL_C(61)
        MSGPACK_PP_LOCAL_MACRO(61)
#    endif
#    if MSGPACK_PP_LOCAL_C(62)
        MSGPACK_PP_LOCAL_MACRO(62)
#    endif
#    if MSGPACK_PP_LOCAL_C(63)
        MSGPACK_PP_LOCAL_MACRO(63)
#    endif
#    if MSGPACK_PP_LOCAL_C(64)
        MSGPACK_PP_LOCAL_MACRO(64)
#    endif
#    if MSGPACK_PP_LOCAL_C(65)
        MSGPACK_PP_LOCAL_MACRO(65)
#    endif
#    if MSGPACK_PP_LOCAL_C(66)
        MSGPACK_PP_LOCAL_MACRO(66)
#    endif
#    if MSGPACK_PP_LOCAL_C(67)
        MSGPACK_PP_LOCAL_MACRO(67)
#    endif
#    if MSGPACK_PP_LOCAL_C(68)
        MSGPACK_PP_LOCAL_MACRO(68)
#    endif
#    if MSGPACK_PP_LOCAL_C(69)
        MSGPACK_PP_LOCAL_MACRO(69)
#    endif
#    if MSGPACK_PP_LOCAL_C(70)
        MSGPACK_PP_LOCAL_MACRO(70)
#    endif
#    if MSGPACK_PP_LOCAL_C(71)
        MSGPACK_PP_LOCAL_MACRO(71)
#    endif
#    if MSGPACK_PP_LOCAL_C(72)
        MSGPACK_PP_LOCAL_MACRO(72)
#    endif
#    if MSGPACK_PP_LOCAL_C(73)
        MSGPACK_PP_LOCAL_MACRO(73)
#    endif
#    if MSGPACK_PP_LOCAL_C(74)
        MSGPACK_PP_LOCAL_MACRO(74)
#    endif
#    if MSGPACK_PP_LOCAL_C(75)
        MSGPACK_PP_LOCAL_MACRO(75)
#    endif
#    if MSGPACK_PP_LOCAL_C(76)
        MSGPACK_PP_LOCAL_MACRO(76)
#    endif
#    if MSGPACK_PP_LOCAL_C(77)
        MSGPACK_PP_LOCAL_MACRO(77)
#    endif
#    if MSGPACK_PP_LOCAL_C(78)
        MSGPACK_PP_LOCAL_MACRO(78)
#    endif
#    if MSGPACK_PP_LOCAL_C(79)
        MSGPACK_PP_LOCAL_MACRO(79)
#    endif
#    if MSGPACK_PP_LOCAL_C(80)
        MSGPACK_PP_LOCAL_MACRO(80)
#    endif
#    if MSGPACK_PP_LOCAL_C(81)
        MSGPACK_PP_LOCAL_MACRO(81)
#    endif
#    if MSGPACK_PP_LOCAL_C(82)
        MSGPACK_PP_LOCAL_MACRO(82)
#    endif
#    if MSGPACK_PP_LOCAL_C(83)
        MSGPACK_PP_LOCAL_MACRO(83)
#    endif
#    if MSGPACK_PP_LOCAL_C(84)
        MSGPACK_PP_LOCAL_MACRO(84)
#    endif
#    if MSGPACK_PP_LOCAL_C(85)
        MSGPACK_PP_LOCAL_MACRO(85)
#    endif
#    if MSGPACK_PP_LOCAL_C(86)
        MSGPACK_PP_LOCAL_MACRO(86)
#    endif
#    if MSGPACK_PP_LOCAL_C(87)
        MSGPACK_PP_LOCAL_MACRO(87)
#    endif
#    if MSGPACK_PP_LOCAL_C(88)
        MSGPACK_PP_LOCAL_MACRO(88)
#    endif
#    if MSGPACK_PP_LOCAL_C(89)
        MSGPACK_PP_LOCAL_MACRO(89)
#    endif
#    if MSGPACK_PP_LOCAL_C(90)
        MSGPACK_PP_LOCAL_MACRO(90)
#    endif
#    if MSGPACK_PP_LOCAL_C(91)
        MSGPACK_PP_LOCAL_MACRO(91)
#    endif
#    if MSGPACK_PP_LOCAL_C(92)
        MSGPACK_PP_LOCAL_MACRO(92)
#    endif
#    if MSGPACK_PP_LOCAL_C(93)
        MSGPACK_PP_LOCAL_MACRO(93)
#    endif
#    if MSGPACK_PP_LOCAL_C(94)
        MSGPACK_PP_LOCAL_MACRO(94)
#    endif
#    if MSGPACK_PP_LOCAL_C(95)
        MSGPACK_PP_LOCAL_MACRO(95)
#    endif
#    if MSGPACK_PP_LOCAL_C(96)
        MSGPACK_PP_LOCAL_MACRO(96)
#    endif
#    if MSGPACK_PP_LOCAL_C(97)
        MSGPACK_PP_LOCAL_MACRO(97)
#    endif
#    if MSGPACK_PP_LOCAL_C(98)
        MSGPACK_PP_LOCAL_MACRO(98)
#    endif
#    if MSGPACK_PP_LOCAL_C(99)
        MSGPACK_PP_LOCAL_MACRO(99)
#    endif
#    if MSGPACK_PP_LOCAL_C(100)
        MSGPACK_PP_LOCAL_MACRO(100)
#    endif
#    if MSGPACK_PP_LOCAL_C(101)
        MSGPACK_PP_LOCAL_MACRO(101)
#    endif
#    if MSGPACK_PP_LOCAL_C(102)
        MSGPACK_PP_LOCAL_MACRO(102)
#    endif
#    if MSGPACK_PP_LOCAL_C(103)
        MSGPACK_PP_LOCAL_MACRO(103)
#    endif
#    if MSGPACK_PP_LOCAL_C(104)
        MSGPACK_PP_LOCAL_MACRO(104)
#    endif
#    if MSGPACK_PP_LOCAL_C(105)
        MSGPACK_PP_LOCAL_MACRO(105)
#    endif
#    if MSGPACK_PP_LOCAL_C(106)
        MSGPACK_PP_LOCAL_MACRO(106)
#    endif
#    if MSGPACK_PP_LOCAL_C(107)
        MSGPACK_PP_LOCAL_MACRO(107)
#    endif
#    if MSGPACK_PP_LOCAL_C(108)
        MSGPACK_PP_LOCAL_MACRO(108)
#    endif
#    if MSGPACK_PP_LOCAL_C(109)
        MSGPACK_PP_LOCAL_MACRO(109)
#    endif
#    if MSGPACK_PP_LOCAL_C(110)
        MSGPACK_PP_LOCAL_MACRO(110)
#    endif
#    if MSGPACK_PP_LOCAL_C(111)
        MSGPACK_PP_LOCAL_MACRO(111)
#    endif
#    if MSGPACK_PP_LOCAL_C(112)
        MSGPACK_PP_LOCAL_MACRO(112)
#    endif
#    if MSGPACK_PP_LOCAL_C(113)
        MSGPACK_PP_LOCAL_MACRO(113)
#    endif
#    if MSGPACK_PP_LOCAL_C(114)
        MSGPACK_PP_LOCAL_MACRO(114)
#    endif
#    if MSGPACK_PP_LOCAL_C(115)
        MSGPACK_PP_LOCAL_MACRO(115)
#    endif
#    if MSGPACK_PP_LOCAL_C(116)
        MSGPACK_PP_LOCAL_MACRO(116)
#    endif
#    if MSGPACK_PP_LOCAL_C(117)
        MSGPACK_PP_LOCAL_MACRO(117)
#    endif
#    if MSGPACK_PP_LOCAL_C(118)
        MSGPACK_PP_LOCAL_MACRO(118)
#    endif
#    if MSGPACK_PP_LOCAL_C(119)
        MSGPACK_PP_LOCAL_MACRO(119)
#    endif
#    if MSGPACK_PP_LOCAL_C(120)
        MSGPACK_PP_LOCAL_MACRO(120)
#    endif
#    if MSGPACK_PP_LOCAL_C(121)
        MSGPACK_PP_LOCAL_MACRO(121)
#    endif
#    if MSGPACK_PP_LOCAL_C(122)
        MSGPACK_PP_LOCAL_MACRO(122)
#    endif
#    if MSGPACK_PP_LOCAL_C(123)
        MSGPACK_PP_LOCAL_MACRO(123)
#    endif
#    if MSGPACK_PP_LOCAL_C(124)
        MSGPACK_PP_LOCAL_MACRO(124)
#    endif
#    if MSGPACK_PP_LOCAL_C(125)
        MSGPACK_PP_LOCAL_MACRO(125)
#    endif
#    if MSGPACK_PP_LOCAL_C(126)
        MSGPACK_PP_LOCAL_MACRO(126)
#    endif
#    if MSGPACK_PP_LOCAL_C(127)
        MSGPACK_PP_LOCAL_MACRO(127)
#    endif
#    if MSGPACK_PP_LOCAL_C(128)
        MSGPACK_PP_LOCAL_MACRO(128)
#    endif
#    if MSGPACK_PP_LOCAL_C(129)
        MSGPACK_PP_LOCAL_MACRO(129)
#    endif
#    if MSGPACK_PP_LOCAL_C(130)
        MSGPACK_PP_LOCAL_MACRO(130)
#    endif
#    if MSGPACK_PP_LOCAL_C(131)
        MSGPACK_PP_LOCAL_MACRO(131)
#    endif
#    if MSGPACK_PP_LOCAL_C(132)
        MSGPACK_PP_LOCAL_MACRO(132)
#    endif
#    if MSGPACK_PP_LOCAL_C(133)
        MSGPACK_PP_LOCAL_MACRO(133)
#    endif
#    if MSGPACK_PP_LOCAL_C(134)
        MSGPACK_PP_LOCAL_MACRO(134)
#    endif
#    if MSGPACK_PP_LOCAL_C(135)
        MSGPACK_PP_LOCAL_MACRO(135)
#    endif
#    if MSGPACK_PP_LOCAL_C(136)
        MSGPACK_PP_LOCAL_MACRO(136)
#    endif
#    if MSGPACK_PP_LOCAL_C(137)
        MSGPACK_PP_LOCAL_MACRO(137)
#    endif
#    if MSGPACK_PP_LOCAL_C(138)
        MSGPACK_PP_LOCAL_MACRO(138)
#    endif
#    if MSGPACK_PP_LOCAL_C(139)
        MSGPACK_PP_LOCAL_MACRO(139)
#    endif
#    if MSGPACK_PP_LOCAL_C(140)
        MSGPACK_PP_LOCAL_MACRO(140)
#    endif
#    if MSGPACK_PP_LOCAL_C(141)
        MSGPACK_PP_LOCAL_MACRO(141)
#    endif
#    if MSGPACK_PP_LOCAL_C(142)
        MSGPACK_PP_LOCAL_MACRO(142)
#    endif
#    if MSGPACK_PP_LOCAL_C(143)
        MSGPACK_PP_LOCAL_MACRO(143)
#    endif
#    if MSGPACK_PP_LOCAL_C(144)
        MSGPACK_PP_LOCAL_MACRO(144)
#    endif
#    if MSGPACK_PP_LOCAL_C(145)
        MSGPACK_PP_LOCAL_MACRO(145)
#    endif
#    if MSGPACK_PP_LOCAL_C(146)
        MSGPACK_PP_LOCAL_MACRO(146)
#    endif
#    if MSGPACK_PP_LOCAL_C(147)
        MSGPACK_PP_LOCAL_MACRO(147)
#    endif
#    if MSGPACK_PP_LOCAL_C(148)
        MSGPACK_PP_LOCAL_MACRO(148)
#    endif
#    if MSGPACK_PP_LOCAL_C(149)
        MSGPACK_PP_LOCAL_MACRO(149)
#    endif
#    if MSGPACK_PP_LOCAL_C(150)
        MSGPACK_PP_LOCAL_MACRO(150)
#    endif
#    if MSGPACK_PP_LOCAL_C(151)
        MSGPACK_PP_LOCAL_MACRO(151)
#    endif
#    if MSGPACK_PP_LOCAL_C(152)
        MSGPACK_PP_LOCAL_MACRO(152)
#    endif
#    if MSGPACK_PP_LOCAL_C(153)
        MSGPACK_PP_LOCAL_MACRO(153)
#    endif
#    if MSGPACK_PP_LOCAL_C(154)
        MSGPACK_PP_LOCAL_MACRO(154)
#    endif
#    if MSGPACK_PP_LOCAL_C(155)
        MSGPACK_PP_LOCAL_MACRO(155)
#    endif
#    if MSGPACK_PP_LOCAL_C(156)
        MSGPACK_PP_LOCAL_MACRO(156)
#    endif
#    if MSGPACK_PP_LOCAL_C(157)
        MSGPACK_PP_LOCAL_MACRO(157)
#    endif
#    if MSGPACK_PP_LOCAL_C(158)
        MSGPACK_PP_LOCAL_MACRO(158)
#    endif
#    if MSGPACK_PP_LOCAL_C(159)
        MSGPACK_PP_LOCAL_MACRO(159)
#    endif
#    if MSGPACK_PP_LOCAL_C(160)
        MSGPACK_PP_LOCAL_MACRO(160)
#    endif
#    if MSGPACK_PP_LOCAL_C(161)
        MSGPACK_PP_LOCAL_MACRO(161)
#    endif
#    if MSGPACK_PP_LOCAL_C(162)
        MSGPACK_PP_LOCAL_MACRO(162)
#    endif
#    if MSGPACK_PP_LOCAL_C(163)
        MSGPACK_PP_LOCAL_MACRO(163)
#    endif
#    if MSGPACK_PP_LOCAL_C(164)
        MSGPACK_PP_LOCAL_MACRO(164)
#    endif
#    if MSGPACK_PP_LOCAL_C(165)
        MSGPACK_PP_LOCAL_MACRO(165)
#    endif
#    if MSGPACK_PP_LOCAL_C(166)
        MSGPACK_PP_LOCAL_MACRO(166)
#    endif
#    if MSGPACK_PP_LOCAL_C(167)
        MSGPACK_PP_LOCAL_MACRO(167)
#    endif
#    if MSGPACK_PP_LOCAL_C(168)
        MSGPACK_PP_LOCAL_MACRO(168)
#    endif
#    if MSGPACK_PP_LOCAL_C(169)
        MSGPACK_PP_LOCAL_MACRO(169)
#    endif
#    if MSGPACK_PP_LOCAL_C(170)
        MSGPACK_PP_LOCAL_MACRO(170)
#    endif
#    if MSGPACK_PP_LOCAL_C(171)
        MSGPACK_PP_LOCAL_MACRO(171)
#    endif
#    if MSGPACK_PP_LOCAL_C(172)
        MSGPACK_PP_LOCAL_MACRO(172)
#    endif
#    if MSGPACK_PP_LOCAL_C(173)
        MSGPACK_PP_LOCAL_MACRO(173)
#    endif
#    if MSGPACK_PP_LOCAL_C(174)
        MSGPACK_PP_LOCAL_MACRO(174)
#    endif
#    if MSGPACK_PP_LOCAL_C(175)
        MSGPACK_PP_LOCAL_MACRO(175)
#    endif
#    if MSGPACK_PP_LOCAL_C(176)
        MSGPACK_PP_LOCAL_MACRO(176)
#    endif
#    if MSGPACK_PP_LOCAL_C(177)
        MSGPACK_PP_LOCAL_MACRO(177)
#    endif
#    if MSGPACK_PP_LOCAL_C(178)
        MSGPACK_PP_LOCAL_MACRO(178)
#    endif
#    if MSGPACK_PP_LOCAL_C(179)
        MSGPACK_PP_LOCAL_MACRO(179)
#    endif
#    if MSGPACK_PP_LOCAL_C(180)
        MSGPACK_PP_LOCAL_MACRO(180)
#    endif
#    if MSGPACK_PP_LOCAL_C(181)
        MSGPACK_PP_LOCAL_MACRO(181)
#    endif
#    if MSGPACK_PP_LOCAL_C(182)
        MSGPACK_PP_LOCAL_MACRO(182)
#    endif
#    if MSGPACK_PP_LOCAL_C(183)
        MSGPACK_PP_LOCAL_MACRO(183)
#    endif
#    if MSGPACK_PP_LOCAL_C(184)
        MSGPACK_PP_LOCAL_MACRO(184)
#    endif
#    if MSGPACK_PP_LOCAL_C(185)
        MSGPACK_PP_LOCAL_MACRO(185)
#    endif
#    if MSGPACK_PP_LOCAL_C(186)
        MSGPACK_PP_LOCAL_MACRO(186)
#    endif
#    if MSGPACK_PP_LOCAL_C(187)
        MSGPACK_PP_LOCAL_MACRO(187)
#    endif
#    if MSGPACK_PP_LOCAL_C(188)
        MSGPACK_PP_LOCAL_MACRO(188)
#    endif
#    if MSGPACK_PP_LOCAL_C(189)
        MSGPACK_PP_LOCAL_MACRO(189)
#    endif
#    if MSGPACK_PP_LOCAL_C(190)
        MSGPACK_PP_LOCAL_MACRO(190)
#    endif
#    if MSGPACK_PP_LOCAL_C(191)
        MSGPACK_PP_LOCAL_MACRO(191)
#    endif
#    if MSGPACK_PP_LOCAL_C(192)
        MSGPACK_PP_LOCAL_MACRO(192)
#    endif
#    if MSGPACK_PP_LOCAL_C(193)
        MSGPACK_PP_LOCAL_MACRO(193)
#    endif
#    if MSGPACK_PP_LOCAL_C(194)
        MSGPACK_PP_LOCAL_MACRO(194)
#    endif
#    if MSGPACK_PP_LOCAL_C(195)
        MSGPACK_PP_LOCAL_MACRO(195)
#    endif
#    if MSGPACK_PP_LOCAL_C(196)
        MSGPACK_PP_LOCAL_MACRO(196)
#    endif
#    if MSGPACK_PP_LOCAL_C(197)
        MSGPACK_PP_LOCAL_MACRO(197)
#    endif
#    if MSGPACK_PP_LOCAL_C(198)
        MSGPACK_PP_LOCAL_MACRO(198)
#    endif
#    if MSGPACK_PP_LOCAL_C(199)
        MSGPACK_PP_LOCAL_MACRO(199)
#    endif
#    if MSGPACK_PP_LOCAL_C(200)
        MSGPACK_PP_LOCAL_MACRO(200)
#    endif
#    if MSGPACK_PP_LOCAL_C(201)
        MSGPACK_PP_LOCAL_MACRO(201)
#    endif
#    if MSGPACK_PP_LOCAL_C(202)
        MSGPACK_PP_LOCAL_MACRO(202)
#    endif
#    if MSGPACK_PP_LOCAL_C(203)
        MSGPACK_PP_LOCAL_MACRO(203)
#    endif
#    if MSGPACK_PP_LOCAL_C(204)
        MSGPACK_PP_LOCAL_MACRO(204)
#    endif
#    if MSGPACK_PP_LOCAL_C(205)
        MSGPACK_PP_LOCAL_MACRO(205)
#    endif
#    if MSGPACK_PP_LOCAL_C(206)
        MSGPACK_PP_LOCAL_MACRO(206)
#    endif
#    if MSGPACK_PP_LOCAL_C(207)
        MSGPACK_PP_LOCAL_MACRO(207)
#    endif
#    if MSGPACK_PP_LOCAL_C(208)
        MSGPACK_PP_LOCAL_MACRO(208)
#    endif
#    if MSGPACK_PP_LOCAL_C(209)
        MSGPACK_PP_LOCAL_MACRO(209)
#    endif
#    if MSGPACK_PP_LOCAL_C(210)
        MSGPACK_PP_LOCAL_MACRO(210)
#    endif
#    if MSGPACK_PP_LOCAL_C(211)
        MSGPACK_PP_LOCAL_MACRO(211)
#    endif
#    if MSGPACK_PP_LOCAL_C(212)
        MSGPACK_PP_LOCAL_MACRO(212)
#    endif
#    if MSGPACK_PP_LOCAL_C(213)
        MSGPACK_PP_LOCAL_MACRO(213)
#    endif
#    if MSGPACK_PP_LOCAL_C(214)
        MSGPACK_PP_LOCAL_MACRO(214)
#    endif
#    if MSGPACK_PP_LOCAL_C(215)
        MSGPACK_PP_LOCAL_MACRO(215)
#    endif
#    if MSGPACK_PP_LOCAL_C(216)
        MSGPACK_PP_LOCAL_MACRO(216)
#    endif
#    if MSGPACK_PP_LOCAL_C(217)
        MSGPACK_PP_LOCAL_MACRO(217)
#    endif
#    if MSGPACK_PP_LOCAL_C(218)
        MSGPACK_PP_LOCAL_MACRO(218)
#    endif
#    if MSGPACK_PP_LOCAL_C(219)
        MSGPACK_PP_LOCAL_MACRO(219)
#    endif
#    if MSGPACK_PP_LOCAL_C(220)
        MSGPACK_PP_LOCAL_MACRO(220)
#    endif
#    if MSGPACK_PP_LOCAL_C(221)
        MSGPACK_PP_LOCAL_MACRO(221)
#    endif
#    if MSGPACK_PP_LOCAL_C(222)
        MSGPACK_PP_LOCAL_MACRO(222)
#    endif
#    if MSGPACK_PP_LOCAL_C(223)
        MSGPACK_PP_LOCAL_MACRO(223)
#    endif
#    if MSGPACK_PP_LOCAL_C(224)
        MSGPACK_PP_LOCAL_MACRO(224)
#    endif
#    if MSGPACK_PP_LOCAL_C(225)
        MSGPACK_PP_LOCAL_MACRO(225)
#    endif
#    if MSGPACK_PP_LOCAL_C(226)
        MSGPACK_PP_LOCAL_MACRO(226)
#    endif
#    if MSGPACK_PP_LOCAL_C(227)
        MSGPACK_PP_LOCAL_MACRO(227)
#    endif
#    if MSGPACK_PP_LOCAL_C(228)
        MSGPACK_PP_LOCAL_MACRO(228)
#    endif
#    if MSGPACK_PP_LOCAL_C(229)
        MSGPACK_PP_LOCAL_MACRO(229)
#    endif
#    if MSGPACK_PP_LOCAL_C(230)
        MSGPACK_PP_LOCAL_MACRO(230)
#    endif
#    if MSGPACK_PP_LOCAL_C(231)
        MSGPACK_PP_LOCAL_MACRO(231)
#    endif
#    if MSGPACK_PP_LOCAL_C(232)
        MSGPACK_PP_LOCAL_MACRO(232)
#    endif
#    if MSGPACK_PP_LOCAL_C(233)
        MSGPACK_PP_LOCAL_MACRO(233)
#    endif
#    if MSGPACK_PP_LOCAL_C(234)
        MSGPACK_PP_LOCAL_MACRO(234)
#    endif
#    if MSGPACK_PP_LOCAL_C(235)
        MSGPACK_PP_LOCAL_MACRO(235)
#    endif
#    if MSGPACK_PP_LOCAL_C(236)
        MSGPACK_PP_LOCAL_MACRO(236)
#    endif

#    if MSGPACK_PP_LOCAL_C(237)
        MSGPACK_PP_LOCAL_MACRO(237)
#    endif
#    if MSGPACK_PP_LOCAL_C(238)
        MSGPACK_PP_LOCAL_MACRO(238)
#    endif
#    if MSGPACK_PP_LOCAL_C(239)
        MSGPACK_PP_LOCAL_MACRO(239)
#    endif
#    if MSGPACK_PP_LOCAL_C(240)
        MSGPACK_PP_LOCAL_MACRO(240)
#    endif
#    if MSGPACK_PP_LOCAL_C(241)
        MSGPACK_PP_LOCAL_MACRO(241)
#    endif
#    if MSGPACK_PP_LOCAL_C(242)
        MSGPACK_PP_LOCAL_MACRO(242)
#    endif
#    if MSGPACK_PP_LOCAL_C(243)
        MSGPACK_PP_LOCAL_MACRO(243)
#    endif
#    if MSGPACK_PP_LOCAL_C(244)
        MSGPACK_PP_LOCAL_MACRO(244)
#    endif
#    if MSGPACK_PP_LOCAL_C(245)
        MSGPACK_PP_LOCAL_MACRO(245)
#    endif
#    if MSGPACK_PP_LOCAL_C(246)
        MSGPACK_PP_LOCAL_MACRO(246)
#    endif
#    if MSGPACK_PP_LOCAL_C(247)
        MSGPACK_PP_LOCAL_MACRO(247)
#    endif
#    if MSGPACK_PP_LOCAL_C(248)
        MSGPACK_PP_LOCAL_MACRO(248)
#    endif
#    if MSGPACK_PP_LOCAL_C(249)
        MSGPACK_PP_LOCAL_MACRO(249)
#    endif
#    if MSGPACK_PP_LOCAL_C(250)
        MSGPACK_PP_LOCAL_MACRO(250)
#    endif
#    if MSGPACK_PP_LOCAL_C(251)
        MSGPACK_PP_LOCAL_MACRO(251)
#    endif
#    if MSGPACK_PP_LOCAL_C(252)
        MSGPACK_PP_LOCAL_MACRO(252)
#    endif
#    if MSGPACK_PP_LOCAL_C(253)
        MSGPACK_PP_LOCAL_MACRO(253)
#    endif
#    if MSGPACK_PP_LOCAL_C(254)
        MSGPACK_PP_LOCAL_MACRO(254)
#    endif
#    if MSGPACK_PP_LOCAL_C(255)
        MSGPACK_PP_LOCAL_MACRO(255)
#    endif
#    if MSGPACK_PP_LOCAL_C(256)
        MSGPACK_PP_LOCAL_MACRO(256)
#    endif
# endif
#
# undef MSGPACK_PP_LOCAL_LIMITS
#
# undef MSGPACK_PP_LOCAL_S
# undef MSGPACK_PP_LOCAL_F
#
# undef MSGPACK_PP_LOCAL_MACRO
