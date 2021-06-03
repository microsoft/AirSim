/*
*  linenoise.hpp -- Multi-platfrom C++ header-only linenoise library.
*
*  All credits and commendations have to go to the authors of the
*  following excellent libraries.
*
*  - linenoise.h and linenose.c (https://github.com/antirez/linenoise)
*  - ANSI.c (https://github.com/adoxa/ansicon)
*  - Win32_ANSI.h and Win32_ANSI.c (https://github.com/MSOpenTech/redis)
*
* ------------------------------------------------------------------------
*
*  Copyright (c) 2015 yhirose
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
*  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
*  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* linenoise.h -- guerrilla line editing library against the idea that a
* line editing lib needs to be 20,000 lines of C code.
*
* See linenoise.c for more information.
*
* ------------------------------------------------------------------------
*
* Copyright (c) 2010, Salvatore Sanfilippo <antirez at gmail dot com>
* Copyright (c) 2010, Pieter Noordhuis <pcnoordhuis at gmail dot com>
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*
*  *  Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*  *  Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
* ANSI.c - ANSI escape sequence console driver.
*
* Copyright (C) 2005-2014 Jason Hood
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the author be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
* 
* Jason Hood
* jadoxa@yahoo.com.au
*/

/*
* Win32_ANSI.h and Win32_ANSI.c
*
* Derived from ANSI.c by Jason Hood, from his ansicon project (https://github.com/adoxa/ansicon), with modifications.
*
* Copyright (c), Microsoft Open Technologies, Inc.
* All rights reserved.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __LINENOISE_HPP
#define __LINENOISE_HPP

#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#else
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <io.h>
#ifndef STDIN_FILENO
#define STDIN_FILENO (_fileno(stdin))
#endif
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#define isatty _isatty
#define write win32_write
#define read _read
#endif
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/types.h>
#include <string>
#include <fstream>
#include <functional>
#include <vector>
#include <iostream>

namespace linenoise
{

typedef std::function<void(const char*, std::vector<std::string>&)> CompletionCallback;

#ifdef _WIN32

namespace ansi
{

#define lenof(array) (sizeof(array) / sizeof(*(array)))

    typedef struct
    {
        BYTE foreground; // ANSI base color (0 to 7; add 30)
        BYTE background; // ANSI base color (0 to 7; add 40)
        BYTE bold; // console FOREGROUND_INTENSITY bit
        BYTE underline; // console BACKGROUND_INTENSITY bit
        BYTE rvideo; // swap foreground/bold & background/underline
        BYTE concealed; // set foreground/bold to background/underline
        BYTE reverse; // swap console foreground & background attributes
    } GRM, *PGRM; // Graphic Rendition Mode

    inline bool is_digit(char c) { return '0' <= c && c <= '9'; }

    // ========== Global variables and constants

    HANDLE hConOut; // handle to CONOUT$

    const char ESC = '\x1B'; // ESCape character
    const char BEL = '\x07';
    const char SO = '\x0E'; // Shift Out
    const char SI = '\x0F'; // Shift In

    const size_t MAX_ARG = 16; // max number of args in an escape sequence
    int state; // automata state
    WCHAR prefix; // escape sequence prefix ( '[', ']' or '(' );
    WCHAR prefix2; // secondary prefix ( '?' or '>' );
    WCHAR suffix; // escape sequence suffix
    int es_argc; // escape sequence args count
    int es_argv[MAX_ARG]; // escape sequence args
    WCHAR Pt_arg[MAX_PATH * 2]; // text parameter for Operating System Command
    int Pt_len;
    BOOL shifted;

    // DEC Special Graphics Character Set from
    // http://vt100.net/docs/vt220-rm/table2-4.html
    // Some of these may not look right, depending on the font and code page (in
    // particular, the Control Pictures probably won't work at all).
    const WCHAR G1[] = {
        ' ', // _ - blank
        L'\x2666', // ` - Black Diamond Suit
        L'\x2592', // a - Medium Shade
        L'\x2409', // b - HT
        L'\x240c', // c - FF
        L'\x240d', // d - CR
        L'\x240a', // e - LF
        L'\x00b0', // f - Degree Sign
        L'\x00b1', // g - Plus-Minus Sign
        L'\x2424', // h - NL
        L'\x240b', // i - VT
        L'\x2518', // j - Box Drawings Light Up And Left
        L'\x2510', // k - Box Drawings Light Down And Left
        L'\x250c', // l - Box Drawings Light Down And Right
        L'\x2514', // m - Box Drawings Light Up And Right
        L'\x253c', // n - Box Drawings Light Vertical And Horizontal
        L'\x00af', // o - SCAN 1 - Macron
        L'\x25ac', // p - SCAN 3 - Black Rectangle
        L'\x2500', // q - SCAN 5 - Box Drawings Light Horizontal
        L'_', // r - SCAN 7 - Low Line
        L'_', // s - SCAN 9 - Low Line
        L'\x251c', // t - Box Drawings Light Vertical And Right
        L'\x2524', // u - Box Drawings Light Vertical And Left
        L'\x2534', // v - Box Drawings Light Up And Horizontal
        L'\x252c', // w - Box Drawings Light Down And Horizontal
        L'\x2502', // x - Box Drawings Light Vertical
        L'\x2264', // y - Less-Than Or Equal To
        L'\x2265', // z - Greater-Than Or Equal To
        L'\x03c0', // { - Greek Small Letter Pi
        L'\x2260', // | - Not Equal To
        L'\x00a3', // } - Pound Sign
        L'\x00b7', // ~ - Middle Dot
    };

#define FIRST_G1 '_'
#define LAST_G1 '~'

    // color constants

#define FOREGROUND_BLACK 0
#define FOREGROUND_WHITE FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE

#define BACKGROUND_BLACK 0
#define BACKGROUND_WHITE BACKGROUND_RED | BACKGROUND_GREEN | BACKGROUND_BLUE

    const BYTE foregroundcolor[8] = {
        FOREGROUND_BLACK, // black foreground
        FOREGROUND_RED, // red foreground
        FOREGROUND_GREEN, // green foreground
        FOREGROUND_RED | FOREGROUND_GREEN, // yellow foreground
        FOREGROUND_BLUE, // blue foreground
        FOREGROUND_BLUE | FOREGROUND_RED, // magenta foreground
        FOREGROUND_BLUE | FOREGROUND_GREEN, // cyan foreground
        FOREGROUND_WHITE // white foreground
    };

    const BYTE backgroundcolor[8] = {
        BACKGROUND_BLACK, // black background
        BACKGROUND_RED, // red background
        BACKGROUND_GREEN, // green background
        BACKGROUND_RED | BACKGROUND_GREEN, // yellow background
        BACKGROUND_BLUE, // blue background
        BACKGROUND_BLUE | BACKGROUND_RED, // magenta background
        BACKGROUND_BLUE | BACKGROUND_GREEN, // cyan background
        BACKGROUND_WHITE, // white background
    };

    const BYTE attr2ansi[8] = // map console attribute to ANSI number
        {
            0, // black
            4, // blue
            2, // green
            6, // cyan
            1, // red
            5, // magenta
            3, // yellow
            7 // white
        };

    GRM grm;

    // saved cursor position
    COORD SavePos;

    // ========== Print Buffer functions

#define BUFFER_SIZE 2048

    int nCharInBuffer;
    WCHAR ChBuffer[BUFFER_SIZE];

    //-----------------------------------------------------------------------------
    //   FlushBuffer()
    // Writes the buffer to the console and empties it.
    //-----------------------------------------------------------------------------

    inline void FlushBuffer(void)
    {
        DWORD nWritten;
        if (nCharInBuffer <= 0) return;
        WriteConsoleW(hConOut, ChBuffer, nCharInBuffer, &nWritten, NULL);
        nCharInBuffer = 0;
    }

    //-----------------------------------------------------------------------------
    //   PushBuffer( WCHAR c )
    // Adds a character in the buffer.
    //-----------------------------------------------------------------------------

    inline void PushBuffer(WCHAR c)
    {
        if (shifted && c >= FIRST_G1 && c <= LAST_G1)
            c = G1[c - FIRST_G1];
        ChBuffer[nCharInBuffer] = c;
        if (++nCharInBuffer == BUFFER_SIZE)
            FlushBuffer();
    }

    //-----------------------------------------------------------------------------
    //   SendSequence( LPWSTR seq )
    // Send the string to the input buffer.
    //-----------------------------------------------------------------------------

    inline void SendSequence(LPWSTR seq)
    {
        DWORD out;
        INPUT_RECORD in;
        HANDLE hStdIn = GetStdHandle(STD_INPUT_HANDLE);

        in.EventType = KEY_EVENT;
        in.Event.KeyEvent.bKeyDown = TRUE;
        in.Event.KeyEvent.wRepeatCount = 1;
        in.Event.KeyEvent.wVirtualKeyCode = 0;
        in.Event.KeyEvent.wVirtualScanCode = 0;
        in.Event.KeyEvent.dwControlKeyState = 0;
        for (; *seq; ++seq) {
            in.Event.KeyEvent.uChar.UnicodeChar = *seq;
            WriteConsoleInput(hStdIn, &in, 1, &out);
        }
    }

    // ========== Print functions

    //-----------------------------------------------------------------------------
    //   InterpretEscSeq()
    // Interprets the last escape sequence scanned by ParseAndPrintANSIString
    //   prefix             escape sequence prefix
    //   es_argc            escape sequence args count
    //   es_argv[]          escape sequence args array
    //   suffix             escape sequence suffix
    //
    // for instance, with \e[33;45;1m we have
    // prefix = '[',
    // es_argc = 3, es_argv[0] = 33, es_argv[1] = 45, es_argv[2] = 1
    // suffix = 'm'
    //-----------------------------------------------------------------------------

    inline void InterpretEscSeq(void)
    {
        int i;
        WORD attribut;
        CONSOLE_SCREEN_BUFFER_INFO Info;
        CONSOLE_CURSOR_INFO CursInfo;
        DWORD len, NumberOfCharsWritten;
        COORD Pos;
        SMALL_RECT Rect;
        CHAR_INFO CharInfo;

        if (prefix == '[') {
            if (prefix2 == '?' && (suffix == 'h' || suffix == 'l')) {
                if (es_argc == 1 && es_argv[0] == 25) {
                    GetConsoleCursorInfo(hConOut, &CursInfo);
                    CursInfo.bVisible = (suffix == 'h');
                    SetConsoleCursorInfo(hConOut, &CursInfo);
                    return;
                }
            }
            // Ignore any other \e[? or \e[> sequences.
            if (prefix2 != 0)
                return;

            GetConsoleScreenBufferInfo(hConOut, &Info);
            switch (suffix) {
            case 'm':
                if (es_argc == 0) es_argv[es_argc++] = 0;
                for (i = 0; i < es_argc; i++) {
                    if (30 <= es_argv[i] && es_argv[i] <= 37)
                        grm.foreground = es_argv[i] - 30;
                    else if (40 <= es_argv[i] && es_argv[i] <= 47)
                        grm.background = es_argv[i] - 40;
                    else
                        switch (es_argv[i]) {
                        case 0:
                        case 39:
                        case 49: {
                            WCHAR def[4];
                            int a;
                            *def = '7';
                            def[1] = '\0';
                            GetEnvironmentVariableW(L"ANSICON_DEF", def, lenof(def));
                            a = wcstol(def, NULL, 16);
                            grm.reverse = FALSE;
                            if (a < 0) {
                                grm.reverse = TRUE;
                                a = -a;
                            }
                            if (es_argv[i] != 49)
                                grm.foreground = attr2ansi[a & 7];
                            if (es_argv[i] != 39)
                                grm.background = attr2ansi[(a >> 4) & 7];
                            if (es_argv[i] == 0) {
                                if (es_argc == 1) {
                                    grm.bold = a & FOREGROUND_INTENSITY;
                                    grm.underline = a & BACKGROUND_INTENSITY;
                                }
                                else {
                                    grm.bold = 0;
                                    grm.underline = 0;
                                }
                                grm.rvideo = 0;
                                grm.concealed = 0;
                            }
                        } break;

                        case 1:
                            grm.bold = FOREGROUND_INTENSITY;
                            break;
                        case 5: // blink
                        case 4:
                            grm.underline = BACKGROUND_INTENSITY;
                            break;
                        case 7:
                            grm.rvideo = 1;
                            break;
                        case 8:
                            grm.concealed = 1;
                            break;
                        case 21: // oops, this actually turns on double underline
                        case 22:
                            grm.bold = 0;
                            break;
                        case 25:
                        case 24:
                            grm.underline = 0;
                            break;
                        case 27:
                            grm.rvideo = 0;
                            break;
                        case 28:
                            grm.concealed = 0;
                            break;
                        }
                }
                if (grm.concealed) {
                    if (grm.rvideo) {
                        attribut = foregroundcolor[grm.foreground] | backgroundcolor[grm.foreground];
                        if (grm.bold)
                            attribut |= FOREGROUND_INTENSITY | BACKGROUND_INTENSITY;
                    }
                    else {
                        attribut = foregroundcolor[grm.background] | backgroundcolor[grm.background];
                        if (grm.underline)
                            attribut |= FOREGROUND_INTENSITY | BACKGROUND_INTENSITY;
                    }
                }
                else if (grm.rvideo) {
                    attribut = foregroundcolor[grm.background] | backgroundcolor[grm.foreground];
                    if (grm.bold)
                        attribut |= BACKGROUND_INTENSITY;
                    if (grm.underline)
                        attribut |= FOREGROUND_INTENSITY;
                }
                else
                    attribut = foregroundcolor[grm.foreground] | grm.bold | backgroundcolor[grm.background] | grm.underline;
                if (grm.reverse)
                    attribut = ((attribut >> 4) & 15) | ((attribut & 15) << 4);
                SetConsoleTextAttribute(hConOut, attribut);
                return;

            case 'J':
                if (es_argc == 0) es_argv[es_argc++] = 0; // ESC[J == ESC[0J
                if (es_argc != 1) return;
                switch (es_argv[0]) {
                case 0: // ESC[0J erase from cursor to end of display
                    len = (Info.dwSize.Y - Info.dwCursorPosition.Y - 1) * Info.dwSize.X + Info.dwSize.X - Info.dwCursorPosition.X - 1;
                    FillConsoleOutputCharacter(hConOut, ' ', len, Info.dwCursorPosition, &NumberOfCharsWritten);
                    FillConsoleOutputAttribute(hConOut, Info.wAttributes, len, Info.dwCursorPosition, &NumberOfCharsWritten);
                    return;

                case 1: // ESC[1J erase from start to cursor.
                    Pos.X = 0;
                    Pos.Y = 0;
                    len = Info.dwCursorPosition.Y * Info.dwSize.X + Info.dwCursorPosition.X + 1;
                    FillConsoleOutputCharacter(hConOut, ' ', len, Pos, &NumberOfCharsWritten);
                    FillConsoleOutputAttribute(hConOut, Info.wAttributes, len, Pos, &NumberOfCharsWritten);
                    return;

                case 2: // ESC[2J Clear screen and home cursor
                    Pos.X = 0;
                    Pos.Y = 0;
                    len = Info.dwSize.X * Info.dwSize.Y;
                    FillConsoleOutputCharacter(hConOut, ' ', len, Pos, &NumberOfCharsWritten);
                    FillConsoleOutputAttribute(hConOut, Info.wAttributes, len, Pos, &NumberOfCharsWritten);
                    SetConsoleCursorPosition(hConOut, Pos);
                    return;

                default:
                    return;
                }

            case 'K':
                if (es_argc == 0) es_argv[es_argc++] = 0; // ESC[K == ESC[0K
                if (es_argc != 1) return;
                switch (es_argv[0]) {
                case 0: // ESC[0K Clear to end of line
                    len = Info.dwSize.X - Info.dwCursorPosition.X + 1;
                    FillConsoleOutputCharacter(hConOut, ' ', len, Info.dwCursorPosition, &NumberOfCharsWritten);
                    FillConsoleOutputAttribute(hConOut, Info.wAttributes, len, Info.dwCursorPosition, &NumberOfCharsWritten);
                    return;

                case 1: // ESC[1K Clear from start of line to cursor
                    Pos.X = 0;
                    Pos.Y = Info.dwCursorPosition.Y;
                    FillConsoleOutputCharacter(hConOut, ' ', Info.dwCursorPosition.X + 1, Pos, &NumberOfCharsWritten);
                    FillConsoleOutputAttribute(hConOut, Info.wAttributes, Info.dwCursorPosition.X + 1, Pos, &NumberOfCharsWritten);
                    return;

                case 2: // ESC[2K Clear whole line.
                    Pos.X = 0;
                    Pos.Y = Info.dwCursorPosition.Y;
                    FillConsoleOutputCharacter(hConOut, ' ', Info.dwSize.X, Pos, &NumberOfCharsWritten);
                    FillConsoleOutputAttribute(hConOut, Info.wAttributes, Info.dwSize.X, Pos, &NumberOfCharsWritten);
                    return;

                default:
                    return;
                }

            case 'X': // ESC[#X Erase # characters.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[X == ESC[1X
                if (es_argc != 1) return;
                FillConsoleOutputCharacter(hConOut, ' ', es_argv[0], Info.dwCursorPosition, &NumberOfCharsWritten);
                FillConsoleOutputAttribute(hConOut, Info.wAttributes, es_argv[0], Info.dwCursorPosition, &NumberOfCharsWritten);
                return;

            case 'L': // ESC[#L Insert # blank lines.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[L == ESC[1L
                if (es_argc != 1) return;
                Rect.Left = 0;
                Rect.Top = Info.dwCursorPosition.Y;
                Rect.Right = Info.dwSize.X - 1;
                Rect.Bottom = Info.dwSize.Y - 1;
                Pos.X = 0;
                Pos.Y = Info.dwCursorPosition.Y + es_argv[0];
                CharInfo.Char.UnicodeChar = ' ';
                CharInfo.Attributes = Info.wAttributes;
                ScrollConsoleScreenBuffer(hConOut, &Rect, NULL, Pos, &CharInfo);
                return;

            case 'M': // ESC[#M Delete # lines.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[M == ESC[1M
                if (es_argc != 1) return;
                if (es_argv[0] > Info.dwSize.Y - Info.dwCursorPosition.Y)
                    es_argv[0] = Info.dwSize.Y - Info.dwCursorPosition.Y;
                Rect.Left = 0;
                Rect.Top = Info.dwCursorPosition.Y + es_argv[0];
                Rect.Right = Info.dwSize.X - 1;
                Rect.Bottom = Info.dwSize.Y - 1;
                Pos.X = 0;
                Pos.Y = Info.dwCursorPosition.Y;
                CharInfo.Char.UnicodeChar = ' ';
                CharInfo.Attributes = Info.wAttributes;
                ScrollConsoleScreenBuffer(hConOut, &Rect, NULL, Pos, &CharInfo);
                return;

            case 'P': // ESC[#P Delete # characters.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[P == ESC[1P
                if (es_argc != 1) return;
                if (Info.dwCursorPosition.X + es_argv[0] > Info.dwSize.X - 1)
                    es_argv[0] = Info.dwSize.X - Info.dwCursorPosition.X;
                Rect.Left = Info.dwCursorPosition.X + es_argv[0];
                Rect.Top = Info.dwCursorPosition.Y;
                Rect.Right = Info.dwSize.X - 1;
                Rect.Bottom = Info.dwCursorPosition.Y;
                CharInfo.Char.UnicodeChar = ' ';
                CharInfo.Attributes = Info.wAttributes;
                ScrollConsoleScreenBuffer(hConOut, &Rect, NULL, Info.dwCursorPosition, &CharInfo);
                return;

            case '@': // ESC[#@ Insert # blank characters.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[@ == ESC[1@
                if (es_argc != 1) return;
                if (Info.dwCursorPosition.X + es_argv[0] > Info.dwSize.X - 1)
                    es_argv[0] = Info.dwSize.X - Info.dwCursorPosition.X;
                Rect.Left = Info.dwCursorPosition.X;
                Rect.Top = Info.dwCursorPosition.Y;
                Rect.Right = Info.dwSize.X - 1 - es_argv[0];
                Rect.Bottom = Info.dwCursorPosition.Y;
                Pos.X = Info.dwCursorPosition.X + es_argv[0];
                Pos.Y = Info.dwCursorPosition.Y;
                CharInfo.Char.UnicodeChar = ' ';
                CharInfo.Attributes = Info.wAttributes;
                ScrollConsoleScreenBuffer(hConOut, &Rect, NULL, Pos, &CharInfo);
                return;

            case 'k': // ESC[#k
            case 'A': // ESC[#A Moves cursor up # lines
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[A == ESC[1A
                if (es_argc != 1) return;
                Pos.Y = Info.dwCursorPosition.Y - es_argv[0];
                if (Pos.Y < 0) Pos.Y = 0;
                Pos.X = Info.dwCursorPosition.X;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 'e': // ESC[#e
            case 'B': // ESC[#B Moves cursor down # lines
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[B == ESC[1B
                if (es_argc != 1) return;
                Pos.Y = Info.dwCursorPosition.Y + es_argv[0];
                if (Pos.Y >= Info.dwSize.Y) Pos.Y = Info.dwSize.Y - 1;
                Pos.X = Info.dwCursorPosition.X;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 'a': // ESC[#a
            case 'C': // ESC[#C Moves cursor forward # spaces
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[C == ESC[1C
                if (es_argc != 1) return;
                Pos.X = Info.dwCursorPosition.X + es_argv[0];
                if (Pos.X >= Info.dwSize.X) Pos.X = Info.dwSize.X - 1;
                Pos.Y = Info.dwCursorPosition.Y;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 'j': // ESC[#j
            case 'D': // ESC[#D Moves cursor back # spaces
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[D == ESC[1D
                if (es_argc != 1) return;
                Pos.X = Info.dwCursorPosition.X - es_argv[0];
                if (Pos.X < 0) Pos.X = 0;
                Pos.Y = Info.dwCursorPosition.Y;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 'E': // ESC[#E Moves cursor down # lines, column 1.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[E == ESC[1E
                if (es_argc != 1) return;
                Pos.Y = Info.dwCursorPosition.Y + es_argv[0];
                if (Pos.Y >= Info.dwSize.Y) Pos.Y = Info.dwSize.Y - 1;
                Pos.X = 0;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 'F': // ESC[#F Moves cursor up # lines, column 1.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[F == ESC[1F
                if (es_argc != 1) return;
                Pos.Y = Info.dwCursorPosition.Y - es_argv[0];
                if (Pos.Y < 0) Pos.Y = 0;
                Pos.X = 0;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case '`': // ESC[#`
            case 'G': // ESC[#G Moves cursor column # in current row.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[G == ESC[1G
                if (es_argc != 1) return;
                Pos.X = es_argv[0] - 1;
                if (Pos.X >= Info.dwSize.X) Pos.X = Info.dwSize.X - 1;
                if (Pos.X < 0) Pos.X = 0;
                Pos.Y = Info.dwCursorPosition.Y;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 'd': // ESC[#d Moves cursor row #, current column.
                if (es_argc == 0) es_argv[es_argc++] = 1; // ESC[d == ESC[1d
                if (es_argc != 1) return;
                Pos.Y = es_argv[0] - 1;
                if (Pos.Y < 0) Pos.Y = 0;
                if (Pos.Y >= Info.dwSize.Y) Pos.Y = Info.dwSize.Y - 1;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 'f': // ESC[#;#f
            case 'H': // ESC[#;#H Moves cursor to line #, column #
                if (es_argc == 0)
                    es_argv[es_argc++] = 1; // ESC[H == ESC[1;1H
                if (es_argc == 1)
                    es_argv[es_argc++] = 1; // ESC[#H == ESC[#;1H
                if (es_argc > 2) return;
                Pos.X = es_argv[1] - 1;
                if (Pos.X < 0) Pos.X = 0;
                if (Pos.X >= Info.dwSize.X) Pos.X = Info.dwSize.X - 1;
                Pos.Y = es_argv[0] - 1;
                if (Pos.Y < 0) Pos.Y = 0;
                if (Pos.Y >= Info.dwSize.Y) Pos.Y = Info.dwSize.Y - 1;
                SetConsoleCursorPosition(hConOut, Pos);
                return;

            case 's': // ESC[s Saves cursor position for recall later
                if (es_argc != 0) return;
                SavePos = Info.dwCursorPosition;
                return;

            case 'u': // ESC[u Return to saved cursor position
                if (es_argc != 0) return;
                SetConsoleCursorPosition(hConOut, SavePos);
                return;

            case 'n': // ESC[#n Device status report
                if (es_argc != 1) return; // ESC[n == ESC[0n -> ignored
                switch (es_argv[0]) {
                case 5: // ESC[5n Report status
                    SendSequence(L"\33[0n"); // "OK"
                    return;

                case 6: // ESC[6n Report cursor position
                {
                    WCHAR buf[32];
                    swprintf(buf, 32, L"\33[%d;%dR", Info.dwCursorPosition.Y + 1, Info.dwCursorPosition.X + 1);
                    SendSequence(buf);
                }
                    return;

                default:
                    return;
                }

            case 't': // ESC[#t Window manipulation
                if (es_argc != 1) return;
                if (es_argv[0] == 21) // ESC[21t Report xterm window's title
                {
                    WCHAR buf[MAX_PATH * 2];
                    DWORD len = GetConsoleTitleW(buf + 3, lenof(buf) - 3 - 2);
                    // Too bad if it's too big or fails.
                    buf[0] = ESC;
                    buf[1] = ']';
                    buf[2] = 'l';
                    buf[3 + len] = ESC;
                    buf[3 + len + 1] = '\\';
                    buf[3 + len + 2] = '\0';
                    SendSequence(buf);
                }
                return;

            default:
                return;
            }
        }
        else // (prefix == ']')
        {
            // Ignore any \e]? or \e]> sequences.
            if (prefix2 != 0)
                return;

            if (es_argc == 1 && es_argv[0] == 0) // ESC]0;titleST
            {
                SetConsoleTitleW(Pt_arg);
            }
        }
    }

    //-----------------------------------------------------------------------------
    //   ParseAndPrintANSIString(hDev, lpBuffer, nNumberOfBytesToWrite)
    // Parses the string lpBuffer, interprets the escapes sequences and prints the
    // characters in the device hDev (console).
    // The lexer is a three states automata.
    // If the number of arguments es_argc > MAX_ARG, only the MAX_ARG-1 firsts and
    // the last arguments are processed (no es_argv[] overflow).
    //-----------------------------------------------------------------------------

    inline BOOL ParseAndPrintANSIString(HANDLE hDev, LPCVOID lpBuffer, DWORD nNumberOfBytesToWrite, LPDWORD lpNumberOfBytesWritten)
    {
        DWORD i;
        LPCSTR s;

        if (hDev != hConOut) // reinit if device has changed
        {
            hConOut = hDev;
            state = 1;
            shifted = FALSE;
        }
        for (i = nNumberOfBytesToWrite, s = (LPCSTR)lpBuffer; i > 0; i--, s++) {
            if (state == 1) {
                if (*s == ESC)
                    state = 2;
                else if (*s == SO)
                    shifted = TRUE;
                else if (*s == SI)
                    shifted = FALSE;
                else
                    PushBuffer(*s);
            }
            else if (state == 2) {
                if (*s == ESC)
                    ; // \e\e...\e == \e
                else if ((*s == '[') || (*s == ']')) {
                    FlushBuffer();
                    prefix = *s;
                    prefix2 = 0;
                    state = 3;
                    Pt_len = 0;
                    *Pt_arg = '\0';
                }
                else if (*s == ')' || *s == '(')
                    state = 6;
                else
                    state = 1;
            }
            else if (state == 3) {
                if (is_digit(*s)) {
                    es_argc = 0;
                    es_argv[0] = *s - '0';
                    state = 4;
                }
                else if (*s == ';') {
                    es_argc = 1;
                    es_argv[0] = 0;
                    es_argv[1] = 0;
                    state = 4;
                }
                else if (*s == '?' || *s == '>') {
                    prefix2 = *s;
                }
                else {
                    es_argc = 0;
                    suffix = *s;
                    InterpretEscSeq();
                    state = 1;
                }
            }
            else if (state == 4) {
                if (is_digit(*s)) {
                    es_argv[es_argc] = 10 * es_argv[es_argc] + (*s - '0');
                }
                else if (*s == ';') {
                    if (es_argc < MAX_ARG - 1) es_argc++;
                    es_argv[es_argc] = 0;
                    if (prefix == ']')
                        state = 5;
                }
                else {
                    es_argc++;
                    suffix = *s;
                    InterpretEscSeq();
                    state = 1;
                }
            }
            else if (state == 5) {
                if (*s == BEL) {
                    Pt_arg[Pt_len] = '\0';
                    InterpretEscSeq();
                    state = 1;
                }
                else if (*s == '\\' && Pt_len > 0 && Pt_arg[Pt_len - 1] == ESC) {
                    Pt_arg[--Pt_len] = '\0';
                    InterpretEscSeq();
                    state = 1;
                }
                else if (Pt_len < lenof(Pt_arg) - 1)
                    Pt_arg[Pt_len++] = *s;
            }
            else if (state == 6) {
                // Ignore it (ESC ) 0 is implicit; nothing else is supported).
                state = 1;
            }
        }
        FlushBuffer();
        if (lpNumberOfBytesWritten != NULL)
            *lpNumberOfBytesWritten = nNumberOfBytesToWrite - i;
        return (i == 0);
    }

} // namespace ansi

HANDLE hOut;
HANDLE hIn;
DWORD consolemodeIn = 0;

inline int win32read(int* c)
{
    DWORD foo;
    INPUT_RECORD b;
    KEY_EVENT_RECORD e;
    BOOL altgr;

    while (1) {
        if (!ReadConsoleInput(hIn, &b, 1, &foo)) return 0;
        if (!foo) return 0;

        if (b.EventType == KEY_EVENT && b.Event.KeyEvent.bKeyDown) {

            e = b.Event.KeyEvent;
            *c = b.Event.KeyEvent.uChar.AsciiChar;

            altgr = e.dwControlKeyState & (LEFT_CTRL_PRESSED | RIGHT_ALT_PRESSED);

            if (e.dwControlKeyState & (LEFT_CTRL_PRESSED | RIGHT_CTRL_PRESSED) && !altgr) {

                /* Ctrl+Key */
                switch (*c) {
                case 'D':
                    *c = 4;
                    return 1;
                case 'C':
                    *c = 3;
                    return 1;
                case 'H':
                    *c = 8;
                    return 1;
                case 'T':
                    *c = 20;
                    return 1;
                case 'B': /* ctrl-b, left_arrow */
                    *c = 2;
                    return 1;
                case 'F': /* ctrl-f right_arrow*/
                    *c = 6;
                    return 1;
                case 'P': /* ctrl-p up_arrow*/
                    *c = 16;
                    return 1;
                case 'N': /* ctrl-n down_arrow*/
                    *c = 14;
                    return 1;
                case 'U': /* Ctrl+u, delete the whole line. */
                    *c = 21;
                    return 1;
                case 'K': /* Ctrl+k, delete from current to end of line. */
                    *c = 11;
                    return 1;
                case 'A': /* Ctrl+a, go to the start of the line */
                    *c = 1;
                    return 1;
                case 'E': /* ctrl+e, go to the end of the line */
                    *c = 5;
                    return 1;
                }

                /* Other Ctrl+KEYs ignored */
            }
            else {

                switch (e.wVirtualKeyCode) {

                case VK_ESCAPE: /* ignore - send ctrl-c, will return -1 */
                    *c = 3;
                    return 1;
                case VK_RETURN: /* enter */
                    *c = 13;
                    return 1;
                case VK_LEFT: /* left */
                    *c = 2;
                    return 1;
                case VK_RIGHT: /* right */
                    *c = 6;
                    return 1;
                case VK_UP: /* up */
                    *c = 16;
                    return 1;
                case VK_DOWN: /* down */
                    *c = 14;
                    return 1;
                case VK_HOME:
                    *c = 1;
                    return 1;
                case VK_END:
                    *c = 5;
                    return 1;
                case VK_BACK:
                    *c = 8;
                    return 1;
                case VK_DELETE:
                    *c = 127;
                    return 1;
                default:
                    if (*c) return 1;
                }
            }
        }
    }

    return -1; /* Makes compiler happy */
}

inline int win32_write(int fd, const void* buffer, unsigned int count)
{
    if (fd == _fileno(stdout)) {
        DWORD bytesWritten = 0;
        if (FALSE != ansi::ParseAndPrintANSIString(GetStdHandle(STD_OUTPUT_HANDLE), buffer, (DWORD)count, &bytesWritten)) {
            return (int)bytesWritten;
        }
        else {
            errno = GetLastError();
            return 0;
        }
    }
    else if (fd == _fileno(stderr)) {
        DWORD bytesWritten = 0;
        if (FALSE != ansi::ParseAndPrintANSIString(GetStdHandle(STD_ERROR_HANDLE), buffer, (DWORD)count, &bytesWritten)) {
            return (int)bytesWritten;
        }
        else {
            errno = GetLastError();
            return 0;
        }
    }
    else {
        return _write(fd, buffer, count);
    }
}
#endif // _WIN32

#define LINENOISE_DEFAULT_HISTORY_MAX_LEN 100
#define LINENOISE_MAX_LINE 4096
static const char* unsupported_term[] = { "dumb", "cons25", "emacs", NULL };
static CompletionCallback completionCallback;

#ifndef _WIN32
static struct termios orig_termios; /* In order to restore at exit.*/
#endif
static bool rawmode = false; /* For atexit() function to check if restore is needed*/
static bool mlmode = false; /* Multi line mode. Default is single line. */
static bool atexit_registered = false; /* Register atexit just 1 time. */
static size_t history_max_len = LINENOISE_DEFAULT_HISTORY_MAX_LEN;
static std::vector<std::string> history;

/* The linenoiseState structure represents the state during line editing.
* We pass this state to functions implementing specific editing
* functionalities. */
struct linenoiseState
{
    int ifd; /* Terminal stdin file descriptor. */
    int ofd; /* Terminal stdout file descriptor. */
    char* buf; /* Edited line buffer. */
    size_t buflen; /* Edited line buffer size. */
    std::string prompt; /* Prompt to display. */
    size_t pos; /* Current cursor position. */
    size_t oldcolpos; /* Previous refresh cursor column position. */
    size_t len; /* Current edited line length. */
    size_t cols; /* Number of columns in terminal. */
    size_t maxrows; /* Maximum num of rows used so far (multiline mode) */
    int history_index; /* The history index we are currently editing. */
};

enum KEY_ACTION
{
    KEY_NULL = 0, /* NULL */
    CTRL_A = 1, /* Ctrl+a */
    CTRL_B = 2, /* Ctrl-b */
    CTRL_C = 3, /* Ctrl-c */
    CTRL_D = 4, /* Ctrl-d */
    CTRL_E = 5, /* Ctrl-e */
    CTRL_F = 6, /* Ctrl-f */
    CTRL_H = 8, /* Ctrl-h */
    TAB = 9, /* Tab */
    CTRL_K = 11, /* Ctrl+k */
    CTRL_L = 12, /* Ctrl+l */
    ENTER = 13, /* Enter */
    CTRL_N = 14, /* Ctrl-n */
    CTRL_P = 16, /* Ctrl-p */
    CTRL_T = 20, /* Ctrl-t */
    CTRL_U = 21, /* Ctrl+u */
    CTRL_W = 23, /* Ctrl+w */
    ESC = 27, /* Escape */
    BACKSPACE = 127 /* Backspace */
};

void linenoiseAtExit(void);
bool AddHistory(const char* line);
void refreshLine(struct linenoiseState* l);

/* ============================ UTF8 utilities ============================== */

static unsigned long unicodeWideCharTable[][2] = {
    { 0x1100, 0x115F },
    { 0x2329, 0x232A },
    {
        0x2E80,
        0x2E99,
    },
    {
        0x2E9B,
        0x2EF3,
    },
    {
        0x2F00,
        0x2FD5,
    },
    {
        0x2FF0,
        0x2FFB,
    },
    {
        0x3000,
        0x303E,
    },
    {
        0x3041,
        0x3096,
    },
    {
        0x3099,
        0x30FF,
    },
    {
        0x3105,
        0x312D,
    },
    {
        0x3131,
        0x318E,
    },
    {
        0x3190,
        0x31BA,
    },
    {
        0x31C0,
        0x31E3,
    },
    {
        0x31F0,
        0x321E,
    },
    {
        0x3220,
        0x3247,
    },
    {
        0x3250,
        0x4DBF,
    },
    {
        0x4E00,
        0xA48C,
    },
    {
        0xA490,
        0xA4C6,
    },
    {
        0xA960,
        0xA97C,
    },
    {
        0xAC00,
        0xD7A3,
    },
    {
        0xF900,
        0xFAFF,
    },
    {
        0xFE10,
        0xFE19,
    },
    {
        0xFE30,
        0xFE52,
    },
    {
        0xFE54,
        0xFE66,
    },
    {
        0xFE68,
        0xFE6B,
    },
    {
        0xFF01,
        0xFFE6,
    },
    {
        0x1B000,
        0x1B001,
    },
    {
        0x1F200,
        0x1F202,
    },
    {
        0x1F210,
        0x1F23A,
    },
    {
        0x1F240,
        0x1F248,
    },
    {
        0x1F250,
        0x1F251,
    },
    {
        0x20000,
        0x3FFFD,
    },
};

static size_t unicodeWideCharTableSize = sizeof(unicodeWideCharTable) / sizeof(unicodeWideCharTable[0]);

static int unicodeIsWideChar(unsigned long cp)
{
    size_t i;
    for (i = 0; i < unicodeWideCharTableSize; i++) {
        if (unicodeWideCharTable[i][0] <= cp && cp <= unicodeWideCharTable[i][1]) {
            return 1;
        }
    }
    return 0;
}

static unsigned long unicodeCombiningCharTable[] = {
    0x0300,
    0x0301,
    0x0302,
    0x0303,
    0x0304,
    0x0305,
    0x0306,
    0x0307,
    0x0308,
    0x0309,
    0x030A,
    0x030B,
    0x030C,
    0x030D,
    0x030E,
    0x030F,
    0x0310,
    0x0311,
    0x0312,
    0x0313,
    0x0314,
    0x0315,
    0x0316,
    0x0317,
    0x0318,
    0x0319,
    0x031A,
    0x031B,
    0x031C,
    0x031D,
    0x031E,
    0x031F,
    0x0320,
    0x0321,
    0x0322,
    0x0323,
    0x0324,
    0x0325,
    0x0326,
    0x0327,
    0x0328,
    0x0329,
    0x032A,
    0x032B,
    0x032C,
    0x032D,
    0x032E,
    0x032F,
    0x0330,
    0x0331,
    0x0332,
    0x0333,
    0x0334,
    0x0335,
    0x0336,
    0x0337,
    0x0338,
    0x0339,
    0x033A,
    0x033B,
    0x033C,
    0x033D,
    0x033E,
    0x033F,
    0x0340,
    0x0341,
    0x0342,
    0x0343,
    0x0344,
    0x0345,
    0x0346,
    0x0347,
    0x0348,
    0x0349,
    0x034A,
    0x034B,
    0x034C,
    0x034D,
    0x034E,
    0x034F,
    0x0350,
    0x0351,
    0x0352,
    0x0353,
    0x0354,
    0x0355,
    0x0356,
    0x0357,
    0x0358,
    0x0359,
    0x035A,
    0x035B,
    0x035C,
    0x035D,
    0x035E,
    0x035F,
    0x0360,
    0x0361,
    0x0362,
    0x0363,
    0x0364,
    0x0365,
    0x0366,
    0x0367,
    0x0368,
    0x0369,
    0x036A,
    0x036B,
    0x036C,
    0x036D,
    0x036E,
    0x036F,
    0x0483,
    0x0484,
    0x0485,
    0x0486,
    0x0487,
    0x0591,
    0x0592,
    0x0593,
    0x0594,
    0x0595,
    0x0596,
    0x0597,
    0x0598,
    0x0599,
    0x059A,
    0x059B,
    0x059C,
    0x059D,
    0x059E,
    0x059F,
    0x05A0,
    0x05A1,
    0x05A2,
    0x05A3,
    0x05A4,
    0x05A5,
    0x05A6,
    0x05A7,
    0x05A8,
    0x05A9,
    0x05AA,
    0x05AB,
    0x05AC,
    0x05AD,
    0x05AE,
    0x05AF,
    0x05B0,
    0x05B1,
    0x05B2,
    0x05B3,
    0x05B4,
    0x05B5,
    0x05B6,
    0x05B7,
    0x05B8,
    0x05B9,
    0x05BA,
    0x05BB,
    0x05BC,
    0x05BD,
    0x05BF,
    0x05C1,
    0x05C2,
    0x05C4,
    0x05C5,
    0x05C7,
    0x0610,
    0x0611,
    0x0612,
    0x0613,
    0x0614,
    0x0615,
    0x0616,
    0x0617,
    0x0618,
    0x0619,
    0x061A,
    0x064B,
    0x064C,
    0x064D,
    0x064E,
    0x064F,
    0x0650,
    0x0651,
    0x0652,
    0x0653,
    0x0654,
    0x0655,
    0x0656,
    0x0657,
    0x0658,
    0x0659,
    0x065A,
    0x065B,
    0x065C,
    0x065D,
    0x065E,
    0x065F,
    0x0670,
    0x06D6,
    0x06D7,
    0x06D8,
    0x06D9,
    0x06DA,
    0x06DB,
    0x06DC,
    0x06DF,
    0x06E0,
    0x06E1,
    0x06E2,
    0x06E3,
    0x06E4,
    0x06E7,
    0x06E8,
    0x06EA,
    0x06EB,
    0x06EC,
    0x06ED,
    0x0711,
    0x0730,
    0x0731,
    0x0732,
    0x0733,
    0x0734,
    0x0735,
    0x0736,
    0x0737,
    0x0738,
    0x0739,
    0x073A,
    0x073B,
    0x073C,
    0x073D,
    0x073E,
    0x073F,
    0x0740,
    0x0741,
    0x0742,
    0x0743,
    0x0744,
    0x0745,
    0x0746,
    0x0747,
    0x0748,
    0x0749,
    0x074A,
    0x07A6,
    0x07A7,
    0x07A8,
    0x07A9,
    0x07AA,
    0x07AB,
    0x07AC,
    0x07AD,
    0x07AE,
    0x07AF,
    0x07B0,
    0x07EB,
    0x07EC,
    0x07ED,
    0x07EE,
    0x07EF,
    0x07F0,
    0x07F1,
    0x07F2,
    0x07F3,
    0x0816,
    0x0817,
    0x0818,
    0x0819,
    0x081B,
    0x081C,
    0x081D,
    0x081E,
    0x081F,
    0x0820,
    0x0821,
    0x0822,
    0x0823,
    0x0825,
    0x0826,
    0x0827,
    0x0829,
    0x082A,
    0x082B,
    0x082C,
    0x082D,
    0x0859,
    0x085A,
    0x085B,
    0x08E3,
    0x08E4,
    0x08E5,
    0x08E6,
    0x08E7,
    0x08E8,
    0x08E9,
    0x08EA,
    0x08EB,
    0x08EC,
    0x08ED,
    0x08EE,
    0x08EF,
    0x08F0,
    0x08F1,
    0x08F2,
    0x08F3,
    0x08F4,
    0x08F5,
    0x08F6,
    0x08F7,
    0x08F8,
    0x08F9,
    0x08FA,
    0x08FB,
    0x08FC,
    0x08FD,
    0x08FE,
    0x08FF,
    0x0900,
    0x0901,
    0x0902,
    0x093A,
    0x093C,
    0x0941,
    0x0942,
    0x0943,
    0x0944,
    0x0945,
    0x0946,
    0x0947,
    0x0948,
    0x094D,
    0x0951,
    0x0952,
    0x0953,
    0x0954,
    0x0955,
    0x0956,
    0x0957,
    0x0962,
    0x0963,
    0x0981,
    0x09BC,
    0x09C1,
    0x09C2,
    0x09C3,
    0x09C4,
    0x09CD,
    0x09E2,
    0x09E3,
    0x0A01,
    0x0A02,
    0x0A3C,
    0x0A41,
    0x0A42,
    0x0A47,
    0x0A48,
    0x0A4B,
    0x0A4C,
    0x0A4D,
    0x0A51,
    0x0A70,
    0x0A71,
    0x0A75,
    0x0A81,
    0x0A82,
    0x0ABC,
    0x0AC1,
    0x0AC2,
    0x0AC3,
    0x0AC4,
    0x0AC5,
    0x0AC7,
    0x0AC8,
    0x0ACD,
    0x0AE2,
    0x0AE3,
    0x0B01,
    0x0B3C,
    0x0B3F,
    0x0B41,
    0x0B42,
    0x0B43,
    0x0B44,
    0x0B4D,
    0x0B56,
    0x0B62,
    0x0B63,
    0x0B82,
    0x0BC0,
    0x0BCD,
    0x0C00,
    0x0C3E,
    0x0C3F,
    0x0C40,
    0x0C46,
    0x0C47,
    0x0C48,
    0x0C4A,
    0x0C4B,
    0x0C4C,
    0x0C4D,
    0x0C55,
    0x0C56,
    0x0C62,
    0x0C63,
    0x0C81,
    0x0CBC,
    0x0CBF,
    0x0CC6,
    0x0CCC,
    0x0CCD,
    0x0CE2,
    0x0CE3,
    0x0D01,
    0x0D41,
    0x0D42,
    0x0D43,
    0x0D44,
    0x0D4D,
    0x0D62,
    0x0D63,
    0x0DCA,
    0x0DD2,
    0x0DD3,
    0x0DD4,
    0x0DD6,
    0x0E31,
    0x0E34,
    0x0E35,
    0x0E36,
    0x0E37,
    0x0E38,
    0x0E39,
    0x0E3A,
    0x0E47,
    0x0E48,
    0x0E49,
    0x0E4A,
    0x0E4B,
    0x0E4C,
    0x0E4D,
    0x0E4E,
    0x0EB1,
    0x0EB4,
    0x0EB5,
    0x0EB6,
    0x0EB7,
    0x0EB8,
    0x0EB9,
    0x0EBB,
    0x0EBC,
    0x0EC8,
    0x0EC9,
    0x0ECA,
    0x0ECB,
    0x0ECC,
    0x0ECD,
    0x0F18,
    0x0F19,
    0x0F35,
    0x0F37,
    0x0F39,
    0x0F71,
    0x0F72,
    0x0F73,
    0x0F74,
    0x0F75,
    0x0F76,
    0x0F77,
    0x0F78,
    0x0F79,
    0x0F7A,
    0x0F7B,
    0x0F7C,
    0x0F7D,
    0x0F7E,
    0x0F80,
    0x0F81,
    0x0F82,
    0x0F83,
    0x0F84,
    0x0F86,
    0x0F87,
    0x0F8D,
    0x0F8E,
    0x0F8F,
    0x0F90,
    0x0F91,
    0x0F92,
    0x0F93,
    0x0F94,
    0x0F95,
    0x0F96,
    0x0F97,
    0x0F99,
    0x0F9A,
    0x0F9B,
    0x0F9C,
    0x0F9D,
    0x0F9E,
    0x0F9F,
    0x0FA0,
    0x0FA1,
    0x0FA2,
    0x0FA3,
    0x0FA4,
    0x0FA5,
    0x0FA6,
    0x0FA7,
    0x0FA8,
    0x0FA9,
    0x0FAA,
    0x0FAB,
    0x0FAC,
    0x0FAD,
    0x0FAE,
    0x0FAF,
    0x0FB0,
    0x0FB1,
    0x0FB2,
    0x0FB3,
    0x0FB4,
    0x0FB5,
    0x0FB6,
    0x0FB7,
    0x0FB8,
    0x0FB9,
    0x0FBA,
    0x0FBB,
    0x0FBC,
    0x0FC6,
    0x102D,
    0x102E,
    0x102F,
    0x1030,
    0x1032,
    0x1033,
    0x1034,
    0x1035,
    0x1036,
    0x1037,
    0x1039,
    0x103A,
    0x103D,
    0x103E,
    0x1058,
    0x1059,
    0x105E,
    0x105F,
    0x1060,
    0x1071,
    0x1072,
    0x1073,
    0x1074,
    0x1082,
    0x1085,
    0x1086,
    0x108D,
    0x109D,
    0x135D,
    0x135E,
    0x135F,
    0x1712,
    0x1713,
    0x1714,
    0x1732,
    0x1733,
    0x1734,
    0x1752,
    0x1753,
    0x1772,
    0x1773,
    0x17B4,
    0x17B5,
    0x17B7,
    0x17B8,
    0x17B9,
    0x17BA,
    0x17BB,
    0x17BC,
    0x17BD,
    0x17C6,
    0x17C9,
    0x17CA,
    0x17CB,
    0x17CC,
    0x17CD,
    0x17CE,
    0x17CF,
    0x17D0,
    0x17D1,
    0x17D2,
    0x17D3,
    0x17DD,
    0x180B,
    0x180C,
    0x180D,
    0x18A9,
    0x1920,
    0x1921,
    0x1922,
    0x1927,
    0x1928,
    0x1932,
    0x1939,
    0x193A,
    0x193B,
    0x1A17,
    0x1A18,
    0x1A1B,
    0x1A56,
    0x1A58,
    0x1A59,
    0x1A5A,
    0x1A5B,
    0x1A5C,
    0x1A5D,
    0x1A5E,
    0x1A60,
    0x1A62,
    0x1A65,
    0x1A66,
    0x1A67,
    0x1A68,
    0x1A69,
    0x1A6A,
    0x1A6B,
    0x1A6C,
    0x1A73,
    0x1A74,
    0x1A75,
    0x1A76,
    0x1A77,
    0x1A78,
    0x1A79,
    0x1A7A,
    0x1A7B,
    0x1A7C,
    0x1A7F,
    0x1AB0,
    0x1AB1,
    0x1AB2,
    0x1AB3,
    0x1AB4,
    0x1AB5,
    0x1AB6,
    0x1AB7,
    0x1AB8,
    0x1AB9,
    0x1ABA,
    0x1ABB,
    0x1ABC,
    0x1ABD,
    0x1B00,
    0x1B01,
    0x1B02,
    0x1B03,
    0x1B34,
    0x1B36,
    0x1B37,
    0x1B38,
    0x1B39,
    0x1B3A,
    0x1B3C,
    0x1B42,
    0x1B6B,
    0x1B6C,
    0x1B6D,
    0x1B6E,
    0x1B6F,
    0x1B70,
    0x1B71,
    0x1B72,
    0x1B73,
    0x1B80,
    0x1B81,
    0x1BA2,
    0x1BA3,
    0x1BA4,
    0x1BA5,
    0x1BA8,
    0x1BA9,
    0x1BAB,
    0x1BAC,
    0x1BAD,
    0x1BE6,
    0x1BE8,
    0x1BE9,
    0x1BED,
    0x1BEF,
    0x1BF0,
    0x1BF1,
    0x1C2C,
    0x1C2D,
    0x1C2E,
    0x1C2F,
    0x1C30,
    0x1C31,
    0x1C32,
    0x1C33,
    0x1C36,
    0x1C37,
    0x1CD0,
    0x1CD1,
    0x1CD2,
    0x1CD4,
    0x1CD5,
    0x1CD6,
    0x1CD7,
    0x1CD8,
    0x1CD9,
    0x1CDA,
    0x1CDB,
    0x1CDC,
    0x1CDD,
    0x1CDE,
    0x1CDF,
    0x1CE0,
    0x1CE2,
    0x1CE3,
    0x1CE4,
    0x1CE5,
    0x1CE6,
    0x1CE7,
    0x1CE8,
    0x1CED,
    0x1CF4,
    0x1CF8,
    0x1CF9,
    0x1DC0,
    0x1DC1,
    0x1DC2,
    0x1DC3,
    0x1DC4,
    0x1DC5,
    0x1DC6,
    0x1DC7,
    0x1DC8,
    0x1DC9,
    0x1DCA,
    0x1DCB,
    0x1DCC,
    0x1DCD,
    0x1DCE,
    0x1DCF,
    0x1DD0,
    0x1DD1,
    0x1DD2,
    0x1DD3,
    0x1DD4,
    0x1DD5,
    0x1DD6,
    0x1DD7,
    0x1DD8,
    0x1DD9,
    0x1DDA,
    0x1DDB,
    0x1DDC,
    0x1DDD,
    0x1DDE,
    0x1DDF,
    0x1DE0,
    0x1DE1,
    0x1DE2,
    0x1DE3,
    0x1DE4,
    0x1DE5,
    0x1DE6,
    0x1DE7,
    0x1DE8,
    0x1DE9,
    0x1DEA,
    0x1DEB,
    0x1DEC,
    0x1DED,
    0x1DEE,
    0x1DEF,
    0x1DF0,
    0x1DF1,
    0x1DF2,
    0x1DF3,
    0x1DF4,
    0x1DF5,
    0x1DFC,
    0x1DFD,
    0x1DFE,
    0x1DFF,
    0x20D0,
    0x20D1,
    0x20D2,
    0x20D3,
    0x20D4,
    0x20D5,
    0x20D6,
    0x20D7,
    0x20D8,
    0x20D9,
    0x20DA,
    0x20DB,
    0x20DC,
    0x20E1,
    0x20E5,
    0x20E6,
    0x20E7,
    0x20E8,
    0x20E9,
    0x20EA,
    0x20EB,
    0x20EC,
    0x20ED,
    0x20EE,
    0x20EF,
    0x20F0,
    0x2CEF,
    0x2CF0,
    0x2CF1,
    0x2D7F,
    0x2DE0,
    0x2DE1,
    0x2DE2,
    0x2DE3,
    0x2DE4,
    0x2DE5,
    0x2DE6,
    0x2DE7,
    0x2DE8,
    0x2DE9,
    0x2DEA,
    0x2DEB,
    0x2DEC,
    0x2DED,
    0x2DEE,
    0x2DEF,
    0x2DF0,
    0x2DF1,
    0x2DF2,
    0x2DF3,
    0x2DF4,
    0x2DF5,
    0x2DF6,
    0x2DF7,
    0x2DF8,
    0x2DF9,
    0x2DFA,
    0x2DFB,
    0x2DFC,
    0x2DFD,
    0x2DFE,
    0x2DFF,
    0x302A,
    0x302B,
    0x302C,
    0x302D,
    0x3099,
    0x309A,
    0xA66F,
    0xA674,
    0xA675,
    0xA676,
    0xA677,
    0xA678,
    0xA679,
    0xA67A,
    0xA67B,
    0xA67C,
    0xA67D,
    0xA69E,
    0xA69F,
    0xA6F0,
    0xA6F1,
    0xA802,
    0xA806,
    0xA80B,
    0xA825,
    0xA826,
    0xA8C4,
    0xA8E0,
    0xA8E1,
    0xA8E2,
    0xA8E3,
    0xA8E4,
    0xA8E5,
    0xA8E6,
    0xA8E7,
    0xA8E8,
    0xA8E9,
    0xA8EA,
    0xA8EB,
    0xA8EC,
    0xA8ED,
    0xA8EE,
    0xA8EF,
    0xA8F0,
    0xA8F1,
    0xA926,
    0xA927,
    0xA928,
    0xA929,
    0xA92A,
    0xA92B,
    0xA92C,
    0xA92D,
    0xA947,
    0xA948,
    0xA949,
    0xA94A,
    0xA94B,
    0xA94C,
    0xA94D,
    0xA94E,
    0xA94F,
    0xA950,
    0xA951,
    0xA980,
    0xA981,
    0xA982,
    0xA9B3,
    0xA9B6,
    0xA9B7,
    0xA9B8,
    0xA9B9,
    0xA9BC,
    0xA9E5,
    0xAA29,
    0xAA2A,
    0xAA2B,
    0xAA2C,
    0xAA2D,
    0xAA2E,
    0xAA31,
    0xAA32,
    0xAA35,
    0xAA36,
    0xAA43,
    0xAA4C,
    0xAA7C,
    0xAAB0,
    0xAAB2,
    0xAAB3,
    0xAAB4,
    0xAAB7,
    0xAAB8,
    0xAABE,
    0xAABF,
    0xAAC1,
    0xAAEC,
    0xAAED,
    0xAAF6,
    0xABE5,
    0xABE8,
    0xABED,
    0xFB1E,
    0xFE00,
    0xFE01,
    0xFE02,
    0xFE03,
    0xFE04,
    0xFE05,
    0xFE06,
    0xFE07,
    0xFE08,
    0xFE09,
    0xFE0A,
    0xFE0B,
    0xFE0C,
    0xFE0D,
    0xFE0E,
    0xFE0F,
    0xFE20,
    0xFE21,
    0xFE22,
    0xFE23,
    0xFE24,
    0xFE25,
    0xFE26,
    0xFE27,
    0xFE28,
    0xFE29,
    0xFE2A,
    0xFE2B,
    0xFE2C,
    0xFE2D,
    0xFE2E,
    0xFE2F,
    0x101FD,
    0x102E0,
    0x10376,
    0x10377,
    0x10378,
    0x10379,
    0x1037A,
    0x10A01,
    0x10A02,
    0x10A03,
    0x10A05,
    0x10A06,
    0x10A0C,
    0x10A0D,
    0x10A0E,
    0x10A0F,
    0x10A38,
    0x10A39,
    0x10A3A,
    0x10A3F,
    0x10AE5,
    0x10AE6,
    0x11001,
    0x11038,
    0x11039,
    0x1103A,
    0x1103B,
    0x1103C,
    0x1103D,
    0x1103E,
    0x1103F,
    0x11040,
    0x11041,
    0x11042,
    0x11043,
    0x11044,
    0x11045,
    0x11046,
    0x1107F,
    0x11080,
    0x11081,
    0x110B3,
    0x110B4,
    0x110B5,
    0x110B6,
    0x110B9,
    0x110BA,
    0x11100,
    0x11101,
    0x11102,
    0x11127,
    0x11128,
    0x11129,
    0x1112A,
    0x1112B,
    0x1112D,
    0x1112E,
    0x1112F,
    0x11130,
    0x11131,
    0x11132,
    0x11133,
    0x11134,
    0x11173,
    0x11180,
    0x11181,
    0x111B6,
    0x111B7,
    0x111B8,
    0x111B9,
    0x111BA,
    0x111BB,
    0x111BC,
    0x111BD,
    0x111BE,
    0x111CA,
    0x111CB,
    0x111CC,
    0x1122F,
    0x11230,
    0x11231,
    0x11234,
    0x11236,
    0x11237,
    0x112DF,
    0x112E3,
    0x112E4,
    0x112E5,
    0x112E6,
    0x112E7,
    0x112E8,
    0x112E9,
    0x112EA,
    0x11300,
    0x11301,
    0x1133C,
    0x11340,
    0x11366,
    0x11367,
    0x11368,
    0x11369,
    0x1136A,
    0x1136B,
    0x1136C,
    0x11370,
    0x11371,
    0x11372,
    0x11373,
    0x11374,
    0x114B3,
    0x114B4,
    0x114B5,
    0x114B6,
    0x114B7,
    0x114B8,
    0x114BA,
    0x114BF,
    0x114C0,
    0x114C2,
    0x114C3,
    0x115B2,
    0x115B3,
    0x115B4,
    0x115B5,
    0x115BC,
    0x115BD,
    0x115BF,
    0x115C0,
    0x115DC,
    0x115DD,
    0x11633,
    0x11634,
    0x11635,
    0x11636,
    0x11637,
    0x11638,
    0x11639,
    0x1163A,
    0x1163D,
    0x1163F,
    0x11640,
    0x116AB,
    0x116AD,
    0x116B0,
    0x116B1,
    0x116B2,
    0x116B3,
    0x116B4,
    0x116B5,
    0x116B7,
    0x1171D,
    0x1171E,
    0x1171F,
    0x11722,
    0x11723,
    0x11724,
    0x11725,
    0x11727,
    0x11728,
    0x11729,
    0x1172A,
    0x1172B,
    0x16AF0,
    0x16AF1,
    0x16AF2,
    0x16AF3,
    0x16AF4,
    0x16B30,
    0x16B31,
    0x16B32,
    0x16B33,
    0x16B34,
    0x16B35,
    0x16B36,
    0x16F8F,
    0x16F90,
    0x16F91,
    0x16F92,
    0x1BC9D,
    0x1BC9E,
    0x1D167,
    0x1D168,
    0x1D169,
    0x1D17B,
    0x1D17C,
    0x1D17D,
    0x1D17E,
    0x1D17F,
    0x1D180,
    0x1D181,
    0x1D182,
    0x1D185,
    0x1D186,
    0x1D187,
    0x1D188,
    0x1D189,
    0x1D18A,
    0x1D18B,
    0x1D1AA,
    0x1D1AB,
    0x1D1AC,
    0x1D1AD,
    0x1D242,
    0x1D243,
    0x1D244,
    0x1DA00,
    0x1DA01,
    0x1DA02,
    0x1DA03,
    0x1DA04,
    0x1DA05,
    0x1DA06,
    0x1DA07,
    0x1DA08,
    0x1DA09,
    0x1DA0A,
    0x1DA0B,
    0x1DA0C,
    0x1DA0D,
    0x1DA0E,
    0x1DA0F,
    0x1DA10,
    0x1DA11,
    0x1DA12,
    0x1DA13,
    0x1DA14,
    0x1DA15,
    0x1DA16,
    0x1DA17,
    0x1DA18,
    0x1DA19,
    0x1DA1A,
    0x1DA1B,
    0x1DA1C,
    0x1DA1D,
    0x1DA1E,
    0x1DA1F,
    0x1DA20,
    0x1DA21,
    0x1DA22,
    0x1DA23,
    0x1DA24,
    0x1DA25,
    0x1DA26,
    0x1DA27,
    0x1DA28,
    0x1DA29,
    0x1DA2A,
    0x1DA2B,
    0x1DA2C,
    0x1DA2D,
    0x1DA2E,
    0x1DA2F,
    0x1DA30,
    0x1DA31,
    0x1DA32,
    0x1DA33,
    0x1DA34,
    0x1DA35,
    0x1DA36,
    0x1DA3B,
    0x1DA3C,
    0x1DA3D,
    0x1DA3E,
    0x1DA3F,
    0x1DA40,
    0x1DA41,
    0x1DA42,
    0x1DA43,
    0x1DA44,
    0x1DA45,
    0x1DA46,
    0x1DA47,
    0x1DA48,
    0x1DA49,
    0x1DA4A,
    0x1DA4B,
    0x1DA4C,
    0x1DA4D,
    0x1DA4E,
    0x1DA4F,
    0x1DA50,
    0x1DA51,
    0x1DA52,
    0x1DA53,
    0x1DA54,
    0x1DA55,
    0x1DA56,
    0x1DA57,
    0x1DA58,
    0x1DA59,
    0x1DA5A,
    0x1DA5B,
    0x1DA5C,
    0x1DA5D,
    0x1DA5E,
    0x1DA5F,
    0x1DA60,
    0x1DA61,
    0x1DA62,
    0x1DA63,
    0x1DA64,
    0x1DA65,
    0x1DA66,
    0x1DA67,
    0x1DA68,
    0x1DA69,
    0x1DA6A,
    0x1DA6B,
    0x1DA6C,
    0x1DA75,
    0x1DA84,
    0x1DA9B,
    0x1DA9C,
    0x1DA9D,
    0x1DA9E,
    0x1DA9F,
    0x1DAA1,
    0x1DAA2,
    0x1DAA3,
    0x1DAA4,
    0x1DAA5,
    0x1DAA6,
    0x1DAA7,
    0x1DAA8,
    0x1DAA9,
    0x1DAAA,
    0x1DAAB,
    0x1DAAC,
    0x1DAAD,
    0x1DAAE,
    0x1DAAF,
    0x1E8D0,
    0x1E8D1,
    0x1E8D2,
    0x1E8D3,
    0x1E8D4,
    0x1E8D5,
    0x1E8D6,
    0xE0100,
    0xE0101,
    0xE0102,
    0xE0103,
    0xE0104,
    0xE0105,
    0xE0106,
    0xE0107,
    0xE0108,
    0xE0109,
    0xE010A,
    0xE010B,
    0xE010C,
    0xE010D,
    0xE010E,
    0xE010F,
    0xE0110,
    0xE0111,
    0xE0112,
    0xE0113,
    0xE0114,
    0xE0115,
    0xE0116,
    0xE0117,
    0xE0118,
    0xE0119,
    0xE011A,
    0xE011B,
    0xE011C,
    0xE011D,
    0xE011E,
    0xE011F,
    0xE0120,
    0xE0121,
    0xE0122,
    0xE0123,
    0xE0124,
    0xE0125,
    0xE0126,
    0xE0127,
    0xE0128,
    0xE0129,
    0xE012A,
    0xE012B,
    0xE012C,
    0xE012D,
    0xE012E,
    0xE012F,
    0xE0130,
    0xE0131,
    0xE0132,
    0xE0133,
    0xE0134,
    0xE0135,
    0xE0136,
    0xE0137,
    0xE0138,
    0xE0139,
    0xE013A,
    0xE013B,
    0xE013C,
    0xE013D,
    0xE013E,
    0xE013F,
    0xE0140,
    0xE0141,
    0xE0142,
    0xE0143,
    0xE0144,
    0xE0145,
    0xE0146,
    0xE0147,
    0xE0148,
    0xE0149,
    0xE014A,
    0xE014B,
    0xE014C,
    0xE014D,
    0xE014E,
    0xE014F,
    0xE0150,
    0xE0151,
    0xE0152,
    0xE0153,
    0xE0154,
    0xE0155,
    0xE0156,
    0xE0157,
    0xE0158,
    0xE0159,
    0xE015A,
    0xE015B,
    0xE015C,
    0xE015D,
    0xE015E,
    0xE015F,
    0xE0160,
    0xE0161,
    0xE0162,
    0xE0163,
    0xE0164,
    0xE0165,
    0xE0166,
    0xE0167,
    0xE0168,
    0xE0169,
    0xE016A,
    0xE016B,
    0xE016C,
    0xE016D,
    0xE016E,
    0xE016F,
    0xE0170,
    0xE0171,
    0xE0172,
    0xE0173,
    0xE0174,
    0xE0175,
    0xE0176,
    0xE0177,
    0xE0178,
    0xE0179,
    0xE017A,
    0xE017B,
    0xE017C,
    0xE017D,
    0xE017E,
    0xE017F,
    0xE0180,
    0xE0181,
    0xE0182,
    0xE0183,
    0xE0184,
    0xE0185,
    0xE0186,
    0xE0187,
    0xE0188,
    0xE0189,
    0xE018A,
    0xE018B,
    0xE018C,
    0xE018D,
    0xE018E,
    0xE018F,
    0xE0190,
    0xE0191,
    0xE0192,
    0xE0193,
    0xE0194,
    0xE0195,
    0xE0196,
    0xE0197,
    0xE0198,
    0xE0199,
    0xE019A,
    0xE019B,
    0xE019C,
    0xE019D,
    0xE019E,
    0xE019F,
    0xE01A0,
    0xE01A1,
    0xE01A2,
    0xE01A3,
    0xE01A4,
    0xE01A5,
    0xE01A6,
    0xE01A7,
    0xE01A8,
    0xE01A9,
    0xE01AA,
    0xE01AB,
    0xE01AC,
    0xE01AD,
    0xE01AE,
    0xE01AF,
    0xE01B0,
    0xE01B1,
    0xE01B2,
    0xE01B3,
    0xE01B4,
    0xE01B5,
    0xE01B6,
    0xE01B7,
    0xE01B8,
    0xE01B9,
    0xE01BA,
    0xE01BB,
    0xE01BC,
    0xE01BD,
    0xE01BE,
    0xE01BF,
    0xE01C0,
    0xE01C1,
    0xE01C2,
    0xE01C3,
    0xE01C4,
    0xE01C5,
    0xE01C6,
    0xE01C7,
    0xE01C8,
    0xE01C9,
    0xE01CA,
    0xE01CB,
    0xE01CC,
    0xE01CD,
    0xE01CE,
    0xE01CF,
    0xE01D0,
    0xE01D1,
    0xE01D2,
    0xE01D3,
    0xE01D4,
    0xE01D5,
    0xE01D6,
    0xE01D7,
    0xE01D8,
    0xE01D9,
    0xE01DA,
    0xE01DB,
    0xE01DC,
    0xE01DD,
    0xE01DE,
    0xE01DF,
    0xE01E0,
    0xE01E1,
    0xE01E2,
    0xE01E3,
    0xE01E4,
    0xE01E5,
    0xE01E6,
    0xE01E7,
    0xE01E8,
    0xE01E9,
    0xE01EA,
    0xE01EB,
    0xE01EC,
    0xE01ED,
    0xE01EE,
    0xE01EF,
};

static unsigned long unicodeCombiningCharTableSize = sizeof(unicodeCombiningCharTable) / sizeof(unicodeCombiningCharTable[0]);

inline int unicodeIsCombiningChar(unsigned long cp)
{
    size_t i;
    for (i = 0; i < unicodeCombiningCharTableSize; i++) {
        if (unicodeCombiningCharTable[i] == cp) {
            return 1;
        }
    }
    return 0;
}

/* Get length of previous UTF8 character
*/
inline size_t unicodePrevUTF8CharLen(char* buf, int pos)
{
    int end = pos--;
    while (pos >= 0 && ((unsigned char)buf[pos] & 0xC0) == 0x80) {
        pos--;
    }
    return end - pos;
}

/* Get length of previous UTF8 character
*/
inline size_t unicodeUTF8CharLen(char* buf, size_t buf_len, size_t pos)
{
    if (pos == buf_len) {
        return 0;
    }
    unsigned char ch = buf[pos];
    if (ch < 0x80) {
        return 1;
    }
    else if (ch < 0xE0) {
        return 2;
    }
    else if (ch < 0xF0) {
        return 3;
    }
    else {
        return 4;
    }
}

/* Convert UTF8 to Unicode code point
*/
inline size_t unicodeUTF8CharToCodePoint(
    const char* buf,
    size_t len,
    int* cp)
{
    if (len) {
        unsigned char byte = buf[0];
        if ((byte & 0x80) == 0) {
            *cp = byte;
            return 1;
        }
        else if ((byte & 0xE0) == 0xC0) {
            if (len >= 2) {
                *cp = (((unsigned long)(buf[0] & 0x1F)) << 6) |
                      ((unsigned long)(buf[1] & 0x3F));
                return 2;
            }
        }
        else if ((byte & 0xF0) == 0xE0) {
            if (len >= 3) {
                *cp = (((unsigned long)(buf[0] & 0x0F)) << 12) |
                      (((unsigned long)(buf[1] & 0x3F)) << 6) |
                      ((unsigned long)(buf[2] & 0x3F));
                return 3;
            }
        }
        else if ((byte & 0xF8) == 0xF0) {
            if (len >= 4) {
                *cp = (((unsigned long)(buf[0] & 0x07)) << 18) |
                      (((unsigned long)(buf[1] & 0x3F)) << 12) |
                      (((unsigned long)(buf[2] & 0x3F)) << 6) |
                      ((unsigned long)(buf[3] & 0x3F));
                return 4;
            }
        }
    }
    return 0;
}

/* Get length of grapheme
*/
inline size_t unicodeGraphemeLen(char* buf, size_t buf_len, size_t pos)
{
    if (pos == buf_len) {
        return 0;
    }
    size_t beg = pos;
    pos += unicodeUTF8CharLen(buf, buf_len, pos);
    while (pos < buf_len) {
        size_t len = unicodeUTF8CharLen(buf, buf_len, pos);
        int cp = 0;
        unicodeUTF8CharToCodePoint(buf + pos, len, &cp);
        if (!unicodeIsCombiningChar(cp)) {
            return pos - beg;
        }
        pos += len;
    }
    return pos - beg;
}

/* Get length of previous grapheme
*/
inline size_t unicodePrevGraphemeLen(char* buf, size_t pos)
{
    if (pos == 0) {
        return 0;
    }
    size_t end = pos;
    while (pos > 0) {
        size_t len = unicodePrevUTF8CharLen(buf, pos);
        pos -= len;
        int cp = 0;
        unicodeUTF8CharToCodePoint(buf + pos, len, &cp);
        if (!unicodeIsCombiningChar(cp)) {
            return end - pos;
        }
    }
    return 0;
}

inline int isAnsiEscape(const char* buf, size_t buf_len, size_t* len)
{
    if (buf_len > 2 && !memcmp("\033[", buf, 2)) {
        size_t off = 2;
        while (off < buf_len) {
            switch (buf[off++]) {
            case 'A':
            case 'B':
            case 'C':
            case 'D':
            case 'E':
            case 'F':
            case 'G':
            case 'H':
            case 'J':
            case 'K':
            case 'S':
            case 'T':
            case 'f':
            case 'm':
                *len = off;
                return 1;
            }
        }
    }
    return 0;
}

/* Get column position for the single line mode.
*/
inline size_t unicodeColumnPos(const char* buf, size_t buf_len)
{
    size_t ret = 0;

    size_t off = 0;
    while (off < buf_len) {
        size_t len;
        if (isAnsiEscape(buf + off, buf_len - off, &len)) {
            off += len;
            continue;
        }

        int cp = 0;
        len = unicodeUTF8CharToCodePoint(buf + off, buf_len - off, &cp);

        if (!unicodeIsCombiningChar(cp)) {
            ret += unicodeIsWideChar(cp) ? 2 : 1;
        }

        off += len;
    }

    return ret;
}

/* Get column position for the multi line mode.
*/
inline size_t unicodeColumnPosForMultiLine(char* buf, size_t buf_len, size_t pos, size_t cols, size_t ini_pos)
{
    size_t ret = 0;
    size_t colwid = ini_pos;

    size_t off = 0;
    while (off < buf_len) {
        int cp = 0;
        size_t len = unicodeUTF8CharToCodePoint(buf + off, buf_len - off, &cp);

        size_t wid = 0;
        if (!unicodeIsCombiningChar(cp)) {
            wid = unicodeIsWideChar(cp) ? 2 : 1;
        }

        int dif = (int)(colwid + wid) - (int)cols;
        if (dif > 0) {
            ret += dif;
            colwid = wid;
        }
        else if (dif == 0) {
            colwid = 0;
        }
        else {
            colwid += wid;
        }

        if (off >= pos) {
            break;
        }

        off += len;
        ret += wid;
    }

    return ret;
}

/* Read UTF8 character from file.
*/
inline size_t unicodeReadUTF8Char(int fd, char* buf, int* cp)
{
    size_t nread = read(fd, &buf[0], 1);

    if (nread <= 0) {
        return nread;
    }

    unsigned char byte = buf[0];

    if ((byte & 0x80) == 0) {
        ;
    }
    else if ((byte & 0xE0) == 0xC0) {
        nread = read(fd, &buf[1], 1);
        if (nread <= 0) {
            return nread;
        }
    }
    else if ((byte & 0xF0) == 0xE0) {
        nread = read(fd, &buf[1], 2);
        if (nread <= 0) {
            return nread;
        }
    }
    else if ((byte & 0xF8) == 0xF0) {
        nread = read(fd, &buf[1], 3);
        if (nread <= 0) {
            return nread;
        }
    }
    else {
        return -1;
    }

    return unicodeUTF8CharToCodePoint(buf, 4, cp);
}

/* ======================= Low level terminal handling ====================== */

/* Set if to use or not the multi line mode. */
inline void SetMultiLine(bool ml)
{
    mlmode = ml;
}

/* Return true if the terminal name is in the list of terminals we know are
* not able to understand basic escape sequences. */
inline bool isUnsupportedTerm(void)
{
#ifndef _WIN32
    char* term = getenv("TERM");
    int j;

    if (term == NULL) return false;
    for (j = 0; unsupported_term[j]; j++)
        if (!strcasecmp(term, unsupported_term[j])) return true;
#endif
    return false;
}

/* Raw mode: 1960 magic shit. */
inline bool enableRawMode(int fd)
{
#ifndef _WIN32
    struct termios raw;

    if (!isatty(STDIN_FILENO)) goto fatal;
    if (!atexit_registered) {
        atexit(linenoiseAtExit);
        atexit_registered = true;
    }
    if (tcgetattr(fd, &orig_termios) == -1) goto fatal;

    raw = orig_termios; /* modify the original mode */
    /* input modes: no break, no CR to NL, no parity check, no strip char,
                         * no start/stop output control. */
    raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    /* output modes - disable post processing */
    raw.c_oflag &= ~(OPOST);
    /* control modes - set 8 bit chars */
    raw.c_cflag |= (CS8);
    /* local modes - choing off, canonical off, no extended functions,
    * no signal chars (^Z,^C) */
    raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    /* control chars - set return condition: min number of bytes and timer.
    * We want read to return every single byte, without timeout. */
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0; /* 1 byte, no timer */

    /* put terminal in raw mode after flushing */
    if (tcsetattr(fd, TCSAFLUSH, &raw) < 0) goto fatal;
    rawmode = true;
#else
    if (!atexit_registered) {
        /* Cleanup them at exit */
        atexit(linenoiseAtExit);
        atexit_registered = true;

        /* Init windows console handles only once */
        hOut = GetStdHandle(STD_OUTPUT_HANDLE);
        if (hOut == INVALID_HANDLE_VALUE) goto fatal;
    }

    DWORD consolemodeOut;
    if (!GetConsoleMode(hOut, &consolemodeOut)) {
        CloseHandle(hOut);
        errno = ENOTTY;
        return false;
    };

    hIn = GetStdHandle(STD_INPUT_HANDLE);
    if (hIn == INVALID_HANDLE_VALUE) {
        CloseHandle(hOut);
        errno = ENOTTY;
        return false;
    }

    GetConsoleMode(hIn, &consolemodeIn);
    SetConsoleMode(hIn, ENABLE_PROCESSED_INPUT);

    rawmode = true;
#endif
    return true;

fatal:
    errno = ENOTTY;
    return false;
}

inline void disableRawMode(int fd)
{
#ifdef _WIN32
    if (consolemodeIn) {
        SetConsoleMode(hIn, consolemodeIn);
        consolemodeIn = 0;
    }
    rawmode = false;
#else
    /* Don't even check the return value as it's too late. */
    if (rawmode && tcsetattr(fd, TCSAFLUSH, &orig_termios) != -1)
        rawmode = false;
#endif
}

/* Use the ESC [6n escape sequence to query the horizontal cursor position
* and return it. On error -1 is returned, on success the position of the
* cursor. */
inline int getCursorPosition(int ifd, int ofd)
{
    char buf[32];
    int cols, rows;
    unsigned int i = 0;

    /* Report cursor location */
    if (write(ofd, "\x1b[6n", 4) != 4) return -1;

    /* Read the response: ESC [ rows ; cols R */
    while (i < sizeof(buf) - 1) {
        if (read(ifd, buf + i, 1) != 1) break;
        if (buf[i] == 'R') break;
        i++;
    }
    buf[i] = '\0';

    /* Parse it. */
    if (buf[0] != ESC || buf[1] != '[') return -1;
    if (sscanf(buf + 2, "%d;%d", &rows, &cols) != 2) return -1;
    return cols;
}

/* Try to get the number of columns in the current terminal, or assume 80
* if it fails. */
inline int getColumns(int ifd, int ofd)
{
#ifdef _WIN32
    CONSOLE_SCREEN_BUFFER_INFO b;

    if (!GetConsoleScreenBufferInfo(hOut, &b)) return 80;
    return b.srWindow.Right - b.srWindow.Left;
#else
    struct winsize ws;

    if (ioctl(1, TIOCGWINSZ, &ws) == -1 || ws.ws_col == 0) {
        /* ioctl() failed. Try to query the terminal itself. */
        int start, cols;

        /* Get the initial position so we can restore it later. */
        start = getCursorPosition(ifd, ofd);
        if (start == -1) goto failed;

        /* Go to right margin and get position. */
        if (write(ofd, "\x1b[999C", 6) != 6) goto failed;
        cols = getCursorPosition(ifd, ofd);
        if (cols == -1) goto failed;

        /* Restore position. */
        if (cols > start) {
            char seq[32];
            snprintf(seq, 32, "\x1b[%dD", cols - start);
            if (write(ofd, seq, strlen(seq)) == -1) {
                /* Can't recover... */
            }
        }
        return cols;
    }
    else {
        return ws.ws_col;
    }

failed:
    return 80;
#endif
}

/* Clear the screen. Used to handle ctrl+l */
inline void linenoiseClearScreen(void)
{
    if (write(STDOUT_FILENO, "\x1b[H\x1b[2J", 7) <= 0) {
        /* nothing to do, just to avoid warning. */
    }
}

/* Beep, used for completion when there is nothing to complete or when all
* the choices were already shown. */
inline void linenoiseBeep(void)
{
    fprintf(stderr, "\x7");
    fflush(stderr);
}

/* ============================== Completion ================================ */

/* This is an helper function for linenoiseEdit() and is called when the
* user types the <tab> key in order to complete the string currently in the
* input.
*
* The state of the editing is encapsulated into the pointed linenoiseState
* structure as described in the structure definition. */
inline int completeLine(struct linenoiseState* ls, char* cbuf, int* c)
{
    std::vector<std::string> lc;
    int nread = 0, nwritten;
    *c = 0;

    completionCallback(ls->buf, lc);
    if (lc.empty()) {
        linenoiseBeep();
    }
    else {
        size_t stop = 0, i = 0;

        while (!stop) {
            /* Show completion or original buffer */
            if (i < lc.size()) {
                struct linenoiseState saved = *ls;

                ls->len = ls->pos = lc[i].size();
                ls->buf = &lc[i][0];
                refreshLine(ls);
                ls->len = saved.len;
                ls->pos = saved.pos;
                ls->buf = saved.buf;
            }
            else {
                refreshLine(ls);
            }

//nread = read(ls->ifd,&c,1);
#ifdef _WIN32
            nread = win32read(c);
            if (nread == 1) {
                cbuf[0] = *c;
            }
#else
            nread = unicodeReadUTF8Char(ls->ifd, cbuf, c);
#endif
            if (nread <= 0) {
                *c = -1;
                return nread;
            }

            switch (*c) {
            case 9: /* tab */
                i = (i + 1) % (lc.size() + 1);
                if (i == lc.size()) linenoiseBeep();
                break;
            case 27: /* escape */
                /* Re-show original buffer */
                if (i < lc.size()) refreshLine(ls);
                stop = 1;
                break;
            default:
                /* Update buffer and return */
                if (i < lc.size()) {
                    nwritten = snprintf(ls->buf, ls->buflen, "%s", &lc[i][0]);
                    ls->len = ls->pos = nwritten;
                }
                stop = 1;
                break;
            }
        }
    }

    return nread;
}

/* Register a callback function to be called for tab-completion. */
void SetCompletionCallback(CompletionCallback fn)
{
    completionCallback = fn;
}

/* =========================== Line editing ================================= */

/* Single line low level line refresh.
*
* Rewrite the currently edited line accordingly to the buffer content,
* cursor position, and number of columns of the terminal. */
inline void refreshSingleLine(struct linenoiseState* l)
{
    char seq[64];
    size_t pcolwid = unicodeColumnPos(l->prompt.c_str(), l->prompt.length());
    int fd = l->ofd;
    char* buf = l->buf;
    size_t len = l->len;
    size_t pos = l->pos;
    std::string ab;

    while ((pcolwid + unicodeColumnPos(buf, pos)) >= l->cols) {
        int glen = unicodeGraphemeLen(buf, len, 0);
        buf += glen;
        len -= glen;
        pos -= glen;
    }
    while (pcolwid + unicodeColumnPos(buf, len) > l->cols) {
        len -= unicodePrevGraphemeLen(buf, len);
    }

    /* Cursor to left edge */
    snprintf(seq, 64, "\r");
    ab += seq;
    /* Write the prompt and the current buffer content */
    ab += l->prompt;
    ab.append(buf, len);
    /* Erase to right */
    snprintf(seq, 64, "\x1b[0K");
    ab += seq;
    /* Move cursor to original position. */
    snprintf(seq, 64, "\r\x1b[%dC", (int)(unicodeColumnPos(buf, pos) + pcolwid));
    ab += seq;
    if (write(fd, ab.c_str(), ab.length()) == -1) {
    } /* Can't recover from write error. */
}

/* Multi line low level line refresh.
*
* Rewrite the currently edited line accordingly to the buffer content,
* cursor position, and number of columns of the terminal. */
inline void refreshMultiLine(struct linenoiseState* l)
{
    char seq[64];
    size_t pcolwid = unicodeColumnPos(l->prompt.c_str(), l->prompt.length());
    int colpos = unicodeColumnPosForMultiLine(l->buf, l->len, l->len, l->cols, pcolwid);
    int colpos2; /* cursor column position. */
    int rows = (pcolwid + colpos + l->cols - 1) / l->cols; /* rows used by current buf. */
    int rpos = (pcolwid + l->oldcolpos + l->cols) / l->cols; /* cursor relative row. */
    int rpos2; /* rpos after refresh. */
    int col; /* colum position, zero-based. */
    int old_rows = (int)l->maxrows;
    int fd = l->ofd, j;
    std::string ab;

    /* Update maxrows if needed. */
    if (rows > (int)l->maxrows) l->maxrows = rows;

    /* First step: clear all the lines used before. To do so start by
    * going to the last row. */
    if (old_rows - rpos > 0) {
        snprintf(seq, 64, "\x1b[%dB", old_rows - rpos);
        ab += seq;
    }

    /* Now for every row clear it, go up. */
    for (j = 0; j < old_rows - 1; j++) {
        snprintf(seq, 64, "\r\x1b[0K\x1b[1A");
        ab += seq;
    }

    /* Clean the top line. */
    snprintf(seq, 64, "\r\x1b[0K");
    ab += seq;

    /* Write the prompt and the current buffer content */
    ab += l->prompt;
    ab.append(l->buf, l->len);

    /* Get text width to cursor position */
    colpos2 = unicodeColumnPosForMultiLine(l->buf, l->len, l->pos, l->cols, pcolwid);

    /* If we are at the very end of the screen with our prompt, we need to
    * emit a newline and move the prompt to the first column. */
    if (l->pos &&
        l->pos == l->len &&
        (colpos2 + pcolwid) % l->cols == 0) {
        ab += "\n";
        snprintf(seq, 64, "\r");
        ab += seq;
        rows++;
        if (rows > (int)l->maxrows) l->maxrows = rows;
    }

    /* Move cursor to right position. */
    rpos2 = (pcolwid + colpos2 + l->cols) / l->cols; /* current cursor relative row. */

    /* Go up till we reach the expected position. */
    if (rows - rpos2 > 0) {
        snprintf(seq, 64, "\x1b[%dA", rows - rpos2);
        ab += seq;
    }

    /* Set column. */
    col = (pcolwid + colpos2) % l->cols;
    if (col)
        snprintf(seq, 64, "\r\x1b[%dC", col);
    else
        snprintf(seq, 64, "\r");
    ab += seq;

    l->oldcolpos = colpos2;

    if (write(fd, ab.c_str(), ab.length()) == -1) {
    } /* Can't recover from write error. */
}

/* Calls the two low level functions refreshSingleLine() or
* refreshMultiLine() according to the selected mode. */
inline void refreshLine(struct linenoiseState* l)
{
    if (mlmode)
        refreshMultiLine(l);
    else
        refreshSingleLine(l);
}

/* Insert the character 'c' at cursor current position.
*
* On error writing to the terminal -1 is returned, otherwise 0. */
inline int linenoiseEditInsert(struct linenoiseState* l, const char* cbuf, int clen)
{
    if (l->len < l->buflen) {
        if (l->len == l->pos) {
            memcpy(&l->buf[l->pos], cbuf, clen);
            l->pos += clen;
            l->len += clen;
            ;
            l->buf[l->len] = '\0';
            if ((!mlmode && unicodeColumnPos(l->prompt.c_str(), l->prompt.length()) + unicodeColumnPos(l->buf, l->len) < l->cols) /* || mlmode */) {
                /* Avoid a full update of the line in the
                * trivial case. */
                if (write(l->ofd, cbuf, clen) == -1) return -1;
            }
            else {
                refreshLine(l);
            }
        }
        else {
            memmove(l->buf + l->pos + clen, l->buf + l->pos, l->len - l->pos);
            memcpy(&l->buf[l->pos], cbuf, clen);
            l->pos += clen;
            l->len += clen;
            l->buf[l->len] = '\0';
            refreshLine(l);
        }
    }
    return 0;
}

/* Move cursor on the left. */
inline void linenoiseEditMoveLeft(struct linenoiseState* l)
{
    if (l->pos > 0) {
        l->pos -= unicodePrevGraphemeLen(l->buf, l->pos);
        refreshLine(l);
    }
}

/* Move cursor on the right. */
inline void linenoiseEditMoveRight(struct linenoiseState* l)
{
    if (l->pos != l->len) {
        l->pos += unicodeGraphemeLen(l->buf, l->len, l->pos);
        refreshLine(l);
    }
}

/* Move cursor to the start of the line. */
inline void linenoiseEditMoveHome(struct linenoiseState* l)
{
    if (l->pos != 0) {
        l->pos = 0;
        refreshLine(l);
    }
}

/* Move cursor to the end of the line. */
inline void linenoiseEditMoveEnd(struct linenoiseState* l)
{
    if (l->pos != l->len) {
        l->pos = l->len;
        refreshLine(l);
    }
}

/* Substitute the currently edited line with the next or previous history
* entry as specified by 'dir'. */
#define LINENOISE_HISTORY_NEXT 0
#define LINENOISE_HISTORY_PREV 1
inline void linenoiseEditHistoryNext(struct linenoiseState* l, int dir)
{
    if (history.size() > 1) {
        /* Update the current history entry before to
        * overwrite it with the next one. */
        history[history.size() - 1 - l->history_index] = l->buf;
        /* Show the new entry */
        l->history_index += (dir == LINENOISE_HISTORY_PREV) ? 1 : -1;
        if (l->history_index < 0) {
            l->history_index = 0;
            return;
        }
        else if (l->history_index >= (int)history.size()) {
            l->history_index = history.size() - 1;
            return;
        }
        memset(l->buf, 0, l->buflen);
        strcpy(l->buf, history[history.size() - 1 - l->history_index].c_str());
        l->len = l->pos = strlen(l->buf);
        refreshLine(l);
    }
}

/* Delete the character at the right of the cursor without altering the cursor
* position. Basically this is what happens with the "Delete" keyboard key. */
inline void linenoiseEditDelete(struct linenoiseState* l)
{
    if (l->len > 0 && l->pos < l->len) {
        int glen = unicodeGraphemeLen(l->buf, l->len, l->pos);
        memmove(l->buf + l->pos, l->buf + l->pos + glen, l->len - l->pos - glen);
        l->len -= glen;
        l->buf[l->len] = '\0';
        refreshLine(l);
    }
}

/* Backspace implementation. */
inline void linenoiseEditBackspace(struct linenoiseState* l)
{
    if (l->pos > 0 && l->len > 0) {
        int glen = unicodePrevGraphemeLen(l->buf, l->pos);
        memmove(l->buf + l->pos - glen, l->buf + l->pos, l->len - l->pos);
        l->pos -= glen;
        l->len -= glen;
        l->buf[l->len] = '\0';
        refreshLine(l);
    }
}

/* Delete the previous word, maintaining the cursor at the start of the
* current word. */
inline void linenoiseEditDeletePrevWord(struct linenoiseState* l)
{
    size_t old_pos = l->pos;
    size_t diff;

    while (l->pos > 0 && l->buf[l->pos - 1] == ' ')
        l->pos--;
    while (l->pos > 0 && l->buf[l->pos - 1] != ' ')
        l->pos--;
    diff = old_pos - l->pos;
    memmove(l->buf + l->pos, l->buf + old_pos, l->len - old_pos + 1);
    l->len -= diff;
    refreshLine(l);
}

/* This function is the core of the line editing capability of linenoise.
* It expects 'fd' to be already in "raw mode" so that every key pressed
* will be returned ASAP to read().
*
* The resulting string is put into 'buf' when the user type enter, or
* when ctrl+d is typed.
*
* The function returns the length of the current buffer. */
inline int linenoiseEdit(int stdin_fd, int stdout_fd, char* buf, size_t buflen, const char* prompt)
{
    struct linenoiseState l;

    /* Populate the linenoise state that we pass to functions implementing
    * specific editing functionalities. */
    l.ifd = stdin_fd;
    l.ofd = stdout_fd;
    l.buf = buf;
    l.buflen = buflen;
    l.prompt = prompt;
    l.oldcolpos = l.pos = 0;
    l.len = 0;
    l.cols = getColumns(stdin_fd, stdout_fd);
    l.maxrows = 0;
    l.history_index = 0;

    /* Buffer starts empty. */
    l.buf[0] = '\0';
    l.buflen--; /* Make sure there is always space for the nulterm */

    /* The latest history entry is always our current buffer, that
                * initially is just an empty string. */
    AddHistory("");

    if (write(l.ofd, prompt, l.prompt.length()) == -1) return -1;
    while (1) {
        int c;
        char cbuf[4];
        int nread;
        char seq[3];

#ifdef _WIN32
        nread = win32read(&c);
        if (nread == 1) {
            cbuf[0] = c;
        }
#else
        nread = unicodeReadUTF8Char(l.ifd, cbuf, &c);
#endif
        if (nread <= 0) return (int)l.len;

        /* Only autocomplete when the callback is set. It returns < 0 when
        * there was an error reading from fd. Otherwise it will return the
        * character that should be handled next. */
        if (c == 9 && completionCallback != NULL) {
            nread = completeLine(&l, cbuf, &c);
            /* Return on errors */
            if (c < 0) return l.len;
            /* Read next character when 0 */
            if (c == 0) continue;
        }

        switch (c) {
        case ENTER: /* enter */
            history.pop_back();
            if (mlmode) linenoiseEditMoveEnd(&l);
            return (int)l.len;
        case CTRL_C: /* ctrl-c */
            errno = EAGAIN;
            return -1;
        case BACKSPACE: /* backspace */
        case 8: /* ctrl-h */
            linenoiseEditBackspace(&l);
            break;
        case CTRL_D: /* ctrl-d, remove char at right of cursor, or if the
                         line is empty, act as end-of-file. */
            if (l.len > 0) {
                linenoiseEditDelete(&l);
            }
            else {
                history.pop_back();
                return -1;
            }
            break;
        case CTRL_T: /* ctrl-t, swaps current character with previous. */
            if (l.pos > 0 && l.pos < l.len) {
                int aux = buf[l.pos - 1];
                buf[l.pos - 1] = buf[l.pos];
                buf[l.pos] = aux;
                if (l.pos != l.len - 1) l.pos++;
                refreshLine(&l);
            }
            break;
        case CTRL_B: /* ctrl-b */
            linenoiseEditMoveLeft(&l);
            break;
        case CTRL_F: /* ctrl-f */
            linenoiseEditMoveRight(&l);
            break;
        case CTRL_P: /* ctrl-p */
            linenoiseEditHistoryNext(&l, LINENOISE_HISTORY_PREV);
            break;
        case CTRL_N: /* ctrl-n */
            linenoiseEditHistoryNext(&l, LINENOISE_HISTORY_NEXT);
            break;
        case ESC: /* escape sequence */
            /* Read the next two bytes representing the escape sequence.
                     * Use two calls to handle slow terminals returning the two
                     * chars at different times. */
            if (read(l.ifd, seq, 1) == -1) break;
            if (read(l.ifd, seq + 1, 1) == -1) break;

            /* ESC [ sequences. */
            if (seq[0] == '[') {
                if (seq[1] >= '0' && seq[1] <= '9') {
                    /* Extended escape, read additional byte. */
                    if (read(l.ifd, seq + 2, 1) == -1) break;
                    if (seq[2] == '~') {
                        switch (seq[1]) {
                        case '3': /* Delete key. */
                            linenoiseEditDelete(&l);
                            break;
                        }
                    }
                }
                else {
                    switch (seq[1]) {
                    case 'A': /* Up */
                        linenoiseEditHistoryNext(&l, LINENOISE_HISTORY_PREV);
                        break;
                    case 'B': /* Down */
                        linenoiseEditHistoryNext(&l, LINENOISE_HISTORY_NEXT);
                        break;
                    case 'C': /* Right */
                        linenoiseEditMoveRight(&l);
                        break;
                    case 'D': /* Left */
                        linenoiseEditMoveLeft(&l);
                        break;
                    case 'H': /* Home */
                        linenoiseEditMoveHome(&l);
                        break;
                    case 'F': /* End*/
                        linenoiseEditMoveEnd(&l);
                        break;
                    }
                }
            }

            /* ESC O sequences. */
            else if (seq[0] == 'O') {
                switch (seq[1]) {
                case 'H': /* Home */
                    linenoiseEditMoveHome(&l);
                    break;
                case 'F': /* End*/
                    linenoiseEditMoveEnd(&l);
                    break;
                }
            }
            break;
        default:
            if (linenoiseEditInsert(&l, cbuf, nread)) return -1;
            break;
        case CTRL_U: /* Ctrl+u, delete the whole line. */
            buf[0] = '\0';
            l.pos = l.len = 0;
            refreshLine(&l);
            break;
        case CTRL_K: /* Ctrl+k, delete from current to end of line. */
            buf[l.pos] = '\0';
            l.len = l.pos;
            refreshLine(&l);
            break;
        case CTRL_A: /* Ctrl+a, go to the start of the line */
            linenoiseEditMoveHome(&l);
            break;
        case CTRL_E: /* ctrl+e, go to the end of the line */
            linenoiseEditMoveEnd(&l);
            break;
        case CTRL_L: /* ctrl+l, clear screen */
            linenoiseClearScreen();
            refreshLine(&l);
            break;
        case CTRL_W: /* ctrl+w, delete previous word */
            linenoiseEditDeletePrevWord(&l);
            break;
        }
    }
    return l.len;
}

/* This function calls the line editing function linenoiseEdit() using
* the STDIN file descriptor set in raw mode. */
inline bool linenoiseRaw(const char* prompt, std::string& line)
{
    bool quit = false;

    if (!isatty(STDIN_FILENO)) {
        /* Not a tty: read from file / pipe. */
        std::getline(std::cin, line);
    }
    else {
        /* Interactive editing. */
        if (enableRawMode(STDIN_FILENO) == false) {
            return quit;
        }

        char buf[LINENOISE_MAX_LINE];
        auto count = linenoiseEdit(STDIN_FILENO, STDOUT_FILENO, buf, LINENOISE_MAX_LINE, prompt);
        if (count == -1) {
            quit = true;
        }
        else {
            line.assign(buf, count);
        }

        disableRawMode(STDIN_FILENO);
        printf("\n");
    }
    return quit;
}

/* The high level function that is the main API of the linenoise library.
* This function checks if the terminal has basic capabilities, just checking
* for a blacklist of stupid terminals, and later either calls the line
* editing function or uses dummy fgets() so that you will be able to type
* something even in the most desperate of the conditions. */
inline bool Readline(const char* prompt, std::string& line)
{
    if (isUnsupportedTerm()) {
        printf("%s", prompt);
        fflush(stdout);
        std::getline(std::cin, line);
        return false;
    }
    else {
        return linenoiseRaw(prompt, line);
    }
}

inline std::string Readline(const char* prompt, bool& quit)
{
    std::string line;
    quit = Readline(prompt, line);
    return line;
}

inline std::string Readline(const char* prompt)
{
    bool quit; // dummy
    return Readline(prompt, quit);
}

/* ================================ History ================================= */

/* At exit we'll try to fix the terminal to the initial conditions. */
inline void linenoiseAtExit(void)
{
    disableRawMode(STDIN_FILENO);
}

/* This is the API call to add a new entry in the linenoise history.
* It uses a fixed array of char pointers that are shifted (memmoved)
* when the history max length is reached in order to remove the older
* entry and make room for the new one, so it is not exactly suitable for huge
* histories, but will work well for a few hundred of entries.
*
* Using a circular buffer is smarter, but a bit more complex to handle. */
inline bool AddHistory(const char* line)
{
    if (history_max_len == 0) return false;

    /* Don't add duplicated lines. */
    if (!history.empty() && history.back() == line) return false;

    /* If we reached the max length, remove the older line. */
    if (history.size() == history_max_len) {
        history.erase(history.begin());
    }
    history.push_back(line);

    return true;
}

/* Set the maximum length for the history. This function can be called even
* if there is already some history, the function will make sure to retain
* just the latest 'len' elements if the new history length value is smaller
* than the amount of items already inside the history. */
inline bool SetHistoryMaxLen(size_t len)
{
    if (len < 1) return false;
    history_max_len = len;
    if (len < history.size()) {
        history.resize(len);
    }
    return true;
}

/* Save the history in the specified file. On success *true* is returned
* otherwise *false* is returned. */
inline bool SaveHistory(const char* path)
{
    std::ofstream f(path); // TODO: need 'std::ios::binary'?
    if (!f) return false;
    for (const auto& h : history) {
        f << h << std::endl;
    }
    return true;
}

/* Load the history from the specified file. If the file does not exist
* zero is returned and no operation is performed.
*
* If the file exists and the operation succeeded *true* is returned, otherwise
* on error *false* is returned. */
inline bool LoadHistory(const char* path)
{
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        AddHistory(line.c_str());
    }
    return true;
}

inline const std::vector<std::string>& GetHistory()
{
    return history;
}

} // namespace linenoise

#ifdef _WIN32
#undef isatty
#undef write
#undef read
#endif

#endif /* __LINENOISE_HPP */