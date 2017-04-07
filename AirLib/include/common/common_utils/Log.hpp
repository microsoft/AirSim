// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef common_utils_Log_hpp
#define common_utils_Log_hpp

#include <cstdarg>
#include <stdio.h>

namespace common_utils {

    // This simple logging interface can be used to redirect debug printf statements to your own app's environment.
    class Log
    {
        static Log* log_;
    public:

        static Log* getLog() {
            return log_;
        }

        static void setLog(Log* log) {
            log_ = log;
        }

        virtual void logMessage(const char* message) {
            printf(message);
            printf("\n");
            fflush(stdout);
        }

        virtual void logError(const char* message) {
            fprintf(stderr, message);
            printf("\n");
            fflush(stderr);
        }

    };
}

#endif

