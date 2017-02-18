// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef commn_utils_AsyncTasker_hpp
#define commn_utils_AsyncTasker_hpp

#include "ctpl_stl.h"
#include <functional>

class AsyncTasker {
public:
    AsyncTasker(unsigned int thread_count = 4)
        : threads_(thread_count), current_([] {}), error_handler_([](std::exception e) {})
    {
    }

    void setErrorHandler(std::function<void(std::exception&)> errorHandler) {
        error_handler_ = errorHandler;
    }

    void execute(std::function<void()> func, unsigned int iterations = 1)
    {
        // must keep the func alive between now and the thread start, right now it is just
        // on the stack, and so between here and the thread start it goes away and then 
        // the thread tries to operate on random memory.  But this AsyncTasker stays alive
        // so it can keep the func state alive as a member.
        current_ = func;
        if (iterations < 1)
            return;
        
        if (iterations == 1)
        {
            threads_.push([&](int i) {
                try {
                    current_();
                }
                catch (std::exception& e) {
                    error_handler_(e);
                };
            });
        } else {
            threads_.push([&](int i) {
                for (unsigned int itr = 0; itr < iterations; ++itr) {
                    try {
                        current_();
                    }
                    catch (std::exception& e) {
                        error_handler_(e);
                        break;
                    };
                }
            });
        }
    }

private:
    ctpl::thread_pool threads_;
    std::function<void()> current_;
    std::function<void(std::exception&)> error_handler_;
};


#endif
