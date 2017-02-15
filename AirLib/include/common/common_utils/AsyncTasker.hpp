// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef commn_utils_AsyncTasker_hpp
#define commn_utils_AsyncTasker_hpp

#include "ctpl_stl.h"


class AsyncTasker {
public:
    AsyncTasker(unsigned int thread_count = 4)
        : threads_(thread_count)
    {
    }

    template<typename TTaskFunction>
    void execute(TTaskFunction func, unsigned int iterations = 1)
    {
        if (iterations < 1)
            return;
        else if (iterations == 1)
            threads_.push([&](int i) { func(); });
        else {
            threads_.push([&](int i) {
                for (unsigned int itr = 0; itr < iterations; ++itr) {
                    func();
                }
            });
        }
    }

private:
    ctpl::thread_pool threads_;
};


#endif
