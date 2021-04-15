// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#if defined(linux) | defined(__linux)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#include <stdlib.h>
#include <string.h>
#include "common_utils/Utils.hpp"

int getWifiRssi(int socket, const char* ifaceName)
{
    unused(socket);
    unused(ifaceName);
    return 0;
// This has not been properly tested
//    struct iw_statistics stats;
//    struct iwreq req;
//    memset(&stats, 0, sizeof(stats));
//    memset(&req, 0, sizeof(iwreq));
//
//    strncpy(req.ifr_name, ifaceName, 16);
//    req.u.data.pointer = &stats;
//    req.u.data.length = sizeof(iw_statistics);
//    
//#ifdef CLEAR_UPDATED
//    req.u.data.flags = 1;
//#endif
//
//    /* Perform the ioctl */
//    if (ioctl(socket, SIOCGIWSTATS, &req) == -1) {
//        //printf("Error performing SIOCGIWSTATS on %s\n", ifaceName);
//        return -127;
//    } 
//
//    return stats.qual.level;
}

#else

// todo: windows version of this...
int getWifiRssi(int socket, const char* ifaceName)
{
    return 0;
}

#endif