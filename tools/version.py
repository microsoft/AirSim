#!/usr/bin/env python3
####################################################################################################
#
#  Project:  Embedded Learning Library (ELL)
#  File:     version.py
#  Authors:  Chris Lovett
#
#  Requires: Python 3.4+
#
####################################################################################################
import argparse
import sys


def check_versions(v1, v2):
    v1parts = [int(x) for x in v1.split('.')]
    v2parts = [int(x) for x in v2.split('.')]
    i = 0
    while i < len(v1parts) and i < len(v2parts):
        if v1parts[i] < v2parts[i]:
            return False
        i += 1

    if i == len(v1parts) and i < len(v2parts):
        return False

    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Version checking, returns error code if version 1 is less than version 2")

    # required arguments
    parser.add_argument("version1", help="first version number to compare")
    parser.add_argument("version2", help="second version number to compare")

    args = parser.parse_args()

    rc = check_versions(args.version1, args.version2)
    if not rc:
        print("version {} insufficient, must be at least {}".format(args.version1, args.version2))
        sys.exit(-1)

    sys.exit(0)
