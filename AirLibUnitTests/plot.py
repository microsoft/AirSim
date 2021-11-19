#!/usr/bin/env python3
"""
Module Docstring
"""

__author__ = "Suman Subedi"
__version__ = "0.1.0"
__license__ = "MIT"

import argparse
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

def main():
    """ Main entry point of the app """

    print("hello world")
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", help="filename of the txt file")
    args = parser.parse_args()

    print(args.filename)

    with open('log.txt', 'rb') as raw_data:
        data = np.loadtxt(raw_data, delimiter='\t')

    X = np.linspace(0, 2*np.pi, 100)
    Y = np.cos(X)

    fig, ax = plt.subplots()
    ax.plot(X, Y, color='C1')

    fig.savefig("figure.pdf")
    fig.show()


if __name__ == "__main__":
    """ This is executed when run from the command line """
    main()