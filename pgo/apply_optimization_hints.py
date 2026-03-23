"""
Apply optimization hints from profiling data
"""

import json
import sys

def main():
    try:
        with open('pgo_optimization_hints.json', 'r') as f:
            hints = json.load(f)
    except FileNotFoundError:
        print("❌ Hints file not found")
        return

    print("/* Auto-generated PGO optimization attributes */")
    for func, hint in hints.items():
        if hint['optimization'] == 'inline':
            print(f"__attribute__((always_inline)) /* {func} (HOT PATH) */")
        elif hint['optimization'] == 'optimize_speed':
            print(f"__attribute__((hot)) /* {func} (CRITICAL PATH) */")

if __name__ == "__main__":
    main()
