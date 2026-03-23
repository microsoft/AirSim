#!/bin/bash
# Automated PGO Pipeline for Arkhe Kernel
set -e

echo "========================================"
echo "ARKHE KERNEL PGO OPTIMIZATION PIPELINE"
echo "========================================"

# Step 1: Clean previous builds
echo "[INFO] Cleaning previous builds..."
make -f pgo/Makefile.pgo clean

# Step 2: Build instrumented kernel
echo "[INFO] Building instrumented kernel..."
make -f pgo/Makefile.pgo instrument

# Step 3: Run chaos tests to generate profile
echo "[INFO] Running chaos tests..."
make -f pgo/Makefile.pgo profile

# Step 4: Build optimized kernel
echo "[INFO] Building PGO-optimized kernel..."
make -f pgo/Makefile.pgo optimize

echo "========================================"
echo "PGO OPTIMIZATION PIPELINE COMPLETE"
echo "========================================"
