#!/bin/bash

cp README.md docs/
sed -i 's/](docs\//](/g' docs/README.md
