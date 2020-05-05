#!/bin/bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

cp README.md docs/
sed -i 's/](docs\//](/g' docs/README.md

# Build API docs
pushd PythonClient/docs >/dev/null
make html

popd >/dev/null

cp -r PythonClient/docs/_build docs/api_docs/

mkdocs build

popd >/dev/null
