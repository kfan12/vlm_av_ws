#!/bin/bash
# Rebuild local/kb/source_guide.html from the code: extract every function per node,
# merge descriptions.py, and write the page. Run after adding a node, file or function;
# the build prints any function without a description and any cycle step that no longer
# resolves, so descriptions.py can be updated.
set -euo pipefail
cd "$(dirname "$0")"
python3 extract.py
python3 build.py "$@"
