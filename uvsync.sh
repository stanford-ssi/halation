#!/bin/bash
set -e

uv export --frozen --no-dev > requirements.txt
uv pip install --system -r requirements.txt
rm -f requirements.txt