#!/bin/bash

find . -name ".DS_Store" -type f -delete
find . -name "*.uvgui*" -type f -delete
find . -name "*.crf" -type f -delete
find . -name "*.d" -type f -delete
find . -name "*.o" -type f -delete
find . -name "*.lst" -type f -delete
find . -name "*.dep" -type f -delete
find . -name "*.htm" -type f -delete
find . -name "*.lnp" -type f -delete
find . -name "*.map" -type f -delete
find . -name "*.axf" -type f -delete
find . -name "*.uvopt" -type f -delete
find . -name "*.bak" -type f -delete

echo "cleaned"
