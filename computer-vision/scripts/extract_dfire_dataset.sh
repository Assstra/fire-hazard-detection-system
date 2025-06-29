#!/bin/bash

echo Extracting the D-Fire dataset...

# check if the data/D-Fire.zip file exists
if [ ! -f "data/D-Fire.zip" ]; then
    echo "D-Fire.zip not found. Please ensure the file is in the current directory
and try again."
    exit 1
fi

echo Done extracting the D-Fire dataset.
