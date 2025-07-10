#!/bin/bash

echo "Extracting the D-Fire dataset..."

# check if the data/D-Fire.zip file exists
if [ ! -f "data/D-Fire.zip" ]; then
    echo "D-Fire.zip not found. Please ensure the file is in the current directory
and try again."
    exit 1
fi

cd data/
mkdir -p D-Fire/
unzip -qo D-Fire.zip -d D-Fire/
echo "> D-Fire dataset extracted."

cd D-Fire/
mv test/ val/ # renaming test to val

echo "Done! The D-Fire dataset is now ready for use."
