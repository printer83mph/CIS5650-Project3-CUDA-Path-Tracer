#! /bin/bash

if [ -d "build" ]; then
    echo "build folder already exists, done!"
    exit 0
fi

mkdir build

echo "setting up Ninja Multi-Config build..."
cmake -S. -B./build -G "Ninja Multi-Config" \
    -DCMAKE_CXX_FLAGS_DEBUG="-g -O0" \
    -DCMAKE_CUDA_FLAGS_DEBUG="-g -G -O0" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "setup complete!"
