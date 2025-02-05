```bash
sudo apt-get install libsdl2-dev
sudo apt-get install libsdl2-gfx-dev
sudo apt-get install libopencv-dev
```

```bash
rm -rf build
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```