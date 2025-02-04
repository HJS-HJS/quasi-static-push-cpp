```bash
sudo apt-get install libsdl2-dev
sudo apt-get install libsdl2-gfx-dev 
```

```bash
rm -rf build
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```