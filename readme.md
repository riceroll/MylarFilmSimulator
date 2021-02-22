
## Mylar Simulator
C++-based simulator for Mylar film, with electrostatic energy, 
edge length energy, bending energy, boosted with openMP.

To run the code, follow the three steps.

### 1. Dependency
```bash
# install openMP
brew install llvm
brew install libomp
brew upgrade libomp
```

### 2. Build
```bash
mkdir build
cd build
cmake ..
make
```

### 3. Run
```bash
./mylar ../data/one_circle.obj
```
