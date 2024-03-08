# Motion++

### Installation

#### Build from source

```bash
git clone --recursive https://git.uibk.ac.at/csba1368/motionplanning.git 
cd motionplanning
cmake -S . -B build
cmake --build build
```

If you want to build the examples and tests use the follwing flags:

```bash
cmake -S . build -DMOTIONPP_BUILD_TESTS=ON -DMOTIONPP_BUILD_EXAMPLES=ON
```
Running the exmaples requires `gnuplot` as a dependency.
