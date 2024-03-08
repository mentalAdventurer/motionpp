# Motion++

Motion++ is a simple implementation of a sampling-based algorithm. 
It finds near optimal solutions by decomposing the state space into cells.

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
> [!WARNING]  
> Running the examples requires `gnuplot` as a runtime dependency.
