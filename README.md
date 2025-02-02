# Motion++

Motion++ is a simple implementation of a sampling-based algorithm. 
It finds near optimal solutions by decomposing the state space into cells.

### Installation

#### Install dependency

##### Ubuntu
```bash
sudo apt install cmake g++
```

##### Arch Linux
```bash
sudo pacman -S cmake gcc base-devel
```

> [!WARNING]
> Running the examples requires `gnuplot` as a runtime dependency.

#### Build from source

```bash
git clone --recursive https://git.uibk.ac.at/csba1368/motionplanning.git 
cd motionplanning
cmake -S . -B build
cmake --build build
```

If you want to build the examples and tests use the follwing flags:

```bash
cmake -S . build -DMOTIONPP_BUILD_EXAMPLES=ON
```

### Recommended integration via `CMake`

The easiest and recommended way to include `motionpp` in your project is to add it as a subdirectory within your existing CMake-based build system.

Clone the Motionpp repository into your project directory (make sure to replace the URL with the correct one for your repository):

```bash
git clone --recursive https://git.uibk.ac.at/csba1368/motionplanning.git motionpp
```

If you are using a git repository, you can add it as a submodule instead:

```bash
git submodule add https://git.uibk.ac.at/csba1368/motionplanning.git motionpp
git submodule update --init --recursive
```
Include `motionpp` into your `CMake` build.
Below is a sample `CMakeLists.txt` you can copy and adapt to your own setup.

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)

# Add the Motionpp library as a subdirectory
add_subdirectory(motionpp)

# (Optional) Set the C++ standard for your project
set(CMAKE_CXX_STANDARD 20)

# Build your executable
add_executable(${PROJECT_NAME} main.cpp model.cpp)

# Link the Motionpp library to your target
target_link_libraries(${PROJECT_NAME} PUBLIC Motionpp)
```
