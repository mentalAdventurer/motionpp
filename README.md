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
git clone --recursive https://github.com/mentalAdventurer/motionpp
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
git clone --recursive https://github.com/mentalAdventurer/motionpp
```

If you are using a git repository, you can add it as a submodule instead:

```bash
git submodule add https://github.com/mentalAdventurer/motionpp
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

### Example Flywhell Pendulum

Below is a minimal example demonstrating how to use Motionpp for a Flywheel Pendulum scenario.
The relevant differential constraints and model parameters are located in the `examples/flywheel` folder.
Additional examples can be found there as well, providing a broader overview of how to utilize Motionpp effectively.

```cpp
#include <tuple>
#include <vector>

#include "cell_based_search.h"
#include "model.h"
#include "helper.h"


int main() {
  // Generate Motion Primitives function with mphelper
  std::vector<std::vector<double>> inputs;
  double step_size = 2*param::ddphi_max/10;
  for (double i = -param::ddphi_max; i <= param::ddphi_max; i += step_size) {
    inputs.push_back({i});
  }
  auto mp = mphelper::Euler(model::system, inputs,0.08);
  auto primitives = mp({0, 0, 0, 0});

  // Define Search Options
  cspace::Options opt(66000,  // Number of Voronoi points
                      {
                          // Region for spreading Voronoi points
                          {0.0, 0.0},  // Position x
                          {0.0, 0.0},   // Speed x
                          {-2*param::pi, 2*param::pi},   // Accel x
                          {-30, 30}
                      });

  opt.target_radius = 0.10;
  opt.distance_metric = [](const std::vector<double>& x1, const std::vector<double>& x2){return std::abs(x1[2]-x2[2])+std::abs(x1[3]-x2[3]);};
  std::vector<double> x0 = {0, 0, 0, 0};
  std::vector<double> xg = {0, 0, param::pi, 0};

  // Start search
  MultiQuerySearch cellBasedSearch(x0.size(), opt, mp);
  auto G = cellBasedSearch(x0, xg);

  auto trajectory = G.get_trajectory(G.back().state);
  auto [input,t] = G.get_input(G.back().state);

  // Terminal Info
  std::cout << "Numer of Primitives: " << primitives.size() << std::endl;
  (G.get_success()) ? std::cout << "Success" << std::endl : std::cout << "Failure" << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;
  std::cout << "Final State: ";
  for (auto& xi : *(G.back().state)) std::cout << xi << " ";
  std::cout << "\nDistance: " << opt.distance_metric(*(G.back().state),xg) << "\n";
  std::cout << std::endl;

  return 0;
}
```
