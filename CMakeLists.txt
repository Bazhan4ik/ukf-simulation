cmake_minimum_required(VERSION 3.10)
project(UKF_Simulation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(ukf_sim data_reader.cpp)

# Link Eigen
target_link_libraries(ukf_sim Eigen3::Eigen)

# If Eigen is not found automatically, you can specify the path manually:
# target_include_directories(ukf_sim PRIVATE /usr/local/include/eigen3)
