# Control Stack
This repo contains "low" and "high" level controllers for robotics control through Dynamical Systems

### Authors/Maintainer

- Bernardo Fichera
- Konstantinos Chatzilygeroudis

### Available Features

- Passive DS Controller:
  "Passive Interaction Control With Dynamical Systems", *Klas Kronander and Aude Billard*, 2016, https://ieeexplore.ieee.org/document/7358081

### Missing Features

- Planners

#### Installing using the waf build system

- `mkdir build && cd buil`
- `cmake -DCMAKE_INSTALL_PREFIX:PATH=/path/to/install/dir/ ..`
- `make && [sudo] make install` (you might need sudo depending on your installation directory)

#### Running the examples

- `./examples/linear_test`

The executables should be inside the `build/examples` folder.

### Documentation

Work in progress...
