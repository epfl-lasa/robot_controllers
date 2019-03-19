# Robot Controllers
This repo contains "low" and "high" level controllers for robotics control through Dynamical Systems

### Authors/Maintainer

- Bernardo Fichera
- Konstantinos Chatzilygeroudis

### Available Controllers

#### Low-Level controllers
- Passive DS Controller:
  "Passive Interaction Control With Dynamical Systems", *Klas Kronander and Aude Billard*, 2016, https://ieeexplore.ieee.org/document/7358081

#### High-Level controllers
- Simple Linear DS high level controller

### Installing

- `mkdir build && cd buil`
- `cmake -DCMAKE_INSTALL_PREFIX:PATH=/path/to/install/dir/ ..`
- `make && [sudo] make install` (you might need sudo depending on your installation directory)

#### Running the examples

- `./src/examples/linear_test`

The executables should be inside the `build/src/examples` folder.

### Documentation

Work in progress...

Copyright (c) 2019, **Bernardo Fichera**
