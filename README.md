# Robot Controllers
This repo contains "low" and "high" level controllers for robot control

### Authors/Maintainers

- Bernardo Fichera (bernardo.fichera@epfl.ch)
- Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@epfl.ch)

### Available Controllers

#### Low-Level controllers
- Passive DS Controller:
  "Passive Interaction Control With Dynamical Systems", *Klas Kronander and Aude Billard*, 2016, https://ieeexplore.ieee.org/document/7358081
- PID Controller

#### High-Level controllers
- Simple Linear DS high level controller

### Installing

#### Dependencies

```sh
cd /source/directory
git clone https://github.com/mosra/corrade.git
cd corrade
mkdir build && cd build
cmake ..
make -j
sudo make install
```

#### Compilation

```sh
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/path/to/install/dir/ ..
make && [sudo] make install (you might need sudo depending on your installation directory)
```

### Contributing

**robot_controllers** is being actively developed and the API is not stable. Please see [CONTRIBUTING](CONTRIBUTING.md) for more on how to help.

### Documentation

UNDER CONSTRUCTION
