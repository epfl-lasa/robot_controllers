# Robot Controllers
This repo contains "low" and "high" level controllers for robotics control through Dynamical Systems

### Authors/Maintainer

- Bernardo Fichera
- Konstantinos Chatzilygeroudis

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

Copyright (c) 2019, **Konstantinos Chatzilygeroudis, Bernardo Fichera**
