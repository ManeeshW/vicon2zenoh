# vicon2zenoh

Publishes Vicon motion-capture pose data over a [Zenoh](https://zenoh.io) session.
An optional Qt6 GUI (`vicon_gui`) plots live position/orientation data.

---

## Dependencies

### Required

| Package | Purpose | Install |
|---------|---------|---------|
| CMake ≥ 3.20 | Build system | `sudo apt install cmake` |
| GCC / Clang (C++17) | Compiler | `sudo apt install build-essential` |
| Eigen3 | Linear algebra | `sudo apt install libeigen3-dev` |
| VRPN + Quat | Vicon tracker protocol | see below |
| zenohc | Zenoh C library | see below |
| zenohcxx | Zenoh C++ bindings | see below |
| nlohmann/json ≥ 3.2 | JSON serialisation | `sudo apt install nlohmann-json3-dev` |

### Optional (GUI only)

| Package | Purpose | Install |
|---------|---------|---------|
| Qt6 Core + Widgets + Charts | Live plot GUI (`vicon_gui`) | see below |

---

## Installing dependencies

### VRPN

```bash
sudo apt install libvrpn-dev
```

If your distro does not package VRPN, build from source:

```bash
git clone https://github.com/vrpn/vrpn.git
cmake -S vrpn -B vrpn/build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build vrpn/build -j$(nproc)
sudo cmake --install vrpn/build
```

### zenohc and zenohcxx

Follow the official releases at <https://github.com/eclipse-zenoh/zenoh-c> and
<https://github.com/eclipse-zenoh/zenoh-cpp>.

```bash
# zenohc
git clone https://github.com/eclipse-zenoh/zenoh-c.git
cmake -S zenoh-c -B zenoh-c/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build zenoh-c/build -j$(nproc)
sudo cmake --install zenoh-c/build

# zenohcxx
git clone https://github.com/eclipse-zenoh/zenoh-cpp.git
cmake -S zenoh-cpp -B zenoh-cpp/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build zenoh-cpp/build -j$(nproc)
sudo cmake --install zenoh-cpp/build
```

### Qt6 (optional — required only for `vicon_gui`)

```bash
sudo apt install qt6-base-dev libqt6charts6-dev libgl-dev
```

If the package manager version is too old, install via the Qt online installer
(<https://www.qt.io/download-qt-installer>) and point CMake at it:

```bash
cmake -DCMAKE_PREFIX_PATH=/path/to/Qt/6.x.x/gcc_64 ..
```

---

## Building

```bash
git clone https://github.com/yourorg/vicon2zenoh.git
cd vicon2zenoh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

If Qt6 is not installed the build still succeeds — only `vicon_gui` is skipped:

```
CMake Warning: Qt6 not found — vicon_gui target will NOT be built.
```

Install Qt6 (see above) and re-run `cmake -S . -B build` to enable it.

---

## Installing

```bash
sudo cmake --install build --prefix /usr/local
```

This installs:

| Artefact | Destination |
|----------|-------------|
| `libvicon_tracker_static.a` | `$prefix/lib/` |
| Public headers | `$prefix/include/vicon_tracker/` |
| `test_vicon_tracker` binary | `$prefix/bin/` |
| `vicon_gui` binary (if built) | `$prefix/bin/` |
| CMake package config | `$prefix/lib/cmake/ViconTracker/` |

To install to a custom prefix (no `sudo` needed):

```bash
cmake --install build --prefix ~/.local
```

### Using the installed library in another CMake project

After installing, downstream projects can locate the library with:

```cmake
find_package(ViconTracker REQUIRED)
target_link_libraries(my_target PRIVATE ViconTracker::vicon_tracker_static)
```

If installed to a non-standard prefix, set `CMAKE_PREFIX_PATH`:

```bash
cmake -DCMAKE_PREFIX_PATH=~/.local ..
```

---

## Running

```bash
# Tracker publisher (edit config.cfg first)
test_vicon_tracker

# Live GUI (requires Qt6 build)
vicon_gui
```

Configuration is read from `config.cfg` / `config_py.cfg` in the working directory.
