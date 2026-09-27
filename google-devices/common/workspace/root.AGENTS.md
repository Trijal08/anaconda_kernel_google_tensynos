# AGENTS.md for Pixel Kernel Codebase

## Project Overview
This repository contains the Pixel kernel codebase, responsible for building the
kernel, kernel modules, and kernel images for Pixel devices.

## Build System
The project uses **Kleaf**, which is a Bazel-based build system for the Linux kernel.

- **Build Command:** Use `tools/bazel` for all build operations.
- **Documentation:** Detailed documentation for Kleaf can be found in `build/kernel/kleaf/docs`.

## Key Directories
- `build/kernel/kleaf`: Kleaf build system source and docs.
- `common/ack`: Android common kernel source and docs.
- `private/devices/google`: Configuration for Pixel devices.
- `private/google-modules`: Kernel modules for Pixel devices.

## Development Guidelines
- **Building:**
  - Always use `tools/bazel`.
  - To build the distribution package of a device, run:
    ```
    tools/bazel run --config=<device> //private/devices/google/<device>:<device>/dist
    ```
  - To build a single module for a device, run:
    ```
    tools/bazel build --config=<device> //<path_to_module>:<module>
    ```
  - Ask the user if you need to know which device configuration to use.
- **Searching:**
  - Prioritize searching in `common/ack` and `private/` directories for relevant source code.
  - Ignore `out` and `bazel-*`, they are for output files.

