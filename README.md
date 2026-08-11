# Destiny
> Core game world simulation engine for popular spaceship based MMO Eve Online.
 
# How to build
The project is build using CMake.

## Configuring
Configuration is best managed using the presets included in the project. To see the list of available presets, run
the following from the project root:

`cmake --list-presets`

To configure your project with a given preset, run the following from the project root:

`cmake --preset [preset]`

This will build using the default generator as configured by the `CMAKE_GENERATOR` environment variable on the host
machine. The generator can also be explicitly provided during configuration:

`cmake -G [generator] --preset [preset]`

For typical use cases, we recommend the `Ninja Multi-Config` generator.

## Building
After configuration is complete, run the following:

`cmake --build [BUILD_DIRECTORY]`

## 📄 License and Legal Notices

© 2026 CCP Games

This software is provided by CCP Games and does not include or distribute any third-party libraries or frameworks.

Core game world simulation engine for MMOs whose titles start with Eve. 

Trademark Notice: CCP Games is a trademark of CCP ehf.

This project is licensed under the [MIT License](LICENSE.md). Nothing in the [MIT License](LICENSE.md) grants any rights to CCP Games' trademarks or game content.
