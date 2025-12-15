# ΑETHERION — Flight Dynamics Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ci](https://github.com/onurtuncer/Aetherion/actions/workflows/linux.yml/badge.svg)](https://github.com/onurtuncer/Aetherion/actions/workflows/linux.yml)
[![ci](https://github.com/onurtuncer/Aetherion/actions/workflows/windows.yml/badge.svg)](https://github.com/onurtuncer/Aetherion/actions/workflows/windows.yml)
[![codecov](https://codecov.io/gh/onurtuncer1/Aetherion/branch/main/graph/badge.svg)](https://codecov.io/gh/onurtuncer/Aetherion)

<p align="center">
  <img src="assets/logo.svg" width="180" alt="AETHERION Monogram"/>
</p>

**AETHERION** is a high-precision, research-grade C++ library for  
**rocket and aerospace flight dynamics**, built on:

- **Featherstone 6-D Spatial Vector Algebra (SVA)**
- **ECI/WGS84 frame transformations**
- **Algorithmic Differentiation (AD-friendly)**
- **Clean ODE formulations for high-fidelity simulation**

The name *ΑΙΘΕΡΙΟΝ* (Aetherion) is inspired by the Greek word **Αιθήρ**,  
referring to the upper atmosphere — the pristine realm of celestial motion.

---

## ✨ Features

- **Full 6-DOF rigid-body dynamics** using Spatial Vector Algebra formalism 
- **Inertial-frame (ECI) equations of motion** with no pseudo-forces  
- **Rotating atmosphere support** via air-relative velocity only  
- **AD-friendly equations** for CppAD, dual numbers, and other libraries 
- **US1976 Standard Atmosphere** and clean gravity models  
- **Lie structure preserving Runge-Kutta-Munthe-Kaas integrator** (with a fully implicit RADAU-IIA solver implementation)  
- **Discrete extended Kalman filter on a product manifold containing Lie groups** for state estimation
- **Post-modern C++23 design**

---

## 📐 Mathematical Foundations

ΑETHERION implements all major constructs from Featherstone's Rigid Body Dynamics Algorithms:

- Spatial velocity `v`
- Spatial acceleration `a`
- Spatial inertia `I`
- Motion  `v×` and its adjoint force `v×*` cross operators

Equation of motion:

<p align="center">
  <img src="assets/equation_of_motion.svg" width="180" alt="Equation of Motion"/>
</p>

Control surfaces can be modeled as rigidly attached bodies.

---

# 📌 Frames & Conventions

ΑETHERION uses:

- **W — Inertial frame (ECI)**
- **E — Earth-fixed frame (ECEF)**
- **B — Rocket body frame**

All transformations follow:

<p align="center">
  <img src="assets/equation_of_motion.svg" width="180" alt="Transform equation"/>
</p>

notation for transforms and spatial transforms.

This keeps the frame algebra clean, explicit, and AD-friendly.

---

# 📊 Atmosphere & Gravity

ΑETHERION provides:

- **US1976 Standard Atmosphere**  
  - temperature  
  - pressure  
  - density  
  - speed of sound  
- **Gravity models**  
- **Branch-free formulations** ideal for AD and Kalman filtering  

---

# 🧪 Testing

The library includes a complete test suite:

- **Catch2 unit tests**
- **Numerical validation** against NASA technical reports


This ensures consistency across symbolic, AD, and numeric implementations.

## Dependencies

- Eigen for linear algebra.
- fmu4cpp for FMI building.
- ecos for simulation.
- cppad for algorithmic differentiation.
- Catch2 v3 is required for testing. 

## 📚 Documentation

- [Detailed Documentation](https://onurtuncer.github.io/Aetherion/)

## Development

### Bug Tracker

- [Github Issues](https://github.com/onurtuncer/Aetherion/issues)


### Forums and Discussions

- [Github Discussions](https://github.com/onurtuncer/Aetherion/discussions/)


# Build & Installation

First you need to dowload the files or clone the repository, then initialize and update the submodules.


## Windows Build Using Visual Studio

1. Open **Visual Studio**
2. Click **Open Folder…**
3. Select the **Aetherion** directory
4. Let Visual Studio configure CMake automatically
5. Choose a configuration (e.g. **x64-Release**)
6. Build the project via **Build > Build All**
7. Run the executable

## 👤 Author

**Prof.Dr. Onur Tuncer**  
Aerospace Engineer, Researcher & C++ Systems Developer  
Email: **onur.tuncer@itu.edu.tr**

<p align="left">
  <img src="assets/itu_logo.png" width="180" alt="Istanbul Technical University"/>
</p>


