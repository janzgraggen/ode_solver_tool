# ODE Solver Project

## General Information

This project is dedicated to solving systems of ordinary differential equations (ODEs) of the form \( f: \mathbb{R}^n \to \mathbb{R}^n \). It offers a flexible framework for solving ODEs based on user-defined equations and configurations.

### Authors:
- Jan Zgraggen 
- Natan ...


### Course:
- This project was developed as part of EPFL's Math 458: PCSC course.

---

## Features

### Supported ODE Solvers
- **Explicit Methods**:
  - Forward Euler
  - Runge-Kutta (4th order)
  - Adams-Bashforth (up to 3rd order)
- **Implicit Methods**:
  - Backward Euler
  - Crank-Nicolson

### Numerical Subroutines
- **Linear System Solvers**:
  - Gaussian Elimination
  - LU Decomposition
- **Root-Finding**:
  - Newton-Raphson Method
  - Fixed-Point Iteration

---

## Getting Started

### Dependencies
To build and run the project, ensure the following libraries are installed:
- [Eigen](https://eigen.tuxfamily.org)
- [GoogleTest](https://github.com/google/googletest)
- [muParser](https://beltoforion.de/en/muparser/)
- [YAML-CPP](https://github.com/jbeder/yaml-cpp)

If the dependencies are not automatically managed by CMake, initialize the submodules as follows:
```bash
git submodule update --init --recursive
```
Here is the content formatted as a complete README document:

---

# ODE Solver Project

## Build Instructions

### Create a build directory and navigate to it:
```bash
mkdir build
cd build
```

### Use CMake to configure and build the project:
```bash
cmake ..
cmake --build .
```

---

## Using the Solver

### Workflow
1. **Build**: Compile the project as described above.
2. **Configure**: Define the ODE problem in `ODE_config.yaml` located in `config/main/`.
3. **Run**: Execute the solver:
   ```bash
   ./main
   ```

---

## Configuration Logic

The configuration file (`ODE_config.yaml`) is structured into sections for flexible and user-friendly setup:

### General Configuration
- `output_file`: Path to the output file. Leave empty or set to `null` for no output.
- `verbosity_level`: Controls logging verbosity (`0` = Silent, `1` = Error, `2` = Warning, `3` = Info, `4` = Debug).

### Function Configuration
- `Dim`: Number of dimensions for the ODE system.
- `Function`: Define the functions \( f_1, f_2, \dots, f_n \) using expressions supported by `muParser`.

### ODE Solver Settings
- `solver_type`: Choose either `"Explicit"` or `"Implicit"`.
- `OdeSolver`:
  - `step_size`: Time step size for integration.
  - `initial_time`: Start time of the simulation.
  - `final_time`: End time of the simulation.
  - `initial_value`: List of initial values for the ODE system.

### Explicit Solver Settings (if `solver_type = "Explicit"`)
- `method`: Choose from `"ForwardEuler"`, `"RungeKutta"`, or `"AdamsBashforth"`.
- `RungeKutta`:
  - `order`: Specify the order (1, 2, 3, 4) or leave empty.
  - `coefficients`: Define custom Runge-Kutta coefficients (`a`, `b`, `c`) if needed.
- `AdamsBashforth`:
  - `max_order`: Maximum order (1 to 4).
  - `coefficients_vector`: Specify coefficients if `max_order` is not used.

### Implicit Solver Settings (if `solver_type = "Implicit"`)
- `method`: Choose from `"BackwardEuler"` or `"CrankNicolson"`.
- `rhs_is_linear`: Boolean to specify if the RHS is linear.
- `linear_system_solver`: Choose from `"GaussianElimination"` or `"LU"`.
- **Linear RHS**:
  - `rhs_system`: Define the system matrix \( A \) and vector \( b \).
- **Nonlinear RHS**:
  - `tolerance`: Convergence tolerance for iterative methods.
  - `max_iterations`: Maximum number of iterations.
  - `root_finder`: Choose from `"NewtonRaphson"` or `"FixedPoint"`.
  - `dx`: Step size for numerical differentiation (used in `NewtonRaphson`).

Refer to example configurations in the `config` folder for additional guidance.

---

## Retrieving Output

Simulation results are saved in CSV format and can be found in the output directory. You can customize the output location and format through the configuration file.

---

## Developer Guide

### Code Structure
The project is modularly organized for easy extensibility:
```
src
├── LinSysSolver
│   ├── GaussElimSolve.cc & .hh
│   ├── LUSolve.cc & .hh
│   └── LinSysSolver.cc & .hh
├── Logger
│   └── Logger.cc & .hh
├── Ode
│   ├── AdamsBashforth.cc & .hh
│   ├── BackwardEuler.cc & .hh
│   ├── Explicit.cc & .hh
│   ├── ForwardEuler.cc & .hh
│   ├── Implicit.cc & .hh
│   ├── OdeSolver.cc & .hh
│   └── RungeKutta.cc & .hh
├── Reader
│   ├── FunctionParser.cc & .hh
│   └── Reader.cc & .hh
├── RootFinder
│   ├── NewtonRaphson.cc & .hh
│   └── RootFinder.cc & .hh
├── Runner
│   └── Runner.cc & .hh
├── Utils
│   └── LinSysStruct.hh
├── Writer
│   └── CSVWriter.cc & .hh
└── main.cc
```

### Adding New Methods
The project is designed for extensibility through polymorphism:
1. **ODE Solvers**: Derive from the `Implicit` or  `Explicit` base classes to add new solving methods.
2. **Linear Solvers**: Derive from `LinSysSolver`.
3. **Root-Finding Methods**: Derive from `RootFinder`.

### Testing
1. Add test configurations in the `config/test` folder.
2. Use [GoogleTest](https://github.com/google/googletest) to verify your implementations:
   ```bash
   cd test
   ./run_tests
   ```

---
