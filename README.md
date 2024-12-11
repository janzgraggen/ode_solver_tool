```
 _____ ____  _____ _____     _             
|     |    \|   __|   __|___| |_ _ ___ ___
|  |  |  |  |   __|__   | . | | | | -_|  _|
|_____|____/|_____|_____|___|_|\_/|___|_|

by Jan Zgraggen and Nathan Kabas Kuroiwa
```

## General Information

Welcome to our ODE-Solver Project!

> Note: The project is part of the **MATH-458: Programming Concepts in Scientific Computing** course at EPFL (Fall semester 2024) thaught by Guillaume Anciaux.

Built using C++, the project is dedicated to solving systems of ordinary differential equations (ODEs) of the form
$\frac{dy}{dt} = f(t,y)$ , where we want to find $$\quad y(t) \in \mathbb{R}^n,$$
given $$\quad f : \mathbb{R} \times \mathbb{R}^n \to \mathbb{R}^n \text{ and } \quad y(t_0)=y_0 \in \mathbb{R}^n $$.

It offers a flexible framework for solving ODEs based on user-defined equations and configurations.

---

## Features

### Supported ODE Solvers
- **Explicit Methods**:
  - Forward Euler
  - Runge-Kutta
  - Adams-Bashforth
- **Implicit Methods**:
  - Backward Euler

### Numerical Subroutines (for implicit methods)
- **Linear System Solvers**:
  - Gaussian Elimination
  - LU Decomposition
- **Root-Finding**:
  - Newton-Raphson Method

---

## Getting Started

### Dependencies
In this project, the following libraries are used:
- [Eigen](https://eigen.tuxfamily.org)
- [GoogleTest](https://github.com/google/googletest)
- [muParser](https://beltoforion.de/en/muparser/)
- [YAML-CPP](https://github.com/jbeder/yaml-cpp)

Dependencies should automatically be managed by CMake. If that's not the case please initialize the submodules as 
follows:
```bash
git submodule update --init --recursive
```

### Build Instructions

Create a build directory and navigate to it:
```bash
mkdir build
cd build
```

Use CMake to configure and build the project:
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

The configuration file (`ODE_config.yaml`) is structured into sections for flexible and user-friendly setup. Examples of configurations (which we used for testing) can be found in `config/test` for additional help!

### General Configuration
- `output_file`: Path to the output file. Leave empty or set to `null` for no output.
- `verbosity_level`: Controls logging verbosity (`0` = Silent, `1` = Error, `2` = Warning, `3` = Info, `4` = Debug).

### Function Configuration
- `Dim`: Number of dimensions for the ODE system.
- `Function`: Define the functions $$ f_1, f_2, \dots, f_n $$ using expressions supported by `muParser` (quite standard
syntax). Please provide as many functions as there are dimensions, and refer to values of the vector y by `y1, ..., yn`.

### ODE Solver Settings
- `solver_type`: Choose either `"Explicit"` or `"Implicit"`.
- `OdeSolver`:
  - `step_size`: Time step size for integration.
  - `initial_time`: Start time of the simulation.
  - `final_time`: End time of the simulation.
  - `initial_value`: List of initial values for the ODE system.

### Explicit Solver Settings (if `solver_type: "Explicit"`)
- `method`: Choose from `"ForwardEuler"`, `"RungeKutta"`, or `"AdamsBashforth"`.
- `RungeKutta`:
  - `order`: Specify the order (1, 2, 3, 4) or leave empty to use custom coefficients.
  - `coefficients`: If needed, define custom Runge-Kutta coefficients (`a`, `b`, `c`) from a Butcher tableau (a:
interactions, b: weights, c: intermediate steps).

More documentation about the Runge-Kutta methods [here](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods).

- `AdamsBashforth`:
  - `max_order`: Maximum order (1 to 4).
  - `coefficients_vector`: Specify coefficients if `max_order` is not used.

For the Adams-Bashforth method, one should note that lower orders are used until having a large enough history for
higher orders, such that only one initial vector is sufficient. More documentation about the Adams-Bashforth methods
[here](https://en.wikipedia.org/wiki/Linear_multistep_method#Adams%E2%80%93Bashforth_methods).

### Implicit Solver Settings (if `solver_type: "Implicit"`)
- `method`: Choose `"BackwardEuler"`.
- `rhs_is_linear`: Boolean to specify if the Right Hand Side (RHS) function is linear.
- `linear_system_solver`: Choose from `"GaussianElimination"` or `"LU"`.
- **Linear RHS**:
  - `rhs_system`: Define the system matrix A and vector b such that $$ f = A y + b $$

  > Note: If rha_is_linear is set to true, a right hand side (rhs) system has to be provided. (As opposed to setting the function via function definition). 
  If a Linear function is desired to be computed with rootfinding rather than linear system solving, `rhs_is_linear` is set to false, the function is passed as above (cf. function configuration) and nonlinear settings apply.

- **Nonlinear RHS**:
  - `tolerance`: Convergence tolerance for iterative methods.
  - `max_iterations`: Maximum number of iterations.
  - `root_finder`: Choose `"NewtonRaphson"`.
  - `dx`: Step size for numerical differentiation (used in `NewtonRaphson`).

---

## Retrieving Output

Simulation results at each time steps are saved in CSV format and can be found in the output directory.

If there is no 
output directory, one will be generated on first run. Once this is done, you can customize the output location in later
runs by creating new folders inside this directory and specifying the right path in the configuration file. Or overwriting the current file by keepint the outputfilename the same. 

---

## Doxygen Documentation

The Doxyfile necessary to generate an .html is included in the repository. An `html` folder containing the `index.html` 
is generated from building the project. Otherwise, it is also possible to run the command:
the project.
   ```bash
   cmake --build . --target doc_doxygen
   ```

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
