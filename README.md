This is a template project structure for the *Programming concepts in scientific computing* (MATH-458)

In order to compile it you should first install *googletest*

```
git submodule update --init 
```

Then, building is done as usual, e.g. with CLion or in the terminal:

```
mkdir build
cd build
cmake ..
make

alternatively: (build all)
mkdir build
cd build
cmake ..
cmake --build .
./main

alternatively (build specific exec)
mkdir build
cd build
cmake ..
cmake --build . --target MyExec
./MyExec
```


