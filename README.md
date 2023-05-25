# EDAA Project

Implementation of the Multigraph Data Structure and application of the following problems:
- Path-Finding;
- Spanning Tree;
- Erdõs Number;

## Dataset

The original dataset was obtained from:
[Flights and Airports Data](https://www.kaggle.com/datasets/tylerx/flights-and-airports-data?select=raw-flight-data.csv)

## Installation Details 

To run this code, you need to have a C++ compiler (such as GCC) and Python installed on your system. 
There's a Makefile to create an executable to run the program. To use this, it's required to have the *make* command installed in your system.
Additionally, you need to install the Python libraries specified in the "requirements.txt" file located in this repository. You can install these libraries using the following command:
```cmd
pip install -r requirements.txt
```

## Run Commands

The Makefile contains two targets: menu and test. The menu target generates an executable that allows you to use the application, while the test target is used for testing the application. To build each target, use the following commands:
```cmd
make menu
```
```cmd
make test
```

After building, you can run the application using the following commands:

For the *menu* target:
```cmd
./project.exe
```

For the *test* target:
```cmd
./tester.exe
```
