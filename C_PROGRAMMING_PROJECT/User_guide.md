# Guide to Compile and run the project


In particular, the repository is organized as follows:
- The `src` folder contains the source code for the Master, Process A, and Process B.
- The `include` folder contains all the data structures and methods used within the ncurses framework to build the two GUIs. 
- The `bin` folder is where the executable files of the codes in the src files are stored
- The `Out` folder is where the bitmaps are stored.

## ncurses installation
To install the ncurses library, simply open a terminal and type the following command:
```console
sudo apt-get install libncurses-dev
```
## litbitmap installation
Follow the step in the README.md file to install the litbitmap library.

## Compiling the project

In the main file, there is a shell executable called build.sh. This executable is run as follows:

```console
./build.sh
```
the content of this executable is the following code lines:
```console
#! /usr/bin/bash
gcc src/processA.c -lbmp -lm -lncurses -o bin/processA -lrt -pthread
gcc src/processB.c -lbmp -lm -lncurses -o bin/processB -lrt -pthread
gcc src/master.c -o bin/master -lrt -pthread
```
This shell code allows to compile all the processes involved in the simulation and also is a more efficient way to make modifications.

## Running the project

In the same address of the shell executable to compile the project, there is the other executable that allows running the project called run.sh. In a terminal, type the next command

```console
./run.sh
```
the content of this executable is the following code lines:

```console

./bin/master
```
This shell code allows running the master code of the project, which executes the other process involved in the project, and clears the current terminal

To finish the process, in the terminal where the ./run.sh programm was run, use the Ctrl+C command to kill the master, and kill the other process



