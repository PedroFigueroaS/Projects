# PROJECT-Summary

The Assignment consists in the design, develop, test and deploy of an interactive simulator of a (simplified) typical
vision system, able to track an object in a 2-D plane. In two consoles it is required to move a circle with the keyboard, obtain the center of this circle in any part of the windows, and with the implementation of shared memory, share this circle in another independent process to plot this position.

## The master process
The main process of the simulation, calls the other processes and gives each one of them a PID.

In the master code, to call other processes, it is used a spawn function that receives a string that contains the address of the desired executable. Into the spawn function, it is used the fork command to create a child process for the executable and run it properly with the execvp command.

## The process A
Process A corresponds to the first part of the simulation, which allows the user to manipulate the position of the circle with the keyboard

Process A will move the circle generated as a 2d matrix(MatrixA) of dimensions 80x30, with a specified radius defined while launching the code. While moving the circle, it will compute the center of the matrix, and show it in the processA window. At the same time, another matrix (Matrix2)with dimensions 1600x600 will be created by resizing the circle and storing it in the matrix.

In this process, it is initialized the shared memory communication and the semaphore thread, required for multitasking. 

With the shared memory communication, matrix 2 is stored in a 1D pointer that has the sh memory segment, to be available for every process that requires access to this address. 

With the semaphores, it is organized the steps required to create the matrix representation of the circle, calculate the center, and shared it with the other process. After that, the process will wait for the actions in process B.

There is a button implemented that after clicking on it, will create the bitmap with the circle in the current position, and save it in the folder OUT.

## The process B

The process corresponds to the second part of the simulation, which accesses the shared memory to manipulate the matrix shared by process A, and obtains the position required to move and trace the path that the arrow keys perform. 

Process B will use the shared memory communication to access the shared memory segment, in which there is the matrix2, which has a dimension of 1600x600. After resizing the 1D pointer, which stores the address of the shared memory, into a 2D matrix, calculates the center and divides it by 20, to obtain the same coordinates as the ones in process A, and proceeds to plot it as a "0" in the process B windows. 

The semaphores are used to wait after process A finishes its actions, to proceed to read and convert the pointer into the matrix, calculate the center and plot it in the window.
