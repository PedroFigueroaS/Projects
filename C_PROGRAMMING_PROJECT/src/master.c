#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/wait.h>


int spawn(const char * program, char * arg_list[]) { //function to spawn a process and open a console for it

  pid_t child_pid = fork();

  if(child_pid < 0) {
    perror("Error while forking...");
    return 1;
  }

  else if(child_pid != 0) {
    return child_pid;
  }

  else {
    if(execvp (program, arg_list) == 0);
    perror("Exec failed");
    return 1;
  }
}

int main() {

  //argument list to call the executable of each process 
  char * arg_list_A[] = { "/usr/bin/konsole", "-e", "./bin/processA","out/testA.bmp","5", NULL };
  char * arg_list_B[] = { "/usr/bin/konsole", "-e", "./bin/processB","5",NULL };
  //The spawn function is called to execute the other processes
  pid_t pid_procA = spawn("/usr/bin/konsole", arg_list_A);
  sleep(1);
  pid_t pid_procB = spawn("/usr/bin/konsole", arg_list_B);

  int status;
  waitpid(pid_procA, &status, 0);
  waitpid(pid_procB, &status, 0);
  
  printf ("Main program exiting with status %d\n", status);
  return 0;
}

