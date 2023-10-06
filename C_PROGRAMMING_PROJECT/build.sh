gcc src/processA.c -lbmp -lm -lncurses -o bin/processA -lrt -pthread
gcc src/processB.c -lbmp -lm -lncurses -o bin/processB -lrt -pthread
gcc src/master.c -o bin/master -lrt -pthread
