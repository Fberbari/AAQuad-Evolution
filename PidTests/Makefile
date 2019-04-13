
build:
	gcc testRunner.c ../src/Common.c ../src/Pid.c ../src/LowPassFilter.c -o testRunner.out

debug:
	gcc -g testRunner.c ../src/Common.c ../src/Pid.c -o testRunner.out
	gdb testRunner.out
run:
	./testRunner.out

clean:
	rm *.out
