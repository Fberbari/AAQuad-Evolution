
build:
	gcc testRunner.c ../src/Common.c ../src/Pid.c ../src/LowPassFilter.c -o testRunner.out
	./testRunner.out
	python3.5 SimulationResults/csvplot.py SimulationResults/stepTest.csv

debug:
	gcc -g testRunner.c ../src/Common.c ../src/Pid.c ../src/LowPassFilter.c -o testRunner.out
	gdb testRunner.out

plot:
	python3.5 SimulationResults/csvplot.py SimulationResults/stepTest.csv

clean:
	rm *.out
