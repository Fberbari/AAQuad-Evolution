
build:
	gcc testRunner.c ../src/Common.c ../src/Pid.c ../src/LowPassFilter.c -o testRunner.out
	chmod +x testRunner.out
	./testRunner.out
	python3.5 SimulationResults/csvplot.py SimulationResults/doubletTest.csv

debug:
	gcc -g testRunner.c ../src/Common.c ../src/Pid.c ../src/LowPassFilter.c -o testRunner.out
	gdb testRunner.out

plot:
	python3.5 SimulationResults/csvplot.py SimulationResults/doubletTest.csv

clean:
	rm *.out
