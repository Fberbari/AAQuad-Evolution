



file = open("AsData.txt", "r")
lines = file.readlines()
file.close()


for line in lines:
	
	index = line.find("]")
	line = line[index + 1:]
	index = line.find("float")
	line = line[:index]
	line = line.strip()
	print(line, ",", end = "")


