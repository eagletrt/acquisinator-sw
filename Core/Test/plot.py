import matplotlib.pyplot as plt 
import csv, numpy

y1 = [] 
y2 = [] 

with open('filtered.csv','r') as csvfile: 
	lines = csv.reader(csvfile, delimiter=',') 
	for row in lines: 
		y1.append(int(row[0])) 
		y2.append(int(row[1])) 

xs = numpy.linspace(1, len(y1), len(y1))
print(xs)
plt.plot(xs, y1)
plt.plot(xs, y2)
plt.show()
