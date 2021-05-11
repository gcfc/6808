import matplotlib.pyplot as plt
import csv

data = [[],[],[],[],[],[],[]]

with open("output1.csv", newline='') as csvfile:
    r = csv.reader(csvfile, delimiter=',')
    for row in r:
        for i in range(7):
            try:
                data[i].append(float(row[i]))
            except IndexError: continue

# t, x0, y0, z0, x1, y1, z1 = data

fig, axs = plt.subplots(6, 1)
for i in range(1,7):
    axs[i-1].plot(data[0], data[i])
plt.show()