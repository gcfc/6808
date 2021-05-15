import matplotlib.pyplot as plt
import csv, os

N_COLS = 13
data = [[] for _ in range(N_COLS)]
curr_dir = os.path.dirname(os.path.realpath(__file__))

with open(os.path.join(curr_dir,"output515-1.csv"), newline='') as csvfile:
    r = csv.reader(csvfile, delimiter=',')
    for row in r:
        for i in range(N_COLS):
            try:
                data[i].append(float(row[i]))
            except IndexError: continue

# t, x0, y0, z0, x1, y1, z1 = data

fig, axs = plt.subplots(N_COLS-1, 1)
for i in range(1,N_COLS):
    axs[i-1].plot(data[0], data[i])
plt.show()