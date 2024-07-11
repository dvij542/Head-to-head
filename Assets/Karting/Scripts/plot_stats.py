import matplotlib.pyplot as plt
import numpy as np

species = (
    "Ours",
    "Ours - CBF",
    "Ours - raceline \n - curriculum learning",
    "MCTS + LQR",
    "End to end",
)
weight_counts = {
    "Ours": np.array([0,7,3,1,0]),
    "Ours - CBF": np.array([13,0,4,0,0]),
    "Ours - raceline - curriculum learning": np.array([17,16,0,5,2]),
    "MCTS + LQR": np.array([19,20,15,0,0]),
    "End to end": np.array([20,20,18,19,1]),
}
width = 0.5

fig, ax = plt.subplots()
bottom = np.zeros(5)

for boolean, weight_count in weight_counts.items():
    p = ax.bar(species, weight_count, width, label=boolean, bottom=bottom)
    bottom += weight_count

ax.set_title("No. of races won")
ax.legend(loc="upper right")

plt.show()

n_steps = [0, 25000, 50000, 100000, 150000, 175000, 200000]
n_races_won_1 = [0, 5, 8, 6, 5, 6, 6]
n_races_won_2 = [0, 2, 4, 4, 4, 4, 4]
n_races_won_3 = [0, 0, 0, 2, 3, 2, 2]
plt.plot(n_steps,n_races_won_1,label='Ours (model+CBF curriculum)')
plt.plot(n_steps,n_races_won_2,label='Ours (only model curriculum)')
plt.plot(n_steps,n_races_won_3,label='Ours (no model or CBF curriculum)')
plt.legend()
plt.xlabel('steps')
plt.ylabel('no of wins')
plt.show()