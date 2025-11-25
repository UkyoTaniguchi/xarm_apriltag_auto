import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

def setup_axes():
    fig, ax = plt.subplots(1,1, figsize=(6,4), dpi=100)
    ax.grid(True)
    return fig, ax
