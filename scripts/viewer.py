import numpy as np
import Tkinter as tk
import matplotlib.pyplot as plt
import seaborn as sns

from threading import Thread

import matplotlib.backends.tkagg as tkagg
from matplotlib.backends.backend_agg import FigureCanvasAgg

import plotting


class Viewer:
    def __init__(self, master, data, bg, cmap):

        self.master = master
        self.master.geometry('600x600')
        self.master.title("Map Visualization")
        self.master.resizable(0, 0)
        self.data = data
        self.bg = bg
        self.cmap = cmap
        self.frame = None

        self.canvas = tk.Canvas(self.master)
        self.canvas.pack(fill='both', expand='yes')

        self.gif_renderer = Thread(target=self.show_gif)
        self.gif_renderer.start()

    def show_gif(self):
        while True:
            self.show_next_frame()

    def show_next_frame(self):

        # Clear figure to fit new one
        plt.gcf().clf()
        plt.gcf().set_dpi(100)
        plt.gcf().set_figwidth(6)
        plt.gcf().set_figheight(6)

        # Advance to next snapshot, or reset to beginning of loop
        self.data.next_data()

        # Generate next heatmap
        sns.heatmap(self.data.get_data(), vmin=0,
                    vmax=np.amax(self.data.get_data()),
                    cmap=self.cmap,
                    xticklabels=False, yticklabels=False, square=1)

        plt.tight_layout(h_pad=0, w_pad=0)

        # show background
        plt.imshow(self.bg)
        plt.xticks([])
        plt.yticks([])
        ax = plt.gca()
        ax.set_xlim(0, self.bg.shape[1])
        ax.set_ylim(self.bg.shape[0], 0)

        # Draw figure over background
        figure_canvas_agg = FigureCanvasAgg(plt.gcf())
        figure_canvas_agg.draw()
        f_x, f_y, f_w, f_h = plt.gcf().bbox.bounds
        f_w, f_h = int(f_w), int(f_h)
        photo = tk.PhotoImage(master=self.canvas, width=f_w, height=f_h)

        # Position: convert from top-left anchor to center anchor
        self.canvas.create_image(f_w / 2, f_h / 2, image=photo)
        tkagg.blit(photo, figure_canvas_agg.get_renderer()._renderer,
                   colormode=2)
        self.frame = photo  # this handle must be kept alive

        # self.canvas.itemconfigure(self.date_label, text=self.data.get_date())

        date_txt = self.data.get_date().strftime("%b %d, %Y")
        self.canvas.create_text((40, 70), anchor='nw',
                                fill='#ffffff', text=date_txt)
