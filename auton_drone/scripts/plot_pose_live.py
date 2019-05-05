import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# Plots all lines on same graph
# data_sources: dict mapping name_id to tuple of xs and ys values
def animate(i, time_axis, data_sources, plot):
    plot.clear()
    plot.plot(time_axis, data_sources['filter'], 'r')
    plot.plot(time_axis, data_sources['tag_0'], 'g')
    plot.plot(time_axis, data_sources['tag_1'], 'b')
    plot.plot(time_axis, data_sources['tag_6'], 'bl')
    plot.plot(time_axis, data_sources['tag_7'], 'br')
    plt.legend(['filter', 'tag_0', 'tag_1', 'tag_6', 'tag_7'])
    # if (is_filter):
    #   plot.plot(time_axis, data_sources['filter'])
    # else:
    #   plot.plot(time_axis, data_sources['tag0'])
    #   plot.plot(time_axis, data_sources['tag1'])
    #   plot.plot(time_axis, data_sources['tag6'])
    #   plot.plot(time_axis, data_sources['tag7'])
    

def init_plots():
  fig = plt.figure()
  x_pos = fig.add_subplot(1, 3, 0)
  y_pos = fig.add_subplot(1, 3, 1)
  z_pos = fig.add_subplot(1, 3, 2)
  # vx = fig.add_subplot(2, 3, 3)
  # vy = fig.add_subplot(2, 3, 4)
  # vz = fig.add_subplot(2, 3, 5)

  position_plots = [x_pos, y_pos, z_pos]
  # velocity_plots = [vx, vy, vz]
  velocity_plots = []

  return fig, position_plots, velocity_plots

class PlotData():
  def __init__(self):
    self.x = {'tag0': [],
              'tag1': [],
              'tag6': [],
              'tag7': [],
              'filter': []}
    self.y = {'tag0': [],
              'tag1': [],
              'tag6': [],
              'tag7': [],
              'filter': []}
    self.z = {'tag0': [],
              'tag1': [],
              'tag6': [],
              'tag7': [],
              'filter': []}
  def add_data(self, name, X):
    self.x[name].append(X[0])
    self.y[name].append(X[1])
    self.z[name].append(X[2])


def init_plot_data():
  return PlotData()

