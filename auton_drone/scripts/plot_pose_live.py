import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

def plot_new_points(position_plots, plot_data, count):
    x_plot, y_plot, z_plot = position_plots
    print(float(plot_data.y['filter'][-1][0]))
    x_plot.scatter(count, float(plot_data.x['filter'][-1][0]), c='r')
    y_plot.scatter(count, float(plot_data.y['filter'][-1][0]), 'r')
    z_plot.scatter(count, float(plot_data.z['filter'][-1][0]), 'r')

    x_plot.scatter(count+1, float(plot_data.x['tag_0'][-1][0]), 'g')
    y_plot.scatter(count+1, float(plot_data.y['tag_0'][-1][0]), 'g')
    z_plot.scatter(count+1, float(plot_data.z['tag_0'][-1][0]), 'g')

    x_plot.scatter(count+2, float(plot_data.x['tag_1'][-1][0]), 'b')
    y_plot.scatter(count+2, float(plot_data.y['tag_1'][-1][0]), 'b')
    z_plot.scatter(count+2, float(plot_data.z['tag_1'][-1][0]), 'b')

def init_plots():
  fig = plt.figure()
  x_pos = fig.add_subplot(1, 3, 1)
  y_pos = fig.add_subplot(1, 3, 2)
  z_pos = fig.add_subplot(1, 3, 3)

  position_plots = [x_pos, y_pos, z_pos]
  # velocity_plots = [vx, vy, vz]
  velocity_plots = []

  return fig, position_plots, velocity_plots

class PlotData():
  def __init__(self):
    self.x = {'tag_0': [],
              'tag_1': [],
              'tag_6': [],
              'tag_7': [],
              'filter': []}
    self.y = {'tag_0': [],
              'tag_1': [],
              'tag_6': [],
              'tag_7': [],
              'filter': []}
    self.z = {'tag_0': [],
              'tag_1': [],
              'tag_6': [],
              'tag_7': [],
              'filter': []}
  def add_data(self, name, X):
    if (X is None):
        self.x[name].append(self.x[name][-1])
        self.y[name].append(self.y[name][-1])
        self.z[name].append(self.z[name][-1])
    else:
        self.x[name].append(X[0])
        self.y[name].append(X[1])
        self.z[name].append(X[2])


def init_plot_data():
  return PlotData()

