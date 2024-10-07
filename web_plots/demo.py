import random
import threading
import time
from threading import Timer, Thread
import remi.gui as gui
from remi import App, start
import uuid
import collections
import math

random.seed(228)


class SvgComposedPoly(gui.SvgGroup):
    """A group of polyline and circles"""

    def __init__(self, maxlen, stroke, color, *args, **kwargs):
        super(SvgComposedPoly, self).__init__(*args, **kwargs)
        self.maxlen = maxlen
        self.plotData = gui.SvgPolyline()
        self.plotData.set_fill('none')
        self.append(self.plotData)
        self.set_stroke(stroke, color)
        self.circle_radius = stroke
        self.circles_list = list()
        self.coordsX = collections.deque(maxlen=maxlen)
        self.coordsY = collections.deque(maxlen=maxlen)

    def add_coord(self, x, y):
        """Adds a coord to the polyline and creates another circle"""
        self.coordsX.append(x)
        self.coordsY.append(y)
        self.plotData.add_coord(x, y)
        circle = gui.SvgCircle(x, y, self.circle_radius)
        self.circles_list.append(circle)
        self.append(circle)
        if len(self.circles_list) > self.maxlen:
            self.remove_child(self.circles_list[0])
            del self.circles_list[0]


class SvgPlot(gui.Svg):
    def __init__(self, _width, _height, window_size=100):
        super(SvgPlot, self).__init__(width=_width, height=_height)
        self.window_size = window_size
        self.width = _width
        self.height = _height
        self.polyList = []
        self.font_size = 15
        self.plot_inner_border = self.font_size + 10  # Padding around the plot area
        self.x_min = 0
        self.x_max = self.window_size  # Initial window size
        self.y_min = 0
        self.y_max = 1
        self.title = gui.SvgText(self.width / 2, -self.plot_inner_border / 2, "")
        self.title.style['font-size'] = gui.to_pix(self.font_size + 5)
        self.title.attributes['text-anchor'] = 'middle'
        self.append(self.title)

        # Axis labels
        self.x_label = gui.SvgText(self.width / 2, self.height + self.plot_inner_border + 5, "")
        self.x_label.style['font-size'] = gui.to_pix(self.font_size)
        self.x_label.attributes['text-anchor'] = 'middle'
        self.append(self.x_label)

        self.y_label = gui.SvgText(-self.plot_inner_border + 20, self.height / 2 - 30, "")
        self.y_label.style['font-size'] = gui.to_pix(self.font_size)
        self.y_label.attributes['text-anchor'] = 'middle'
        self.y_label.attributes['transform'] = f'rotate(-90 {-self.plot_inner_border + 20} {self.height / 2})'
        self.append(self.y_label)

        # Grid group
        self.grid_group = gui.SvgGroup()
        self.append(self.grid_group)

        # Axis lines
        self.axis_group = gui.SvgGroup()
        self.append(self.axis_group)

        # Tick labels group
        self.ticks_group = gui.SvgGroup()
        self.append(self.ticks_group)

    def set_axis_limits(self, y_min, y_max):
        """Set the Y-axis limits for the plot"""
        self.y_min = y_min
        self.y_max = y_max

    def set_labels(self, title, x_label, y_label):
        """Set the labels for title and axes"""
        self.title.set_text(title)
        self.x_label.set_text(x_label)
        self.y_label.set_text(y_label)

    def append_poly(self, polys):
        for poly in polys:
            self.append(poly)
            self.polyList.append(poly)

    def remove_poly(self, poly):
        self.remove_child(poly)
        self.polyList.remove(poly)

    def render(self):
        # Clear previous grid and ticks
        self.grid_group.empty()
        self.ticks_group.empty()
        self.axis_group.empty()

        # Define plot area dimensions
        plot_width = self.width
        plot_height = self.height

        # Set viewbox
        self.set_viewbox(-self.plot_inner_border, -self.plot_inner_border,
                         self.width + self.plot_inner_border * 2, self.height + self.plot_inner_border * 2)

        # Adjust x-axis limits for sliding window
        for poly in self.polyList:
            if len(poly.coordsX) > 0:
                latest_x = poly.coordsX[-1]
                self.x_min = latest_x - self.window_size
                self.x_max = latest_x

        # Dynamically adjust the number of ticks to prevent overlapping
        max_ticks_x = int(plot_width / (self.font_size * 3))  # Approximate width per tick label
        max_ticks_y = int(plot_height / (self.font_size * 1.5))  # Approximate height per tick label

        x_range = self.x_max - self.x_min
        y_range = self.y_max - self.y_min

        # Calculate suitable grid steps
        grid_x_step = self._nice_number(x_range / max_ticks_x)
        grid_y_step = self._nice_number(y_range / max_ticks_y)

        # Horizontal grid lines and Y-axis ticks
        y_start = math.ceil(self.y_min / grid_y_step) * grid_y_step
        y = y_start
        while y <= self.y_max:
            y_pos = self._map_y(y, plot_height)
            line = gui.SvgLine(0, y_pos, plot_width, y_pos)
            line.set_stroke(1, 'lightgray')
            self.grid_group.append(line)

            # Y-axis tick labels
            tick = gui.SvgText(-5, y_pos + 5, f"{y:.2f}")
            tick.attributes['text-anchor'] = 'end'
            tick.style['font-size'] = gui.to_pix(self.font_size - 2)
            self.ticks_group.append(tick)

            y += grid_y_step

        # Vertical grid lines and X-axis ticks
        x_start = math.ceil(self.x_min / grid_x_step) * grid_x_step
        x = x_start
        while x <= self.x_max:
            x_pos = self._map_x(x, plot_width)
            line = gui.SvgLine(x_pos, 0, x_pos, plot_height)
            line.set_stroke(1, 'lightgray')
            self.grid_group.append(line)

            # X-axis tick labels
            tick = gui.SvgText(x_pos, plot_height + self.font_size, f"{x:.2f}")
            tick.attributes['text-anchor'] = 'middle'
            tick.style['font-size'] = gui.to_pix(self.font_size - 2)
            self.ticks_group.append(tick)

            x += grid_x_step

        # Draw axes
        x_axis = gui.SvgLine(0, plot_height, plot_width, plot_height)
        y_axis = gui.SvgLine(0, 0, 0, plot_height)
        x_axis.set_stroke(2, 'black')
        y_axis.set_stroke(2, 'black')
        self.axis_group.append([x_axis, y_axis])

        # Render polylines
        for poly in self.polyList:
            # Map data points to plot coordinates, only if they are within x_min and x_max
            mapped_points = []
            visible_indices = []
            for idx, (x_val, y_val) in enumerate(zip(poly.coordsX, poly.coordsY)):
                if self.x_min <= x_val <= self.x_max:
                    x_pos = self._map_x(x_val, plot_width)
                    y_pos = self._map_y(y_val, plot_height)
                    mapped_points.append((x_pos, y_pos))
                    visible_indices.append(idx)

            # Update polyline points
            poly.plotData.attributes['points'] = ' '.join(f"{x},{y}" for x, y in mapped_points)

            # Update circles (data points)
            # Remove all existing circles first
            for circle in poly.circles_list:
                poly.remove_child(circle)
            poly.circles_list = []

            # Add only visible circles
            for idx in visible_indices:
                x_val = poly.coordsX[idx]
                y_val = poly.coordsY[idx]
                x_pos = self._map_x(x_val, plot_width)
                y_pos = self._map_y(y_val, plot_height)
                circle = gui.SvgCircle(x_pos, y_pos, poly.circle_radius)
                poly.circles_list.append(circle)
                poly.append(circle)

    def _map_x(self, x_value, plot_width):
        """Map data x value to plot coordinate"""
        return (x_value - self.x_min) / (self.x_max - self.x_min) * plot_width

    def _map_y(self, y_value, plot_height):
        """Map data y value to plot coordinate"""
        return plot_height - (y_value - self.y_min) / (self.y_max - self.y_min) * plot_height

    def _nice_number(self, value):
        """Round the number to a nice value for grid steps"""
        if value == 0:
            return 1  # Prevent division by zero
        exponent = math.floor(math.log10(abs(value)))
        fraction = abs(value) / (10 ** exponent)
        if fraction <= 1:
            nice_fraction = 1
        elif fraction <= 2:
            nice_fraction = 2
        elif fraction <= 5:
            nice_fraction = 5
        else:
            nice_fraction = 10
        return nice_fraction * (10 ** exponent)


class PlotsServer(App):
    my_instance = None

    def __init__(self, *args):
        # presets
        self.svgplots = {}  # Dictionary to hold multiple plots
        self.plot_data = {}  # Dictionary to hold data for each plot
        self.plot_container = []
        self.update_lock = threading.Lock()
        # main methods applying
        super(PlotsServer, self).__init__(*args)
        PlotsServer.my_instance = self
        self.id = uuid.uuid4()
        self.stop_flag = False

    @staticmethod
    def get_my_instance():
        return PlotsServer.my_instance

    def add_plot(self, plot_id, width=700, height=400, window_size=100):
        with self.update_lock:
            # Create a new plot
            svgplot = SvgPlot(width, height, window_size)
            svgplot.style['margin'] = '10px'
            svgplot.style['overflow'] = 'visible'
            self.svgplots[plot_id] = svgplot

            # Initialize plot data
            plot_data = SvgComposedPoly(window_size, 2, 'blue')
            svgplot.append_poly([plot_data])
            self.plot_data[plot_id] = plot_data

            # Add the plot to the container
            self.plot_container.append(svgplot)

    def set_labels(self, plot_id, title="", x_label="", y_label=""):
        with self.update_lock:
            if plot_id in self.svgplots:
                svgplot = self.svgplots[plot_id]
                svgplot.set_labels(title=title, x_label=x_label, y_label=y_label)

    def set_axis_limits(self, plot_id, y_min=None, y_max=None):
        with self.update_lock:
            if plot_id in self.svgplots:
                svgplot = self.svgplots[plot_id]
                if y_min is not None:
                    svgplot.y_min = y_min
                if y_max is not None:
                    svgplot.y_max = y_max

    def set_window_size(self, plot_id, window_size):
        with self.update_lock:
            if plot_id in self.svgplots:
                svgplot = self.svgplots[plot_id]
                svgplot.window_size = window_size
                plot_data = self.plot_data[plot_id]
                plot_data.maxlen = window_size  # Update maxlen to match window size

    def update_coords(self, plot_id, x, y):
        with self.update_lock:
            if plot_id in self.plot_data:
                plot_data = self.plot_data[plot_id]
                plot_data.add_coord(x, y)
                # Adjust y-axis limits automatically based on data
                svgplot = self.svgplots[plot_id]
                svgplot.y_min = min(svgplot.y_min, y)
                svgplot.y_max = max(svgplot.y_max, y)

    def main(self):
        # plot settings
        self.wid = gui.VBox()
        self.wid.style['overflow'] = 'visible'

        # Container for multiple plots
        self.plot_container = gui.VBox()
        self.wid.append(self.plot_container)

        # Interface elements (optional)
        bt_zoom_in = gui.Button("Zoom + ")
        bt_zoom_out = gui.Button("Zoom - ")
        bt_zoom_in.onclick.do(self.zoom_in_all)
        bt_zoom_out.onclick.do(self.zoom_out_all)
        self.wid.append(bt_zoom_in)
        self.wid.append(bt_zoom_out)

        # Start the rendering loop
        self.stop_flag = False
        self.start_draw()

        # returning the root widget
        PlotsServer.ready = True
        return self.wid

    def on_close(self):
        self.stop_flag = True
        super(PlotsServer, self).on_close()

    def zoom_out_all(self, emitter):
        with self.update_lock:
            for svgplot in self.svgplots.values():
                scale_factor = 1.2
                y_center = (svgplot.y_max + svgplot.y_min) / 2
                y_range = (svgplot.y_max - svgplot.y_min) * scale_factor
                svgplot.set_axis_limits(
                    y_center - y_range / 2,
                    y_center + y_range / 2,
                )

    def zoom_in_all(self, emitter):
        with self.update_lock:
            for svgplot in self.svgplots.values():
                scale_factor = 0.8
                y_center = (svgplot.y_max + svgplot.y_min) / 2
                y_range = (svgplot.y_max - svgplot.y_min) * scale_factor
                svgplot.set_axis_limits(
                    y_center - y_range / 2,
                    y_center + y_range / 2,
                )

    def start_draw(self):
        with self.update_lock:
            for svgplot in self.svgplots.values():
                svgplot.render()
        if not self.stop_flag:
            Timer(0.1, self.start_draw).start()


def start_in_thread(port):
    start(PlotsServer, address='0.0.0.0', port=port, update_interval=0.1, multiple_instance=False)


def start_plots_app(port):
    import socket

    def check_port_available(host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind((host, port))
                return True  # accessible
            except socket.error:
                return False  # forbidden

    if check_port_available('0.0.0.0', port):
        Thread(target=start_in_thread, args=(port,)).start()