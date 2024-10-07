import numpy as np
import pandas as pd
import random
from nav_helpers import degrees_to_radians
import remi.gui as gui
from remi import App, start
from threading import Timer
import math
import uuid

random.seed(228)
from profiler import Profiler

my_profiler = Profiler()
my_profiler.set_output_file("profiler.txt")

class SvgComposedPoly(gui.SvgGroup):
    """
    A group of polyline and circles
    """

    def __init__(self, maxlen, stroke, color, *args, **kwargs):
        super(SvgComposedPoly, self).__init__(*args, **kwargs)
        self.maxlen = maxlen
        self.plotData = gui.SvgPolyline(self.maxlen)
        self.plotData.set_fill('none')
        self.append(self.plotData)
        self.set_stroke(stroke, color)
        self.set_fill(color)
        self.circle_radius = stroke
        self.circles_list = list()

    def add_move(self, x, y):
        """ Adds a coord to the polyline and creates another circle
        """
        self.plotData.add_coord(x, y)
        self.circles_list.append(gui.SvgCircle(x, y, self.circle_radius))
        self.append(self.circles_list[-1])
        if len(self.circles_list) > self.maxlen:
            self.remove_child(self.circles_list[0])
            del self.circles_list[0]

    def add_full_trajectory(self, traj):
        for pt in traj:
            x = pt[0]
            y = pt[1]
            self.plotData.add_coord(x, y)
            self.circles_list.append(gui.SvgCircle(x, y, self.circle_radius))
            self.append(self.circles_list[-1])


class RobotPlot(gui.Svg):
    def __init__(self, _width, _height):
        super(RobotPlot, self).__init__(width=_width, height=_height)
        self.width = _width
        self.height = _height
        self.polyList = []  # gt - 0, real - 1
        self.font_size = 15
        self.plot_inner_border = self.font_size
        self.scale_x = 10.0
        self.scale_y = 10.0
        self.robot_element = None
        self.wheel_status = [False, False, False, False]
        self.wheels = None
        self.robot_dir_element = None
        self.road_visor_detections = None
        self.road_visor_detections_svg = None

    def scale(self, x_scale, y_scale):
        self.scale_x *= x_scale
        self.scale_y *= y_scale

    def append_poly(self, polys):
        for poly in polys:
            self.append(poly)
            self.polyList.append(poly)

    def remove_poly(self, poly):
        self.remove_child(poly)
        self.polyList.remove(poly)
        self.remove_child(poly.textXMin)
        self.remove_child(poly.textXMax)
        self.remove_child(poly.textYVal)

    @my_profiler.profile
    def draw_robot(self, x, y, direction):
        if self.robot_element is not None:
            self.remove_child(self.robot_element)
        if self.robot_dir_element is not None:
            self.remove_child(self.robot_dir_element)
        if self.wheels is not None:
            for wheel in self.wheels:
                self.remove_child(wheel)

        robot_width = 1.5 * self.scale_x
        robot_length = 4 * self.scale_y
        wheel_radius = 0.5 * self.scale_x

        # draw robot body
        robot = gui.SvgRectangle(x - robot_width / 2, y - robot_length / 2, robot_width, robot_length)
        robot.set_fill('orange')
        self.append(robot)
        self.robot_element = robot

        # draw robot direction
        arrow_length = robot_length / 8 * self.scale_y
        arrow = gui.SvgPolyline()
        arrow.add_coord(x, y)
        arrow.add_coord(x + arrow_length * math.cos(direction), y + arrow_length * math.sin(direction))
        arrow.set_stroke(1, 'red')
        self.append(arrow)
        self.robot_dir_element = arrow

        # draw robot wheels
        self.wheels = []
        wheel_offsets = [(robot_width / 2, robot_length / 3), (-robot_width / 2, robot_length / 3),
                         (robot_width / 2, -robot_length / 3), (-robot_width / 2, -robot_length / 3)]
        for i, offset in enumerate(wheel_offsets):
            wheel_x = x + offset[0] - wheel_radius / 2
            wheel_y = y + offset[1] - wheel_radius / 2
            wheel_color = 'green' if self.wheel_status[i] else 'red'
            wheel = gui.SvgRectangle(wheel_x, wheel_y, wheel_radius, wheel_radius)
            wheel.set_fill(wheel_color)
            self.append(wheel)
            self.wheels.append(wheel)

    def update_road_visor_detections(self, detections):
        if detections is None:
            return

        self.road_visor_detections = detections

    def draw_road_visor_detections(self, centerX, centerY):
        if self.road_visor_detections is None:
            return

        if self.road_visor_detections_svg is not None:
            for element in self.road_visor_detections_svg:
                self.remove_child(element)
        self.road_visor_detections_svg = []

        for det in self.road_visor_detections:
            for segment in det:
                point1, point2 = segment[0], segment[1]

                x1, y1 = self.convert_coordinates(point1, centerX, centerY)
                x2, y2 = self.convert_coordinates(point2, centerX, centerY)

                y1 = self.height - y1
                y2 = self.height - y2

                line = gui.SvgPolyline()
                line.add_coord(x1, y1)
                line.add_coord(x2, y2)
                line.set_stroke(1, 'green')

                self.road_visor_detections_svg.append(line)
                self.append(line)

    def convert_coordinates(self, point, translateX, translateY):
        """Converts 3D coordinates to 2D for SVG display and applies translation."""
        x, y = point[0] * self.scale_x + translateX, point[1] * self.scale_y + translateY
        return x, y

    @my_profiler.profile
    def render(self):
        # choose ROI (region of interest)
        self.set_viewbox(-self.plot_inner_border, -self.plot_inner_border, self.width + self.plot_inner_border * 2,
                         self.height + self.plot_inner_border * 2)

        centerX = self.width / 2.0
        centerY = self.height / 2.0

        # draw real path
        need_make_trasform = None
        if len(self.polyList) > 1 and len(self.polyList[1].plotData.coordsX) > 0:
            poly = self.polyList[1]
            pointX = (poly.plotData.coordsX[-1]) * self.scale_x
            pointY = (poly.plotData.coordsY[-1]) * self.scale_y

            # Calculate the required displacement to center the last point
            translateX = centerX - pointX
            translateY = centerY + pointY

            if len(poly.plotData.coordsX) < 2:
                yaw = -90  # straight ahead
            else:
                diffX = poly.plotData.coordsX[-1] - poly.plotData.coordsX[-2]
                diffY = poly.plotData.coordsY[-1] - poly.plotData.coordsY[-2]
                # Calculate the angle in radians and convert it to degrees
                diff = math.degrees(math.atan2(diffY, diffX))
                yaw = -90 + diff  # so that the robot always looks forward

            # Apply the offset to all elements of the graph, including the polygon
            need_make_trasform = 'rotate({}, {}, {}) translate({},{}) scale({}, {})'.format(
                yaw, centerX, centerY, translateX, translateY, self.scale_x, -1 * self.scale_y)
            poly.attributes['transform'] = need_make_trasform

        # draw GT path
        if need_make_trasform is not None:
            self.polyList[0].attributes['transform'] = need_make_trasform

        # draw detections
        self.draw_road_visor_detections(centerX, centerY)
        # choose ROI (region of interest)
        self.set_viewbox(-self.plot_inner_border, -self.plot_inner_border, self.width + self.plot_inner_border * 2,
                         self.height + self.plot_inner_border * 2)

        centerX = self.width / 2.0
        centerY = self.height / 2.0

        # draw real path
        need_make_trasform = None
        if len(self.polyList) > 1 and len(self.polyList[1].plotData.coordsX) > 0:
            poly = self.polyList[1]
            pointX = (poly.plotData.coordsX[-1]) * self.scale_x
            pointY = (poly.plotData.coordsY[-1]) * self.scale_y

            # Calculate the required displacement to center the last point
            translateX = centerX - pointX
            translateY = centerY + pointY

            if len(poly.plotData.coordsX) < 2:
                yaw = -90  # straight ahead
            else:
                diffX = poly.plotData.coordsX[-1] - poly.plotData.coordsX[-2]
                diffY = poly.plotData.coordsY[-1] - poly.plotData.coordsY[-2]
                # Calculate the angle in radians and convert it to degrees
                diff = math.degrees(math.atan2(diffY, diffX))
                yaw = -90 + diff  # so that the robot always looks forward

            # Apply the offset to all elements of the graph, including the polygon
            need_make_trasform = 'rotate({}, {}, {}) translate({},{}) scale({}, {})'.format(
                yaw, centerX, centerY, translateX, translateY, self.scale_x, -1 * self.scale_y)
            poly.attributes['transform'] = need_make_trasform


        # draw GT path
        if need_make_trasform is not None:
            self.polyList[0].attributes['transform'] = need_make_trasform

        # draw detections
        self.draw_road_visor_detections(centerX, centerY)

        # draw robot
        direction_up = math.radians(270)
        self.draw_robot(centerX, centerY, direction_up)


class NavigatorServer(App):
    my_instance = None

    def __init__(self, *args):
        # presets
        self.real_driving = None
        self.gt_driving = None
        self.svgplot = None

        # main methods applying
        super(NavigatorServer, self).__init__(*args)
        NavigatorServer.my_instance = self
        self.id = uuid.uuid4()

    @staticmethod
    def get_my_instance():
        return NavigatorServer.my_instance

    def get_gt_path(self, csv_path):
        data = pd.read_csv(csv_path, sep='\t')
        gt_path = [(0, 0)]
        detections_right_pole = []
        for index, row in data.iterrows():
            angle = degrees_to_radians(row['raw_angle'])
            dx = row['passed'] * np.cos(angle)
            dy = row['passed'] * np.sin(angle)
            new_gt_point = (gt_path[-1][0] + dx, gt_path[-1][1] + dy)
            detections_right_pole.append(row['right_pole_detection'])
            gt_path.append(new_gt_point)

        return gt_path

    def set_GT_trajectory(self, path):
        if len(path) < 5:
            return
        while self.gt_driving is None:
            pass
        gt_path = self.get_gt_path(path)
        self.gt_driving.add_full_trajectory(gt_path)

    def update_real_path(self, x, y):
        while self.real_driving is None:
            pass
        self.real_driving.add_move(x, y)

    def update_road_visor_detections(self, detections):
        while self.svgplot is None:
            pass
        self.svgplot.update_road_visor_detections(detections)


    def main(self):
        # robot drive settings
        self.draw_only_n = 100

        # plot settings
        self.wid = gui.VBox(margin='0px auto')
        self.svgplot = RobotPlot(600, 600)
        self.svgplot.style['margin'] = '10px'
        self.real_driving = SvgComposedPoly(self.draw_only_n, 0.3, 'rgba(0,0,255,0.8)')
        self.gt_driving = SvgComposedPoly(9999999, 0.3, 'rgba(0,200,0,1.9)')
        self.svgplot.append_poly([self.gt_driving, self.real_driving])
        self.wid.append(self.svgplot)

        self.stop_flag = False
        self.count = 0
        self.start_draw()

        # interface
        bt_zoom_in = gui.Button("Zoom + ")
        bt_zoom_out = gui.Button("Zoom - ")
        bt_zoom_in.onclick.do(self.zoom_in)
        bt_zoom_out.onclick.do(self.zoom_out)
        self.wid.append(bt_zoom_in)
        self.wid.append(bt_zoom_out)

        # returning the root widget
        NavigatorServer.ready = True
        return self.wid

    def on_close(self):
        self.stop_flag = True
        super(NavigatorServer, self).on_close()

    def zoom_out(self, emitter):
        scale_factor_x = 0.5
        scale_factor_y = 0.5
        self.svgplot.scale(scale_factor_x, scale_factor_y)

    def zoom_in(self, emitter):
        scale_factor_x = 2.0
        scale_factor_y = 2.0
        self.svgplot.scale(scale_factor_x, scale_factor_y)

    def start_draw(self):
        with self.update_lock:
            self.svgplot.render()
            if not self.stop_flag:
                Timer(0.1, self.start_draw).start()


def start_navigator_app():
    import socket
    def check_port_available(host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind((host, port))
                return True  # accessible
            except socket.error:
                return False  # forbidden

    if check_port_available('0.0.0.0', 1491):
        start(NavigatorServer, address='0.0.0.0', port=1491, update_interval=0.1, multiple_instance=False)
