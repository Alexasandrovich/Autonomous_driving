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
        self.robot_down_shift = 250
        self.robot_element = None
        self.wheel_status = [False, False, False, False]
        self.wheels = None
        self.robot_dir_element = None
        self.road_visor_detections = None
        self.road_visor_detections_svg = None
        self.people_detections = None
        self.people_svg = None
        self.critical_zone_svg = None

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

    def draw_robot(self, x, y, direction=math.radians(270)):
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
            wheel_color = 'green' if self.wheel_status[i] else 'orange'
            wheel = gui.SvgRectangle(wheel_x, wheel_y, wheel_radius, wheel_radius)
            wheel.set_fill(wheel_color)
            self.append(wheel)
            self.wheels.append(wheel)

        # Draw critical area in front of the robot
        # self.draw_critical_zone(x, y)

    def draw_critical_zone(self, x, y):
        """Draws a red rectangle indicating the critical zone in front of the robot."""
        if self.critical_zone_svg is not None:
            self.remove_child(self.critical_zone_svg)

        self.critical_zone_svg = gui.SvgRectangle(x - 1.5 * self.scale_x, y - 3 * self.scale_y,
                                                  3 * self.scale_x, 3 * self.scale_y)
        self.critical_zone_svg.set_fill('none')
        self.critical_zone_svg.set_stroke(2, 'red')
        self.append(self.critical_zone_svg)

    def draw_people(self, people_positions, centerX, centerY):
        """Draws detected people as circles relative to the robot."""
        if self.people_svg is not None:
            self.remove_child(self.people_svg)

        if people_positions is None:
            return

        self.people_svg = gui.SvgGroup()

        for person in people_positions:
            # Convert person's coordinates relative to robot's frame of reference
            person_x, person_y = self.convert_coordinates((person[0], person[1]), centerX, centerY)

            # Draw each person as a blue circle
            person_circle = gui.SvgCircle(person_x, person_y, self.scale_x / 3)
            person_circle.set_fill('red')
            self.people_svg.append(person_circle)

        self.append(self.people_svg)

    def update_people_detections(self, people_positions):
        """Updates the positions of detected people and redraws them."""
        if self.people_svg is not None:
            return
        self.people_detections = people_positions

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

                # y1 = centerY + y1
                # y2 = centerY + y2

                line = gui.SvgPolyline()
                line.add_coord(x1, y1)
                line.add_coord(x2, y2)
                line.set_stroke(1, 'green')

                self.road_visor_detections_svg.append(line)
                self.append(line)

    def draw_path(self, centerX, centerY):
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

    def convert_coordinates(self, point, translateX, translateY, direction=math.radians(180)):
        """Converts 2D for SVG display and applies translation, scaling, and rotation."""
        x, y = point[0], point[1]

        # Применяем масштабирование
        x *= self.scale_x
        y *= self.scale_y

        # Применяем отзеркаливание по оси X для учета направления оси координат в SVG (оси X направлены вправо)
        x = -x

        # Применяем поворот относительно направления робота
        x_rotated = x * math.cos(direction) - y * math.sin(direction)
        y_rotated = x * math.sin(direction) + y * math.cos(direction)

        # Применяем сдвиг относительно центра робота
        x_final = x_rotated + translateX
        y_final = y_rotated + translateY
        return x_final, y_final

    def render(self):
        # choose ROI (region of interest)
        self.set_viewbox(-self.plot_inner_border, -self.plot_inner_border, self.width + self.plot_inner_border * 2,
                         self.height + self.plot_inner_border * 2)

        centerX = self.width / 2.0
        centerY = self.height / 2.0 + self.robot_down_shift

        # draw real path
        self.draw_path(centerX, centerY)

        # draw detections
        self.draw_road_visor_detections(centerX, centerY)

        # draw robot
        self.draw_robot(centerX, centerY)

        # draw obstacles
        self.draw_people(self.people_detections, centerX, centerY)

class ManeuverSvg(gui.HBox):
    def __init__(self, width, height):
        super(ManeuverSvg, self).__init__()
        self.width = width
        self.height = height
        self.arrow_svg = gui.Svg(width=self.width + 45, height=self.height)  # Создаём отдельный SVG для стрелки
        self.distance_table = gui.Table(width=self.width, height=self.height / 1.4)  # Создаём таблицу для текста

        # Настраиваем таблицу для текста
        self.distance_table.append_from_list([['Оставшееся расстояние']], fill_title=True)
        self.distance_table.set_style({
            'font-size': '24px',  # Увеличиваем размер шрифта
            'text-align': 'center',
            'border': '2px solid black',
            'margin-top': '10px',
            'width': '100%',
            'padding': '10px',
            'background-color': '#FFFFFF',
        })
        self.set_style({'margin': '0px'})

        # Добавляем стрелку и таблицу в контейнер
        self.append(self.arrow_svg)
        self.append(self.distance_table)

    def update_maneuver(self, maneuver_name, distance_to_maneuver):
        # Очищаем SVG перед добавлением новой стрелки
        self.arrow_svg.empty()

        # Обновляем стрелку в зависимости от манёвра
        if maneuver_name == 'left':
            self.draw_left_arrow()
        elif maneuver_name == 'right':
            self.draw_right_arrow()
        elif maneuver_name == 'straight':
            self.draw_straight_arrow()
        elif maneuver_name == 'shift_left_to_right':
            self.draw_shift_left_to_right()
        elif maneuver_name == 'shift_right_to_left':
            self.draw_shift_right_to_left()

        # Обновляем текст в таблице
        self.distance_table.empty()
        self.distance_table.append_from_list([
            [f'{distance_to_maneuver:.2f} м']
        ])

    # Пример манёвра налево
    def draw_left_arrow(self):
        # Окружение знака (рамка)
        sign = gui.SvgGroup()
        background = gui.SvgRectangle(10, 10, self.width - 20, self.height - 20)
        background.set_fill('#FFFFFF')  # Светло-серый фон
        background.set_stroke(2, 'black')  # Чёрная рамка
        sign.append(background)

        # Стрелка налево
        arrow = gui.SvgPolyline()
        up_line = gui.SvgPolyline()
        mid_x = self.width / 2
        mid_y = self.height / 2
        size = min(self.width, self.height) / 2 - 20

        points_arrow = [
            (mid_x + size / 2, mid_y - size / 2),
            (mid_x - size / 2, mid_y),
            (mid_x + size / 2, mid_y + size / 2)
        ]
        points_up = [
            (mid_x - size / 2, mid_y),
            (mid_x + size, mid_y),
            (mid_x + size, mid_y + size)
        ]

        for point in points_arrow:
            arrow.add_coord(point[0], point[1])

        for point in points_up:
            up_line.add_coord(point[0], point[1])

        arrow.set_stroke(8, 'black')
        arrow.set_fill('none')
        up_line.set_stroke(8, 'black')
        up_line.set_fill('none')
        sign.append(arrow)
        sign.append(up_line)

        # Добавляем стрелку в SVG
        self.arrow_svg.append(sign)

    def draw_right_arrow(self):
        # Окружение знака (рамка)
        sign = gui.SvgGroup()
        background = gui.SvgRectangle(10, 10, self.width - 20, self.height - 20)
        background.set_fill('#FFFFFF')  # Светло-серый фон
        background.set_stroke(2, 'black')  # Чёрная рамка
        sign.append(background)

        # Стрелка направо
        arrow = gui.SvgPolyline()
        up_line = gui.SvgPolyline()
        mid_x = self.width / 2
        mid_y = self.height / 2
        size = min(self.width, self.height) / 2 - 20

        points_arrow = [
            (mid_x - size / 2, mid_y - size / 2),
            (mid_x + size / 2, mid_y),
            (mid_x - size / 2, mid_y + size / 2)
        ]
        points_up = [
            (mid_x + size / 2, mid_y),
            (mid_x - size, mid_y),
            (mid_x - size, mid_y + size)
        ]

        for point in points_arrow:
            arrow.add_coord(point[0], point[1])

        for point in points_up:
            up_line.add_coord(point[0], point[1])

        arrow.set_stroke(8, 'black')
        arrow.set_fill('none')
        up_line.set_stroke(8, 'black')
        up_line.set_fill('none')
        sign.append(arrow)
        sign.append(up_line)

        # Добавляем стрелку в SVG
        self.arrow_svg.append(sign)

    # Манёвр: смещение по кромке слева направо
    def draw_shift_left_to_right(self):
        sign = gui.SvgGroup()
        background = gui.SvgRectangle(10, 10, self.width - 20, self.height - 20)
        background.set_fill('#FFFFFF')  # Светло-серый фон
        background.set_stroke(2, 'black')  # Чёрная рамка
        sign.append(background)

        # Две вертикальные линии
        left_line = gui.SvgPolyline()
        right_line = gui.SvgPolyline()
        snake = gui.SvgPolyline()

        mid_x = self.width / 2
        mid_y = self.height / 2
        size = min(self.width, self.height) / 2 - 20

        # Координаты для вертикальных линий
        points_left_line = [
            (mid_x - size, 10),
            (mid_x - size, self.height - 10)
        ]
        points_right_line = [
            (mid_x + size, 10),
            (mid_x + size, self.height - 10)
        ]

        # Змейка: слева направо
        points_snake = [
            (mid_x - size + 10, self.height - 10),  # Старт на нижней левой линии
            (mid_x - size + 10, mid_y + size / 2),  # Вверх до середины
            (mid_x - size + 40, mid_y + size / 2),  # Переход немного вправо
            (mid_x - size + 40, mid_y - size),  # Вверх
        ]

        # Маленькая стрелка
        arrow = gui.SvgPolyline()
        points_arrow = [
            (mid_x - size / 2 + 10, mid_y - size / 2),
            (mid_x + 10, mid_y - size / 2 - size / 2),
            (mid_x + size / 2 + 10, mid_y - size / 2)
        ]

        # Добавляем координаты для линий и змейки
        for point in points_left_line:
            left_line.add_coord(point[0], point[1])

        for point in points_right_line:
            right_line.add_coord(point[0], point[1])

        for point in points_snake:
            snake.add_coord(point[0], point[1])

        for point in points_arrow:
            arrow.add_coord(point[0], point[1])

        # Настройка линий
        left_line.set_stroke(8, 'black')
        left_line.set_fill('none')
        right_line.set_stroke(8, 'black')
        right_line.set_fill('none')
        snake.set_stroke(8, 'black')
        snake.set_fill('none')
        arrow.set_stroke(8, 'black')
        arrow.set_fill('none')

        # Добавляем линии, змейку и стрелку
        sign.append(left_line)
        sign.append(right_line)
        sign.append(snake)
        sign.append(arrow)

        # Добавляем знак в SVG
        self.arrow_svg.append(sign)

    # Манёвр: смещение по кромке справа налево
    def draw_shift_right_to_left(self):
        sign = gui.SvgGroup()
        background = gui.SvgRectangle(10, 10, self.width - 20, self.height - 20)
        background.set_fill('#FFFFFF')  # Светло-серый фон
        background.set_stroke(2, 'black')  # Чёрная рамка
        sign.append(background)

        # Две вертикальные линии
        left_line = gui.SvgPolyline()
        right_line = gui.SvgPolyline()
        snake = gui.SvgPolyline()

        mid_x = self.width / 2
        mid_y = self.height / 2
        size = min(self.width, self.height) / 2 - 20

        # Координаты для вертикальных линий
        points_left_line = [
            (mid_x - size, 10),
            (mid_x - size, self.height - 10)
        ]
        points_right_line = [
            (mid_x + size, 10),
            (mid_x + size, self.height - 10)
        ]

        # Змейка: справа налево
        points_snake = [
            (mid_x + size - 10, self.height - 10),  # Старт на нижней правой линии
            (mid_x + size - 10, mid_y + size / 2),  # Вверх до середины
            (mid_x + size - 40, mid_y + size / 2),  # Переход немного влево
            (mid_x + size - 40, mid_y - size),  # Вверх
        ]

        # Маленькая стрелка
        arrow = gui.SvgPolyline()
        points_arrow = [
            (mid_x + size / 2 - 10, mid_y - size / 2),
            (mid_x - 10, mid_y - size / 2 - size / 2),
            (mid_x - size / 2 - 10, mid_y - size / 2)
        ]

        # Добавляем координаты для линий и змейки
        for point in points_left_line:
            left_line.add_coord(point[0], point[1])

        for point in points_right_line:
            right_line.add_coord(point[0], point[1])

        for point in points_snake:
            snake.add_coord(point[0], point[1])

        for point in points_arrow:
            arrow.add_coord(point[0], point[1])

        # Настройка линий
        left_line.set_stroke(8, 'black')
        left_line.set_fill('none')
        right_line.set_stroke(8, 'black')
        right_line.set_fill('none')
        snake.set_stroke(8, 'black')
        snake.set_fill('none')
        arrow.set_stroke(8, 'black')
        arrow.set_fill('none')

        # Добавляем линии, змейку и стрелку
        sign.append(left_line)
        sign.append(right_line)
        sign.append(snake)
        sign.append(arrow)

        # Добавляем знак в SVG
        self.arrow_svg.append(sign)

    def draw_straight_arrow(self):
        # Окружение знака (рамка)
        sign = gui.SvgGroup()
        background = gui.SvgRectangle(10, 10, self.width - 20, self.height - 20)
        background.set_fill('#FFFFFF')  # Светло-серый фон
        background.set_stroke(2, 'black')  # Чёрная рамка
        sign.append(background)

        # Прямая стрелка вверх
        arrow = gui.SvgPolyline()
        up_line = gui.SvgPolyline()
        mid_x = self.width / 2
        mid_y = self.height / 2
        size = min(self.width, self.height) / 2 - 20

        points_arrow = [
            (mid_x - size / 2, mid_y),
            (mid_x, mid_y - size / 2),
            (mid_x + size / 2, mid_y)
        ]
        points_up = [
            (mid_x, mid_y + size),
            (mid_x, mid_y - size / 2)
        ]

        for point in points_arrow:
            arrow.add_coord(point[0], point[1])

        for point in points_up:
            up_line.add_coord(point[0], point[1])

        arrow.set_stroke(8, 'black')
        arrow.set_fill('none')
        up_line.set_stroke(8, 'black')
        up_line.set_fill('none')
        sign.append(arrow)
        sign.append(up_line)

        # Добавляем стрелку в SVG
        self.arrow_svg.append(sign)

class ProgressBar(gui.HBox):
    def __init__(self, width, height):
        super(ProgressBar, self).__init__()
        self.width = width
        self.height = height

        # Создаём рамку для отображения процента
        self.progress_table = gui.Table(width='100%')

        # Настраиваем таблицу для отображения текста
        self.progress_table.append_from_list([['Пройденный маршрут']], fill_title=True)
        self.progress_table.set_style({
            'font-size': '24px',
            'text-align': 'center',
            'border': '2px solid black',  # Чёрная рамка
            'padding': '5px',
            'margin': '-9px',
            'color': 'black',  # Чёрный цвет текста
            'background-color': '#D3D3D3',  # Светло-серый фон
        })

        # Добавляем таблицу в контейнер
        self.append(self.progress_table)

    def update_progress(self, progress_percentage):
        # Обновляем текст с процентом прохождения
        self.progress_table.empty()
        self.progress_table.append_from_list([
            [f'{progress_percentage:.2f}%']
        ])


class MetricsGrid(gui.GridBox):
    def __init__(self, width, height):
        super(MetricsGrid, self).__init__(width=width, height=height)

        # Определяем ASCII-схему сетки для трех колонок и двух строк
        self.set_from_asciiart("""
            | LK     | IS_ROAD  | RK     |
            | lk_val | is_road  | rk_val |
        """, 10, 10)

        # Добавляем стили для текста и рамок
        cell_style = {
            'text-align': 'center',
            'font-weight': 'bold',
            'border': '1px solid black',
            'padding': '5px',
        }

        # Создаем виджеты для заголовков и значений
        self.lk_label = gui.Label('LK', style=cell_style)
        self.is_road_label = gui.Label('IS_ROAD', style=cell_style)
        self.rk_label = gui.Label('RK', style=cell_style)

        self.lk_value_label = gui.Label('0.00', style=cell_style)
        self.is_road_value_label = gui.Label('False', style=cell_style)
        self.rk_value_label = gui.Label('0.00', style=cell_style)

        # Добавляем виджеты в контейнер
        self.append({
            'LK': self.lk_label,
            'IS_ROAD': self.is_road_label,
            'RK': self.rk_label,
            'lk_val': self.lk_value_label,
            'is_road': self.is_road_value_label,
            'rk_val': self.rk_value_label,
        })

    # Функция для обновления метрик
    def update_metrics(self, lk_value, is_road_value, rk_value):
        self.lk_value_label.set_text(f'{lk_value:.2f}')
        self.is_road_value_label.set_text(str(is_road_value))
        self.rk_value_label.set_text(f'{rk_value:.2f}')


class ZoomControl(gui.HBox):
    def __init__(self, zoom_in_callback, zoom_out_callback):
        super(ZoomControl, self).__init__(margin='0px')

        # Кнопка увеличения
        self.bt_zoom_in = gui.Button("+", width=100, height=50)
        self.bt_zoom_in.set_style({
            'font-size': '30px',
            'margin': '0px',
            'color': 'black',
            'background-color': '#FFFFFF',
            'border': '2px solid black'
        })

        # Кнопка уменьшения
        self.bt_zoom_out = gui.Button("-", width=100, height=50)
        self.bt_zoom_out.set_style({
            'font-size': '30px',
            'margin': '0px',
            'color': 'black',
            'background-color': '#FFFFFF',
            'border': '2px solid black'
        })

        # Привязка событий к кнопкам
        self.bt_zoom_in.onclick.do(zoom_in_callback)
        self.bt_zoom_out.onclick.do(zoom_out_callback)

        # Добавляем кнопки в контейнер
        self.append(self.bt_zoom_out)
        self.append(self.bt_zoom_in)


class NavigatorServer(App):
    my_instance = None

    def __init__(self, *args):
        # presets
        self.real_driving = None
        self.gt_driving = None
        self.svgplot = None
        self.metrics_grid = None

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

    def update_maneuver(self, maneuver, distance):
        while self.svg_maneuvers is None:
            pass
        self.svg_maneuvers.update_maneuver(maneuver, distance)

    def update_traj_progres(self, percentage):
        while self.progress_widget is None:
            pass
        self.progress_widget.update_progress(percentage)

    def update_people_detections(self, people_positions):
        """Updates the detection of people by calling RobotPlot's update method."""
        while self.svgplot is None:
            pass
        self.svgplot.update_people_detections(people_positions)

    def update_metrics(self, lk_value, is_road_value, rk_value):
        while self.metrics_grid is None:
            pass
        self.metrics_grid.update_metrics(lk_value, is_road_value, rk_value)

    def main(self):
        # Основной контейнер
        self.wid = gui.VBox(margin='0px auto')

        # Добавляем график робота
        self.svgplot = RobotPlot(600, 600)
        self.svgplot.style['margin'] = '10px'
        self.real_driving = SvgComposedPoly(100, 0.3, 'rgba(0,0,255,0.8)')
        self.gt_driving = SvgComposedPoly(9999999, 0.3, 'rgba(0,200,0,1.9)')
        self.svgplot.append_poly([self.gt_driving, self.real_driving])
        self.wid.append(self.svgplot)

        # Добавляем отображение манёвров
        self.svg_maneuvers = ManeuverSvg(width=100, height=100)
        self.svg_maneuvers.update_maneuver("shift_right_to_left", 100.1)
        self.wid.append(self.svg_maneuvers)

        # Добавляем прогресс-бар
        self.progress_widget = ProgressBar(width=200, height=1)
        self.wid.append(self.progress_widget)

        # Добавляем таблицу метрик
        self.metrics_grid = MetricsGrid(width=250, height=50)
        self.metrics_grid.update_metrics(0.0, False, 0.0)
        self.metrics_grid.set_style({
            'margin': '10px auto',  # Центрирование по горизонтали
        })
        self.wid.append(self.metrics_grid)

        # Добавляем управление зумом
        self.zoom_control = ZoomControl(self.zoom_in, self.zoom_out)
        self.wid.append(self.zoom_control)

        self.stop_flag = False
        self.count = 0
        self.start_draw()

        # Устанавливаем готовность сервера
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
