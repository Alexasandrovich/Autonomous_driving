import random
import time

from demo import start_plots_app, PlotsServer

if __name__ == "__main__":
    start_plots_app(14229)
    start_plots_app(14228)
    while PlotsServer.get_my_instance() is None:
        time.sleep(0.1)
    print("start_navigator_app - SUCCESS!")

    instance = PlotsServer.get_my_instance()

    # Добавляем несколько графиков
    instance.add_plot('plot1', width=700, height=400, window_size=100)
    instance.add_plot('plot2', width=700, height=400, window_size=100)

    # Устанавливаем параметры для каждого графика
    instance.set_labels('plot1', title="График 1", x_label="Время", y_label="Значение")
    instance.set_axis_limits('plot1', y_min=0, y_max=1)

    instance.set_labels('plot2', title="График 2", x_label="Время", y_label="Значение")
    instance.set_axis_limits('plot2', y_min=-1, y_max=1)

    base1 = [0, 0.0]
    base2 = [0, 0.0]

    while True:
        time.sleep(0.1)
        # Обновляем данные для графика 1
        base1[0] += 1
        base1[1] = random.random()
        instance.update_coords('plot1', base1[0], base1[1])

        # Обновляем данные для графика 2
        base2[0] += 1
        base2[1] = random.uniform(-1, 1)
        instance.update_coords('plot2', base2[0], base2[1])