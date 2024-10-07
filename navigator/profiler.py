from functools import wraps
import time
import numpy as np


class Profiler:
    """
    example of usage:

    profiler = Profiler()
    profiler.set_logger(logger)
    profiler.set_output_file('path/to/your/log_file.txt')

    class MyNode():
        @profiler.profile
        def myMethod(self):
            pass
    """

    def __init__(self):
        self.logger = None
        self.output_file = None
        self.stats = {}
        self.save_interval = 60  # Save stats every 60 seconds

    def set_logger(self, logger):
        self.logger = logger

    def set_output_file(self, file_path):
        self.output_file = file_path
        with open(self.output_file, 'w') as f:
            header = "{:<30} {:<10} {:<10} {:<10} {:<10} {:<15}\n".format(
                "Method", "Min", "Max", "Mean", "Variance", "Period (sec)"
            )
            f.write(header)
            f.write('-' * len(header) + '\n')

    def _log_message(self, message):
        if self.logger:
            self.logger.info(message, 60)
        else:
            print(message)

    def _update_stats(self, func_name, duration):
        now = time.time()
        if func_name not in self.stats:
            self.stats[func_name] = {'durations': [], 'last_save': now}

        self.stats[func_name]['durations'].append(duration)

        if now - self.stats[func_name]['last_save'] >= self.save_interval:
            self._dump_stats(func_name, now - self.stats[func_name]['last_save'])
            self.stats[func_name]['last_save'] = now

    def _dump_stats(self, func_name, period):
        durations = np.array(self.stats[func_name]['durations'])
        if durations.size == 0:
            return

        min_duration = np.min(durations)
        max_duration = np.max(durations)
        mean_duration = np.mean(durations)
        var_duration = np.var(durations)

        stats_message = "{:<30} {:<10.2f} {:<10.2f} {:<10.2f} {:<10.2f} {:<15.2f}\n".format(
            func_name, min_duration, max_duration, mean_duration, var_duration, period
        )

        if self.output_file:
            with open(self.output_file, 'a') as f:
                f.write(stats_message)

        self._log_message(stats_message)
        self.stats[func_name]['durations'] = []

    def profile(self, func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            duration = (end_time - start_time) * 1000

            self._update_stats(func.__name__, duration)

            return result

        return wrapper
