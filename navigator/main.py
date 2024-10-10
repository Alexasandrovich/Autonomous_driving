import random
import threading
import time

from demo import start_navigator_app, NavigatorServer


if __name__ == "__main__":
    threading.Thread(target=start_navigator_app).start()
    while NavigatorServer.get_my_instance() is None:
        pass
    print("start_navigator_app - SUCCESS!")

    instance = NavigatorServer.get_my_instance()

    print(instance.set_GT_trajectory("trajectories/2024_03_14_16_42_41.csv"))
    print("set_GT_trajectory - SUCCESS!")

    road_visor_detection = [[[[-1.9472453594207764, 2.320467472076416, 0.0],
                              [-6.384272575378418, 25.27199935913086, 0.0]],

                             [[1.627353310585022, 1.9412848949432373, 0.0],
                              [-2.849727153778076, 25.27199935913086, 0.0]]]]


    gt = instance.get_gt_path("trajectories/2024_03_14_16_42_41.csv")
    for step, pt in enumerate(gt):
        time.sleep(0.1)
        instance.update_road_visor_detections(road_visor_detection)
        distance_to_maneuver = random.uniform(50, 200)
        maneuver = random.choice(['left', 'right', 'straight', 'shift_right_to_left', 'shift_left_to_right'])
        instance.update_maneuver(maneuver, distance_to_maneuver)
        instance.update_traj_progres(step / len(gt) * 100)
        print(f"{step / len(gt)}%")
        x = pt[0]
        y = pt[1]
        instance.update_real_path(x, y)