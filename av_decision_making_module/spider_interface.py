import spider
import spider.elements as elm


import math
import csv
import tqdm
import numpy as np
import time


def _get_vxy_from_speed(long_speed, lat_speed, yaw):
    # the positive direction is to the left for the lat speed
    cosyaw, sinyaw = math.cos(yaw), math.sin(yaw)
    vx = cosyaw * long_speed - sinyaw * lat_speed
    vy = sinyaw * long_speed + cosyaw * lat_speed
    return vx, vy

def extract_route(route_filename):
    # csv file: "av_decision_making_module/initial_information/route.csv"
    csv_reader = csv.reader(open(route_filename))
    route_pts = []
    for i, pt in tqdm.tqdm(enumerate(csv_reader), desc='loading the route'):
        if i == 0:
            continue
        route_pts.append([float(x) for x in pt])
    route_pts = np.array(route_pts[:-250])

    #### preprocess the routes (resample & smooth) ####
    from spider.utils.geometry import resample_polyline
    from spider.elements.curves import ParametricCubicSpline
    # 1. resample the centerline to a fixed resolution
    route_pts = resample_polyline(route_pts, resolution=10.0)
    # # 2. use cubic spline to smooth the centerline
    csp = ParametricCubicSpline(route_pts[:,0], route_pts[:,1])
    ss = np.arange(csp.s[0], csp.s[-1], 1.0)
    route_pts = csp(ss)

    local_map = elm.RoutedLocalMap.from_centerlines(
        [route_pts], lane_width=3.5, speed_limit=20.0/3.6)
    return local_map

def wrap_observation(step_info:dict, route_filename=None):
    ###### agents ######
    context_info = step_info['av_context_info']
    obbs_with_vel = []
    for vid in context_info:
        veh:dict = context_info[vid]
        vx, vy = _get_vxy_from_speed(veh['speed_long'], veh['speed_lat'], veh['orientation'])
        obbs_with_vel.append([
            veh['x'], veh['y'], veh['length'], veh['width'], veh['orientation'],
            vx, vy
        ])
    tbox_list = elm.TrackingBoxList.from_obbs(obbs_with_vel)
    
    ###### ego ######
    ego = step_info['av_info']
    vx, vy = _get_vxy_from_speed(ego['speed_long'], ego['speed_lat'], ego['orientation'])
    ax, ay = _get_vxy_from_speed(ego['accel_long'], ego['accel_lat'], ego['orientation'])   
    ego_state = elm.VehicleState.from_kine_states(
        ego['x'], ego['y'], ego['orientation'], 
        vx, vy, ax, ay, ego['length'], ego['width']
    )
    
    ###### map ######
    if route_filename is None:
        local_map = None
    else:
        local_map = extract_route(route_filename)
        # 注意，现在没有加入红绿灯信息！
        
    return ego_state, tbox_list, local_map

def convert_trajectory(trajectory:elm.Trajectory) -> dict:
    # import pdb
    # pdb.set_trace()
    assert trajectory.dt == 0.1, 'The trajectory must have intervals of 0.1'
    planning_result = {
        "timestamp": time.time(),
        "time_resolution": 0.1,
        "next_x": trajectory.x[1],
        "next_y": trajectory.y[1],
        "next_speed": trajectory.v[1],
        "next_orientation": trajectory.heading[1],
    }
    return planning_result

if __name__ == '__main__':
    vx, vy = _get_vxy_from_speed( 0.06613948320364216, -0.7651043701645932, 0.847253513157084)
    ax, ay = _get_vxy_from_speed(0.06613948320364216, -0.7651043701645932, 0.847253513157084)   
    ego_state = elm.VehicleState.from_kine_states(
        0,0, 0.847253513157084, 
        vx, vy, ax, ay, 5,2.
    )
    print(ego_state)