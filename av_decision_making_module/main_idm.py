import csv
import math
import numpy as np
import time
import matplotlib
matplotlib.use('Agg')

import spider
import spider.visualize as vis
from spider.planner_zoo.IDMPlanner import IDMPlanner

from mrav.mcity_mr_av import (
    MRAVTemplateMcity,
)  # This Python class is a basic component for any developed AV decision-making module and the user should inherit from it.
from spider_interface import extract_route, wrap_observation, convert_trajectory

import pdb


class AVDecisionMakingModule(MRAVTemplateMcity):
    """This is an example AV decision making module that reads a logged trajectory from a file and follows it."""

    def initialize_av_algorithm(self):
        """This function will be used to initialize the developed AV ddecision-making module. In this example, we read the predefined trajectory from a file."""
        
        self.route_filename = '/app/av_decision_making_module/initial_information/route.csv'
        self.local_map = extract_route(self.route_filename)

        # self.start_planner = DummyPlanner({
        #     "steps": 30,
        #     "dt": 0.1,
        #     "ego_veh_length": 5.0,
        #     "ego_veh_width": 1.8,
        #     "target_speed": 5/3.6,
        #     "max_acceleration": 1.0,
        #     "max_deceleration": 7.0,
        # })

        # self.fallback_planner = FallbackPlanner({
        #     "steps": 30,
        #     "dt": 0.1,
        #     "ego_veh_length": 5.0,
        #     "ego_veh_width": 1.8,
        #     "min_TTC": 2.0,
        #     "max_deceleration": 7.0,
        # })

        self.planner = IDMPlanner({
            "steps": 40,
            "dt": 0.1,
            "ego_veh_length": 5.0,
            "ego_veh_width": 1.8,
            "max_speed": 21/3.6,
            "min_speed": 0,
            "max_acceleration": 1.0,
            "max_deceleration": 7.0,
            "max_lateral_jerk": 1.0,
            "max_longitudinal_jerk": 1.0,
            # # "max_centripetal_acceleration" : 100,
            # "max_curvature": 2.0,
            # "safe_distance": (1.0, 0.2),  # 目前好像还没用
            # "end_s_candidates": (5,20,30),
            # "end_l_candidates": (-3.0, 0.5, 0, 0.5, 3.0),# (-0.8,0,0.8), # s,d采样生成横向轨迹 (-3.5, 0, 3.5), #
            # "end_v_candidates": tuple(i*20/3.6/7 for i in range(8)), # 改这一项的时候，要连着限速一起改了
            # "end_T_candidates": (2,4,8), # s_dot, T采样生成纵向轨迹

            # "print_info": True,

            # "constraint_flags": {
            #     spider.CONSTRIANT_SPEED_UB,
            #     spider.CONSTRIANT_SPEED_LB,
            #     spider.CONSTRIANT_ACCELERATION,
            #     spider.CONSTRIANT_DECELERATION,
            #     spider.CONSTRIANT_CURVATURE,
            #     spider.CONSTRIANT_LATERAL_JERK,
            #     spider.CONSTRIANT_LONGITUDINAL_JERK
            # },
        })
        self.planner.set_local_map(self.local_map)

        ###### replan######
        from spider.utils.collision import BoxCollisionChecker
        self.last_trajectory = None
        self._replan_interval_count = 0
        self.max_replan_interval = 10 # frames
        self.collision_checker = BoxCollisionChecker(5.0, 1.8)
        ###################

        self.last_acc = None

        self.VISUALIZE = True

    def trigger_replanning(self, tracking_box_list) -> bool:
        # judge whether to replan
        if self.last_trajectory is None or self._replan_interval_count > self.max_replan_interval:
            self._replan_interval_count = 0
            return True
        
        # check if it is going to collide
        tracking_box_list.predict(self.last_trajectory.t)
        if self.collision_checker.check_trajectory(self.last_trajectory, tracking_box_list): # if collide
            return True
        
        return False
    
    def clip_store_traj(self,trajectory):
        self._replan_interval_count += 1
        self.last_trajectory = trajectory[1:]
        

    def derive_planning_result(self, step_info):
        """This function will be used to compute the planning results based on the observation from "step_info". In this example, we find the closest point in the predefined trajectory and return the next waypoint as the planning results."""

        # parse the step_info
        obs = ego_state, tbox_list, _ =  wrap_observation(step_info)

        if self.trigger_replanning(tbox_list):
            # print('00')
            # if step_info['av_info']['speed_long'] < 0.5:
            #     traj = self.start_planner.plan(*obs)
            # else:
            traj = self.planner.plan(*obs)
            # print('11')
        else:
            traj = self.last_trajectory
        
        planning_result = convert_trajectory(traj)
        # pdb.set_trace()
        ### constraint ###
        if self.last_acc is not None:
            current_speed = math.sqrt(step_info['av_info']['speed_long'] ** 2 + step_info['av_info']['speed_lat'] ** 2)
            planning_result['next_speed'] = min([planning_result['next_speed'],abs(self.last_acc+0.1) * 0.1 + current_speed])
            planning_result['next_speed'] = max([planning_result['next_speed'],-abs(self.last_acc+0.1) * 0.1 + current_speed])
            displacement = 0.5*(current_speed + planning_result['next_speed']) * 0.1
            dx = planning_result['next_x'] - step_info['av_info']['x']
            dy = planning_result['next_y'] - step_info['av_info']['y']
            distance = np.linalg.norm([dx,dy])
            planning_result['next_x'] = displacement * dx / distance + step_info['av_info']['x']
            planning_result['next_y'] = displacement * dy / distance + step_info['av_info']['y']
        self.last_acc = math.sqrt(step_info['av_info']['accel_long'] ** 2 + step_info['av_info']['accel_lat'] ** 2)
        
        ####### debug #######
        print(max(traj.a))
        try:
            print(traj.s_2dot[:2])
            print(traj.l_2prime[:2])
            print(max(traj.s_3dot))
            print(max(traj.l_3dot))
        except Exception:
            pass
        
        if self.VISUALIZE:
            vis.figure(0)
            vis.cla()
            # vis.lazy_draw(*obs, traj)
            vis.draw_ego_vehicle(obs[0])
            vis.draw_trackingbox_list(obs[1])
            vis.draw_local_map(self.local_map)
            vis.draw_trajectory(traj, 'g.-', show_footprint=False)
            vis.plot(traj.x[1], traj.y[1], 'r*')
            # csp = self.local_map.lanes[0].centerline_csp
            # csp_line =  csp(np.arange(1.0, 50.0, 0.2),0)
            # vis.draw_polyline(csp_line, 'k-')


            vis.ego_centric_view(obs[0].x(), obs[0].y(), [-10, 10], [-10,10])
            vis.savefig('/app/av_decision_making_module/temp.png')
        
        self.clip_store_traj(traj)
        
        print(step_info['av_info'])
        print(planning_result)
        print(ego_state.to_dict(), ego_state.a())
        return planning_result


# Create an instance of the AV decision-making module and run it
av_decision_making_module = AVDecisionMakingModule()
av_decision_making_module.run()
