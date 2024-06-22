## 1.Usage
```bash
sudo docker compose -f docker-compose-development.yml up -d
```

```bash
sudo docker exec -it avalgo /bin/bash
```

To run 2000 iterations of testing, run
```bash
bash main.sh
```

else if you want to run the test for only one time, run
```bash
python main.py
```

For visualization, run
```bash
python3 /app/av_decision_making_module/visualization_tool.py --trajectory_path output/trajectory_data/mcity_av_challenge_results/raw_data/0_1188
```



## 2. observation & action definition
step_info: dict \
dict_keys(['sim_time', 'av_info', 'tls_info', 'av_context_info', 'terasim_status', 'external_step_info'])
\
\
example:
```json
{
    'sim_time': '902.1', 
    'av_info': {
        'x': 124.7523096154207, 
        'y': 15.917477574911015, 
        'length': 5.0, 
        'width': 1.8, 
        'height': 1.5, 
        'orientation': 0.3267946478091318, 
        'yaw_rate': 0, 
        'speed_long': 0.0, 
        'speed_lat': 0.0, 
        'accel_long': 0.0, 
        'accel_lat': 0, 
        'edge_id': 'EG_35_1_14', 
        'lane_id': 'EG_35_1_14_0', 
        'sim_time': 902.1, 
        'leading_info': {
            'is_leading_cav': False, 
            'distance': None
        }
    }, 
    'tls_info': {
        'next_tls_id': 'NODE_17', 
        'distance_to_next_tls': 645.6, 
        'next_tls_state': 'r', 
        'tls_info': {
            'NODE_11': {'tls_state': 'rGgrrGgr'}, 
            'NODE_12': {'tls_state': 'rrrG'}, 
            'NODE_17': {'tls_state': 'rrryrrrry'}, 
            'NODE_18': {'tls_state': 'rrGgrrGg'}, 
            'NODE_23': {'tls_state': 'rrGr'}, 
            'NODE_24': {'tls_state': 'rrGr'}
        }
    }, 
    'av_context_info': {
        'VEHICLE_ID':[same structure as av info]

    }, 
    'terasim_status': True, 
    'external_step_info': None
}
```


planning_result , returned by the  derive_planning_result  method, is a dictionary with the following keys:

'timestamp': <float> Current timestamp (seconds).

'time_resolution': <float> Time resolution of the planning results (seconds).Currently, we only support 0.1s time resolution. So, please make sure the timeresolution of your planning results is 0.1s.

'next_x': <float> The x coordinate of the vehicle center in the next 0.1s within the SUMO coordinate system (meters).

'next_y': <float> The next y coordinate of the vehicle center in the next 0.1s within the SUMO coordinate system (meters).

'next_speed': <float> The desired overall velocity in the next 0.1s (meters persecond).

'next_orientation': <float> The planned orientation in the next 0.1s (in radians)
