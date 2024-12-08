path_planner_pso_node.py
  reference_path.csv 파일에서 경로 데이터를 읽어온다.
  PSO 알고리즘을 사용하여 최적의 경로를 계산힌다.
  계산된 최적 경로를 ROS의 /optimized_path라는 토픽에 publish한다.
  (optimized_path 토픽에 nav_msgs/Path 메시지를 발행)



vehicle_controller_node.py
  path_planner_pso_node에서 계산된 최적 경로 subscribe + 차량 현재상태 subscribe 
  차량 상태와 최적 경로를 기반으로 쓰로틀, 조향각, break 값을 계산하여 ROS의 ontrol_msg로 제어 명령을 전송


carla_integration_node.py
  vehicle_controller_node에서 발행한 차량 제어 명령(control_msg)을 구독하고,
   CARLA에서 사용하는 제어 메시지 형식(carla_msgs/CarlaEgoVehicleControl)으로 변환하여 CARLA 시뮬레이터로 전송




GPT기반 작동 순서 요약
작동 순서

    path_planner_pso_node.py:
        프로젝트가 실행되면, 먼저 reference_path.csv를 읽어 경로 데이터를 로드.
        PSO 알고리즘을 통해 최적 경로를 계산.
        최적 경로를 /optimized_path 토픽에 publish.

    vehicle_controller_node.py:
        path_planner_pso_node에서 발행한 /optimized_path 토픽을 구독하여 최적 경로 데이터를 수신.
        /mobile_system_control/ego_vehicle에서 차량의 현재 상태 데이터를 구독.
        차량의 현재 상태와 경로를 기반으로 제어 명령(throttle, steering, brake)을 계산.
        계산된 명령을 /mobile_system_control/control_msg 토픽에 publish.

    carla_integration_node.py:
        vehicle_controller_node에서 발행한 /mobile_system_control/control_msg 토픽을 구독하여 제어 명령을 수신.
        이를 CARLA에서 사용하는 제어 메시지 형식으로 변환.
        변환된 명령을 CARLA로 전송하여 차량을 제어.



