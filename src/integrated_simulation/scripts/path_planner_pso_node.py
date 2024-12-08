#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# CSV 파일 경로
REFERENCE_PATH_FILE = "/home/chohyunjun/catkin_ws/src/integrated_simulation/data/reference_path.csv"

# 경로 데이터 로드 함수
def load_reference_path(file_path):
    try:
        # CSV 파일 읽기 및 전처리
        data = pd.read_csv(file_path, header=None, delimiter=",", dtype=float)
        points = data.values[:, :2]  # x, y 좌표만 추출
        return points
    except Exception as e:
        rospy.logerr(f"Error loading reference path: {e}")
        return None

# 목적 함수
def objective_function(path, reference_path):
    # 경로와 참조 경로 간 거리 계산 (간단한 Euclidean 거리)
    distance = np.sum([np.linalg.norm(p - rp) for p, rp in zip(path, reference_path)])
    return distance

# PSO 알고리즘
def pso_optimize(reference_path):
    num_particles = 30
    max_iterations = 50
    path_length = len(reference_path)

    particles = [reference_path + np.random.uniform(-0.5, 0.5, reference_path.shape) for _ in range(num_particles)]
    velocities = [np.random.uniform(-0.1, 0.1, reference_path.shape) for _ in range(num_particles)]
    personal_best = particles[:]
    global_best = min(personal_best, key=lambda p: objective_function(p, reference_path))

    for _ in range(max_iterations):
        for i in range(num_particles):
            r1, r2 = np.random.rand(), np.random.rand()
            velocities[i] = 0.5 * velocities[i] + \
                            1.5 * r1 * (personal_best[i] - particles[i]) + \
                            1.5 * r2 * (global_best - particles[i])
            particles[i] += velocities[i]
            if objective_function(particles[i], reference_path) < objective_function(personal_best[i], reference_path):
                personal_best[i] = particles[i]
        global_best = min(personal_best, key=lambda p: objective_function(p, reference_path))
    return global_best

def path_planner():
    rospy.init_node('path_planner_node', anonymous=True)
    pub = rospy.Publisher('/optimized_path', Path, queue_size=10)
    rate = rospy.Rate(10)

    reference_path = load_reference_path(REFERENCE_PATH_FILE)
    if reference_path is None:
        rospy.logerr("Failed to load reference path. Exiting node.")
        return

    while not rospy.is_shutdown():
        optimized_path = pso_optimize(reference_path)
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for point in optimized_path:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = point
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        path_planner()
    except rospy.ROSInterruptException:
        pass
