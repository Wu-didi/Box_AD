"如果转角阈值过大或者过小，做一个滤波"
import os 
from pyproj import CRS, Transformer
import sys
import can
from math import radians, cos, sin, asin, sqrt, degrees, atan2
import matplotlib.pyplot as plt
import time
import csv
import numpy as np
import threading
import math
import cvxpy
from can_use import Can_use


# 初始化 WGS84 到 UTM 的投影转换器
wgs84 = CRS("EPSG:4326")
utm_zone_number = 50  # 根据实际情况选择合适的 UTM 区域
utm_crs = CRS(f"EPSG:{32600 + utm_zone_number}")  # 例如，UTM Zone 50N 对应 EPSG:32650
projector_to_utm = Transformer.from_crs(wgs84, utm_crs, always_xy=True)
projector_to_wgs84 = Transformer.from_crs(utm_crs, wgs84, always_xy=True)

def latlon_to_utm(lon, lat):
    """将经纬度转换为 UTM 坐标"""
    x, y = projector_to_utm.transform(lon, lat)
    return x, y

def utm_to_latlon(x, y):
    """将 UTM 坐标转换为经纬度"""
    lon, lat = projector_to_wgs84.transform(x, y)
    return lon, lat

def smooth_yaw_iter(previous_yaw, new_yaw):
    """
    Smooth the yaw angle based on the previous yaw to ensure continuity.

    :param previous_yaw: (float) Previous yaw angle in radians
    :param new_yaw: (float) New yaw angle in radians (not yet normalized)
    :return: (float) Smoothed and normalized yaw angle in radians within (-pi, pi]
    """
    dyaw = new_yaw - previous_yaw

    # 调整 dyaw，使其在 [-pi, pi] 范围内
    dyaw = (dyaw + np.pi) % (2.0 * np.pi) - np.pi

    # 平滑后的 yaw
    smoothed_yaw = previous_yaw + dyaw

    return smoothed_yaw


# 车辆参数
VEHICLE_WIDTH = 1.9   # m
VEHICLE_LENGTH = 4.5  # m
WHEEL_FACTOR = 7.2
manual_triggered = False
stop_record = False
mod_666 = 0
mod_AE = 0

#MPC参数

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 3  # horizon length

# mpc parameters
R = np.diag([0.1, 0.1])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1000.0, 1000.0, 0.01, 2])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 1  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 40  # Search index number

DT = 0.33  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 3  # [m]

MAX_STEER = np.deg2rad(69)  # 63 maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 0.0  # maximum accel [m/ss]


def convert_angle(angle):
    '''angle: deg'''
    if angle < 0:
        angle_2 = angle + 360
        if abs(angle) > abs(angle_2):
            return angle_2
        else:
            return angle
    else:
        angle_2 = angle-360
        if abs(angle) >abs(angle_2):
            return angle_2
        else:
            return angle


def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)
    
    # C = np.zeros(NX)
    # # 在这里正确计算偏置项 C，确保其仅包含常数项
    # # 例如，如果线性化点为 x0, u0，则 C = f(x0, u0) - A @ x0 - B @ u0
    # # 假设 f(x0, u0) 是非线性动力学方程的值
    # # 这里需要根据具体的非线性动力学方程进行计算
    # # 暂时设为零，如果线性模型是基于原点线性化的
    # C[:] = 0.0

    return A, B, C

def pi_2_pi(angle):
    return angle_mod(angle)


def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle.

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle



class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None



def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


# def normalize_angle(angle):
#     normalized_angle = ((angle + 180) % 360) - 180
#     return normalized_angle

def normalize_angle(angle):
    normalized_angle = angle % 360
    if normalized_angle < 0:
        normalized_angle += 360
    return normalized_angle


class ISGSpeedFilter:
    def __init__(self):
        self.isg_sum_filtspd = 0  # 总和
        self.isg_mot_spd_filt = 0  # 滤波后的速度
        self.isg_mot_spd = 0  # 实时速度
        self.MAT_Moto_spd = 0  # 最终输出的速度

    def update_speed(self, isg_mot_spd):
        self.isg_mot_spd = isg_mot_spd
        self.isg_sum_filtspd += self.isg_mot_spd  # 加上当前速度
        self.isg_sum_filtspd -= self.isg_mot_spd_filt  # 减去上一次的滤波结果
        
        # 计算滤波后的速度
        self.isg_mot_spd_filt = self.isg_sum_filtspd / 15
        self.MAT_Moto_spd = self.isg_mot_spd_filt  # 更新最终输出速度

        return self.MAT_Moto_spd

def normalize_angle_rad(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    copied from https://atsushisakai.github.io/PythonRobotics/modules/path_tracking/stanley_control/stanley_control.html
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

class VehicleTrajectoryFollower:
    def __init__(self):
        """
        初始化，读取轨迹点
        :param trajectory_csv: 包含轨迹点的CSV文件路径，轨迹点格式为[经度, 纬度, 航向角度]
        """
        self.dl = 1  # 轨迹点之间的间隔
        self.closest_index = 0
        self.far_index = 45
        self.offset_point = 5 # 最近点朝前挪动点数
        self.previous_turn_angle = 0
        self.max_turn_rate = 6
        self.real_speed = 0
        self.target_ind = 0
        
        # self.cx, self.cy, self.cyaw, self.ck = read_csv(trajectory_csv)
        
        self.cx, self.cy, self.cyaw, self.ck,self.sp = None, None, None, None, None
        # self.cyaw = [one_cyaw-math.radians(95) for one_cyaw in self.cyaw]
        if self.cx is not None:
            self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED)  # 主要作用是根据给定的路径规划和控制要求，计算一个适合的速度轨迹
            self.init_mpc()
        else:
            self.sp = None
            

        
    def init_mpc(self):
        self.goal = [self.cx[-1], self.cy[-1]]
        
        self.state = State(x=self.cx[0], y=self.cy[0], yaw=self.cyaw[0], v=0.0)
        
        # initial yaw compensation
        if self.state.yaw - self.cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - self.cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0
        
        self.target_ind, _ = self.calc_nearest_index(self.state, self.cx, self.cy, self.cyaw, 0)
        self.odelta, self.oa = None, None
        self.cyaw = self.smooth_yaw(self.cyaw)
        
    def update_target_index(self, ego_state, ego_yaw, ego_v):
        self.state.x = ego_state[0]
        self.state.y = ego_state[1]
        # self.state.yaw = math.radians(ego_yaw)
        self.state.yaw = ego_yaw
        self.state.v = ego_v
        self.state.predelta = 0
        self.target_ind, _ = self.calc_nearest_index(self.state, self.cx, self.cy, self.cyaw, 0)
        # print("self target indx",  self.target_ind)
        
    # 得到mpc迭代的结果
    def calculate_turn_angle(self, ego_state, ego_yaw, ego_v):
        # 暂时定死ego_v
        self.real_speed = ego_v
        ego_v = 2.7 
        
        # 计算参考轨迹
        if self.sp is None:
            self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED)  # 主要作用是根据给定的路径规划和控制要求，计算一个适合的速度轨迹
            self.init_mpc()
            
        self.state.x = ego_state[0]
        self.state.y = ego_state[1]
        # self.state.yaw = math.radians(ego_yaw)
        self.state.yaw = ego_yaw
        self.state.v = ego_v
        self.state.predelta = 0


        self.xref, self.target_ind, self.dref = self.calc_ref_trajectory(
            self.state, self.cx, self.cy, self.cyaw, self.ck, self.sp, self.dl, self.target_ind
        )
        # print("reference :", self.xref)

        # 当前状态
        self.x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]
        # 使用MPC控制器计算控制输入
        oa, odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
            self.xref, self.x0, self.dref, self.oa, self.odelta
        )
        # print("x0:         ", self.x0)
        # print("ox:         ", ox)
        # print("x reference:", self.xref[0])
        # print("oy: ", oy)
        # print("y reference:", self.xref[1])
        # print("oyaw: ", oyaw)
        # print("yaw reference:", self.xref[3])
        # print("ov: ", ov)
        # print("v reference:", self.xref[2])
        # print("o delt",odelta)
        # print('------------------------------------')
        
        # plt.figure()
        # plt.plot(ox, oy, c='r')
        # plt.plot(self.cx, self.cy, c='g')
        # plt.plot(self.xref[0],self.xref[1],c='b')
        # plt.axis("equal")
        # plt.scatter(self.state.x, self.state.y, c='y')
        # plt.scatter(self.cx[self.target_ind], self.cy[self.target_ind], c='b')
        # plt.savefig("prediction_points_bank.png")
        
        # print(self.state.x,self.state.y,self.state.v,self.state.yaw)
        # print(self.cx[self.target_ind], self.cy[self.target_ind],self.cyaw[self.target_ind])

        di, ai = 0.0, 0.0
        if odelta is not None:
            di = odelta[0]
            # yaw0 = oyaw[5]
            # yaw1 = oyaw[6]
            # di = yaw1-yaw0
            ai = oa[0]
            yawi = oyaw[0]
            # print("di = ", di, "ai = ", ai)

            if di >= MAX_STEER:
                di = MAX_STEER
            elif di <= -MAX_STEER:
                di = -MAX_STEER

            # di = pi_2_pi(di)
            # print("yawi: ", yawi)
            # print("di = ", di, "ai = ", ai)

            di_deg = math.degrees(di)
            di_deg = convert_angle(di_deg)
            # -pi,pi
            # print("di deg = ", di_deg, "ai = ", ai)
            
            if di_deg*WHEEL_FACTOR>500:
                turn_angle = 500
            elif di_deg*WHEEL_FACTOR<-500:
                turn_angle = -500
            else:
                # print("di_deg in else: ",di_deg)
                turn_angle = di_deg*WHEEL_FACTOR 

            # print("===================turn angle: ",turn_angle)
            # 将转向角转换为度并平滑处理
            # filtered_angle = self.smooth_turn_angle(-turn_angle)
            filtered_angle = -turn_angle
            # print("===================filtered_angle: ", filtered_angle)
            
            # 更新MPC控制器的上一轮控制输入
            self.oa = oa
            self.odelta = odelta
            return filtered_angle
        else:
            print("MPC computation failed, using default values.")
            return 0, 0
                    
    def calc_ref_trajectory(self, state, cx, cy, cyaw, ck, sp, dl, pind):
        xref = np.zeros((NX, T + 1))
        dref = np.zeros((1, T + 1))
        ncourse = len(cx)

        temp_ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, 0)
        self.closest_index = temp_ind
        
        # 根据速度定偏移量
        if self.real_speed >= 8:
            self.offset_point = 5
        else:
            self.offset_point = 3
        
        ind = temp_ind + self.offset_point
        
        if pind >= ind:
            ind = pind
        ind = min(len(cx)-1,ind)
        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0
        # dref[0, 0] = self.calculate_reference_steer(state, cyaw[ind])

        travel = 0.0

        for i in range(T + 1):
            travel += abs(state.v) * DT  # 累计形式的距离
            dind = int(round(travel / dl))  # dl是路径点的间隔，travel/dl是当前车辆已经行驶的路径点数

            if (ind + dind) < ncourse:  #n course是路径点的总数，判断是否超出路径点的总数
                # xref[0, i] = cx[ind + dind]
                # xref[1, i] = cy[ind + dind]
                # xref[2, i] = sp[ind + dind]
                # xref[3, i] = cyaw[ind + dind]
                # dref[0, i] = 0.0
            # if (ind + i) < ncourse:
                xref[0, i] = cx[ind + i]
                xref[1, i] = cy[ind + i]
                xref[2, i] = sp[ind + i]
                xref[3, i] = cyaw[ind + i]
                dref[0, i] = 0.0
                dref[0, i] = self.calculate_reference_steer(state, cyaw[ind + dind])
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref

    def calculate_reference_steer(self, state, ref_yaw):
        # 简单的参考转向角计算，可以根据需要调整
        delta = pi_2_pi(ref_yaw - state.yaw)
        # 限制转向角在物理范围内
        delta = max(-MAX_STEER, min(MAX_STEER, delta))
        return delta    
    
    def smooth_yaw(self,yaw):

        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw    
      
    def calc_nearest_index(self, state, cx, cy, cyaw, pind):
        # print(len(cx[pind:(pind + N_IND_SEARCH)]))
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        # ind = d.index(mind) + pind
        ind = d.index(mind)

        mind = math.sqrt(mind) # 最小距离

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind

    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        MPC control with updating operational point iteratively
        """
        ox, oy, oyaw, ov = None, None, None, None

        if oa is None or od is None:
            oa = [0.0] * T  # 上一轮优化得到的加速度序列，如果是第一次迭代，就初始化为0
            od = [0.0] * T  # 上一轮优化得到的转角序列，如果是第一次迭代，就初始化为0

        for i in range(MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value 用于判断是否收敛
            if du <= DU_TH: # 如果u的变化小于阈值，说明收敛了, 就退出迭代
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov

    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        x = cvxpy.Variable((NX, T + 1))
        u = cvxpy.Variable((NU, T))

        cost = 0.0
        constraints = []

        for t in range(T):
            cost += cvxpy.quad_form(u[:, t], R)  # cost function 控制输入的cost

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)  # state cost function

            A, B, C = get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C] # 动力学约束 保证车辆的运动符合动力学模型

            if t < (T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)  # input difference cost function 控制输入的变化cost，目的是让控制输入尽量平滑，减小震荡
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=  # 控制输入的变化率约束，限制转角的变化率，防止转角变化过快
                                MAX_DSTEER * DT]

        cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf) # final state cost function 终端状态的cost

        constraints += [x[:, 0] == x0]  
        constraints += [x[2, :] <= MAX_SPEED]  # 速度约束
        constraints += [x[2, :] >= MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL] # 加速度约束
        constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.OSQP, verbose=False)
        

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = get_nparray_from_matrix(x.value[0, :])
            oy = get_nparray_from_matrix(x.value[1, :])
            ov = get_nparray_from_matrix(x.value[2, :])
            oyaw = get_nparray_from_matrix(x.value[3, :])
            oa = get_nparray_from_matrix(u.value[0, :])
            odelta = get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def predict_motion(self, x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, T + 1)):
            state = self.update_state(state, ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar

    def update_state(self, state, a, delta):
        # input check
        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER
        state.x = state.x + state.v * math.cos(state.yaw) * DT
        state.y = state.y + state.v * math.sin(state.yaw) * DT
        state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
        # state.yaw = normalize_angle(state.yaw)
        state.v = state.v + a * DT
        if state.v > MAX_SPEED:
            state.v = MAX_SPEED
        elif state.v < MIN_SPEED:
            state.v = MIN_SPEED
        return state

    # 用于平滑转角
    def smooth_turn_angle(self, turn_angle):
        # 限制转向角的最大变化速率
        angle_difference = turn_angle - self.previous_turn_angle
        if angle_difference > self.max_turn_rate:
            update_turn_angle = self.previous_turn_angle + 4
        elif angle_difference < -self.max_turn_rate:
            update_turn_angle = self.previous_turn_angle - 4
        else:
            update_turn_angle = turn_angle

        # 更新上一次的转向角
        self.previous_turn_angle = update_turn_angle
        return turn_angle

        # 计算期望速度和加速度
    
    
    def calculate_speedAndacc(self, turn_angle, current_position, current_speed, is_obstacle = False, points_num_threshold=20,
                              high_speed = 30,
                              low_speed = 5):
        if current_speed < 1:
            speed = low_speed
            acc = 0
            return speed, acc
        
        if abs(turn_angle) >= 25:
            if current_speed >= 15:
                speed = low_speed
                acc = -1
            else:
                speed = low_speed
                acc = 0
            return speed, acc 
        
        current_lat, current_lon, current_heading = current_position
        next_ref_x       = self.cx[min(self.closest_index + self.far_index, len(self.cx) - 1)] 
        next_ref_y       = self.cy[min(self.closest_index + self.far_index, len(self.cx) - 1)] 
        next_ref_heading = self.cyaw[min(self.closest_index + self.far_index, len(self.cx) - 1)] 

        next_heading = math.degrees(next_ref_heading)
        next_heading = 90 - next_heading
        if next_heading <= 0:
            next_heading += 360
        if next_heading > 360:
            next_heading -= 360
        
        next_lon, next_lat = utm_to_latlon(next_ref_x, next_ref_y)
        print(f"current_heading: {current_heading}, next_heading: {next_heading}")
        
        if current_heading <= 0:
            current_heading += 360
        diff_heading = abs(current_heading-next_heading)
        print(f"current_heading: {current_heading}, next_heading: {next_heading}, diff_heading: {diff_heading}")
        # 计算目标点相对当前位置的方位角
        far_desired_heading = self.calculate_bearing(current_lat, current_lon, next_lat, next_lon)

        # 计算转向角
        far_turn_angle = (far_desired_heading - current_heading + 360) % 360
        if far_turn_angle > 180:
            far_turn_angle -= 360        
        # 映射到方向盘转角
        if far_turn_angle * WHEEL_FACTOR > 500:
            far_turn_angle = 500
        elif far_turn_angle * WHEEL_FACTOR < -500:
            far_turn_angle = -500
        else:
            far_turn_angle = far_turn_angle * WHEEL_FACTOR    
        print("====== far_turn_angle: ",far_turn_angle)
        if abs(far_turn_angle) >= 50 or diff_heading >=120:
            print("========================将会发生转弯，减速！减速！减速！=============================")
            if current_speed >= high_speed-5:
                speed = low_speed+10
                acc = -2
            elif current_speed >= low_speed+5:
                speed = low_speed
                acc = -1
            else:
                speed = low_speed
                acc = 0
        else:
            speed = high_speed
            acc = 0
        
        if is_obstacle:
            print("==================== find obstacle reduce speed ===========================")
            if current_speed >= low_speed+2:
                speed = low_speed
                acc = -3
            else:
                speed = low_speed
                acc = 0
        return speed, acc    

    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing

        
        
    
if __name__ == '__main__':
    main()

