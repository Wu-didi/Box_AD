
class Bbox:
    """用于存放每个检测框信息的简单类"""
    def __init__(self, x, y, z, w, l, h, theta, score, label):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.l = l
        self.h = h
        self.theta = theta
        self.score = score
        self.label = label

    def __str__(self):
        return (f"Bbox(x={self.x}, y={self.y}, z={self.z}, w={self.w}, "
                f"l={self.l}, h={self.h}, theta={self.theta}, "
                f"score={self.score}, label={self.label})")
        
        
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
        self.isg_mot_spd_filt = self.isg_sum_filtspd / 5
        self.MAT_Moto_spd = self.isg_mot_spd_filt  # 更新最终输出速度

        return self.MAT_Moto_spd


class ISGLowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha  # 平滑因子
        self.isg_mot_spd_filt = 0  # 滤波后的速度

    def update_speed(self, isg_mot_spd):
        # 应用低通滤波器公式
        self.isg_mot_spd_filt = self.alpha * isg_mot_spd + (1 - self.alpha) * self.isg_mot_spd_filt
        return self.isg_mot_spd_filt


# 滑动窗口滤波器
class SlideWindowISGSpeedFilter:
    def __init__(self, window_size=15):
        self.window_size = window_size
        self.buffer = []
    
    def update_speed(self, current_speed):
        self.buffer.append(current_speed)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)
