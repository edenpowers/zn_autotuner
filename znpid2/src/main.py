import math
# ----------------------------------------------------------------------------- #
#                                                                               #              
#    Project:        Using Threads                                              #
#    Module:         main.py                                                    #
#    Author:         VEX                                                        #
#    Created:        Fri Aug 05 2022                                            #
#    Description:    This example will show how to run multiple threads (tasks) # 
#                    in a project at the same time                              #
#                                                                               #                                                                          
#    Configuration:  None                                                       #
#                                                                               #                                                                          
# ----------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain = Brain()
controller = Controller()
f_left_motor = Motor(Ports.PORT11, True)
b_left_motor = Motor(Ports.PORT12, True)
t_left_motor = Motor(Ports.PORT14, False)
f_right_motor = Motor(Ports.PORT17, False)
b_right_motor = Motor(Ports.PORT18, False)
t_right_motor = Motor(Ports.PORT19, True)
intake = Motor(Ports.PORT10, False)
inertial = Inertial(Ports.PORT2)
pto = DigitalOut(brain.three_wire_port.h)
wings = DigitalOut(brain.three_wire_port.g)


def cw_turn_pid(adegrees, kp, ki, kd, max_steps, return_period = 0):
    inertial.set_heading(0, DEGREES)
    cur_degrees = inertial.heading(DEGREES)
    prev_degrees = 0
    cur_sum = 0
    cur_step = 0
    err = adegrees
    tai94 = 0
    prev_der = 0
    osc_started = 0
    osc_start_step = 0
    osc_periods = []
    while( abs(err) > 0.5 and cur_step <= max_steps or abs(prev_degrees - cur_degrees) > 0.1):
        if(cur_step != 0):
            wait(10, MSEC)
        prev_degrees = cur_degrees
        cur_degrees = inertial.heading(DEGREES)
        if(cur_degrees > 350):
            cur_degrees -= 360
        #tai94 += 0.02 * (prev_degrees + cur_degrees) * 0.5
        if(prev_der > prev_degrees - cur_degrees):
            tai94 += 1.5 * err
        if(ki != 0):
            if(tai94 > 12 / ki):
                tai94 = 12 / ki
        else:
            tai94 = 0
        cout = kp * err + ki * tai94 + kd * (prev_degrees - cur_degrees)
        if(abs(err) > 10):
            cout+=1
        if(cout > 12):
            cout = 12
        f_left_motor.spin(FORWARD, cout, VoltageUnits.VOLT)
        b_left_motor.spin(FORWARD, cout, VoltageUnits.VOLT)
        f_right_motor.spin(REVERSE, cout, VoltageUnits.VOLT)
        b_right_motor.spin(REVERSE, cout, VoltageUnits.VOLT)
        err = adegrees - cur_degrees
        cur_step+=1
        prev_der = prev_degrees - cur_degrees
        #print("err: " + str(abs(err)))
        #print("deriv = " + str(prev_degrees - cur_degrees))
        #print("cout time:  " + str(cout))
        #print("tai's model : " +str(tai94))
        #osc logging
        if(err < 0):
            if(osc_started % 2==0):
                osc_started+=1
                osc_start_step = cur_step
        else:
            if(osc_started % 2 == 1):
                osc_started += 1
                osc_periods+=[cur_step-osc_start_step]

            
    f_left_motor.stop()
    b_left_motor.stop()
    f_right_motor.stop()
    b_right_motor.stop()
    print("exit")
    print(inertial.heading(DEGREES))
    if(len(osc_periods) == 0):
        return 1000
    mean = sum(osc_periods) / len(osc_periods)
    if return_period == 0:
        return sum([abs(i - mean) for i in osc_periods]) / len(osc_periods)
    if return_period == 2:
        return cur_step
    else:
        return mean
        
def ccw_turn_pid(adegrees, kp, ki, kd, max_steps):
    inertial.set_heading(0, DEGREES)
    cur_degrees = inertial.heading(DEGREES)
    prev_degrees = 0
    cur_sum = 0
    cur_step = 0
    err = adegrees
    tai94 = 0
    prev_der = 0
    while( abs(err) > 0.5 and cur_step <= max_steps or abs(prev_degrees - cur_degrees) > 0.1):
        if(cur_step != 0):
            wait(10, MSEC)
        prev_degrees = cur_degrees
        cur_degrees = inertial.heading(DEGREES)
        if(cur_degrees < 10):
            cur_degrees *= -1
        else:
            cur_degrees = 360 - cur_degrees
        #tai94 += 0.02 * (prev_degrees + cur_degrees) * 0.5
        if(prev_der > prev_degrees - cur_degrees):
            tai94 += 1.5 * err
        if(ki != 0):
            if(tai94 > 12 / ki):
                tai94 = 12 / ki
        else:
            tai94 = 0
        cout = kp * err + ki * tai94 + kd * (prev_degrees - cur_degrees)
        if(abs(err) > 10):
            cout+=1
        if(cout > 12):
            cout = 12
        f_left_motor.spin(FORWARD, -1 * cout, VoltageUnits.VOLT)
        b_left_motor.spin(FORWARD, -1 * cout, VoltageUnits.VOLT)
        f_right_motor.spin(REVERSE, -1 * cout, VoltageUnits.VOLT)
        b_right_motor.spin(REVERSE, -1 * cout, VoltageUnits.VOLT)
        err = adegrees - cur_degrees
        cur_step+=1
        prev_der = prev_degrees - cur_degrees
        #print("err: " + str(abs(err)))
        #print("deriv = " + str(prev_degrees - cur_degrees))
        #print("cout time:  " + str(cout))
        #print("tai's model : " +str(tai94))
    f_left_motor.stop()
    b_left_motor.stop()
    f_right_motor.stop()
    b_right_motor.stop()
    print("exit")

def autotune_degrees(degrees):
    #this is pretty fast but i'm a speedy gal what can i say
    kp = 0.1
    kd = 0
    ki = 0
    last_osc = 1000
    while(True):
        cur_osc = cw_turn_pid(degrees, kp, ki, kd,1000)
        if(last_osc >= cur_osc):
            last_osc = cur_osc
            kp += 0.05
        else:
            break
    last_osc = cw_turn_pid(degrees, kp, ki, kd, 1000, 1)
    return [0.6 * kp, last_osc * 0.5, last_osc * 0.125]


def check_func(degrees, kp, ki, kd, _a, _b):
    return cw_turn_pid(degrees, kp, ki, kd, 1000, 2)
class Firefly:
    def __init__(self, kp, ki, kd, speed =250) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.speed = speed
    def find_speed(self, target) -> None:
        self.speed = 250 - check_func(target, self.kp, self.ki, self.kd, 250, 2)
        #self.speed = check_func(target, self.kp, self.ki, self.kd, 250, 2)
    def calculate_next_self(self, other_kp, other_ki, other_kd, other_speed, batch_num = 8) -> None:
        max_speed = 1.0 #1.0 = clamped growth, actual max value = explosive growth
        #kp_absorption = 5.6e-4
        #ki_absorption = 4e-5
        #kd_absorption = 0.0048
        absorption = 0.017
        next_vector = [0,0,0]
        for i in range(batch_num - 1):
            fly_distance_sq = (self.kp-other_kp[i])**2 + (self.ki-other_ki[i])**2 + (self.kd-other_kd[i])**2
            if(other_speed[i] < 0):
                attractiveness = 0
            else:
                attractiveness = math.exp(-1 * absorption * fly_distance_sq)  * max_speed * (other_speed[i]/ (1.0 * (batch_num - 1)))
                #attractiveness = math.exp(-1 * absorption * fly_distance_sq) * (other_speed[i]/ (30.0))
            if(attractiveness > (max_speed / (batch_num -1))):
                attractiveness = (max_speed / (batch_num -1))
            #print(attractiveness)
            #print("exp")
            #print(math.exp(-1 * absorption * fly_distance_sq))
            #print((other_speed[i]/ (60.0 * batch_num - 1)))
            
            if(self.speed <= other_speed[i]):
                next_vector[0] += (other_kp[i] - self.kp) * attractiveness
                next_vector[1] += (other_ki[i] - self.ki) * attractiveness
                next_vector[2] += (other_kd[i] - self.kd) * attractiveness

        #print(next_vector)
        self.kp += next_vector[0]
        self.ki += next_vector[1]
        self.kd += next_vector[2]

def run_FA(target, start_kp, start_ki, start_kd, kp_scale = 0.1, ki_scale = 0.01, kd_scale = 0.3, generations = 3):
    kp_dir = [(x) * kp_scale + start_kp for x in [1,1,1,1,-1,-1,-1,-1]]
    ki_dir = [(x) * ki_scale + start_ki for x in [1,1,-1,-1,1,1,-1,-1]]
    kd_dir = [(x) * kd_scale + start_kd for x in [1,-1,1,-1,1,-1,1,-1]]
    flys = []
    for i in range(8):
        flys.append(Firefly(kp_dir[i], ki_dir[i], kd_dir[i]))

    print("--==GENERATION: " + str(0) + "==--")
    for i in flys:
        print("("+ str(i.kp) + ", " + str(i.ki) + ", " + str(i.kd) + ")")

    #other_speed = [100 for x in range(7)]
    for i99 in range(generations):
        new_flys = []
        for i in range(8):
            flys[i].find_speed(target)
            #print(flys[i].speed)
        for i in range(8):
            new_flys.append(Firefly(flys[i].kp,flys[i].ki,flys[i].kd, flys[i].speed))
            other_kp = [flys[x + 1].kp if x >= i else flys[x].kp for x in range(7)]
            other_ki = [flys[x + 1].ki if x >= i else flys[x].ki for x in range(7)]
            other_kd = [flys[x + 1].kd if x >= i else flys[x].kd for x in range(7)]
            other_speed = [flys[x + 1].speed if x >= i else flys[x].speed for x in range(7)]
            #print(other_speed)
            #print(i)
            #print(other_kp)
            new_flys[i].calculate_next_self(other_kp, other_ki, other_kd, other_speed)
        flys = new_flys
        print("--==GENERATION: " + str(i99 + 1) + "==--")
        for i in flys:
            print("("+ str(i.kp) + ", " + str(i.ki) + ", " + str(i.kd) + ")")
    sum_kp = 0
    sum_ki = 0
    sum_kd = 0
    for i in flys:
        sum_kp += i.kp
        sum_ki += i.ki
        sum_kd += i.kd
    print("--==FINAL MEAN==--")
    print("("+ str(sum_kp / 8) + ", " + str(sum_ki / 8) + ", " + str(sum_kd / 8) + ")")
    
inertial.set_heading(0,DEGREES)
"""
while(True):
    print(inertial.heading())
"""
print("hello")
while(inertial.heading() == 0):
    wait(10,MSEC)
print("world")
#cur kd = 1.8
#cw_turn_pid(90,0.24,0.018,2.5*1.2,1000)
#fa vals
print(cw_turn_pid(90,0.2875219,0.01349781,4.560751,250, 2))
#og vals
print(cw_turn_pid(90,0.24,0.01825,4.5625,250, 2))
"""
kvals = []
kvals = autotune_degrees(90)
#cw_turn_pid(90, kvals[0], kvals[1], kvals[2], 1000)
print("--==KVALS==--")
print("KP = " + str(kvals[0]))
print("KI = " + str(kvals[1] / 1000))
print("KD = " + str(kvals[2]))
#cw_turn_pid(90,0.27,0.025,3.6875,1000)
run_FA(90, kvals[0], kvals[1] / 1000, kvals[2])
wait(1,SECONDS)
print(inertial.heading())
#ccw_turn_pid(180,0.35,0.025,5.5,1000)
#wait(1,SECONDS)
print(inertial.heading())
#print(kvals)
"""