import torch
import numpy as np
from torch import optim
from lynxmotion_grad import LynxmotionGrad
import time

pos_tols = np.logspace(1,-3,5,base=2)
ori_tols = np.logspace(1,-3,5,base=2) * np.pi/180
results = {}
no_tests = 1000
no_iterations = 100
lr = 7.74e-05

for i in range(pos_tols.size):
    pos_tol = pos_tols[i]
    ori_tol = ori_tols[i]
    print('Testing position tolerance={} orientation tolerance={}'.format(pos_tol, ori_tol))
    tolerances = np.array([pos_tol, ori_tol, ori_tol])
    times = []

    for i in range(no_tests):
        robot = LynxmotionGrad()
        with torch.no_grad():
            target_pose = LynxmotionGrad().forward_kinematics()
        optimiser = optim.SGD(robot.parameters(), lr=lr, momentum=0.9)

        time_start = time.time()
        loss, errors = robot.inverse_kinematics_step(optimiser, target_pose)
        iteration = 0
        while (errors > tolerances).any() and iteration < no_iterations:
            loss, errors = robot.inverse_kinematics_step(optimiser, target_pose)
            iteration += 1

        time_elapsed = time.time() - time_start
        if iteration < no_iterations:
            times.append(time_elapsed)
        else:
            times.append(-1)

    results[(pos_tol, ori_tol)] = times

filename = 'tolerance_times.csv'
with open(filename, 'w') as f:
    f.write('pos_tol,ori_tol,')
    f.write(','.join(str(i) for i in range(no_tests)))
    f.write('\n')
    for pos_tol, ori_tol in results:
        f.write('{},{},'.format(pos_tol, ori_tol))
        f.write(','.join(str(time_elapsed) for time_elapsed in results[(pos_tol, ori_tol)]))
        f.write('\n')
print('Times written to {}'.format(filename))