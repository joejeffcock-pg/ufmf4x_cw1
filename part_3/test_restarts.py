import torch
import numpy as np
from torch import optim
from lynxmotion_grad import LynxmotionGrad
from collections import defaultdict

pos_tols = np.logspace(1,-3,5,base=2)
ori_tols = np.logspace(1,-3,5,base=2) * np.pi/180
results = defaultdict(int)
no_tests = 1000
no_iterations = 150 # approx 5Hz
lr = 1.29e-04

for i in range(pos_tols.size):
    pos_tol = float(pos_tols[i])
    ori_tol = float(ori_tols[i])
    print('Testing position tolerance={} orientation tolerance={}'.format(pos_tol, ori_tol))
    tolerances = np.array([pos_tol, ori_tol, ori_tol])

    for i in range(no_tests):
        with torch.no_grad():
            target_pose = LynxmotionGrad().forward_kinematics()

        solved = 0
        for restarts in range(4):
            if not solved:
                robot = LynxmotionGrad()
                optimiser = optim.SGD(robot.parameters(), lr=lr, momentum=0.9)

                loss, errors = robot.inverse_kinematics_step(optimiser, target_pose)
                iteration = 0
                while (errors > tolerances).any() and iteration < no_iterations:
                    loss, errors = robot.inverse_kinematics_step(optimiser, target_pose)
                    iteration += 1

                if iteration < no_iterations:
                    solved = 1

            results[(pos_tol, ori_tol, restarts)] += solved

filename = 'restart_success.csv'
with open(filename, 'w') as f:
    f.write('pos_tol,ori_tol,restarts,')
    f.write('successes')
    f.write('\n')
    for pos_tol, ori_tol, restarts in results:
        f.write('{},{},{},{}'.format(pos_tol, ori_tol, restarts, results[pos_tol, ori_tol, restarts]))
        f.write('\n')
print('Restart written to {}'.format(filename))