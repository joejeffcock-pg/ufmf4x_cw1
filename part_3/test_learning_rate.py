import torch
import numpy as np
from torch import optim
from lynxmotion_grad import LynxmotionGrad

learning_rates = np.logspace(-5, -3, num=10)
results = {}
no_tests = 1000
no_iterations = 100

for lr in learning_rates:
    print('Testing lr={}'.format(lr))
    losses = []

    for i in range(no_tests):
        robot = LynxmotionGrad()
        with torch.no_grad():
            target_pose = LynxmotionGrad().forward_kinematics()
        optimiser = optim.SGD(robot.parameters(), lr=lr, momentum=0.9)

        for j in range(no_iterations):
            loss, errors = robot.inverse_kinematics_step(optimiser, target_pose)
        losses.append(loss.item())

    results[lr] = losses

filename = 'learning_rate_loss.csv'
with open(filename, 'w') as f:
    f.write('lr,')
    f.write(','.join(str(i) for i in range(no_tests)))
    f.write('\n')
    for lr in results:
        f.write('{},'.format(lr))
        f.write(','.join(str(loss) for loss in results[lr]))
        f.write('\n')
print('Loss values written to {}'.format(filename))