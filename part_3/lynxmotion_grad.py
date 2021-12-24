import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import optim

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

import numpy as np
from math import sin, cos, radians, pi

import time
import argparse

class LynxmotionGrad(nn.Module):
    def __init__(self):
        super(LynxmotionGrad, self).__init__()
        self.q1 = nn.Parameter(torch.rand(1) * pi)
        self.q2 = nn.Parameter(torch.rand(1) * pi)
        self.q3 = nn.Parameter(torch.rand(1) * pi)
        self.q4 = nn.Parameter(torch.rand(1) * pi)
        self.q5 = nn.Parameter(torch.rand(1) * pi)
        # link lengths (cm)
        self.d1 = 18.5
        self.d2 = 14.5
        self.d3 = 18.75
        self.d4 = 4.3
        # gripper finger lengths
        self.f1 = 3.2
        self.f2 = 4.3


    def forward_kinematics(self):
        pose = torch.zeros(5)
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3
        q4 = self.q4
        q5 = self.q5

        pose[0] = torch.cos(q1)*(self.d3*torch.cos(q2 + q3) + self.d2*torch.cos(q2) - self.d4*torch.sin(q2 + q3 + q4)) # x
        pose[1] = torch.sin(q1)*(self.d3*torch.cos(q2 + q3) + self.d2*torch.cos(q2) - self.d4*torch.sin(q2 + q3 + q4)) # y
        pose[2] = self.d1 - self.d3*torch.sin(q2 + q3) - self.d2*torch.sin(q2) - self.d4*torch.cos(q2 + q3 + q4) # z
        pose[3] = q2 + q3 + q4 + radians(90) # psi
        pose[4] = q5 # mu
        return pose

    def loss_function(self, target_pose):
        current_pose = self.forward_kinematics()
        error = torch.zeros(5)

        # positional error
        error[:3] = target_pose[:3] - current_pose[:3]

        # rotational error (degrees)
        error[3:] = (target_pose[3:] - current_pose[3:]) * (180.0/pi)

        loss = torch.sum(torch.pow(error, 2))
        return loss

    def l2_norm(self, target_pose):
        target_pose_np = target_pose.detach().numpy()
        current_pose_np = self.forward_kinematics().detach().numpy()

        pos_dist = np.linalg.norm(target_pose_np[:3] - current_pose_np[:3])
        psi_dist = np.linalg.norm(target_pose_np[3] - current_pose_np[3])
        mu_dist = np.linalg.norm(target_pose_np[4] - current_pose_np[4])

        return np.array([pos_dist, psi_dist, mu_dist])

    def inverse_kinematics_step(self, optimiser, target_pose):
        optimiser.zero_grad()
        loss = self.loss_function(target_pose)
        loss.backward()
        optimiser.step()

        errors = self.l2_norm(target_pose)
        return loss, errors

    def draw(self, ax, colour='gray'):
        q1 = self.q1.detach().numpy()
        q2 = self.q2.detach().numpy()
        q3 = self.q3.detach().numpy()
        q4 = self.q4.detach().numpy()
        q5 = self.q5.detach().numpy()
        
        # transforms of lynxmotion
        T = np.zeros((7,4,4))
        T[0,:,:] = np.eye(4)
        T[1,:,:] = tf_from_distal(0, radians(-90), self.d1, q1)
        T[2,:,:] = tf_from_distal(self.d2, 0, 0, q2)
        T[3,:,:] = tf_from_distal(self.d3, 0, 0, q3)
        T[4,:,:] = tf_from_distal(0, radians(-90), 0, q4)
        T[5,:,:] = tf_from_distal(0, 0, self.d4, 0)
        T[6,:,:] = tf_from_distal(0, 0, 0, q5)

        # joint positions from FK
        T0eef = np.eye(4)
        points = np.zeros((7,3))
        for i in range(7):
            T0eef = T0eef.dot(T[i,:,:])
            points[i,0] = T0eef[0,3]
            points[i,1] = T0eef[1,3]
            points[i,2] = T0eef[2,3]

        # gripper points
        T0g1f1 = T0eef.dot( tf_from_distal( self.f1, 0, 0      , 0))
        T0g1f2 = T0g1f1.dot(tf_from_distal( 0      , 0, self.f2, 0))
        T0g2f1 = T0eef.dot( tf_from_distal(-self.f1, 0, 0      , 0))
        T0g2f2 = T0g2f1.dot(tf_from_distal( 0      , 0, self.f2, 0))
        g1 = np.array([T0eef[:3, 3], T0g1f1[:3, 3], T0g1f2[:3, 3]])
        g2 = np.array([T0eef[:3, 3], T0g2f1[:3, 3], T0g2f2[:3, 3]])

        ax.plot3D(points[:,0], points[:,1], points[:,2], colour)
        ax.plot3D(g1[:,0], g1[:,1], g1[:,2], colour)
        ax.plot3D(g2[:,0], g2[:,1], g2[:,2], colour)


def tf_from_distal(a, alpha, d, theta):
    return np.array([[cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)],
                     [sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                     [0            ,  sin(alpha)              ,  cos(alpha)              , d              ],
                     [0            ,  0                          ,  0                          , 1              ]])



def draw():
    plt.clf()
    ax = plt.axes(projection='3d')
    ax.set_xlim(-50,50)
    ax.set_ylim(-50,50)
    ax.set_zlim(-30,70)
    target.draw(ax, 'gray')
    robot.draw(ax, 'blue')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--display', action='store_true', dest='display', help='show animated plot of IK optimisation')
    args = parser.parse_args()

    robot = LynxmotionGrad()
    optimiser = optim.SGD(robot.parameters(), lr=1e-4, momentum=0.9)

    target = LynxmotionGrad()
    with torch.no_grad():
        target_pose = target.forward_kinematics()

    draw()
    plt.show()
    
    max_iters = 1000
    tolerances = np.array([0.5, 0.0175, 0.0175]) # 0.5cm xyz; 1deg psi,mu
    iteration = 0
    loss, errors = robot.inverse_kinematics_step(optimiser, target_pose)

    if args.display:
        plt.ion()

    time_start = time.time()
    while (errors > tolerances).any() and iteration < max_iters:
        loss, errors = robot.inverse_kinematics_step(optimiser, target_pose)

        if args.display:
            print(errors)
            draw()
            plt.show()
            plt.pause(0.03)
    time_elapsed = time.time() - time_start

    if args.display:
        plt.ioff()

    print('time taken: {:.2f} seconds'.format(time_elapsed))
    draw()
    plt.show()
