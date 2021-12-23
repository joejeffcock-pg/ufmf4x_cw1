import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import optim

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

import numpy as np
from math import sin, cos, radians, pi

class LynxmotionGrad(nn.Module):
    def __init__(self):
        super(LynxmotionGrad, self).__init__()
        self.q1 = nn.Parameter(torch.rand(1) * pi)
        self.q2 = nn.Parameter(torch.rand(1) * pi)
        self.q3 = nn.Parameter(torch.rand(1) * pi)
        self.q4 = nn.Parameter(torch.rand(1) * pi)
        self.d1 = 18.5
        self.d2 = 14.5
        self.d3 = 18.75
        self.d4 = 4.3

    def forward(self, q1, q2, q3, q4):
        x = torch.cos(q1)*(self.d3*torch.cos(q2 + q3) + self.d2*torch.cos(q2) - self.d4*torch.sin(q2 + q3 + q4))
        y = torch.sin(q1)*(self.d3*torch.cos(q2 + q3) + self.d2*torch.cos(q2) - self.d4*torch.sin(q2 + q3 + q4))
        z = self.d1 - self.d3*torch.sin(q2 + q3) - self.d2*torch.sin(q2) - self.d4*torch.cos(q2 + q3 + q4)
        psi = q2 + q3 + q4 + radians(90)
        return x,y,z,psi

    def draw(self, q1, q2, q3, q4, ax, colour='gray'):
        q1 = q1.detach().numpy()
        q2 = q2.detach().numpy()
        q3 = q3.detach().numpy()
        q4 = q4.detach().numpy()
        
        T = np.zeros((7,4,4))
        T[0,:,:] = np.eye(4)
        T[1,:,:] = tf_from_distal(0, radians(-90), self.d1, q1)
        T[2,:,:] = tf_from_distal(self.d2, 0, 0, q2)
        T[3,:,:] = tf_from_distal(self.d3, 0, 0, q3)
        T[4,:,:] = tf_from_distal(0, radians(-90), 0, q4)
        T[5,:,:] = tf_from_distal(0, 0, self.d4, 0)
        T[6,:,:] = tf_from_distal(0, 0, 0, 0)

        T0eef = np.eye(4)
        points = np.zeros((7,3))
        for i in range(7):
            T0eef = T0eef.dot(T[i,:,:])
            points[i,0] = T0eef[0,3]
            points[i,1] = T0eef[1,3]
            points[i,2] = T0eef[2,3]
        ax.plot3D(points[:,0], points[:,1], points[:,2], colour)


def tf_from_distal(a, alpha, d, theta):
    return np.array([[cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)],
                     [sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                     [0            ,  sin(alpha)              ,  cos(alpha)              , d              ],
                     [0            ,  0                          ,  0                          , 1              ]])

def loss_function(current_pose, target_pose):
    # sum contribution of each dim to loss
    loss = torch.Tensor([0])
    for i in range(len(current_pose)):
        loss += torch.pow(target_pose[i] - current_pose[i], 2)
    return loss

def draw():
    plt.clf()
    ax = plt.axes(projection='3d')
    ax.set_xlim(-50,50)
    ax.set_ylim(-50,50)
    ax.set_zlim(-30,70)
    robot.draw(q1, q2, q3, q4, ax, 'gray')
    robot.draw(robot.q1, robot.q2, robot.q3, robot.q4, ax, 'blue')


if __name__ == "__main__":
    robot = LynxmotionGrad()
    optimiser = optim.SGD(robot.parameters(), lr=1e-3, momentum=0.9)

    q1 = torch.rand(1) * pi
    q2 = torch.rand(1) * pi
    q3 = torch.rand(1) * pi
    q4 = torch.rand(1) * pi
    target_pose = robot.forward(q1, q2, q3, q4)

    draw()
    plt.show()
    
    plt.ion()
    for i in range(200):
        optimiser.zero_grad()
        current_pose = robot.forward(robot.q1, robot.q2, robot.q3, robot.q4)
        loss = loss_function(current_pose, target_pose)
        loss.backward()
        optimiser.step()

        print(loss)
        draw()
        plt.show()
        plt.pause(0.03)
    plt.ioff()

    with torch.no_grad():
        print (target_pose)
        print (robot.forward(robot.q1, robot.q2, robot.q3, robot.q4))
        print (q1, q2, q3, q4)
        print (robot.q1, robot.q2, robot.q3, robot.q4)
    draw()
    plt.show()
