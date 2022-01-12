clear 
close all
clc

% create robot
robot = planarRobot();

% parameter for IK
px = 0;
py = 0;
phi_1 = pi/6+0;

% draw robot
draw_ik(robot, px, py, phi_1)

% draw work space of the robot
draw_ws(robot);

function draw_ik(robot, px, py, phi_1)
    %% This function takes in the position of ee and angle alpha repected to the first origin    
    %% output is the plot of the robot

    ik_angle = robot.IK(px, py, phi_1);
    theta_1 = ik_angle(1,1);
    theta_2 = ik_angle(2,1);
    theta_3 = ik_angle(3,1);

    % displace angles
    disp(theta_1*180/pi)
    disp(theta_2*180/pi)
    disp(theta_3*180/pi)

    % draw robot
    figure(1)
    robot.draw(px, py, phi_1, theta_1, theta_2, theta_3);

end

function draw_ws(robot)
    %% This function will sample the ee position and orientation
    %% and check if that configuration can be solved by IK or not
    %% if yes, that config is in work space.

    % set min max 
    xmin = -200;
    xmax = 200;
    ymin = -200;
    ymax = 200;

    % set sampling rate
    samp_rate = 200;
    phi_samp_rate =20;
    
    % create 0 array for putting in the positions in work space
    x_result = zeros(1,samp_rate*samp_rate*phi_samp_rate);
    y_result = zeros(1,samp_rate*samp_rate*phi_samp_rate);

    % create line space
    x = linspace(xmin, xmax, samp_rate);
    y = linspace(ymin, ymax, samp_rate);
    phi = linspace(pi/6-pi/4 ,pi/6+pi/4,phi_samp_rate);
    
    % loop for testing the point is in the work space or not
    for i = 1:phi_samp_rate
        for j = 1:samp_rate
            for k = 1:samp_rate
                px = x(k);
                py = y(j);
                angle = phi(i);

                % do the IK fo all angles
                ik_angle = robot.IK(px, py, angle);
                theta_1 = ik_angle(1,1);
                theta_2 = ik_angle(2,1);
                theta_3 = ik_angle(3,1);

                % if all angles are real, register x and y in work space
                if  and(and(isreal(theta_1),isreal(theta_2)),isreal(theta_3))
                    x_result(k + (j-1)*samp_rate + (i-1)*samp_rate*samp_rate) = px;
                    y_result(k + (j-1)*samp_rate + (i-1)*samp_rate*samp_rate) = py;
                end
            end
        end
    end
  
    % plotting work space
    figure(2)
    scatter(x_result,y_result,'rx')
    hold on 

    % plotting the frame
    x_offset = 500/2;
    y_offset = 500*sin(60*pi/180)/2;    
    frame_x = [-x_offset x_offset 0 -x_offset];
    frame_y = [-y_offset -y_offset y_offset -y_offset];
    xlim([-300,300])
    ylim([-300,300])
    xlabel('x (mm)')
    ylabel('y (mm)')
    grid on 
    plot(frame_x,frame_y,"LineStyle","--","Color",'blue');

end