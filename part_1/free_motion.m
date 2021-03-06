arm = LynxmotionAL5D();

poses = [
%     33.25, 0 , 14.2, -pi/2, 0; % parked
    10, 0 , 18.5, -pi/2, 0; % parked
    0    , 20, 45,  pi/2, 2*pi; % remove bulb 1
    0    , 20, 45,  pi/2, 0; % remove bulb 2
    7.5   , 10, -10  , -pi/2, 0; % dispose of bulb
    -15   , 25, 15  , 0, pi/2; % pick new bulb 1 
    -18   , 30, 15  , 0, pi/2; % pick new bulb 2
    0    , 20, 45,  pi/2, 0; % replace bulb 1
    0    , 20, 45,  pi/2, 2*pi; % replace bulb 2
    10, 0 , 18.5, -pi/2, 0; % parked
];

t_i = 0; % time at pose i
t_f = 2; % time at pose f = i + 1
samples = 50; % samples between each pose
points = zeros(size(poses,1) * samples, 3); % store points to plot trajectory
az = -37.5; % initial azimuth angle for view

for i = 0:size(poses,1)-2
    p1 = poses(i+1,:);
    p2 = poses(i+2,:);
    
    % compute IK for pose i and i + 1
    j1 = arm.inverse_kinematics(p1(1), p1(2), p1(3), p1(4), p1(5));
    j2 = arm.inverse_kinematics(p2(1), p2(2), p2(3), p2(4), p2(5));

    % compute joint trajectories
    q1_spj = joint_space_trajectory(j1(1),j2(1),0,0,t_i,t_f,samples);
    q2_spj = joint_space_trajectory(j1(2),j2(2),0,0,t_i,t_f,samples);
    q3_spj = joint_space_trajectory(j1(3),j2(3),0,0,t_i,t_f,samples);
    q4_spj = joint_space_trajectory(j1(4),j2(4),0,0,t_i,t_f,samples);
    q5_spj = joint_space_trajectory(j1(5),j2(5),0,0,t_i,t_f,samples);

    for j = 1:samples
        % store points to plot trajectory
        eef = arm.forward_kinematics(q1_spj(j),q2_spj(j),q3_spj(j),q4_spj(j),q5_spj(j));
        points(i*samples+j,1) = eef(1,4);
        points(i*samples+j,2) = eef(2,4);
        points(i*samples+j,3) = eef(3,4);

        % animated joint trajectory
        figure(1);
        arm.draw(q1_spj(j),q2_spj(j),q3_spj(j),q4_spj(j),q5_spj(j));
        hold on;
        plot3(points(1:i*samples+j,1),points(1:i*samples+j,2),points(1:i*samples+j,3),'-','Linewidth',2,'Color',[0 0 1 0.5]);

        axis([-50 50 -50 50 -30 70]);
        view([az, 30]);
        az = az - 0.2;
        grid on;
        pause(0.01);
        hold off;
    end

    % advance time
    t_i = t_i + 2;
    t_f = t_f + 2;
end

% plot trajectory
figure(2);
plot3(points(:,1),points(:,2),points(:,3),'b-','Linewidth',2);
axis([-50 50 -50 50 -30 70])
grid on;

% plot last motion angle vs time
time_steps = linspace(0,2,samples);

figure(3);
plot(time_steps,q1_spj*180/pi)
hold on
plot(time_steps,q2_spj*180/pi)
plot(time_steps,q3_spj*180/pi)
plot(time_steps,q4_spj*180/pi)
plot(time_steps,q5_spj*180/pi)
legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5')
xlabel('time(s)')
ylabel('degree(??)')
xlim([0,2])
ylim([-360,360])
yticks(-360:60:360);
grid on
title('Joint Angles over Time (Joint Space Motion)')
