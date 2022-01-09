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
c_samples = 50; % cartesian samples between each pose
j_samples = 50; % joint samples to form our trajectory
poly = 10;
points = zeros(size(poses,1) * c_samples, 3); % store points to plot trajectory
az = -37.5; % initial azimuth angle for view

for i = 0:size(poses,1)-2
    p1 = poses(i+1,:);
    p2 = poses(i+2,:);

    % compute joint trajectories
    trajectory = cartesian_trajectory(arm, p1, p2, c_samples, j_samples, poly);

    for j = 1:c_samples
        j1 = trajectory(j,:);

        % store points to plot trajectory
        eef = arm.forward_kinematics(j1(1),j1(2),j1(3),j1(4),j1(5));
        points(i*c_samples+j,1) = eef(1,4);
        points(i*c_samples+j,2) = eef(2,4);
        points(i*c_samples+j,3) = eef(3,4);

        % animated joint trajectory
        figure(1);
        arm.draw(j1(1),j1(2),j1(3),j1(4),j1(5));
        hold on;
        plot3(points(1:i*c_samples+j,1),points(1:i*c_samples+j,2),points(1:i*c_samples+j,3),'-','Linewidth',2,'Color',[0 0 1 0.5]);

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


function trajectory = cartesian_trajectory(arm,p1,p2,c_samples,j_samples,poly)
    % interpolate between P1 and P2 to sample poses
    % equivalent to f(t) = P1(1-t) + P2(t) for 0<=t<=1
    sample_poses = zeros(c_samples, length(p1));
    for j = 1:length(p1)
        sample_poses(:,j) = linspace(p1(j), p2(j), c_samples);
    end

    % compute IK for sampled poses in cartesian space
    % to obtain c_samples in joint space coordinates
    sample_joints = zeros(c_samples,5);
    for i = 1:size(sample_poses,1)
        ps = sample_poses(i,:);
        sample_joints(i,:) = arm.inverse_kinematics(ps(1),ps(2),ps(3),ps(4),ps(5));
    end

    % for each joint
    trajectory = zeros(j_samples,5);
    for i = 1:5
        % fit a polynomial to the joint space coordinates over time
        yy1 = sample_joints(:,i);
        xx1 = linspace(0,1,c_samples); % increments of 1 unit
        p = polyfit(xx1,yy1,poly);

        % resample from the fit polynomial
        % to obtain cartesian trajectory in joint space
        xx2 = linspace(0,1,j_samples); % increments of 1 unit
        trajectory(:,i) = polyval(p,xx2,poly);
    end
end