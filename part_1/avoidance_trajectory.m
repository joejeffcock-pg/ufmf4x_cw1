arm = LynxmotionAL5D();
obstacle = Cuboid([-10 27 30], [5 5 5]);

poses = [
%     33.25, 0 , 14.2, -pi/2, 0; % parked
%     10, 0 , 18.5, -pi/2, 0; % parked
%     0    , 20, 45,  pi/2, 2*pi; % remove bulb 1
%     0    , 20, 45,  pi/2, 0; % remove bulb 2
%     7.5   , 10, -10  , -pi/2, 0; % dispose of bulb
%     -15   , 25, 15  , 0, pi/2; % pick new bulb 1 
    -18   , 30, 15  , 0, pi/2; % pick new bulb 2
    0    , 20, 45,  pi/2, 0; % replace bulb 1
    0    , 20, 45,  pi/2, 2*pi; % replace bulb 2
    10, 0 , 18.5, -pi/2, 0; % parked
];

c_samples = 50; % cartesian samples between each pose
j_samples = 50; % joint samples to form our trajectory
poly = 10;
points = zeros(0, 3); % store points to plot trajectory
az = -37.5; % initial azimuth angle for view

for i = 0:size(poses,1)-2
    p1 = poses(i+1,:);
    p2 = poses(i+2,:);

    % compute joint trajectories
    path = bug_algorithm(p1, p2, obstacle);
    for j = 1:size(path,1)-1
        bugp1 = path(j+0,:);
        bugp2 = path(j+1,:);
        trajectory = cartesian_space_trajectory(arm, bugp1, bugp2, c_samples, j_samples, poly);

        for k = 1:c_samples
            j1 = trajectory(k,:);
    
            % store points to plot trajectory
            eef = arm.forward_kinematics(j1(1),j1(2),j1(3),j1(4),j1(5));
            points = [points;[eef(1,4) eef(2,4) eef(3,4)]];
            size(points)
    
            % animated joint trajectory
            figure(1);
            arm.draw(j1(1),j1(2),j1(3),j1(4),j1(5));
            hold on;
            obstacle.draw()
            plot3(points(1:size(points,1),1),points(1:size(points,1),2),points(1:size(points,1),3),'-','Linewidth',2,'Color',[0 0 1 0.5]);
    
            axis([-50 50 -50 50 -30 70]);
            view([az, 30]);
            az = az - 0.2;
            grid on;
            pause(0.01);
            hold off;
        end
    end
end

% plot trajectory
figure(2);
obstacle.draw()
hold on
plot3(points(:,1),points(:,2),points(:,3),'b-','Linewidth',2);
hold off
axis([-50 50 -50 50 -30 70])
grid on;






function path = get_closest_plane(obj, start, goal)
    % choose the closest plane and entry point
    min_dist = inf;
    entry_args = [1 1]; % default: plane 1 point 1
    for i=1:size(obj.skeleton,1)
        for j=1:5
            point = zeros(1,3);
            point(:) = obj.skeleton(i,j,:);
            dist = norm(start - point);
            if dist < min_dist
                min_dist = dist;
                entry_args = [i j];
            end
        end
    end

    % perform bug1 on 2D plane
    % exit point is point closest to goal
    min_dist = inf;
    exit_args = [1 1]; % default: plane 1 point 1
    for j=1:5
        point = zeros(1,3);
        point(:) = obj.skeleton(entry_args(1),j,:);
        dist = norm(goal - point);
        if dist < min_dist
            min_dist = dist;
            exit_args = [entry_args(1) j];
        end
    end

    % return points along plane from entry point to exit point
    if entry_args(2) <= exit_args(2)
        path = obj.skeleton(entry_args(1), entry_args(2):exit_args(2), :);
    else
        path = [obj.skeleton(entry_args(1), entry_args(2):4, :) obj.skeleton(entry_args(1), 1:exit_args(2), :)];
    end
end

function path = bug_algorithm(start, goal, obstacle)
    dist = norm(start(1:3) - goal(1:3));
    steps = round(dist) * 5;
    poses = zeros(steps, 5);
    for j = 1:5
        poses(:,j) = linspace(start(j), goal(j), steps);
    end

    path = [start; goal];
    for i = 1:steps - 1
        pose = poses(i,:);
        next = poses(i+1,:);
        if obstacle.point_inside(next(1:3)) == 1
            % get closest plane to point in pose
            plane = get_closest_plane(obstacle,next(1:3), goal(1:3));
            plane = reshape(plane,size(plane,2), 3);

            % add psi,mu dims to plane to create a path
            bug_path = zeros(size(plane,1),5);
            bug_path(:,1:3) = plane;
            bug_path(:,4) = pose(4);
            bug_path(:,5) = pose(5);

            % create bug solution
            path = [start; pose; bug_path; goal];
            break
        end
    end
end