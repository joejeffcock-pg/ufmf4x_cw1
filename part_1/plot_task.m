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

for i = 1:size(poses,1)
    p1 = poses(i,:);
    joints = arm.inverse_kinematics(p1(1), p1(2), p1(3), p1(4), p1(5));

    figure(i)
    arm.draw(joints(1), joints(2), joints(3), joints(4), joints(5));
    axis([-50 50 -50 50 -30 70])
    xlabel("x (cm)");
    ylabel("y (cm)");
    zlabel("z (cm)");
    grid on;
end

figure(size(poses,1) + 1)
xx = poses(:,1);
yy = poses(:,2);
zz = poses(:,3);
plot3(xx,yy,zz)
axis([-50 50 -50 50 -30 70])
xlabel("x (cm)");
ylabel("y (cm)");
zlabel("z (cm)");
title("Path of Light Bulb Replacement Task")
grid on;