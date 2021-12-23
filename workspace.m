arm = LynxmotionAL5D();

% try lots of angles
t1 = linspace(0,deg2rad(360),10);
t2 = linspace(0,deg2rad(360),10);
t3 = linspace(0,deg2rad(360),10);
t4 = linspace(0,deg2rad(360),10);

% memory for samples
xwork = zeros(length(t1),length(t2),length(t3),length(t4));
ywork = zeros(length(t1),length(t2),length(t3),length(t4));
zwork = zeros(length(t1),length(t2),length(t3),length(t4));

for i = 1:length(t1)
    for j = 1:length(t2)
        for k = 1:length(t3)
            for l = 1:length(t4)
                eef = arm.forward_kinematics(t1(i), t2(j), t3(k), t4(l), 0);

                xwork(i,j,k,l) = eef(1,4);
                ywork(i,j,k,l) = eef(2,4);
                zwork(i,j,k,l) = eef(3,4);
            end
        end
    end
end

% plot workspace in 3D
figure(1)
scatter3(xwork(:),ywork(:),zwork(:),'.')
axis equal