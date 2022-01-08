arm = LynxmotionAL5D();

% try lots of angles
t1 = linspace(deg2rad(-180),deg2rad(180),36);
t2 = linspace(deg2rad(-180),deg2rad(180),10);
t3 = linspace(deg2rad(-180),deg2rad(180),10);
t4 = linspace(deg2rad(-180),deg2rad(180),10);
t5 = linspace(deg2rad(-180),deg2rad(180),10);

% memory for samples
xwork = zeros(length(t1),length(t2),length(t3),length(t4),length(t5));
ywork = zeros(length(t1),length(t2),length(t3),length(t4),length(t5));
zwork = zeros(length(t1),length(t2),length(t3),length(t4),length(t5));

for i = 1:length(t1)
    for j = 1:length(t2)
        for k = 1:length(t3)
            for l = 1:length(t4)
                for m = 1:length(t5)
                    eef = arm.forward_kinematics(t1(i), t2(j), t3(k), t4(l), t5(m));
                    xwork(i,j,k,l,m) = eef(1,4);
                    ywork(i,j,k,l,m) = eef(2,4);
                    zwork(i,j,k,l,m) = eef(3,4);
                end
            end
        end
    end
end

% plot workspace in 3D
figure(1)
scatter3(xwork(:),ywork(:),zwork(:),'.')
xlabel("x (cm)");
ylabel("y (cm)");
zlabel("z (cm)");
title("Reachable Workspace of Lynxmotion AL5D")
axis equal

figure(2)
scatter(xwork(:),ywork(:),'.')
xlabel("x (cm)");
ylabel("y (cm)");
title("Reachable Workspace of Lynxmotion AL5D")
axis equal

figure(3)
scatter(xwork(:),zwork(:),'.')
xlabel("x (cm)");
ylabel("z (cm)");
title("Reachable Workspace of Lynxmotion AL5D")
axis equal

figure(4)
scatter(ywork(:),zwork(:),'.')
xlabel("y (cm)");
ylabel("z (cm)");
title("Reachable Workspace of Lynxmotion AL5D")
axis equal