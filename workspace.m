d1 = 5;
d2 = 10;
d3 = 10;
d4 = 5;

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
                % transforms of lynxmotion
                T01  = tf_from_distal(0, deg2rad(90), d1, t1(i));
                T12  = tf_from_distal(d2, 0, 0, t2(j));
                T23  = tf_from_distal(d3, 0, 0, t3(k));
                T33p = tf_from_distal(0, deg2rad(90), 0, t4(l));
                T3p4 = tf_from_distal(0, 0, d4, 0);

                % forward kinematics
                eef = T01 * T12 * T23 * T33p * T3p4;

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

% Transform from DH Parameters (Distal)
function T = tf_from_distal(a, alpha, d, theta)
    T = [cos(theta) -cos(alpha)*sin(theta)  sin(alpha)*sin(theta) a*cos(theta);
         sin(theta)  cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta);
         0           sin(alpha)             cos(alpha)            d           ;
         0           0                      0                     1            ];
end