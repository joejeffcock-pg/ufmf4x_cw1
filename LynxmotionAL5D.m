classdef LynxmotionAL5D
    properties (Constant)
        % link lengths (cm)
        d1 = 18.5
        d2 = 14.5
        d3 = 18.75
        d4 = 4.3
        % gripper finger lengths
        f1 = 3.2
        f2 = 4.3
    end
    properties
        eef_pose
    end
    methods
        function T = forward_kinematics(obj, q1, q2, q3, q4, q5)
            % transforms of lynxmotion
            T01  = tf_from_distal(0, deg2rad(90), obj.d1, q1);
            T12  = tf_from_distal(obj.d2, 0, 0, q2);
            T23  = tf_from_distal(obj.d3, 0, 0, q3);
            T33p = tf_from_distal(0, deg2rad(90), 0, q4);
            T3p4 = tf_from_distal(0, 0, obj.d4, 0);
            T45  = tf_from_distal(0, 0, 0, q5);

            % forward kinematics
            T = T01 * T12 * T23 * T33p * T3p4 * T45;
        end

        function joints = inverse_kinematics(obj, x, y, z, psi, mu)
            q1 = atan2(y,x);
            q5 = mu;
            
            % compute wrist position
            % base->wrist is a 3DoF problem
            vec_z = obj.d4 * sin(psi);
            D1 = obj.d4 * cos(psi);
            vec_x = D1 * cos(q1);
            vec_y = D1 * sin(q1);
            xw = x - vec_x;
            yw = y - vec_y;
            zw = z - vec_z;

            % solve for q2 from wrist
            r1 = sqrt(xw^2 + yw^2);
            r2 = zw - obj.d1;
            r3 = sqrt(r1^2 + r2^2);
            phi2 = atan2(r2,r1);
            phi1 = acos((obj.d3^2 - obj.d2^2 - r3^2)/(-2 * obj.d2 * r3));
            q2 = phi2 + phi1;

            % solve for q3 from wrist
            phi3 = acos((r3^2 - obj.d2^2 - obj.d3^2)/(-2*obj.d2*obj.d3));
            q3 = -(pi - phi3);

            % add pi/2 in psi to obtain rotation in robot frame
            q4 = (psi + pi/2) - q2 - q3;
            joints = [q1 q2 q3 q4 q5];
        end

        function draw(obj, q1, q2, q3, q4, q5)
            % transforms of lynxmotion
            T = zeros(7,4,4);
            T(1,:,:) = eye(4);
            T(2,:,:) = [tf_from_distal(0, deg2rad(90), obj.d1, q1)];
            T(3,:,:) = [tf_from_distal(obj.d2, 0, 0, q2)];
            T(4,:,:) = [tf_from_distal(obj.d3, 0, 0, q3)];
            T(5,:,:) = [tf_from_distal(0, deg2rad(90), 0, q4)];
            T(6,:,:) = [tf_from_distal(0, 0, obj.d4, 0)];
            T(7,:,:) = [tf_from_distal(0, 0, 0, q5)];

            % joint positions from FK
            T0eef = eye(4);
            points = zeros(7,3);
            for i = 1:7
                T0eef = T0eef * squeeze(T(i,:,:));
                points(i,1) = T0eef(1,4);
                points(i,2) = T0eef(2,4);
                points(i,3) = T0eef(3,4);
            end

            % gripper points
            T0g1f1 = T0eef  * [tf_from_distal( obj.f1, 0, 0     , 0)];
            T0g1f2 = T0g1f1 * [tf_from_distal( 0     , 0, obj.f2, 0)];
            T0g2f1 = T0eef  * [tf_from_distal(-obj.f1, 0, 0     , 0)];
            T0g2f2 = T0g2f1 * [tf_from_distal( 0     , 0, obj.f2, 0)];
            g1 = [T0eef(1:3, 4) T0g1f1(1:3, 4) T0g1f2(1:3, 4)];
            g2 = [T0eef(1:3, 4) T0g2f1(1:3, 4) T0g2f2(1:3, 4)];

            % plot points
            plot3(points(:,1), points(:,2), points(:,3), 'ko-', 'Linewidth', 2);
            hold on
            plot3(g1(1,:), g1(2,:), g1(3,:), 'k-', 'Linewidth', 2);
            plot3(g2(1,:), g2(2,:), g2(3,:), 'k-', 'Linewidth', 2);
            hold off
        end
    end
end

% Transform from DH Parameters (Distal)
function T = tf_from_distal(a, alpha, d, theta)
    T = [cos(theta) -cos(alpha)*sin(theta)  sin(alpha)*sin(theta) a*cos(theta);
         sin(theta)  cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta);
         0           sin(alpha)             cos(alpha)            d           ;
         0           0                      0                     1            ];
end