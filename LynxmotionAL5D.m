classdef LynxmotionAL5D
    properties (Constant)
        d1 = 5
        d2 = 10
        d3 = 10
        d4 = 5
    end
    properties
        eef_pose
    end
    methods
        function T = forward_kinematics(obj, q1, q2, q3, q4)
            % transforms of lynxmotion
            T01  = tf_from_distal(0, deg2rad(90), obj.d1, q1);
            T12  = tf_from_distal(obj.d2, 0, 0, q2);
            T23  = tf_from_distal(obj.d3, 0, 0, q3);
            T33p = tf_from_distal(0, deg2rad(90), 0, q4);
            T3p4 = tf_from_distal(0, 0, obj.d4, 0);

            % forward kinematics
            T = T01 * T12 * T23 * T33p * T3p4;
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
            q2 = phi2 - phi1;

            % solve for q3 from wrist
            phi3 = acos((r3^2 - obj.d2^2 - obj.d3^2)/(-2*obj.d2*obj.d3));
            q3 = pi - phi3;

            % add pi/2 in psi to obtain rotation in robot frame
            q4 = (psi + pi/2) - q2 - q3;
            joints = [q1 q2 q3 q4 q5];
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