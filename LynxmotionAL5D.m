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
    end
end

% Transform from DH Parameters (Distal)
function T = tf_from_distal(a, alpha, d, theta)
    T = [cos(theta) -cos(alpha)*sin(theta)  sin(alpha)*sin(theta) a*cos(theta);
         sin(theta)  cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta);
         0           sin(alpha)             cos(alpha)            d           ;
         0           0                      0                     1            ];
end