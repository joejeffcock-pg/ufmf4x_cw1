classdef planarRobot
    properties
        Sa = 150;   % first link
        L = 150;    % second link
        R = 500;    % outer size
        r = 80;     % center plate size
        x_offset = 500/2;
        y_offset = 500*sin(60*pi/180)/2;
    end

    methods
        
        % function for calculating angle for each origin
        function angle = IK(obj, px, py, phi_1)
            
            px1 = px + obj.x_offset;
            py1 = py + obj.y_offset;

            % Transformation matrix from point 1 to 2
            T12_prime = DH_transform(obj.R,0,0,0);
            T12 = DH_transform(0,0,0, 2*pi/3);
            T12 = T12_prime*T12;
            T21 = T12^(-1);

            % Transformation matrix from point 1 to 3
            T13_prime = DH_transform(0,0,0,4*pi/3);
            T13 = DH_transform(-obj.R,0,0,0);
            T13 = T13_prime*T13;
            T31 = T13^(-1);
            
            %P2x = T21 * P1x         
            P1 = [px1 py1 0 1]';
            P2 = T21*P1;
            P3 = T31*P1;
            
            % extract x y position for each pose
            px2 = P2(1,1);
            py2 = P2(2,1);
            px3 = P3(1,1);
            py3 = P3(2,1);
            
            
            % calculation each theta 
            theta_1 = CalTheta(obj.Sa, obj.L, obj.r, phi_1, px1, py1);
            theta_2 = CalTheta(obj.Sa, obj.L, obj.r, phi_1, px2, py2);
            theta_3 = CalTheta(obj.Sa, obj.L, obj.r, phi_1, px3, py3);
            
            % angle for each origin [c+d c-d], use c+d
            angle = [theta_1
                     theta_2
                     theta_3];
        end   

        
        function draw(obj, px, py, phi_1, theta_1, theta_2, theta_3)
            %% function for plotting the robot configuration

            px1 = px + obj.x_offset;
            py1 = py + obj.y_offset;

            % transformation matrix
            T12_prime = DH_transform(obj.R,0,0,0);
            T12 = DH_transform(0,0,0, 2*pi/3);
            T12 = T12_prime*T12;
            T21 = T12^(-1);

            % Transformation matrix from point 1 to 3
            T13_prime = DH_transform(0,0,0,4*pi/3);
            T13 = DH_transform(-obj.R,0,0,0);
            T13 = T13_prime*T13;
            T31 = T13^(-1);
            
            % center point viewd from different origin
            P1 = [px1 py1 0 1]';
            P2 = T21*P1;
            P3 = T31*P1;
            
            px2 = P2(1,1);
            py2 = P2(2,1);
            px3 = P3(1,1);
            py3 = P3(2,1);
      
            P = [px1 py1;
                 px2 py2;
                 px3 py3];
            
            ANGLE = [theta_1;
                     theta_2;
                     theta_3];
            
            % XY = 3 origins x 2 dimensions x 3 points
            % 3 points = each joints of one link
            XY = zeros(3,2,3);
            for i =1:3
                XY(i,1,2) = obj.Sa*cos(ANGLE(i,1));
                XY(i,2,2) = obj.Sa*sin(ANGLE(i,1));
                XY(i,1,3) = P(i,1) - obj.r*cos(phi_1);
                XY(i,2,3) = P(i,2) - obj.r*sin(phi_1);
            end
            
            % change P2xy to P1xy by P1xy = T12 x P2xy
            for i = 1:3
                x2 = squeeze(XY(2,1,i));
                y2 = squeeze(XY(2,2,i));
                p2 = [x2 y2 0 1]';
                p1 = T12*p2;
                XY(2,1,i) = p1(1,1);
                XY(2,2,i) = p1(2,1);
            end
            
            % change P3xy to P1xy by P1xy = T13 x P3xy
            for i = 1:3
                x3 = squeeze(XY(3,1,i));
                y3 = squeeze(XY(3,2,i));
                p3 = [x3 y3 0 1]';
                p1 = T13*p3;
                XY(3,1,i) = p1(1,1);
                XY(3,2,i) = p1(2,1);
            end

            % prepare for drawing
            x1 = squeeze(XY(1,1,:) - obj.x_offset);
            y1 = squeeze(XY(1,2,:) - obj.y_offset);
            x2 = squeeze(XY(2,1,:) - obj.x_offset);
            y2 = squeeze(XY(2,2,:) - obj.y_offset);
            x3 = squeeze(XY(3,1,:) - obj.x_offset);
            y3 = squeeze(XY(3,2,:) - obj.y_offset);
            
            % draw center plate
            planar_x = [x1(3) x2(3) x3(3) x1(3)];
            planar_y = [y1(3) y2(3) y3(3) y1(3)];
            
            % draw frame
            frame_x = [x1(1) x2(1) x3(1) x1(1)];
            frame_y = [y1(1) y2(1) y3(1) y1(1)];

            % draw center point
            plot(px,py,'kx')
            hold on

            % draw each link
            plot(x1,y1,'ko-', 'Linewidth', 2)
            plot(x2,y2,'ko-', 'Linewidth', 2)
            plot(x3,y3,'ko-', 'Linewidth', 2)
            plot(planar_x, planar_y, 'ko-', 'Linewidth', 2)
            plot(frame_x,frame_y,"LineStyle","--","Color",'blue');
            xlim([-300,300])
            ylim([-300,300])
            xlabel('x (mm)')
            ylabel('y (mm)')
            hold on
            grid on
        end
                
    end

end



function T = DH_transform(a,alpha,d,theta)
    T = [cos(theta) -cos(alpha)*sin(theta)   sin(alpha)*sin(theta)   a*cos(theta);
         sin(theta)  cos(alpha)*cos(theta)  -sin(alpha)*cos(theta)   a*sin(theta);
         0           sin(alpha)              cos(alpha)              d           ;
         0           0                       0                       1           ;];
end


function theta = CalTheta(Sa, L, r, phi, px, py)
    %% function for calculating theta
    x = px - r*cos(phi);
    y = py - r*sin(phi);
    r = sqrt(x^2 + y^2);
    c = atan2(y,x);
    d = acos((Sa^2 + r^2 - L^2)/(2 * Sa *r));
    theta = [c+d c-d];
end
