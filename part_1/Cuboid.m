classdef Cuboid
    properties
        x = [-1 1]
        y = [-1 1]
        z = [-1 1]
        skeleton
        offset
    end
    methods
        function obj = Cuboid(offset, scale)
            % scale box
            obj.x = obj.x * scale(1);
            obj.y = obj.y * scale(2);
            obj.z = obj.z * scale(3);

            % define cross sections
            cross = zeros(2,5,3);
            cross(1,:,:) = [obj.x(1) 0 obj.x(1); obj.x(2) 0 obj.x(1); obj.x(2) 0 obj.x(2); obj.x(1) 0 obj.x(2); obj.x(1) 0 obj.x(1)];
            cross(2,:,:) = [0 obj.y(1) obj.x(1); 0 obj.y(2) obj.x(1); 0 obj.y(2) obj.x(2); 0 obj.y(1) obj.x(2); 0 obj.y(1) obj.x(1)];

            % apply offsets
            obj.offset = offset;
            cross(:,:,1) = cross(:,:,1) + offset(1);
            cross(:,:,2) = cross(:,:,2) + offset(2);
            cross(:,:,3) = cross(:,:,3) + offset(3);

            % create skeleton from cross sections
            obj.skeleton = zeros(4,5,3);
            obj.skeleton(1,:,:) = cross(1,:,:) + reshape([0 obj.y(1) 0],1,1,3);
            obj.skeleton(2,:,:) = cross(1,:,:) + reshape([0 obj.y(2) 0],1,1,3);
            obj.skeleton(3,:,:) = cross(2,:,:) + reshape([obj.x(1) 0 0],1,1,3);
            obj.skeleton(4,:,:) = cross(2,:,:) + reshape([obj.x(2) 0 0],1,1,3);
        end

        function draw(obj)
            hold_start = ishold;
            % draw XZ sides
            for i=1:size(obj.skeleton,1)
                plot3(obj.skeleton(i,:,1),obj.skeleton(i,:,2),obj.skeleton(i,:,3),'g-')
                if hold_start == 0
                    hold on
                end
            end
            if hold_start == 0
                hold off
            end
        end

        function inside = point_inside(obj, point)
            inside = 0;
            px = point(1);
            py = point(2);
            pz = point(3);
            
            ox = obj.x + obj.offset(1);
            oy = obj.y + obj.offset(2);
            oz = obj.z + obj.offset(3);
            if px > ox(1) && px < ox(2) && py > oy(1) && py < oy(2) && pz > oz(1) && pz < oz(2)
                inside = 1;
            end
        end
    end
end
