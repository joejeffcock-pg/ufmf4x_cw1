function trajectory = cartesian_space_trajectory(arm,p1,p2,c_samples,j_samples,poly)
    % interpolate between P1 and P2 to sample poses
    % equivalent to f(t) = P1(1-t) + P2(t) for 0<=t<=1
    sample_poses = zeros(c_samples, length(p1));
    for j = 1:length(p1)
        sample_poses(:,j) = linspace(p1(j), p2(j), c_samples);
    end

    % compute IK for sampled poses in cartesian space
    % to obtain c_samples in joint space coordinates
    sample_joints = zeros(c_samples,5);
    for i = 1:size(sample_poses,1)
        ps = sample_poses(i,:);
        sample_joints(i,:) = arm.inverse_kinematics(ps(1),ps(2),ps(3),ps(4),ps(5));
    end

    % for each joint
    trajectory = zeros(j_samples,5);
    for i = 1:5
        % fit a polynomial to the joint space coordinates over time
        yy1 = sample_joints(:,i);
        xx1 = linspace(0,1,c_samples); % increments of 1 unit
        p = polyfit(xx1,yy1,poly);

        % resample from the fit polynomial
        % to obtain cartesian trajectory in joint space
        xx2 = linspace(0,1,j_samples); % increments of 1 unit
        trajectory(:,i) = polyval(p,xx2,poly);
    end
end