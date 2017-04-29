function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
x1 = video_pts(:,1);
x2 = video_pts(:,2);
x1_p = logo_pts(:,1);
x2_p = logo_pts(:,2);
A = zeros(8,9);
for i = 1:4
    ax = [-x1(i), -x2(i), -1, 0, 0, 0, x1(i)*x1_p(i), x2(i)*x1_p(i), x1_p(i)];
    ay = [0, 0, 0, -x1(i), -x2(i), -1, x1(i)*x2_p(i), x2(i)*x2_p(i), x2_p(i)];
    A(2*i-1,:) = ax;
    A(2*i,:) = ay;
end
[U,S,V] = svd(A);
h = V(:,end);
[H, p] = vec2mat(h,3);

end

