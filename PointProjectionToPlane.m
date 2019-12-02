function [I] = PointProjectionToPlane(imX,imY, Kc1, GT_pose, normal, d)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% GT_pose - projects points from camera to world coordinate frame
% imX,imY are pixel coordinates on the image
% normal, d are the parameters of the plane

imCoord = [imX; imY; 1];
imCoordNorm = inv(Kc1) * imCoord;
% normalize to Z=1 image plane
imCoordNorm = imCoordNorm / imCoordNorm(3);

% projection vector P0P1 in camera 3D coordinate system
P0 = [0;0;0];
P1 = imCoordNorm;
P1 = P1 / norm(P1);

% projection vector
proj_vect = P1;

% apply camera pose to projection vector and origin point
P03D = GT_pose * [P0; 1];
P03D = P03D(1:3);
% P13D = GT_pose * [P1; 1];
proj_vect_WCF = GT_pose(1:3,1:3) * proj_vect;

% % check projection line
% line = [P03D(1:3)'; P13D(1:3)'];
% %figure; pcshow(points_filtdist_hpr);
% %hold on; plot3(line(:,1),line(:,2),line(:,3));

% find intersection point of camera projection ray, with plane defined with
% n,d
% projection ray is already placed in 3D WCF by applying pose to origin,
% and rotation of pose to unit direction vector
syms t
lineEq = P03D+t*proj_vect_WCF;  %equation of 3d line
Intersect = normal(1)*lineEq(1) + normal(2)*lineEq(2) + normal(3)*lineEq(3) + d*1 == 0;
t_intersect = solve(Intersect,t);
intersection = double(subs(lineEq,t_intersect));

I = intersection;

% check plane equation with intersection point
error = intersection' * normal + d;



end

