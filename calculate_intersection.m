function [result] = calculate_intersection(i1, i2, stereoParams, plane_properties)
    close all;

    [arm_properties] = arm_segmenter(i1,i2, stereoParams);
    I1 = imread(i1);
    I2 = imread(i2);

    x1 = 226;
    y1 = 172;
    x2 = 503;
    y2 = 141;
    x3 = 503;
    y3 = 311;
    
    %scale
    
    proj_tl_pixel = [238,151];
    proj_tr_pixel = [497, 161];
    proj_bl_pixel = [233, 296];
    
%     z1 = Z(y1,x1)
%     z2 = Z(y2,x2)
%     z3 = Z(y3,x3)
    z1 = 5.1378e+03;
    z2 = 5.1707e+03;
    z3 = 4.9487e+03;

    P1 = [x1,y1,z1];
    P2 = [x2,y2,z2];
    P3 = [x3,y3,z3];
%     
%     normal = cross(P1-P2, P1-P3)   
%     
%     syms x y z
%     P = [x,y,z]
%     planefunction = dot(normal, P-P1)
%     
%     dot(P-P1, normal)
%     
%     realdot = @(u, v) u*transpose(v);
%     
%     realdot(P-P1,normal)
    
    % LINE
    
    %P4 = [240,288,vall];
    %P5 = [274,267,ujj];
    
    line_x1 = arm_properties(1,1);
    line_y1 = arm_properties(1,2);
    line_z1 = arm_properties(1,3);
    
    line_x2 = arm_properties(2,1);
    line_y2 = arm_properties(2,2);
    line_z2 = arm_properties(2,3);
    
    P4 = [line_x1, line_y1, line_z1];
    P5 = [line_x2, line_y2, line_z2];
    
    %Emil
    figure
    [normal_test, ~] = plot_line(P1,P2,P3); hold on
    v=[P4;P5];
    plot3(v(:,1),v(:,2),v(:,3),'r')
    
    [I,check] = plane_line_intersect(normal_test,P1, P4, P5);
    plot3(I(1),I(2),I(3),'-r*', 'Markersize',20)
    
%     syms t
%     line = P4 + t*(P5-P4)
%     line = rad-(sin(theta)*)
%     
%     newfunction = subs(planefunction, P, line)
%     t0 = solve(newfunction)
%     point = subs(line, t, t0)
%     subs(planefunction, P, point)
%     
%     zplane = solve(planefunction, z)
%     
%     
%     
%     
%     szilva = 200
%     korte = 800
%     cseresznye = 600 
%     megy = 0
%     
%     
%     ezplot3(line(1), line(2), line(3), [-1,3]), hold on
%     ezmesh(zplane, [szilva, korte, szilva, korte]), hold off
%     axis([szilva, korte, szilva, korte, megy, cseresznye]), title 'C�m'
%    
%     on_projector_X = round(point(1))
%     on_projector_Y = round(point(2))
%     on_projector_Z = round(point(3))
%     
%     k1 = -tan(theta);
%     b1 = rad/cos(theta);
%     X = 0:600;
%     hold off;
%     figure;
%     imshow(I1_r);hold on;
%     plot(X, k1*X+b1,'r')
%     
%     I1_r = draw_pointer(I1_r,on_projector_X,on_projector_Y);
%     
%     I1_r(line_x1,round(line_y1), 1) = 255;
%     I1_r(line_x1,round(line_y1), 2) = 0;
%     I1_r(line_x1,round(line_y1), 3) = 255;
%     
%     I1_r(line_x2,round(line_y2), 1) = 255;
%     I1_r(line_x2,round(line_y2), 2) = 0;
%     I1_r(line_x2,round(line_y2), 3) = 255;
    
    
    %Emil
    figure;
    imshow(I1); hold on;
    plot(I(1), I(2),'-r*', 'Markersize',20)
    %plot(v(:,1),v(:,2), 'r')
    %[1170, 66] 1920 × 1080
%     tmp1 = I(1:2) - proj_tl_pixel;
%     tmp2 = proj_bl_pixel - proj_tl_pixel;
%     scaled_y = tmp1(2)/ tmp2(2);
%     
%     tmp1 = I(1:2) - proj_tl_pixel;
%     tmp2 = proj_tr_pixel - proj_tl_pixel;
%     scaled_x = tmp1(1) / tmp2(1);
%     result = [scaled_y, scaled_x] .*  [1920, 1080];
   
    %GT = [1183, 30];
    point = I(1:2);
    
    projWidth = 1920;
    projHeight = 1080;
    
    A_x_ratio = proj_tr_pixel(1) - proj_tl_pixel(1);
    A_y_ratio = proj_bl_pixel(2) - proj_tl_pixel(2);
    
    A_x_ratio = A_x_ratio / projWidth;
    A_y_ratio = A_y_ratio / projHeight;
    
    pointXScale = point(1) - proj_tl_pixel(1);
    pointX = pointXScale/A_x_ratio;
    
    pointYScale = point(2) - proj_tl_pixel(2);
    pointY = pointYScale/A_y_ratio;
    result = [pointX, pointY];
%     tmp1 = GT(1:2) - proj_tl_pixel;
%     tmp2 = proj_bl_pixel - proj_tl_pixel;
%     scaled_y = tmp1(2)/ tmp2(2);
%     
%     tmp1 = GT(1:2) - proj_tl_pixel;
%     tmp2 = proj_tr_pixel - proj_tl_pixel;
%     scaled_x = tmp1(1) / tmp2(1);
%     
%     resultGT = [scaled_x, scaled_y] .*  [1920, 1080];
end