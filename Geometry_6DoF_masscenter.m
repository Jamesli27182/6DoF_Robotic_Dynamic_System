%%  Geometry of mass center of 6DoF robotic manipulator system
% which in used many T-RO papers. such as
% A. Dietrich and C. Ott, "Hierarchical Impedance-Based Tracking Control of Kinematically Redundant Robots," in IEEE Transactions on Robotics, vol. 36, no. 1, pp. 204-221, Feb. 2020, doi: 10.1109/TRO.2019.2945876.
% keywords: {Task analysis;Robots;Aerospace electronics;Asymptotic stability;Stability analysis;Dynamics;Mathematical model;Force control;hierarchical control;impedance control;physical human–robot interaction;redundant robots;whole-body control},
% AND
%Y. Yuan and W. Sun, "Continuously Shaping Prioritized Jacobian Approach for Hierarchical Optimal Control With Task Priority Transition," in IEEE Transactions on Robotics, vol. 41, pp. 1639-1656, 2025, doi: 10.1109/TRO.2025.3539204. keywords: {Robots;Jacobian matrices;Stability analysis;Switches;Optimal control;Aerospace electronics;Vectors;Null space;Asymptotic stability;Sun;Control continuity;hierarchy of tasks;priority switching;redundant robots;torque-based control},

% This matlab function file is the Mass center of each link of 
% this 6DoF robotic dynamic system. 
% An animation file and test file should be posted in the same folder.
% Written by Chengyue Li, University of Louisiana at Lafayette, June 25, 2025.
%% Input 
%q -----> 6x1 unit:m,rad,rad,rad,rad,rad                ||| state variable 
%L -----> 6x1 unit:m,m,m,m,m,m                          ||| Length of each link

%% Output
% Pos_output ---> 6x2
% column represents the position x and y
% row    represents the linkage index
function [Pos_output] = Geometry_6DoF_masscenter(q,L)
x1            = q(1);
q2            = q(2);
q3            = q(3);
q4            = q(4);
q5            = q(5);
q6            = q(6);
L1            = L(1);
L2            = L(2);
L3            = L(3);
L4            = L(4);
L5            = L(5);
L6            = L(6);
Pos_output    = zeros(6,2);

% Geometry
Joint1_x      = x1; 
Joint2_x      = Joint1_x;
Joint3_x      = Joint2_x + L2*cos(q2);
Joint4_x      = Joint3_x + L3*cos(q2+q3);
Joint5_x      = Joint4_x + L4*cos(q2+q3+q4);
Joint6_x      = Joint5_x + L5*cos(q2+q3+q4+q5);


Joint1_y      = 0;
Joint2_y      = Joint1_y + L1;
Joint3_y      = Joint2_y + L2*sin(q2); 
Joint4_y      = Joint3_y + L3*sin(q2+q3);
Joint5_y      = Joint4_y + L4*sin(q2+q3+q4);
Joint6_y      = Joint5_y + L5*sin(q2+q3+q4+q5);


%% Dynamic
obj_1_cx = x1;
obj_2_cx = Joint2_x + L2/2*cos(q2);
obj_3_cx = Joint3_x + L3/2*cos(q2+q3);
obj_4_cx = Joint4_x + L4/2*cos(q2+q3+q4);
obj_5_cx = Joint5_x + L5/2*cos(q2+q3+q4+q5);
obj_6_cx = Joint6_x + L6/2*cos(q2+q3+q4+q5+q6);

obj_1_cy = L1/2;
obj_2_cy = Joint2_y + L2/2*sin(q2);
obj_3_cy = Joint3_y + L3/2*sin(q2+q3);
obj_4_cy = Joint4_y + L4/2*sin(q2+q3+q4);
obj_5_cy = Joint5_y + L5/2*sin(q2+q3+q4+q5);
obj_6_cy = Joint6_y + L6/2*sin(q2+q3+q4+q5+q6);

Pos_output(1,1) = obj_1_cx;
Pos_output(2,1) = obj_2_cx;
Pos_output(3,1) = obj_3_cx;
Pos_output(4,1) = obj_4_cx;
Pos_output(5,1) = obj_5_cx;
Pos_output(6,1) = obj_6_cx;


Pos_output(1,2) = obj_1_cy;
Pos_output(2,2) = obj_2_cy;
Pos_output(3,2) = obj_3_cy;
Pos_output(4,2) = obj_4_cy;
Pos_output(5,2) = obj_5_cy;
Pos_output(6,2) = obj_6_cy;


end