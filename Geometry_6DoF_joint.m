%%  Geometry of joint position and end effector of 6DoF robotic manipulator system
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
% Pos_output ---> 7x2
% column represents the position x and y
% row    represents the linkage index
% row 1-6 -----> joints
% row 7   -----> end effector
function [Pos_output,Orie] = Geometry_6DoF_joint(q,L)
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
Pos_output    = zeros(7,2);

Joint1_x      = x1; 
Joint2_x      = Joint1_x;
Joint3_x      = Joint2_x + L2*cos(q2);
Joint4_x      = Joint3_x + L3*cos(q2+q3);
Joint5_x      = Joint4_x + L4*cos(q2+q3+q4);
Joint6_x      = Joint5_x + L5*cos(q2+q3+q4+q5);
endeffector_x = Joint6_x + L6*cos(q2+q3+q4+q5+q6);

Joint1_y      = 0;
Joint2_y      = Joint1_y + L1;
Joint3_y      = Joint2_y + L2*sin(q2); 
Joint4_y      = Joint3_y + L3*sin(q2+q3);
Joint5_y      = Joint4_y + L4*sin(q2+q3+q4);
Joint6_y      = Joint5_y + L5*sin(q2+q3+q4+q5);
endeffector_y = Joint6_y + L6*sin(q2+q3+q4+q5+q6);

Pos_output(1,1) = Joint1_x;
Pos_output(2,1) = Joint2_x;
Pos_output(3,1) = Joint3_x;
Pos_output(4,1) = Joint4_x;
Pos_output(5,1) = Joint5_x;
Pos_output(6,1) = Joint6_x;
Pos_output(7,1) = endeffector_x;

Pos_output(1,2) = Joint1_y;
Pos_output(2,2) = Joint2_y;
Pos_output(3,2) = Joint3_y;
Pos_output(4,2) = Joint4_y;
Pos_output(5,2) = Joint5_y;
Pos_output(6,2) = Joint6_y;
Pos_output(7,2) = endeffector_y;
Orie = zeros(5,1);
Orie(1)            = q2;            % joint3
Orie(2)            = q2+q3;         % joint4
Orie(3)            = q2+q3+q4;      % joint5
Orie(4)            = q2+q3+q4+q5;   % joint6
Orie(5)            = q2+q3+q4+q5+q6;% linkee
end