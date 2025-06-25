%%  Gravity term of 6DoF robotic manipulator system
% which in used many T-RO papers. such as
% A. Dietrich and C. Ott, "Hierarchical Impedance-Based Tracking Control of Kinematically Redundant Robots," in IEEE Transactions on Robotics, vol. 36, no. 1, pp. 204-221, Feb. 2020, doi: 10.1109/TRO.2019.2945876.
% keywords: {Task analysis;Robots;Aerospace electronics;Asymptotic stability;Stability analysis;Dynamics;Mathematical model;Force control;hierarchical control;impedance control;physical human–robot interaction;redundant robots;whole-body control},
% AND
%Y. Yuan and W. Sun, "Continuously Shaping Prioritized Jacobian Approach for Hierarchical Optimal Control With Task Priority Transition," in IEEE Transactions on Robotics, vol. 41, pp. 1639-1656, 2025, doi: 10.1109/TRO.2025.3539204. keywords: {Robots;Jacobian matrices;Stability analysis;Switches;Optimal control;Aerospace electronics;Vectors;Null space;Asymptotic stability;Sun;Control continuity;hierarchy of tasks;priority switching;redundant robots;torque-based control},

% This matlab function file is the gravity term of 
% this 6DoF robotic dynamic system. 
% An animation file and test file should be posted in the same folder.
% Written by Chengyue Li, University of Louisiana at Lafayette, June 25, 2025.

%% Input
%q -----> 6x1 unit:m,rad,rad,rad,rad,rad                ||| state variable 
%L -----> 6x1 unit:m,m,m,m,m,m                          ||| Length of each link
%m -----> 6x1 unit:kg,kg,kg,kg,kg,kg                    ||| Mass of each link

%% Output G_ -----> 6x1 unit:Nm
function [G_] = dyn_6DoF_gravity_term(q,L,m)

x1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);

L1=L(1);
L2=L(2);
L3=L(3);
L4=L(4);
L5=L(5);
L6=L(6);

m1=m(1);
m2=m(2);
m3=m(3);
m4=m(4);
m5=m(5);
m6=m(6);


G_1 = 0;

G_2 = m3*((L3*cos(q2 + q3))/2 + L2*cos(q2)) + m6*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2)) + m4*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3) + L2*cos(q2)) + m5*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2)) + (L2*m2*cos(q2))/2;
 
G_3 = m6*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + m4*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3)) + m5*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3)) + (L3*m3*cos(q2 + q3))/2;
 
G_4 = m6*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + m5*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4)) + (L4*m4*cos(q2 + q3 + q4))/2;
   
G_5 = m6*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + (L5*m5*cos(q2 + q3 + q4 + q5))/2;
 
G_6 = (L6*m6*cos(q2 + q3 + q4 + q5 + q6))/2;




G_ = [G_1 ;
      G_2 ;
      G_3 ;
      G_4 ;
      G_5 ;
      G_6 ];
end