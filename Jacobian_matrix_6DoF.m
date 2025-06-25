%%  Jacobian matrix of 6DoF robotic manipulator system
% which in used many T-RO papers. such as
% A. Dietrich and C. Ott, "Hierarchical Impedance-Based Tracking Control of Kinematically Redundant Robots," in IEEE Transactions on Robotics, vol. 36, no. 1, pp. 204-221, Feb. 2020, doi: 10.1109/TRO.2019.2945876.
% keywords: {Task analysis;Robots;Aerospace electronics;Asymptotic stability;Stability analysis;Dynamics;Mathematical model;Force control;hierarchical control;impedance control;physical human–robot interaction;redundant robots;whole-body control},
% AND
%Y. Yuan and W. Sun, "Continuously Shaping Prioritized Jacobian Approach for Hierarchical Optimal Control With Task Priority Transition," in IEEE Transactions on Robotics, vol. 41, pp. 1639-1656, 2025, doi: 10.1109/TRO.2025.3539204. keywords: {Robots;Jacobian matrices;Stability analysis;Switches;Optimal control;Aerospace electronics;Vectors;Null space;Asymptotic stability;Sun;Control continuity;hierarchy of tasks;priority switching;redundant robots;torque-based control},

% This matlab function file is the Jacobian matrix of 
% this 6DoF robotic dynamic system. 
% An animation file and test file should be posted in the same folder.
% Written by Chengyue Li, University of Louisiana at Lafayette, June 25, 2025.

%% Input 
%q -----> 6x1 unit:m,rad,rad,rad,rad,rad                ||| state variable 
%L -----> 6x1 unit:m,m,m,m,m,m                          ||| Length of each link
%dq ----> 6x1 velocity 

%% Output
% Jacobian matrix of each joints and ee meas end effector with respect to
% the position
% dJ is the dot of Jacobian matrix
function [J1,J3,J4,J6,Jee,dJee] = Jacobian_matrix_6DoF(q,L,dq)

x1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);
L1 =L(1);
L2 =L(2);
L3 =L(3);
L4 =L(4);
L5 =L(5);
L6 =L(6);
dx1 =dq(1);
dq2 =dq(2);
dq3 =dq(3);
dq4 =dq(4);
dq5 =dq(5);
dq6 =dq(6);
J1 = [1 0 0 0 0 0;
      0 0 0 0 0 0];
Jee_16 = -L6*sin(q2 + q3 + q4 + q5 + q6);
Jee_15 = - L5*sin(q2 + q3 + q4 + q5) - L6*sin(q2 + q3 + q4 + q5 + q6);
Jee_14 = - L5*sin(q2 + q3 + q4 + q5) - L4*sin(q2 + q3 + q4) - L6*sin(q2 + q3 + q4 + q5 + q6);
Jee_13 = - L5*sin(q2 + q3 + q4 + q5) - L4*sin(q2 + q3 + q4) - L3*sin(q2 + q3) - L6*sin(q2 + q3 + q4 + q5 + q6);
Jee_12 = - L2*sin(q2) - L5*sin(q2 + q3 + q4 + q5) - L4*sin(q2 + q3 + q4) - L3*sin(q2 + q3) - L6*sin(q2 + q3 + q4 + q5 + q6);
Jee_11 = 1;

Jee_26 = L6*cos(q2 + q3 + q4 + q5 + q6);
Jee_25 = L5*cos(q2 + q3 + q4 + q5) + L6*cos(q2 + q3 + q4 + q5 + q6);
Jee_24 = L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L6*cos(q2 + q3 + q4 + q5 + q6);
Jee_23 = L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L6*cos(q2 + q3 + q4 + q5 + q6);
Jee_22 = L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L6*cos(q2 + q3 + q4 + q5 + q6) + L2*cos(q2);
Jee_21 = 0;
Jee    =[Jee_11,Jee_12,Jee_13,Jee_14,Jee_15,Jee_16;
         Jee_21,Jee_22,Jee_23,Jee_24,Jee_25,Jee_26];
J6_11 = 1;
J6_12 = - L2*sin(q2) - L5*sin(q2 + q3 + q4 + q5) - L4*sin(q2 + q3 + q4) - L3*sin(q2 + q3);
J6_13 = - L5*sin(q2 + q3 + q4 + q5) - L4*sin(q2 + q3 + q4) - L3*sin(q2 + q3);
J6_14 = - L5*sin(q2 + q3 + q4 + q5) - L4*sin(q2 + q3 + q4);
J6_15 = -L5*sin(q2 + q3 + q4 + q5);
J6_16 = 0;

J6_21 = 0;
J6_22 = L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2);
J6_23 = L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3);
J6_24 = L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4);
J6_25 = L5*cos(q2 + q3 + q4 + q5);
J6_26 = 0;
J6    =[J6_11,J6_12,J6_13,J6_14,J6_15,J6_16;
        J6_21,J6_22,J6_23,J6_24,J6_25,J6_26];

J3_21 = 0;
J3_22 = L2*cos(q2);
J3_23 = 0;
J3_24 = 0;
J3_25 = 0;
J3_26 = 0;

J3_11 = 1;
J3_12 = -L2*sin(q2);
J3_13 = 0;
J3_14 = 0;
J3_15 = 0;
J3_16 = 0;
J3    =[J3_11,J3_12,J3_13,J3_14,J3_15,J3_16;
        J3_21,J3_22,J3_23,J3_24,J3_25,J3_26];



J4_21 = 0;
J4_22 = L3*cos(q2 + q3) + L2*cos(q2);
J4_23 = L3*cos(q2 + q3);
J4_24 = 0;
J4_25 = 0;
J4_26 = 0;

J4_11 = 1;
J4_12 = - L3*sin(q2 + q3) - L2*sin(q2);
J4_13 = - L3*sin(q2 + q3);
J4_14 = 0;
J4_15 = 0;
J4_16 = 0;
J4    =[J4_11,J4_12,J4_13,J4_14,J4_15,J4_16;
        J4_21,J4_22,J4_23,J4_24,J4_25,J4_26];









dJee_16 = -L6*cos(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6);
dJee_15 = - L5*cos(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5) - L6*cos(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6);
 
dJee_14 = - L5*cos(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5) - L4*cos(q2 + q3 + q4)*(dq2 + dq3 + dq4) - L6*cos(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6);
 

dJee_13 = - L3*cos(q2 + q3)*(dq2 + dq3) - L5*cos(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5) - L4*cos(q2 + q3 + q4)*(dq2 + dq3 + dq4) - L6*cos(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6);
 
dJee_12 = - L3*cos(q2 + q3)*(dq2 + dq3) - L5*cos(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5) - L2*cos(q2)*dq2 - L4*cos(q2 + q3 + q4)*(dq2 + dq3 + dq4) - L6*cos(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6);
 

dJee_11 = 0;

dJee_26 = -L6*sin(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6);
 

dJee_25 = - L6*sin(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6) - L5*sin(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5);
 

dJee_24 = - L6*sin(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6) - L5*sin(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5) - L4*sin(q2 + q3 + q4)*(dq2 + dq3 + dq4);
 

dJee_23 = - L6*sin(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6) - L3*sin(q2 + q3)*(dq2 + dq3) - L5*sin(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5) - L4*sin(q2 + q3 + q4)*(dq2 + dq3 + dq4);
 

dJee_22 = - L6*sin(q2 + q3 + q4 + q5 + q6)*(dq2 + dq3 + dq4 + dq5 + dq6) - L3*sin(q2 + q3)*(dq2 + dq3) - L5*sin(q2 + q3 + q4 + q5)*(dq2 + dq3 + dq4 + dq5) - L2*sin(q2)*dq2 - L4*sin(q2 + q3 + q4)*(dq2 + dq3 + dq4);
 

dJee_21 = 0;
dJee    =[dJee_11,dJee_12,dJee_13,dJee_14,dJee_15,dJee_16;
          dJee_21,dJee_22,dJee_23,dJee_24,dJee_25,dJee_26];


end

