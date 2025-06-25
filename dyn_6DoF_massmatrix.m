%%  Mass matrix of 6DoF robotic manipulator system
% which in used many T-RO papers. such as
% A. Dietrich and C. Ott, "Hierarchical Impedance-Based Tracking Control of Kinematically Redundant Robots," in IEEE Transactions on Robotics, vol. 36, no. 1, pp. 204-221, Feb. 2020, doi: 10.1109/TRO.2019.2945876.
% keywords: {Task analysis;Robots;Aerospace electronics;Asymptotic stability;Stability analysis;Dynamics;Mathematical model;Force control;hierarchical control;impedance control;physical human–robot interaction;redundant robots;whole-body control},
% AND
%Y. Yuan and W. Sun, "Continuously Shaping Prioritized Jacobian Approach for Hierarchical Optimal Control With Task Priority Transition," in IEEE Transactions on Robotics, vol. 41, pp. 1639-1656, 2025, doi: 10.1109/TRO.2025.3539204. keywords: {Robots;Jacobian matrices;Stability analysis;Switches;Optimal control;Aerospace electronics;Vectors;Null space;Asymptotic stability;Sun;Control continuity;hierarchy of tasks;priority switching;redundant robots;torque-based control},

% This matlab function file is the Mass matrix of 
% this 6DoF robotic dynamic system. 
% An animation file and test file should be posted in the same folder.
% Written by Chengyue Li, University of Louisiana at Lafayette, June 25, 2025.

%% Input 
%q -----> 6x1 unit:m,rad,rad,rad,rad,rad                ||| state variable 
%L -----> 6x1 unit:m,m,m,m,m,m                          ||| Length of each link
%m -----> 6x1 unit:kg,kg,kg,kg,kg,kg                    ||| Mass of each link
%I -----> 5x1 unit:kg*m^2,kg*m^2,kg*m^2,kg*m^2,kg*m^2   ||| Moment of
 %inertia and first joint is translational joint no moment of inertia
%% Output
% M_ ---> 6x6 
function [M_]=dyn_6DoF_massmatrix(q,L,m,I)
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

I2=I(1);
I3=I(2);
I4=I(3);
I5=I(4);
I6=I(5);

M11 = m1 + m2 + m3 + m4 + m5 + m6;

M12 = - (m3*(2*L2*sin(q2) + L3*sin(q2 + q3)))/2 - (m4*(2*L2*sin(q2) + L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (m6*(2*L2*sin(q2) + 2*L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (m5*(2*L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (L2*m2*sin(q2))/2;
 
M13 = - (m4*(L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (m6*(2*L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (m5*(L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (L3*m3*sin(q2 + q3))/2;

M14 = - (m6*(2*L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (m5*(L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4)))/2 - (L4*m4*sin(q2 + q3 + q4))/2;
 
M15 = - (m6*(2*L5*sin(q2 + q3 + q4 + q5) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (L5*m5*sin(q2 + q3 + q4 + q5))/2;
 
M16 = -(L6*m6*sin(q2 + q3 + q4 + q5 + q6))/2;
 

M21 = - (m3*(2*L2*sin(q2) + L3*sin(q2 + q3)))/2 - (m4*(2*L2*sin(q2) + L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (m6*(2*L2*sin(q2) + 2*L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (m5*(2*L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (L2*m2*sin(q2))/2;
 
M22 = I2 + I3 + I4 + I5 + I6 + (m6*(2*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)^2 + 2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2))^2))/2 + (m4*(2*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3) + L2*cos(q2))^2 + 2*(L2*sin(q2) + (L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3))^2))/2 + (m5*(2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2))^2 + 2*(L2*sin(q2) + (L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))^2))/2 + (m2*((L2^2*cos(q2)^2)/2 + (L2^2*sin(q2)^2)/2))/2 + (m3*(2*(L2*sin(q2) + (L3*sin(q2 + q3))/2)^2 + 2*((L3*cos(q2 + q3))/2 + L2*cos(q2))^2))/2;
 
M23 = I3 + I4 + I5 + I6 + (m5*(2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))*(L2*sin(q2) + (L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3)) + 2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3))*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2))))/2 + (m6*(2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2))))/2 + (m3*(L3*cos(q2 + q3)*((L3*cos(q2 + q3))/2 + L2*cos(q2)) + L3*sin(q2 + q3)*(L2*sin(q2) + (L3*sin(q2 + q3))/2)))/2 + (m4*(2*((L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3))*(L2*sin(q2) + (L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3)) + 2*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3))*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3) + L2*cos(q2))))/2;
 
M24 = I4 + I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2)) + 2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4))*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2)) + 2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4))*(L2*sin(q2) + (L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2 + (m4*(L4*sin(q2 + q3 + q4)*(L2*sin(q2) + (L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3)) + L4*cos(q2 + q3 + q4)*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3) + L2*cos(q2))))/2;
 
M25 = I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2)) + 2*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(L5*cos(q2 + q3 + q4 + q5)*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2)) + L5*sin(q2 + q3 + q4 + q5)*(L2*sin(q2) + (L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2;
 
M26 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2)) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 

M31 = - (m4*(L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (m6*(2*L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (m5*(L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + 2*L3*sin(q2 + q3)))/2 - (L3*m3*sin(q2 + q3))/2;
 
M32 = I3 + I4 + I5 + I6 + (m5*(2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))*(L2*sin(q2) + (L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3)) + 2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3))*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2))))/2 + (m6*(2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2))))/2 + (m3*(L3*cos(q2 + q3)*((L3*cos(q2 + q3))/2 + L2*cos(q2)) + L3*sin(q2 + q3)*(L2*sin(q2) + (L3*sin(q2 + q3))/2)))/2 + (m4*(2*((L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3))*(L2*sin(q2) + (L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3)) + 2*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3))*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3) + L2*cos(q2))))/2;
 
M33 = I3 + I4 + I5 + I6 + (m6*(2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)^2 + 2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)^2))/2 + (m3*((L3^2*sin(q2 + q3)^2)/2 + (L3^2*cos(q2 + q3)^2)/2))/2 + (m4*(2*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3))^2 + 2*((L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3))^2))/2 + (m5*(2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3))^2 + 2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))^2))/2;

M34 = I4 + I5 + I6 + (m4*(L4*sin(q2 + q3 + q4)*((L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3)) + L4*cos(q2 + q3 + q4)*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3))))/2 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4))*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3)) + 2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4))*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2;

M35 = I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(L5*cos(q2 + q3 + q4 + q5)*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3)) + L5*sin(q2 + q3 + q4 + q5)*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2;
 
M36 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 

M41 = - (m6*(2*L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (m5*(L5*sin(q2 + q3 + q4 + q5) + 2*L4*sin(q2 + q3 + q4)))/2 - (L4*m4*sin(q2 + q3 + q4))/2;
 
M42 = I4 + I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2)) + 2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4))*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2)) + 2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4))*(L2*sin(q2) + (L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2 + (m4*(L4*sin(q2 + q3 + q4)*(L2*sin(q2) + (L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3)) + L4*cos(q2 + q3 + q4)*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3) + L2*cos(q2))))/2;
 
M43 = I4 + I5 + I6 + (m4*(L4*sin(q2 + q3 + q4)*((L4*sin(q2 + q3 + q4))/2 + L3*sin(q2 + q3)) + L4*cos(q2 + q3 + q4)*((L4*cos(q2 + q3 + q4))/2 + L3*cos(q2 + q3))))/2 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4))*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3)) + 2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4))*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2;
 
M44 = I4 + I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)^2 + 2*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)^2))/2 + (m5*(2*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4))^2 + 2*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4))^2))/2 + (m4*((L4^2*cos(q2 + q3 + q4)^2)/2 + (L4^2*sin(q2 + q3 + q4)^2)/2))/2;
 
M45 = I5 + I6 + (m5*(L5*cos(q2 + q3 + q4 + q5)*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4)) + L5*sin(q2 + q3 + q4 + q5)*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4))))/2 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 
M46 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 

M51 = - (m6*(2*L5*sin(q2 + q3 + q4 + q5) + L6*sin(q2 + q3 + q4 + q5 + q6)))/2 - (L5*m5*sin(q2 + q3 + q4 + q5))/2;
 
M52 = I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2)) + 2*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(L5*cos(q2 + q3 + q4 + q5)*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + L2*cos(q2)) + L5*sin(q2 + q3 + q4 + q5)*(L2*sin(q2) + (L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2;
 
M53 = I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2 + (m5*(L5*cos(q2 + q3 + q4 + q5)*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3)) + L5*sin(q2 + q3 + q4 + q5)*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3))))/2;
 
M54 = I5 + I6 + (m5*(L5*cos(q2 + q3 + q4 + q5)*((L5*cos(q2 + q3 + q4 + q5))/2 + L4*cos(q2 + q3 + q4)) + L5*sin(q2 + q3 + q4 + q5)*((L5*sin(q2 + q3 + q4 + q5))/2 + L4*sin(q2 + q3 + q4))))/2 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + 2*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 
M55 = I5 + I6 + (m6*(2*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2)^2 + 2*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)^2))/2 + (m5*((L5^2*cos(q2 + q3 + q4 + q5)^2)/2 + (L5^2*sin(q2 + q3 + q4 + q5)^2)/2))/2;

M56 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;


M61 = -(L6*m6*sin(q2 + q3 + q4 + q5 + q6))/2;

M62 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2 + L2*cos(q2)) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L2*sin(q2) + L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 
M63 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + L3*cos(q2 + q3) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + L3*sin(q2 + q3) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 
M64 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + L4*cos(q2 + q3 + q4) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L5*sin(q2 + q3 + q4 + q5) + L4*sin(q2 + q3 + q4) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;
 
M65 = I6 + (m6*(L6*cos(q2 + q3 + q4 + q5 + q6)*(L5*cos(q2 + q3 + q4 + q5) + (L6*cos(q2 + q3 + q4 + q5 + q6))/2) + L6*sin(q2 + q3 + q4 + q5 + q6)*(L5*sin(q2 + q3 + q4 + q5) + (L6*sin(q2 + q3 + q4 + q5 + q6))/2)))/2;

M66 = I6 + (m6*((L6^2*cos(q2 + q3 + q4 + q5 + q6)^2)/2 + (L6^2*sin(q2 + q3 + q4 + q5 + q6)^2)/2))/2;

M_ = [M11,M12,M13,M14,M15,M16;
      M21,M22,M23,M24,M25,M26;
      M31,M32,M33,M34,M35,M36;
      M41,M42,M43,M44,M45,M46;
      M51,M52,M53,M54,M55,M56;
      M61,M62,M63,M64,M65,M66];
end
