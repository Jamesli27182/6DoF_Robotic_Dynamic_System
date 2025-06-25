%% This is the dynamic simulation for the 6DoF robotic manipulator system
% The dynamic motion is free fall, so the total energy should be
% Conservation. 
% Also this file check the of 

% If you want to see the animation, please do NOT uSE CLEAR function after
% finish the simualtino directly run the animation_6DoF.m
% Written by Chengyue Li, University of Louisiana at Lafayette, June 25, 2025.

%% parameters
N          = 100001;
T          = 5;
t          = linspace(0,T,N);
dt         = T/(N-1);
g          = 9.81;
q_initial  = [0, pi/4+pi/2,-pi/4,-pi/4,-pi/4,-pi/4];
dq_initial = zeros(6,1);
L          = [0.25,0.5,0.5,0.5,0.5,0.5];
Inertia    = 1/12*0.5^2*ones(5,1);
mass       = [1.2;1.7;1.1;1.8;1.2;1.8];
q          = zeros(6,N);
dq         = zeros(6,N);
ddq        = zeros(6,N);
k          = 1;
q(:,k)     = q_initial';
M_matrix   = dyn_6DoF_massmatrix(q(:,k),L,mass ,Inertia);
C_term     = dyn_6DoF_Coriolisterm(q(:,k),dq(:,k),L,mass);
G_term     = dyn_6DoF_gravity_term(q(:,k),L,mass)*g;
ddq(:,k)   = M_matrix\(-C_term -G_term);
dddq       = 0;
pM_matrix = M_matrix 
for k = 2:N
%% Please DO NOT USE THIS Integrator IN YOUR ApPLicAtIoNs
q(:,k)              = q(:,k-1)  + dq(:,k-1)*dt  + 0.5*ddq(:,k-1)*dt^2 + 1/6*dddq*dt^3;
dq(:,k)             = dq(:,k-1) + ddq(:,k-1)*dt + 0.5*dddq*dt^2;
M_matrix            = dyn_6DoF_massmatrix(q(:,k),L,mass ,Inertia);
C_term              = dyn_6DoF_Coriolisterm(q(:,k),dq(:,k),L,mass);
C_matrix            = dyn_6DoF_Coriolismatrix(q(:,k),dq(:,k),L,mass);
G_term              = dyn_6DoF_gravity_term(q(:,k),L,mass)*g;
ddq(:,k)            = M_matrix\(-C_term -G_term);
dddq                = (ddq(:,k)-ddq(:,k-1))/dt;
Pos_c               = Geometry_6DoF_masscenter(q(:,k),L);
%% The total energy of this system should be 
% In the simulation, it should be around a constant number
0.5*dq(:,k)'*M_matrix*dq(:,k) + mass'*Pos_c(:,2)*g
% Check the dotM = C+C'. The result should be around zero matrix 6x6
dM = (M_matrix-pM_matrix)/dt - C_matrix'-C_matrix
pM_matrix = M_matrix;

end