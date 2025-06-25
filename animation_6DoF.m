%% High computational efficiency than using URDF file from MATLAB Robotic ToolBox
% Written by Chengyue Li, University of Louisiana at Lafayette, June 25, 2025.
q_intial = [0, pi/4+pi/2,-pi/4,-pi/4,-pi/4,-pi/4];
L        = [0.25,0.5,0.5,0.5,0.5,0.5];
g        = 9.81;
Pos_out  = Geometry_6DoF_joint(q_intial,L);
figure(101);
hold on
for k = 1:5:N
    k 
Pos_out = Geometry_6DoF_joint(q(:,k),L);
for j = 1:6
    plot([Pos_out(j,1),Pos_out(j+1,1)],[Pos_out(j,2),Pos_out(j+1,2)],'LineWidth',2)
    hold on
end
grid on
axis equal
xlim([-1.5 1.5])
ylim([-0.5 2])
hold off
pause(0.01)
Pos_c               = Geometry_6DoF_masscenter(q(:,k),L);

end
axis equal
grid on
