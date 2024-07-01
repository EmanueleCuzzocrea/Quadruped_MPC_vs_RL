clc
clear
close all

fileID = fopen('posx.txt', 'r');
px = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('posy.txt', 'r');
py = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('posz.txt', 'r');
pz = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('velx.txt', 'r');
vx = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('vely.txt', 'r');
vy = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('velz.txt', 'r');
vz = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('angularx.txt', 'r');
wx = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('angulary.txt', 'r');
wy = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('angularz.txt', 'r');
wz = fscanf(fileID, '%f');
fclose(fileID);

dt=0.01;
t=0:dt:dt*length(px);
t=t(1:end-1);

figure()
hold on
plot(t,px,'r','linewidth',1);
plot(t,py,'g','linewidth',1);
plot(t,pz,'b','linewidth',1);
%title('Position', 'Interpreter', 'latex', 'FontSize', 18)
grid
xlabel('$[\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 22)
ylabel('$[\mathrm{m/s}]$', 'Interpreter', 'latex', 'FontSize', 22)
legend('$p_x$','$p_y$','$p_z$', 'Interpreter', 'latex','Location','northoutside','Orientation','horizontal', 'FontSize', 14)
ax = gca;
ax.FontSize = 16;

figure()
hold on
plot(t,vx,'r','linewidth',1)
plot(t,vy,'g','linewidth',1)
plot(t,vz,'b','linewidth',1)
%title('Linear Velocity', 'Interpreter', 'latex', 'FontSize', 18)
xlabel('$[\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 22)
ylabel('$[\mathrm{m/s}]$', 'Interpreter', 'latex', 'FontSize', 22)
legend('$\dot{p}_x$','$\dot{p}_y$','$\dot{p}_z$', 'Interpreter', 'latex','Location','northoutside','Orientation','horizontal', 'FontSize', 14)
ax = gca;
ax.FontSize = 16;
grid

figure()
hold on
plot(t,wx,'r','linewidth',1)
plot(t,wy,'g','linewidth',1)
plot(t,wz,'b','linewidth',1)
%title('Angular Velocity', 'Interpreter', 'latex', 'FontSize', 18)
xlabel('$[\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 22)
ylabel('$[\mathrm{rad/s}]$', 'Interpreter', 'latex', 'FontSize', 22)
legend('$\omega_x$','$\omega_y$','$\omega_z$', 'Interpreter', 'latex','Location','northoutside','Orientation','horizontal', 'FontSize', 14)
ax = gca;
ax.FontSize = 16;
grid
