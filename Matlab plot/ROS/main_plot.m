clc
clear
close all

tsample=0.001;
t2sample=0.01;
tend=10;

sel=1;
switch sel
   case 1 %trot
        bag=rosbag('trot_v03.bag');
        
        %cut initial phase
        in_grf=round(4.5/0.01);
        end_grf=round(14.5/0.01);
        in_com=round(5.3/0.001);
        end_com=round(15.3/0.001);
   
    case 2 %walk
        bag=rosbag('walk_v02.bag');
        
        %cut initial phase
        in_grf=round(3.7/0.01);
        end_grf=round(13.7/0.01);
        in_com=round(4.3/0.001);
        end_com=round(14.3/0.001);
 
    case 3 %pace
        bag=rosbag('pace_v03.bag');
        
        %cut initial phase
        in_grf=round(3.4/0.01);
        end_grf=round(13.4/0.01);
        in_com=round(4.2/0.001);
        end_com=round(14.2/0.001);
   
    case 4 %gallop
        bag=rosbag('gallop_v03.bag');
        
        %cut initial phase
        in_grf=round(3.7/0.01);
        end_grf=round(13.7/0.01);
        in_com=round(4.5/0.001);
        end_com=round(14.5/0.001);

    case 5 %bound
        bag=rosbag('bound_v03.bag');
        
        %cut initial phase
        in_grf=round(4.4/0.01);
        end_grf=round(14.4/0.01);
        in_com=round(5/0.001);
        end_com=round(15/0.001);

    case 6 %bound_old
        bag=rosbag('BOUND_OLD_V03.bag');
        
        %cut initial phase   
        in_grf=round(3.2/0.01);
        end_grf=round(13.2/0.01);
        in_com=round(4/0.001);
        end_com=round(14/0.001);

   otherwise
      disp('ERROR rosbag selection')
end
 
 t=0:tsample:tend;
 t2=0:t2sample:tend;

 bSel1 = select(bag,'Topic','/gazebo/model_states');
 bSel2 = select(bag,'Topic','/visual/FL_foot_contact/the_force');
 bSel3 = select(bag,'Topic','/visual/FR_foot_contact/the_force');
 bSel4 = select(bag,'Topic','/visual/RL_foot_contact/the_force');
 bSel5 = select(bag,'Topic','/visual/RR_foot_contact/the_force');
 
 msgSTRUCT1 = readMessages(bSel1,'DataFormat','struct');
 msgSTRUCT2 = readMessages(bSel2,'DataFormat','struct');
 msgSTRUCT3 = readMessages(bSel3,'DataFormat','struct');
 msgSTRUCT4 = readMessages(bSel4,'DataFormat','struct');
 msgSTRUCT5 = readMessages(bSel5,'DataFormat','struct');

 for i=1:length(msgSTRUCT2)
    grf1(:,i)=msgSTRUCT2{i,1}.Wrench.Force.X;
end

for i=1:length(msgSTRUCT3)
    grf2(:,i)=msgSTRUCT3{i,1}.Wrench.Force.X;
end

for i=1:length(msgSTRUCT4)
    grf3(:,i)=msgSTRUCT4{i,1}.Wrench.Force.X;
end

for i=1:length(msgSTRUCT5)
    grf4(:,i)=msgSTRUCT5{i,1}.Wrench.Force.X;
end

for i=1:length(msgSTRUCT1)
   if (length(msgSTRUCT1{i, 1}.Twist)<3)
        vx(:,i)=vx(:,i-1);
        vy(:,i)=vy(:,i-1);
        vz(:,i)=vz(:,i-1);
        wx(:,i)=wx(:,i-1);
        wy(:,i)=wy(:,i-1);
        wz(:,i)=wz(:,i-1);
        px(:,i)=px(:,i-1);
        py(:,i)=py(:,i-1);
        pz(:,i)=pz(:,i-1);
   else
        vx(:,i)=msgSTRUCT1{i, 1}.Twist(3).Linear.X;
        vy(:,i)=msgSTRUCT1{i,1}.Twist(3).Linear.Y;
        vz(:,i)=msgSTRUCT1{i,1}.Twist(3).Linear.Z;
        wx(:,i)=msgSTRUCT1{i,1}.Twist(3).Angular.X;
        wy(:,i)=msgSTRUCT1{i,1}.Twist(3).Angular.Y;
        wz(:,i)=msgSTRUCT1{i,1}.Twist(3).Angular.Z;
        px(:,i)=msgSTRUCT1{i,1}.Pose(3).Position.X;
        py(:,i)=msgSTRUCT1{i,1}.Pose(3).Position.Y;
        pz(:,i)=msgSTRUCT1{i,1}.Pose(3).Position.Z;
   end
end


figure()
hold on
plot(t2,grf1(in_grf:end_grf),'r','linewidth',1)
plot(t2,grf2(in_grf:end_grf),'g','linewidth',1)
plot(t2,grf3(in_grf:end_grf),'b','linewidth',1)
plot(t2,grf4(in_grf:end_grf),'k','linewidth',1)
xlabel('$[\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 22)
ylabel('$[\mathrm{N}]$', 'Interpreter', 'latex', 'FontSize', 22)
legend('$f_{gr,1}^z$','$f_{gr,2}^z$','$f_{gr,3}^z$','$f_{gr,4}^z$', 'Interpreter', 'latex','Location','northoutside','Orientation','horizontal', 'FontSize', 24)
ax = gca;
ax.FontSize = 16;
grid

figure()
hold on
plot(t,vx(in_com:end_com),'r','linewidth',1)
plot(t,vy(in_com:end_com),'g','linewidth',1)
plot(t,vz(in_com:end_com),'b','linewidth',1)
%title('Linear Velocity', 'Interpreter', 'latex', 'FontSize', 18)
xlabel('$[\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 22)
ylabel('$[\mathrm{m/s}]$', 'Interpreter', 'latex', 'FontSize', 22)
legend('$\dot{p}_x$','$\dot{p}_y$','$\dot{p}_z$', 'Interpreter', 'latex','Location','northoutside','Orientation','horizontal', 'FontSize', 14)
ax = gca;
ax.FontSize = 16;
grid

figure()
hold on
plot(t,wx(in_com:end_com),'r','linewidth',1)
plot(t,wy(in_com:end_com),'g','linewidth',1)
plot(t,wz(in_com:end_com),'b','linewidth',1)
%title('Angular Velocity', 'Interpreter', 'latex', 'FontSize', 18)
xlabel('$[\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 22)
ylabel('$[\mathrm{rad/s}]$', 'Interpreter', 'latex', 'FontSize', 22)
legend('$\omega_x$','$\omega_y$','$\omega_z$', 'Interpreter', 'latex','Location','northoutside','Orientation','horizontal', 'FontSize', 14)
ax = gca;
ax.FontSize = 16;
grid

figure()
hold on
plot(t,px(in_com:end_com),'r','linewidth',1)
plot(t,py(in_com:end_com),'g','linewidth',1)
plot(t,pz(in_com:end_com),'b','linewidth',1)
%title('Position', 'Interpreter', 'latex', 'FontSize', 18)
xlabel('$[\mathrm{s}]$', 'Interpreter', 'latex', 'FontSize', 22)
ylabel('$[\mathrm{m}]$', 'Interpreter', 'latex', 'FontSize', 22)
legend('$p_x$','$p_y$','$p_z$', 'Interpreter', 'latex','Location','northoutside','Orientation','horizontal', 'FontSize', 14)
ax = gca;
ax.FontSize = 16;
grid