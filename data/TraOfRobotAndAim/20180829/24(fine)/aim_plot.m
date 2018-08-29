load aim_point_1_20180829.txt;
sec = aim_point_1_20180829(:,1);
aim_x = aim_point_1_20180829(:,2);
aim_y = aim_point_1_20180829(:,3);
aim_z = aim_point_1_20180829(:,4);

load arm_end_point_20180829.txt;
arm_x = arm_end_point_20180829(:,2);
arm_y = arm_end_point_20180829(:,3);
arm_z = arm_end_point_20180829(:,4);

box_x=0.4727*exp(sec-sec);
box_y=-0.1754*exp(sec-sec);
box_z=-0.038*exp(sec-sec);
figure(1);
subplot(2,2,1);
plot(sec, arm_x,'-b',sec, aim_x,'-r',sec,box_x,'g');

xlabel('time(s)');
ylabel('x distance(m)');
subplot(2,2,2);
plot(sec, arm_y,'-b',sec, aim_y,'-r',sec,box_y,'g');
xlabel('time(s)');
ylabel('y distance(m)');

subplot(2,2,3);
plot(sec, arm_z,'-b',sec, aim_z,'-r',sec,box_z,'g');
xlabel('time(s)');
ylabel('z distance(m)');
legend('arm','parper','box');

