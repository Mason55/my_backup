load pose_point.txt;
sec = pose_point(:,1);
arm_x = pose_point(:,2);
arm_y = pose_point(:,3);
arm_z = pose_point(:,4);

box_x=0.6032*exp(sec-sec);
box_y=-0.1754*exp(sec-sec);
box_z=-0.032*exp(sec-sec);
figure(1);
subplot(2,2,1);
plot(sec, arm_x,'-b');

xlabel('time(s)');
ylabel('x distance(m)');
subplot(2,2,2);
plot(sec, arm_y,'-b');
xlabel('time(s)');
ylabel('y distance(m)');

subplot(2,2,3);
plot(sec, arm_z,'-b');
xlabel('time(s)');
ylabel('z distance(m)');
legend('arm','parper','box');

