load aim_point_20180820.txt;
sec = aim_point_20180820(:,1)
aim_x = aim_point_20180820(:,2);
aim_y = aim_point_20180820(:,3);
aim_z = aim_point_20180820(:,4);

load arm_end_point_20180820.txt;
arm_x = arm_end_point_20180820(:,2);
arm_y = arm_end_point_20180820(:,3);
arm_z = arm_end_point_20180820(:,4);
plot(sec, arm_z,'-b',sec, aim_z,'-r')