% read the parameterized path (trajectory) from TOPP
% the time array
% joint variable, joint velocity, joint acceleration
time = csvread('data/time.csv');
jointVariable = csvread('data/jointVariable.csv');
jointVel = csvread('data/jointVel.csv');
jointAccl = csvread('data/jointAccl.csv');

% construct the full UR5
ur5_L(1) = Link('d', 0.182, 'a', 0, 'alpha', pi/2);
ur5_L(2) = Link('d', 0, 'a', -0.620, 'alpha', 0);
ur5_L(3) = Link('d', 0, 'a', -0.559, 'alpha', 0);
%ur5_L(4) = Link('d', 0, 'a', 0, 'alpha', pi/2);
%ur5_L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
%ur5_L(6) = Link('d', 0, 'a', 0, 'alpha', 0);
ur5_L(4) = Link('d', 0.10915, 'a', 0, 'alpha', pi/2);
ur5_L(5) = Link('d', 0.09565, 'a', 0, 'alpha', -pi/2);
ur5_L(6) = Link('d', 0.0823, 'a', 0, 'alpha', 0);

ur5_full = SerialLink(ur5_L, 'name', 'ur5-6axis');
ur5_full.ikineType = 'puma';

% plot the trajectory
while(true)
    ur5_full.plot(jointVariable);
end

% compute the difference of desired orientation and real orientation
pose_real = zeros(length(time), 1);
pose_desired = zeros(length(time), 1);

% ori_real = ur5_full.fkine(jointVariable);
% for i=1:length(time)
%     pose_real(i) = acos(ori_real(2,2,i));
% end
% 
% for i=1:length(time)
%     %the jacobian matrix
%     J = ur5_full.jacob0(jointVariable(i,:));
%     %the product of the differential of jacobian matrix and the joint
%     %velocity
%     Jd = ur5_full.jacob_dot(jointVariable(i,:), jointVel(i,:));
%     %get the cartesian acceleration of end effector
%     cAccel = J*jointAccl(i,:)' + Jd;
%     
%     %get the orientation
%     pose_x = -atan2(cAccel(2),(cAccel(3)+9.81));
%     pose_y = -atan2(cAccel(1),(cAccel(3)+9.81));
%     pose_desired(i) = pose_x;
%     %get the complete transition matrix and update
%     %ori_desired(:,:,i) = rotx(pose_x) * roty(pose_y);
% end
% 
% %plot(time, pose_real);
