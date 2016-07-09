start = [0.5, 0.6, 0.5];
goal = [0.5, -0.6, 0.5];

%get the transition matrix of the initial states
Tstart = transl(start);
Tgoal = transl(goal);

t = [0:0.01:2]';

%plan the path in Cartesian Space
%get the inverse kinematics
Ts = ctraj(Tstart, Tgoal, length(t));

%compute the end effecter orientation
Ts_x = reshape(Ts(1,4,:),length(t),1);
vel_x = gradient(Ts_x)./gradient(t);
acl_x = gradient(vel_x)./gradient(t);

Ts_y = reshape(Ts(2,4,:),length(t),1);
vel_y = gradient(Ts_y)./gradient(t);
acl_y = gradient(vel_y)./gradient(t);

Ts_z = reshape(Ts(3,4,:),length(t),1);
vel_z = gradient(Ts_z)./gradient(t);
acl_z = gradient(vel_z)./gradient(t);

alpha_y = atan2(acl_x, 9.81 + acl_z);
alpha_x = atan2(acl_y, 9.81 + acl_z);

%construct the UR5 robot
ur5_L(1) = Link('d', 0.182, 'a', 0, 'alpha', pi/2);
ur5_L(2) = Link('d', 0, 'a', -0.620, 'alpha', 0);
ur5_L(3) = Link('d', 0, 'a', -0.559, 'alpha', 0);
ur5_L(4) = Link('d', 0, 'a', 0, 'alpha', pi/2);
ur5_L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
ur5_L(6) = Link('d', 0, 'a', 0, 'alpha', 0);

ur5_full = SerialLink(ur5_L, 'name', 'ur5-6axis');
ur5_full.ikineType = 'puma';

for i=1:length(t)
    T(:,:,i) =  Ts(:,:,i) * trotx(alpha_x(i)) * troty(alpha_y(i));
end

qc = ur5_full.ikine6s(T);
while(true)
    ur5_full.plot(qc)
end

% p = transl(T);
% plot3(p(:,1), p(:,2), p(:,3));
