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
ur5_L(4) = Link('d', 0, 'a', 0, 'alpha', pi/2);
ur5_L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
ur5_L(6) = Link('d', 0, 'a', 0, 'alpha', 0);

ur5_full = SerialLink(ur5_L, 'name', 'ur5-6axis');
ur5_full.ikineType = 'puma';

% plot the trajectory
while(true)
    ur5_full.plot(jointVariable);
end