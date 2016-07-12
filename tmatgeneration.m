clear
clc

start = [0.5, 0.3, 0.3];
goal = [0.5, -0.3, 0.3];

%get the transition matrix of the initial states
Tstart = transl(start);
Tgoal = transl(goal);

t = 0:0.1:2;
t = t';

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

alpha_y = -atan2(acl_x, 9.81 + acl_z);
alpha_x = -atan2(acl_y, 9.81 + acl_z);

for i=1:length(t)
    T(:,:,i) =  Ts(:,:,i) * trotx(alpha_x(i)) * troty(alpha_y(i));
    reshapeT = reshape(T(:,:,i)',1,16);
    mat(i,:) = reshapeT;
end
csvwrite('matforcpp.csv',mat)

qc =[3.978311 4.417378 4.746586 3.402407 5.549107 3.175982;
3.973785 4.421950 4.740534 3.403887 5.544582 3.193153;
3.959989 4.435420 4.722831 3.408120 5.530785 3.210292;
3.936139 4.457152 4.694675 3.414544 5.506935 3.210292;
3.900951 4.485966 4.658121 3.422284 5.471748 3.210292;
3.852477 4.520124 4.615950 3.430297 5.423274 3.209340;
3.788109 4.557096 4.571743 3.437531 5.358905 3.195058;
3.705478 4.592971 4.530299 3.443100 5.276275 3.165480;
3.612126 4.620633 4.499328 3.446409 5.182923 3.145415;
3.511736 4.637702 4.480648 3.448021 5.082532 3.141593;
3.405974 4.643476 4.474403 3.448492 4.976770 3.141593;
3.297098 4.637702 4.480648 3.448021 4.867894 3.141593;
3.187684 4.620633 4.499328 3.446409 4.758480 3.137770;
3.080275 4.592971 4.530299 3.443100 4.651071 3.117705;
2.980407 4.557096 4.571743 3.437531 4.551203 3.088127;
2.899402 4.520135 4.615937 3.430299 4.470198 3.073845;
2.836457 4.485966 4.658121 3.422284 4.407254 3.072894;
2.789727 4.457140 4.694691 3.414540 4.360523 3.072894;
2.757590 4.435420 4.722831 3.408120 4.328386 3.072894;
2.738786 4.421936 4.740552 3.403883 4.309583 3.090033;
2.732613 4.417378 4.746586 3.402407 4.303409 3.107203];

dist = zeros(length(t),1);
for i=1:length(t)
    dist(i) = norm(T(1:3,4,i)-T(1:3,4,1));
end

qc_d = zeros(length(t), 6);
% compute the differential of joint variable via distance
for i=1:6
    qc_d(:,i) = gradient(qc(:,i))./gradient(dist);
    % uncomment the following lines to plot the differential of joint variable via distance
    % plot(dist, qc(:,i));
    % hold on;
end

% output in TOPP format
qc_for = qc(1:end-1,:);
qc_lat = qc(2:end,:);
qc_d_for = qc_d(1:end-1,:);
qc_d_lat = qc_d(2:end,:);
dist_d = gradient(dist);

% save in csv file
output = [qc_for, qc_lat, qc_d_for, qc_d_lat, dist_d(1:end-1,:)];
csvwrite('input.csv',output)