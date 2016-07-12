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
for i=1:length(t)
    reshapeT = reshape(Ts(:,:,i)',1,16);
    mat(i,:) = reshapeT;
end
%csvwrite('zeroOrienMatforcpp.csv',mat)

qc = [3.978311 4.417378 4.746585 3.402407 5.549107 3.141593;
3.973785 4.421950 4.740534 3.403887 5.544582 3.141593;
3.959989 4.435420 4.722831 3.408119 5.530785 3.141593;
3.936139 4.457152 4.694675 3.414544 5.506935 3.141593;
3.900951 4.485966 4.658121 3.422284 5.471748 3.141593;
3.852477 4.520124 4.615949 3.430298 5.423274 3.141593;
3.788109 4.557096 4.571744 3.437531 5.358905 3.141593;
3.705478 4.592971 4.530300 3.443099 5.276275 3.141593;
3.612126 4.620633 4.499329 3.446409 5.182923 3.141593;
3.511736 4.637702 4.480648 3.448021 5.082532 3.141593;
3.405974 4.643476 4.474403 3.448492 4.976770 3.141593;
3.297098 4.637702 4.480648 3.448021 4.867894 3.141593;
3.187684 4.620633 4.499329 3.446409 4.758480 3.141593;
3.080275 4.592971 4.530300 3.443099 4.651071 3.141593;
2.980407 4.557096 4.571744 3.437531 4.551203 3.141593;
2.899402 4.520135 4.615936 3.430300 4.470198 3.141593;
2.836457 4.485966 4.658121 3.422284 4.407254 3.141593;
2.789727 4.457140 4.694691 3.414540 4.360523 3.141593;
2.757590 4.435420 4.722831 3.408119 4.328386 3.141593;
2.738786 4.421936 4.740552 3.403883 4.309583 3.141593;
2.732613 4.417378 4.746585 3.402407 4.303409 3.141593];

dist = zeros(length(t),1);
for i=1:length(t)
    dist(i) = norm(Ts(1:3,4,i)-Ts(1:3,4,1));
end

for i=1:6
    qc_d(:,i) = gradient(qc(:,i))./gradient(dist);
    % uncomment the following lines to plot the differential of joint variable via distance
    plot(dist, qc_d(:,i));
    hold on;
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