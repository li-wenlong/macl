T = load('/home/kimiwings/Desktop/robot_1_track.txt');
%T=T(40:60,:);
plot(T(:,1), T(:,2), 'r');
hold on
plot(T(:,3), T(:,4), 'g');
hold on
plot(T(:,5), T(:,6), 'b');
hold off