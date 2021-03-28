%% Generate path points
r1 = 0;
r2 = 10;
num = 4;
% x = (r2-r1).*rand(num,1) + r1;
% y = (r2-r1).*rand(num,1) + r1;
% z = (r2-r1).*rand(num,1) + r1;
x = [3.063, 3.568, 4.359, 4.868]/5;
y = [7.948, 8.176, 5.108, 5.085]/5;
z = [5.348, 8.116, 3.786, 6.443]/5;

P1 = [x(1); y(1); z(1)];
P2 = [x(2); y(2); z(2)];
P3 = [x(3); y(3); z(3)];
P4 = [x(4); y(4); z(4)];
G1 = [P1, P2, P3, P4];
G2 = [-1, 3, -3, 1; 3, -6, 3, 0; -3, 3, 0, 0; 1, 0, 0, 0];


%% Construct Bezier Curve
s = 0:0.01:1;
P = zeros(size(s,2),3);
for i=1:size(s,2)
    Gs = [s(i)^3; s(i)^2; s(i); 1];
    Pi = G1*G2*Gs;
    P(i,:) = Pi.';
end

%% Plot data
close all
figure;

subplot(2,2,1);
hold on, grid on, view([45 45]);
plot3(P(:,1),P(:,2),P(:,3));
scatter3(x,y,z,'r','filled');
legend({'Bezier Curve', 'Control Points'});
xlabel('x');
ylabel('y');
zlabel('z');

subplot(2,2,2);
hold on, grid on;
plot(P(:,1),P(:,2));
scatter(x,y,'r','filled');
xlabel('x');
ylabel('y');

subplot(2,2,3);
hold on, grid on;
plot(P(:,2),P(:,3));
scatter(y,z,'r','filled');
xlabel('y');
ylabel('z');

subplot(2,2,4);
hold on, grid on;
plot(P(:,1),P(:,3));
scatter(x,z,'r','filled');
xlabel('x');
ylabel('z');