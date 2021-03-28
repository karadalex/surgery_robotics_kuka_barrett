%% Generate path points
r1 = 0;
r2 = 10;
size = 5;
x = (r2-r1).*rand(size,1) + r1;
y = (r2-r1).*rand(size,1) + r1;
z = (r2-r1).*rand(size,1) + r1;

% xd = 2.*rand(size,1);
% yd = 2.*rand(size,1);
% zd = 2.*rand(size,1);
xd = 1*ones(size,1);
yd = 1*ones(size,1);
zd = 1*ones(size,1);


%% Construct polynomials
[px, py, pz] = cubicInterpolation(x,y,z,xd,yd,zd);

%% Plot data
close all
figure, hold on
plot3(x,y,z);
plot3(px,py,pz);
scatter3(x,y,z, 'r', 'filled');
legend({'Linear Interpolation','Cubic Spline Interpolation', 'Path Points'});
grid on
view([45 45]);