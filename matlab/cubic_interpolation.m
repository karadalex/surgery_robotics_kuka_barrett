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
px = [];
for i = 1:length(x)-1
    tau = 1;
    ai = (1/tau^3)*(xd(i+1)*tau - 2*x(i+1) + xd(i)*tau + 2*x(i));
    bi = (1/tau^2)*(3*x(i+1) - 3*x(i) - xd(i+1)*tau - 2*xd(i)*tau);
    ci = xd(i);
    di = x(i);
    for j = i:0.05:i+1
        ji = j-i;
        pxi = ai*ji^3 + bi*ji^2 +ci*ji + di;
        px = [px; pxi];
    end
end

py = [];
for i = 1:length(y)-1
    tau = 1;
    ai = (1/tau^3)*(yd(i+1)*tau - 2*y(i+1) + yd(i)*tau + 2*y(i));
    bi = (1/tau^2)*(3*y(i+1) - 3*y(i) - yd(i+1)*tau - 2*yd(i)*tau);
    ci = yd(i);
    di = y(i);
    for j = i:0.05:i+1
        ji = j-i;
        pyi = ai*ji^3 + bi*ji^2 +ci*ji + di;
        py = [py; pyi];
    end
end

pz = [];
for i = 1:length(z)-1
    tau = 1;
    ai = (1/tau^3)*(zd(i+1)*tau - 2*z(i+1) + zd(i)*tau + 2*z(i));
    bi = (1/tau^2)*(3*z(i+1) - 3*z(i) - zd(i+1)*tau - 2*zd(i)*tau);
    ci = zd(i);
    di = z(i);
    for j = i:0.05:i+1
        ji = j-i;
        pzi = ai*ji^3 + bi*ji^2 +ci*ji + di;
        pz = [pz; pzi];
    end
end

close all
figure, hold on
plot3(x,y,z);
plot3(px,py,pz);
scatter3(x,y,z, 'r', 'filled');
legend({'Linear Interpolation','Cubic Spline Interpolation', 'Path Points'});
grid on
view([45 45]);