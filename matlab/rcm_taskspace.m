x = []; y = []; z = [];
r = [];
x0 = 0;
y0 = 0;
s = 0:0.005:0.5;

L = 0.48;
k = 4.5;

for z0=-L:0.01:0
    if abs(z0)*sqrt(2) <= L
        rmax = abs(z0);
    else
        rmax = sqrt(L^2-z0^2);
    end
    for r0=0:0.02:rmax
        x = [x, r0*cos(2*pi*s) + x0];
        y = [y, r0*sin(2*pi*s) + y0];
        z = [z, z0*ones(size(s))];
        lp = 1-exp(-k*(rmax-r0));
        r = [r, lp*ones(size(s))];
    end
end

yr = pi/2;
Ry = [cos(yr) 0 sin(yr); 0 1 0; -sin(yr) 0 cos(yr)];

close all
figure = gcf;
hold on, grid on
xlabel('x');
ylabel('y');
zlabel('z');
view([45 45]);
colorbar;
set(gca,'DataAspectRatio',[1 1 1]);
scatter3(x, y, z, 40, r);
scatter3(0, 0, 0, 'r', 'filled');
legend({'Task space', 'Remote Center of Motion (Fulcrum)'});
%xyzRy = Ry*[x; y; z];
%scatter3(xyzRy(1,:), xyzRy(2,:), xyzRy(3,:), 'b', 'filled');