function [px, py, pz] = cubicInterpolation(x,y,z,xd,yd,zd)
    %CUBIC INTERPOLATION Construct a cubic interpolation trajectory of
    % given input points
    %   Detailed explanation goes here
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
end

