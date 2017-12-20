% Linearization of robot motion model
% param x - state [x[mm] y[mm] alfa[deg] v[mm/s] omega[deg/s]]
% param dt - sampling time
% return A - linearized state matrix

function [A] = linearization(x, dt)

    A = [
            1       0       0       dt*cos(deg2rad(x(3)))       0       0;
            0       1       0       dt*sin(deg2rad(x(3)))       0       0;
            0       0       1       0                           dt      0;
            0       0       0       1                           0       0;
            0       0       0       0                           1       0;
            0       0       0       0                           0       1;
        ];

end