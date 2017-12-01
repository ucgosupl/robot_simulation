% Scope:
% 1. Robot motion simulation - save outputs.
% 2. Validate linearized model - compare with original outputs.
% 3. Generate signals from encoders and gyroscope.
% 4. Extended Kalman Filter
% 5. Unscented Kalman Filter
% 6. Square root UKF/EKF.

% state vector: [x[mm] y[mm] alfa[deg]]'
% input vector: [v[mm/s] omega[dps]]'

% Robot params
L = 80; % [mm]

% Time
Tmax = 10;
dt = 0.01;
t = 0:dt:Tmax;
t_size = max(size(t)) - 1;

% Generate input
input = zeros(2, t_size + 1);

% 0 - 1s - v = 2000 mm/s omega = 0 dps
input(1,2 : t_size/10 + 1) =  ones(1,t_size/10) * 2000;
input(2,2 : t_size/10 + 1) =  zeros(1,t_size/10);

% 1 - 3s - v = 1000 mm/s omega = 360 dps
input(1,t_size/10 + 1 : 3 * t_size/10) =  ones(1,2 * t_size/10) * 1000;
input(2,t_size/10 + 1 : 3 * t_size/10) =  ones(1,2 * t_size/10) * 360;

% 3 - 6s - v = 500 mm/s omega = -180 dps
input(1,3 * t_size/10 + 1 : 6 * t_size/10) =  ones(1,3 * t_size/10) * 500;
input(2,3 * t_size/10 + 1 : 6 * t_size/10) =  ones(1,3 * t_size/10) * -180;

% 6 - 8s - v = -1500 mm/s omega = 90 dps
input(1,6 * t_size/10 + 1 : 8 * t_size/10) =  ones(1,2 * t_size/10) * -1500;
input(2,6 * t_size/10 + 1 : 8 * t_size/10) =  ones(1,2 * t_size/10) * 90;

% 8 - 10s - v = 1000 mm/s omega = -30 dps
input(1,8 * t_size/10 + 1 : t_size) =  ones(1,2 * t_size/10) * 1000;
input(2,8 * t_size/10 + 1 : t_size) =  ones(1,2 * t_size/10) * -30;

% initial state
x0 = [50 100 30]';

% state
state = zeros(3, t_size + 1);
state(:,1) = x0;

% state linearized
state_lin = zeros(5, t_size + 1);
state_lin(1:3, 1) = x0;

% position of wheels
wheel_left = state(1:2,:);
wheel_right = wheel_left;

% encoder readings
encoder_left = zeros(1, max(size(wheel_left)));
encoder_right = zeros(1, max(size(wheel_right)));

enc = zeros(2, t_size + 1);

% gyro readings
gyro = zeros(1, t_size);

% EKF params
C = [0      0       0       dt      0;     % velocity calculated based on encoders
     0      0       0       0       dt;    % angular velocity calculated based on encoders
     0      0       0       0       1;     % angular velocity calculated based on gyroscope
    ];
 
enc_sdev = 1;
enc_ang_sdev = 2;
gyro_sdev = 5;

statex_sdev = 1;
statey_sdev = 1;
statealfa_sdev = 1;
statev_sdev = 5;
stateomega_sdev = 20;

varvx = ((statex_sdev)^2)*dt;
varvy = ((statey_sdev)^2)*dt;
varvalfa = ((statealfa_sdev)^2)*dt;
varv = ((statev_sdev)^2)*dt;
varomega = ((stateomega_sdev)^2)*dt;

varwv_enc = ((enc_sdev)^2)*dt;
varwalfa_enc = ((enc_ang_sdev)^2)*dt;
varwalfa_gyro = ((gyro_sdev)^2)*dt;

 % V - macierz szumu procesu
 V = [
       varvx    0       0           0       0;
       0        varvy	0           0       0;
       0        0       varvalfa    0       0;
       0        0       0           varv    0;
       0        0       0           0       varomega;
     ];
 
 % W - macierz szumu pomiaru
 W = [
       varwv_enc*dt    0                0;
       0               varwalfa_enc*dt  0;
       0                0               varwalfa_gyro;
     ];

 Y = zeros(3, t_size + 1);
 X = zeros(5, t_size + 1);
 
 xpri = [x0; 0; 0;];
 xpost = xpri;
 
 Ppri = [
            1       0       0       0       0;
            0       1       0       0       0;
            0       0       1       0       0;
            0       0       0       5       0;
            0       0       0       0       5;
        ];
 Ppost = Ppri;
 
for i = 1:t_size
    
    wheel_left(1,i) = state(1,i) + L/2 * cos(deg2rad(state(3,i) + 90));
    wheel_left(2,i) = state(2,i) + L/2 * sin(deg2rad(state(3,i) + 90));
    
    wheel_right(1,i) = state(1,i) + L/2 * cos(deg2rad(state(3,i) - 90));
    wheel_right(2,i) = state(2,i) + L/2 * sin(deg2rad(state(3,i) - 90));
    
    % Encoder readings
    if i > 1
        %calculate sign of encoder value
        dir_left = direction(state(3,i), wheel_left(:,i), wheel_left(:, i - 1));
        dir_right = direction(state(3,i), wheel_right(:,i), wheel_right(:, i - 1));
        
        encoder_left(i) = dir_left * (sqrt(sum((wheel_left(:,i) - wheel_left(:,i - 1)).^ 2))) + randn(1)*enc_sdev;
        encoder_right(i) = dir_right * (sqrt(sum((wheel_right(:,i) - wheel_right(:,i - 1)) .^ 2))) + randn(1)*enc_sdev;
        
        gyro(i) = (state(3, i) - state(3, i - 1))/dt + randn(1)*gyro_sdev;
    end
    
    % State equations:
    % x(t + 1) = x(t) + v(t) cos alfa(t) dt
    % y(t + 1) = y(t) + v(t) sin alfa(t) dt
    % alfa(t + 1) = alfa(t) + omega(t) dt
    state(1,i + 1) = state(1, i) + input(1, i) * cos(deg2rad(state(3,i))) * dt;
    state(2,i + 1) = state(2, i) + input(1, i) * sin(deg2rad(state(3,i))) * dt;
    state(3,i + 1) = state(3, i) + input(2, i) * dt;
    
    % Velocities from encoders
    v_enc = (encoder_left(i) + encoder_right(i))/2;
    omega_enc = (encoder_left(i) - encoder_right(i))/L;
    
    enc(:,i) = [v_enc; omega_enc];
    
    % Y from readings
    Y(1, i) = v_enc;
    Y(2, i) = omega_enc;
    Y(3, i) = gyro(i);
    
    %EKF
    A = linearization(xpost, dt);
    
    %xpri = A*xpost;
    xpri(1) = xpost(1) + xpost(4) * cos(deg2rad(xpost(3))) * dt;
    xpri(2) = xpost(2) + xpost(4) * sin(deg2rad(xpost(3))) * dt;
    xpri(3) = xpost(3) + xpost(5) * dt;
    xpri(4) = xpost(4);
    xpri(5) = xpost(5);
    
    Ppri = A*Ppost*A' + V;
    e = Y(:,i) - C*xpri;
    S = C*Ppri*C' + W;
    K = Ppri*C'*S^-1;
    xpost = xpri + K*e;
    % Joseph formula for elimination of subtraction of covariance matrix.
    Ppost = (eye(5)-K*C)*Ppri*(eye(5)-K*C)' + K*W*K';
    
    X(:,i) = xpost;
end


figure
plot(t, input(1,:))
title('Input velocity')
xlabel('time [s]')
ylabel('velocity [mm/s]')

figure
plot(t, input(2,:))
title('Input angular velocity')
xlabel('time [s]')
ylabel('angular velocity [dps]')

figure
plot(state(1,:), state(2,:), 'b', X(1,:), X(2,:), 'r.')
title('Robot position + EKF estimation')
xlabel('position x [mm]')
ylabel('position y [mm]')

figure
plot(state(1,:), state(2,:), 'b', wheel_left(1,1:end-1), wheel_left(2,1:end-1), 'r', wheel_right(1,1:end-1), wheel_right(2,1:end-1), 'r')
title('Robot position + wheel positions')
xlabel('position x [mm]')
ylabel('position y [mm]')

figure
plot(t(2:end-1), encoder_left(2:end-1), 'b', t(2:end-1), encoder_right(2:end-1), 'r')
title('Encoder readings')
xlabel('time [s]')
ylabel('speed [mm/s]')

figure
plot(t, state(1,:), t, X(1,:), 'r')
title('Robot X position + EKF estimation')
xlabel('time [s]')
ylabel('velocity [mm/s]')

figure
plot(t, state(2,:), t, X(2,:), 'r')
title('Robot Y position + EKF estimation')
xlabel('time [s]')
ylabel('velocity [mm/s]')

figure
plot(t, state(3,:), t, X(3,:), 'r')
title('Robot alfa orientation + EKF estimation')
xlabel('time [s]')
ylabel('velocity [mm/s]')

figure
plot(t, input(1,:), t, X(4,:), 'r')
title('Robot linear velocity + EKF estimation')
xlabel('time [s]')
ylabel('velocity [mm/s]')

figure
plot(t, input(2,:), t, X(5,:), 'r')
title('Robot angular velocity + EKF estimation')
xlabel('time [s]')
ylabel('angular velocity [dps]')