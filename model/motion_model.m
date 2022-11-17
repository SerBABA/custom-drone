
timestep = 0.01;
maxtime = 5;
numsteps = maxtime/timestep;

u = 0; % longitudinal velocity
v = 0; %lateral velocity
w = 0; %normal velocity

p = 0;  %roll rate
q = 0; %pitch rate
r = 0; %yaw rate

F_x = 0; %force body x
F_y = 0; %force body y
F_z = 0; %force body z

L = 0; %torque body x
M = 0; %torque body y
N = 0; %torque body z

phi = 0; %bank angle
theta = 0; %attitude
psi = 0; %heading

x = 0; %x coordinate
y = 0; %y coordinate
h = 40; %altitude

m = 0.1; % drone mass
I_xx = 0.00062; %moment of inertia
I_yy = 0.00113;
I_zz = 0.9*(I_xx + I_yy);

dx = 0.114;
dy = 0.0825;

g = 9.81;

x_data = zeros(numsteps,1);
y_data = zeros(numsteps,1);
h_data = zeros(numsteps,1);

simstep = 0;

for time = 0:timestep:maxtime
    simstep = simstep + 1;
    
    F1 = 0; % front right propeller
    F2 = 0; % rear left propeller
    F3 = 0; % front left propeller
    F4 = 0; % rear right propeller
    
    F_z = F1 + F2 + F3 + F4; %total normal thrust
    
    L = F1*dy-F2*dy-F3*dy+F4*dy;
    M = F1*dx - F2*dx + F3*dx - F4*dx;
    N = 0;
    
    %body-frame accelerations
    u_dot = -g*sin(theta) + r*v - q*w;
    v_dot = g*sin(phi)*cos(theta) - r*u+p*w;
    w_dot = (1/m)*-F_z+g*cos(phi)*cos(theta)+q*u-p*v;
    
    %body-frame angular accelerations
    p_dot = (1/I_xx)*(L + (I_yy - I_zz)*q*r);
    q_dot = (1/I_yy)*(M + (I_zz - I_xx)*p*r);
    r_dot = (1/I_zz)*(N + (I_xx - I_yy)*p*q);
    
    %inertial-frame rotations
    phi_dot = p + (q*sin(phi)+r*cos(phi))*tan(theta);
    theta_dot = q*cos(phi)-r*sin(phi);
    psi_dot = (q*sin(phi) + r*cos(phi))*sec(theta);
    
    %body-frame velocities
    u = u + u_dot * timestep;
    v = v + v_dot * timestep;
    w = w + w_dot * timestep;
    
    %body-frame angular velocities
    p = p + p_dot * timestep;
    q = q + q_dot * timestep;
    r = r + r_dot * timestep;
    
    %inertial-frame angles
    phi = phi + phi_dot * timestep;
    theta = theta + theta_dot * timestep;
    psi = psi + psi_dot * timestep;
    
    %inertial-frame accelerations
    x_dot = cos(theta)*cos(psi)*u + (-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi))*v + (sin(theta)*sin(psi)+cos(phi)*sin(theta)*cos(psi))*w;
    y_dot = cos(theta)*sin(psi)*u + (cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))*v + (-sin(theta)*cos(psi)+cos(phi)*sin(theta)*sin(psi))*w;
    h_dot = -1 * (-sin(theta)*u + sin(phi)*cos(theta)*v+cos(phi)*cos(theta)*w); %is negative to make up positive
    
    %inertial-frame positions
    x = x + x_dot;
    y = y + y_dot;
    h = h + h_dot;
    
    %RECORD DATA
    x_data(simstep) = x;
    y_data(simstep) = y;
    h_data(simstep) = h;
end

figure
plot3(x_data, y_data, h_data)
grid on

