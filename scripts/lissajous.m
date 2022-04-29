%% lissajous
% written by Stefan Leutenegger 03/2022

%% parameters
fps = 30; % animation framerate (approx)

% distance from target -- point will move on sphere
R = 1.5;

% lissajous curve parameters
a = 1;
b = 0.5;
m = 1;
n = 2;
d = 0.5;

% timing
T = 3.0; % [sec]

%% static plot
phi = 0:0.01:2*pi;
x = [a*cos(m*phi);
     b*cos(n*phi+pi*d)];
z = sqrt(R^2-x(1,:).^2-x(2,:).^2);
plot3(x(1,:),x(2,:), z(:))
axis equal
hold on

% initial point
phi0 = 0;
x = [a*cos(m*phi0);
     b*cos(n*phi0+pi*d)];
z = sqrt(R^2-x(1)^2-x(2)^2);
p = plot3(x(1),x(2),z,'rx');

%% Loop on the line angle (from 0 radians to 2pi radians
for t=0:1.0/fps:T
    phi=2*pi*sin(pi/2*t/T)^2;
    % Update the line coordinates
    x = [a*cos(m*phi);
         b*cos(n*phi+pi*d)];
    z = sqrt(R^2-x(1)^2-x(2)^2);
    set (p, 'XData', x(1), 'YData',x(2), 'ZData',z );

    % Refresh the figure
    drawnow;

    % run at desired framerate
    pause(1.0/fps)
end
ginput();

