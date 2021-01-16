function [xsim, usim, x_ref,y_ref] = Controller(P1)
% ------------ modele du agent: -----------
%Two-wheeled mobile robot 
%d_x = v*cos(theta)s
%d_y = v*sin(theta)
%d_theta = w
%z1 = x z2 = y
%theta = a_tg(d_z2/d_z1)
%v = sqrt(d_z1^2 + d_z2^2)
%eps = [x y theta]'
%u = [v w]'

% P = [ 15 15 15.1 15.4 16 16 15.8  15.3   15;
%        38 35 30   28   25 15 10    5.2    3];
% P2 = [ 0 5  8  13 15 20 25 30 36;
%        0 10 15 16 18 20 26 35 36];
% P3 = [ 38 36 30 25 22 16 12 8  5;
%        0  5  10 15 18 24 28 32 36];

ref = IP_ex_bezier(P1); % duas linhas, (x,y)


z1 = ref(1,:);
z2 = ref(2,:);

Ts = 0.1; %sample time

d_z1 = [0 (z1(2:end)-z1(1:end-1))/Ts];
d_z2 = [0 (z2(2:end)-z2(1:end-1))/Ts];
theta_ref = atan2(d_z2,d_z1);
v_ref = sqrt(d_z1.^2+d_z1.^2);


% partie MPC
x_ref = z1;
y_ref = z2;

v_max = 5;
v_min = -5;

w_max = 1;
w_min = -1;

dv_max = 1;
dv_min = -1;

dw_max = 0.5;
dw_min = -0.5;

% system dimentions
dx = 3;
du = 2;
dy = 3;

C = eye(3);

%Weighting matrices
Q = 10*eye(dy); % cost state x
P = 1000*Q;        % terminal cost
R = eye(du);    % cost for the input
Rdelta = eye(du); % cost for the input u variation

Npred = 10;
Nsim = length(z1) - 1;

% Optimization problem
u = sdpvar(repmat(du,1,Npred), ones(1,Npred));
x = sdpvar(repmat(dx,1,Npred+1), ones(1,Npred+1));
utmp=sdpvar(du, 1);
u_init=sdpvar(du,1);

xref = sdpvar(Npred+1,1);
yref = sdpvar(Npred+1,1);
thetaref = sdpvar(Npred+1,1);

constraints=[];
objective=0;

for k=1:Npred
    if(k==1)
        utmp=u_init;
    else
        utmp=u{k-1}; 
    end  
    B = Ts*[cos(x{k}(3)) 0;sin(x{k}(3)) 0; 0 1];
    constraints=[constraints,...
        x{k+1}==x{k}+B*u{k},...
        v_min<=u{k}(1)<=v_max,...
        w_min<=u{k}(2)<=w_max,...
        %ymin<=C*x{k}+D*u{k}<=ymax,...
        dv_min<=u{k}(1)-utmp(1)<=dv_max];
      
    objective=objective+(C*x{k}-[xref(k);yref(k);thetaref(k)])'*Q*(C*x{k}-[xref(k);yref(k);thetaref(k)])+u{k}'*R*u{k};
end

objective=objective+(C*x{Npred+1}-[xref(Npred+1);yref(Npred+1);thetaref(Npred+1)])'*P*(C*x{Npred+1}-[xref(Npred+1);yref(Npred+1);thetaref(Npred+1)]);

options = [];

parameters = {x{1}, u_init, xref, yref, thetaref};
output = u{1};
controller = optimizer(constraints, objective, options, parameters, output);

usim = zeros(du, Nsim);
ysim = zeros(dy, Nsim);
xsim = zeros(dx, Nsim+1);

%condições iniciais 
x_init = [P1(1,1) P1(2,1)]; % primeiro pont da matrix P

xsim(:,1) = [x_init(1) ; x_init(2);0]; 
usim_init = zeros(du,1);
D = 0;

for i=1:Nsim
    if i<=length(z1)-Npred
        u = controller{{xsim(:,i), usim_init, x_ref(i:i+Npred)', y_ref(i:i+Npred)', theta_ref(i:i+Npred)'}};
    else
        u = controller{{xsim(:,i), usim_init, ...
            [x_ref(i:end) repmat(x_ref(end), 1, Npred - (length(z1)-i))]',...
            [y_ref(i:end) repmat(y_ref(end), 1, Npred - (length(z1)-i))]',...
            [theta_ref(i:end) repmat(theta_ref(end), 1, Npred - (length(z1)-i))]'}};
    end
    usim_init = u;
    usim(:,i) = u;
    B = Ts*[cos(xsim(3,i)) 0;sin(xsim(3,i)) 0; 0 1];
    
    xsim(:,i+1) = xsim(:,i) + B*usim(:,i); %update the dynamics
    ysim(:,i) = C*xsim(:,i);   
end

end