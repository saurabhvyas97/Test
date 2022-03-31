clear all
close all
clc
%% Vehicle Parameter
%global m Jz lf lr C12 C34 delta velocity time_delta fz12 fz34
m=1435;
Jz=2380;
L=2.55;
C12=90e3;
C34=100e3;
lambda=0.41;
lf=lambda*L;
lr=(1-lambda)*L;
file_data=read_ascii('DLC_50_english.ASC');
load("yawrate_ref_data.mat")
vx = out.vx;


% Constraints
u0 = 0;
umin = -20/57.3 - u0;
umax = 20/57.3 - u0;
xmin = [-0.4;-0.87];
xmax = [ 0.4;0.87];

% Objective function
Q = diag([0 10]);
QN = Q;
R = 0.1;

% Initial and reference states
x0 = [0;0];

xr = zeros(2,length(out.vx));
xr(2,:) = out.yawrate_ref;
% Prediction horizon
N = 50;

% Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
% - quadratic objective
P = blkdiag( kron(speye(N), Q), QN, kron(speye(N), R) );


% Create an OSQP object
prob = osqp;


figure;


% Simulate in closed loop
nsim = length(vx);
for i = 1 : nsim
    
    %vx = 12;
    A_c = [-(C12+C34)/m*vx(i) ((lr*C34-lf*C12)/(m*vx(i)))-vx(i);
     (lr*C34-lf*C12)/(Jz*vx(i)) -(lf*lf*C12+lr*lr*C34)/(Jz*vx(i))];

    B_c = [C12/m; C12*lf/Jz];

    C = eye(2);

    D = [0;0];

    sys_c = ss(A_c,B_c,C,D);
    sys_d = c2d(sys_c,0.01);

    Ad = sys_d.A;
    Bd = sys_d.B;
    [nx, nu] = size(Bd);
    % - linear dynamics
    Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), Ad);
    Bu = kron([sparse(1, N); speye(N)], Bd);
    Aeq = [Ax, Bu];
    
    leq = [-x0; zeros(N*nx, 1)];
    ueq = leq;
    % - input and state constraints
    Aineq = speye((N+1)*nx + N*nu);
    lineq = [repmat(xmin, N+1, 1); repmat(umin, N, 1)];
    uineq = [repmat(xmax, N+1, 1); repmat(umax, N, 1)];
    % - OSQP constraints
    A = [Aeq; Aineq];
  
    l = [leq; lineq];
    u = [ueq; uineq];
    
    qxr_temp = reshape(Q*xr(:,i:N+i-1),2*N,1);
    % - linear objective
    q = [-qxr_temp; -QN*xr(:,N+i); zeros(N*nu, 1)];

    if i ==1 

        % Setup workspace
        prob.setup(P, q, A, l, u, 'warm_start', true);
    else
        prob.update('q',q);
        prob.update('Ax',nonzeros(A));
    end

    % Solve
    res = prob.solve();
    x_pred = reshape(res.x(1:N*2),2,N);
      subplot(2,1,1);
      
      plot(xr(2,:))
      xlim([i N+i+100])
      hold on
      %grid on;
      plot(i:i+N-1,x_pred(2,:),'k')
      plot(i,x0(2),'r*')
      
      hold off
      
      
    
    

    
    % Check solver status
    if ~strcmp(res.info.status, 'solved')
        error('OSQP did not solve the problem!')
    end

    % Apply first control input to the plant
%     velocity = vx(i);
    ctrl = res.x(N * nx + nx + 1);
    x0 = Ad*x0 + Bd*ctrl+[0;(rand(1)-0.5)*0.02];
      subplot(2,1,2);
      
      plot(zeros(length(vx),1),'w')
      xlim([i N+i+100])
            ylim([-20/57.3 20/57.3])

      hold on;
      %grid on;
      plot(i:i+N-1,res.x(N*nx+nx+1:end),'k')
      plot(i,ctrl,'*')
      hold off
%     subplot(3,1,3)
%     hold on
%     plot(i,ctrl,'k*')
    % Update initial state
    % Update initial state
    l(1:nx) = -x0;
    u(1:nx) = -x0;
    prob.update('l', l, 'u', u);
    pause(0.01)
    x_out(:,i) = x0;
    u_out(:,i) = ctrl;
end