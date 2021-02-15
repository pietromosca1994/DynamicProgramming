%% Test function for the DP framework used to solve the fishery problem
%% Author: Pietro Mosca
%% Email: pietromosca1994@gmail.com
%% Date: 04.02.2021

clear all;

% Fishery Problem test

% link resources 
addpath('../');
addpath('../benchmark_functions/');

date='20210214';

%% input definition
% grid definition
grd.Nx=101;
grd.Xn.lo=0;
grd.Xn.hi=1000;
grd.XN.lo=750;
grd.XN.hi=1000;
grd.Nu=11;
grd.Un.lo=0;
grd.Un.hi=10;
grd.X0=500;

% problem definition
prb.Ts=1/5;                       % float               [day]
prb.start_T=0.4;                  % float               [day]
prb.end_T=200;                    % float               [day]
prb.InfCost=1000;                 % float               infinite cost
prb.G=@(x)(0);                    % function handler    final cost function
prb.J=@(inp, par)(fishery(inp, par));

% options definition
options.interp='round';   % str               interpolation method
options.log=true;           % bool              logs intermidiate results
options.verbose=false;       % bool              shows logs if true
options.gN=[];              % array(n-dim)      final cost matrix

%% Functionality test
dynamic_programming=DP();
dynamic_programming.init(grd, prb, options);

T_step=1000;
% test step
%dynamic_programming.compute_step(T_step, prb, options);

%test algorithm
dynamic_programming.get_BP(prb, options)

% forward computation
dynamic_programming.forward_sim(grd, prb);

%% result plot
% plot final cost array
figure();

subplot(3,1,1)
plot(dynamic_programming.DX, dynamic_programming.Jtogo(end, :));
xlabel('X');
ylabel('Jtogo');
title(['Final Cost to Go']);
xticks(dynamic_programming.DX);
grid on;

% plot jtogo for one step
step_Jtogo=size(dynamic_programming.Jtogo,1)-1;

subplot(3,1,2)
plot(dynamic_programming.DX, dynamic_programming.Jtogo(step_Jtogo, :));
xlabel('X');
ylabel('Jtogo');
title(['Jtogo @', num2str(step_Jtogo)]);
xticks(dynamic_programming.DX);
grid on;

% plot optimal input for one step
step_u_opt=step_Jtogo;

subplot(3,1,3)
plot(dynamic_programming.DX, dynamic_programming.u_opt(step_u_opt, :));
xlabel('X');
ylabel('u_opt');
title(['u_o_p_t @', num2str(step_u_opt)]);
xticks(dynamic_programming.DX);
grid on;

%
figure()
subplot(2,1,1)
contourf(dynamic_programming.time, dynamic_programming.DX,  dynamic_programming.Jtogo');
colorbar;
xlabel('Time');
ylabel('State');
title('J-to-go');

subplot(2,1,2)
contourf( dynamic_programming.time, dynamic_programming.DX, dynamic_programming.u_opt');
colorbar;
xlabel('Time');
ylabel('State');
title('u_o_p_t');

figure();
plot(dynamic_programming.time, dynamic_programming.X_opt);
xlabel('Time');
ylabel('State');
ylim([dynamic_programming.DX(1), dynamic_programming.DX(end)]);
title('Optimal State-Trajectory');

figure();
plot(dynamic_programming.time(1:end-1), dynamic_programming.log.elapsed_time);
xlabel('Time');
ylabel('Computational Time per Step [ms]');
title('Computational Time per Time ');

% save data
Jtogo=dynamic_programming.Jtogo;
u_opt=dynamic_programming.u_opt;
log=dynamic_programming.log;
time=dynamic_programming.time;
DX=dynamic_programming.DX;
DU=dynamic_programming.DU;

save([date, '_workspace.mat']);





