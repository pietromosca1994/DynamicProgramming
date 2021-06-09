%% Test function for the DP framework used to solve torque split problem for an Hybrid Vehicle
%% Author: Pietro Mosca
%% Email: pietromosca1994@gmail.com
%% Date: 08.05.2021

clear all;

% link resources 
run('init_lib_DP.m');

run('init_lib_VehicleModel.m');
run('init_sim_env.m');
run('init_driving_cycle.m');

% initialize VehicleModel
global vehicle
vehicle=sim_system;
vehicle.init(p_sim_env.Ts);
vehicle.vehicle.log_flag=false;
vehicle.EMG.log_flag=false;
vehicle.gearbox.log_flag=false;
vehicle.ICE.log_flag=false;
vehicle.battery.log_flag=false;
vehicle.T_split.log_flag=false;

% initialize driving cycle
driving_cycle=driving_cycle;
driving_cycle.init(p_driving_cycle);

%% input definition
% grid definition
grd.Nx=91;                        % float               number of grid points in the state grid
grd.Xn.lo=0.05;                   % float               lower limit for each state
grd.Xn.hi=0.95;                   % float               upper limit for each state
grd.XN.lo=0.59;                   % float               final state lower constraint
grd.XN.hi=0.61;                   % float               final state upper constarint
grd.Nu=16;                        % float               number of grid points in input grid
grd.Un.lo=-2;                     % float               lower limit for each input
grd.Un.hi=1;                      % float               upper limit for each input            
grd.X0=0.6;                       % float               initial state

% problem definition
prb.Ts=p_sim_env.Ts;                                                  % float               sampple time [day]
prb.start_T=p_sim_env.StartTime;                                      % float               start time  [day]
prb.end_T=p_sim_env.EndTime;                                          % float               end time  [day]
prb.InfCost=1000;                                                     % float               infinite cost
time=prb.start_T:prb.Ts:prb.end_T;
prb.W(:,1)=interp1(driving_cycle.time, driving_cycle.v, time);        % float               speed
prb.W(:,2)=interp1(driving_cycle.time, driving_cycle.a, time);        % float               acceleration 
prb.W(:,3)=interp1(driving_cycle.time, driving_cycle.alpha, time);    % float               road gradient
prb.W(:,4)=interp1(driving_cycle.time, driving_cycle.g, time);        % float               gear
prb.G=@(x)(0);                                                        % function handler    final cost function
prb.J=@(inp, par)(TorqueSplit(inp, par));                             % function handler    problem definition   

% options definition
options.interp='interp1';   % str               interpolation method
options.log=true;           % bool              logs intermidiate results
options.verbose=false;      % bool              shows logs if true
options.gN=[];              % array(n-dim)      final cost matrix

%% Functionality test
dynamic_programming=DP();
dynamic_programming.init(grd, prb, options);

%test algorithm
dynamic_programming.get_BP(options)

% forward computation
dynamic_programming.forward_sim(options);

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
subplot(3,1,1)
plot(dynamic_programming.time, prb.W(:,1));
xlabel('Time');
ylabel('State');

subplot(3,1,2)
plot(dynamic_programming.time, dynamic_programming.X_opt);
xlabel('Time');
ylabel('State');
ylim([dynamic_programming.DX(1), dynamic_programming.DX(end)]);

subplot(3,1,3)
plot(dynamic_programming.time(1:end-1), dynamic_programming.log.u_opt_t);
xlabel('Time');
ylabel('U_{opt}');
title('Optimal Input');

figure();
plot(dynamic_programming.time(1:end-1), dynamic_programming.log.elapsed_time);
xlabel('Time');
ylabel('Computational Time per Step [ms]');
title('Computational Time per Time ');