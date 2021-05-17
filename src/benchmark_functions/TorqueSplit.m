function [X, C, I, signals] = TorqueSplit(inp, par)
  global vehicle;
  vehicle.battery.SOC=0.9; % prevents errors
  
  %% input parsing
  v=inp.W(1);       % vehicle speed
  a=inp.W(2);       % vehicle acceleration
  alpha=inp.W(3);   % road gradient
  gear=inp.W(4);    % gear
  
  u=inp.U(1);
  x=[];
  w=[v, a, alpha, gear];
  
  SOC=inp.X(1);
  [dSOC, fc_act]=vehicle.update(x, u, w);
  SOC=SOC+dSOC*inp.Ts;
  
  %% outputs
  I = 0;
  signals.U(1) = inp.U(1);
  C(1)=inp.Ts.*inp.U(1);
  X(1)=SOC;

  end
