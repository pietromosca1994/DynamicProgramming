%% Implementation of 1-D Dynamic Programming framework
%% Author: Pietro Mosca
%% Email: pietromosca1994@gmail.com
%% Date: 04.02.2021

classdef DP<handle

    properties
        Jtogo;      % array(time, n_states)                 cost-to-go table            
        u_opt;      % array(time, n_states)                 optimal policy
        time;       % array(time)                           time array
        DX;         % array(n_states)                       discrete states
        DU;         % array(n_inputs)                       discrete input
        W;          % array(time, n_disturbances)           distrubances 
        log;        % struct                                log 
        X_opt;      % array(time, n_inputs)                 optimal state trajectory
        
        prb;
        grd;
        par;
    
    end % end of properties
    
    
    methods 
        %% function to initialize the DP 
        % grd           struct     grid definition
        % prb           struct     problem description
        % options       struct     options          
        
        function init(obj, grd, prb, options)
            obj.prb=prb;
            obj.grd=grd;
            obj.time=obj.prb.start_T:obj.prb.Ts:obj.prb.end_T;
            obj.DX=linspace(obj.grd.Xn.lo, obj.grd.Xn.hi, obj.grd.Nx);                        
            obj.DU=linspace(obj.grd.Un.lo, obj.grd.Un.hi, obj.grd.Nu);
                       
            % initialize cost-to-go grid to Inf value
            % scale to n dim
            obj.Jtogo=ones(numel(obj.time), obj.grd.Nx)*obj.prb.InfCost;
            % initialize control grid
            obj.u_opt=zeros(size(obj.Jtogo));
            
            % initialize forward simulation
            obj.X_opt=zeros(size(obj.time));          
            
            % final cost initialization
            if isempty(options.gN)
             obj.Jtogo(end, (obj.DX>obj.grd.XN.hi | obj.DX<obj.grd.XN.lo))=obj.prb.InfCost;
             obj.Jtogo(end, (obj.DX<obj.grd.XN.hi & obj.DX>obj.grd.XN.lo))=prb.G(obj.DX(obj.DX<obj.grd.XN.hi & obj.DX>obj.grd.XN.lo));           
            else
              obj.Jtogo=options.gN;
            end
            
            if options.log==true
              obj.log.Jtogo_t=zeros(numel(obj.time), numel(obj.DX), numel(obj.DU)); % array(time, n_states, n_inputs)  temporary cost to go
              obj.log.elapsed_time=zeros(1, numel(obj.time)-1);                     % array(time)                      elapsed time per time step
              obj.log.total_elapsed_time=0;                                       % float                            total elapsed time
              obj.log.u_opt_t=zeros(1, numel(obj.time)-1);
            end
        end
        
        %% Function to compute the Jtogo for a time step
        % T_step        float       time step
        % prb           struct      problem description
        % options       struct      options sructure
        function compute_step(obj, T_step, options)
            
            if options.verbose==true
              disp(['[INFO] T_step:', num2str(T_step)]);
            end        
            
            if not(isempty(obj.W))
              inp.W=obj.W(T_step, :);
            end
            
            % loop through the states
            for i=1:numel(obj.DX)               
                
                % loop through the inputs
                for j=1:numel(obj.DU)
                  % group inputs
                  inp.X=obj.DX(i);
                  inp.U=obj.DU(j);
                  inp.Ts=obj.prb.Ts;
                  if isempty(obj.prb.W)
                    inp.W=[];
                  else
                    inp.W=obj.prb.W(T_step, :);
                  end
                  
                  par=[];                  
                  
                  % cost evaluation
                  [X(j), J(j), I(j), signals]=obj.prb.J(inp, par);
                  
                  if strcmp(options.interp, 'interp1')
                    % interpolation on next cost-to-go vector
                    Jtogo_t(j)=J(j)+interp1(obj.DX, obj.Jtogo(T_step+1, :), X(j));
                  end
                                    
                 [obj.Jtogo(T_step, i), idx]=min(Jtogo_t);
                 obj.u_opt(T_step, i)=obj.DU(idx);
                                
                end
                
                % log result
                if options.log==true
                  obj.log.Jtogo_t(T_step, i, :)=Jtogo_t;
                end
                
            end
        end
      
      %% function for backpropagation 
      % prb           struct      problem description
      % options       struct      options structure
      function get_BP(obj, options)
        disp('[INFO] Started Dynamic Programming')
        
        h = waitbar (0, '0.00%');
        % loop through time steps
        for T_step=numel(obj.time)-1:-1:1
          
          waitbar((numel(obj.time)-T_step)/numel(obj.time), h, sprintf ('%.2f%%', 100*(numel(obj.time)-T_step)/numel(obj.time)));

          
          t0 = clock (); % start time
          obj.compute_step(T_step, options)
          elapsed_time=etime (clock (), t0); % elapsed time 
          
          % log execution time
          if options.log==true
            obj.log.elapsed_time(T_step)=elapsed_time; 
          end          
        end
        
        delete(h)
        disp('[INFO] Backpropagation completed');
        
        if options.log==true
            obj.log.total_elapsed_time=sum(obj.log.elapsed_time);
            disp(['[INFO] Elapsed ', num2str(obj.log.total_elapsed_time) , ' s']);
        end
       
      end
      
      %% function for forward simulation
      % grd           struct     grid definition
      % prb           struct      problem description
      function forward_sim(obj, options)
        
        % initial state initialization
        obj.X_opt(1)=obj.grd.X0;
            
        % loop through time forward
        for T_step=1:numel(obj.time)-1
          inp.X=obj.X_opt(T_step);
          inp.U=interp1(obj.DX, obj.u_opt(T_step, :), inp.X);
          inp.Ts=obj.prb.Ts;
          if isempty(obj.prb.W) 
            inp.W=[];
          else          
            inp.W=obj.prb.W(T_step, :);
          end
          
          par=[];         
          [obj.X_opt(T_step+1), ~, ~, ~]=obj.prb.J(inp, par);
          
          if options.log==true
            obj.log.u_opt_t(T_step)=inp.U;
          end
          
        end
        
        % check if final cost is between the boundaries specified
        if obj.X_opt(end)<obj.grd.XN.lo || obj.X_opt(end)>obj.grd.XN.hi
          disp('[WARNING] Final state NOT betweend Final State bounds');
        else
          disp('[INFO] Final State between Final State bounds');	
        end
		
      end
    
    end % end of methods
    
end % end of class
