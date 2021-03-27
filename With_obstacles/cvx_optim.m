%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  CVX optimization begins with velocity and acceleration constraints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
function [cvx_optval, optimal_vel] = cvx_optim(guess,A,Q,C,min_vel,max_vel,min_accel,max_accel,dt,n_steps,initial_vx,initial_vy,final_vx,final_vy,constraint_obsx,constraint_obsy,constraint_obsr,num_obs)
    
    cvx_begin quiet
    
    variables optimal_vel(2*(n_steps-1),1);
    
    % min(xT.A.x + QT.x + C)
    minimize ((optimal_vel'*A*optimal_vel) + ( Q'*optimal_vel) + C);   

    subject to
    
    % velocity constraint : minimim_velocity <= optimal_vel <= maximum_velocity 
    min_vel <= optimal_vel <= max_vel;
    
    % acceleration constraint : minimim_acceleration*dt <= optimal_vel(t+1)-optimal_vel(t) <= maximum_acceleration*dt 
    min_accel*dt <= optimal_vel(2:n_steps-1,1) - optimal_vel(1:n_steps-2,1) <= max_accel*dt;
    min_accel*dt <= optimal_vel(1,1) - initial_vx <= max_accel*dt;
    
    
    min_accel*dt <= optimal_vel(n_steps+1:end,1)-optimal_vel(n_steps:end-1,1) <= max_accel*dt;
    min_accel*dt <= optimal_vel(n_steps,1) - initial_vy <= max_accel*dt;
   
    optimal_vel(n_steps-1,1)== final_vx;
    optimal_vel(end,1)== final_vy;
    
    for iter_obs = 1: num_obs
        for iter_step = 1:n_steps-1
        
            mat_x = constraint_obsx(iter_step,iter_obs)*(2*dt);
            mat_y = constraint_obsy(iter_step,iter_obs)*(2*dt);
            
            placeholder_mat = [ones(iter_step,1);zeros(n_steps-iter_step-1,1)];
            holonomic_mat = [mat_x*placeholder_mat ; mat_y*placeholder_mat ];
            (holonomic_mat' * optimal_vel) >= (holonomic_mat'*guess) + (constraint_obsr(1,iter_obs)).^2 - (constraint_obsx(iter_step,iter_obs)^2 + constraint_obsy(iter_step,iter_obs)^2); 
      
        end
    end
    
    cvx_end
end