%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  CVX optimization begins with velocity and acceleration constraints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
function [cvx_optval, optimal_vel] = cvx_optim(guess,A,Q,C,min_vel,max_vel,min_accel,max_accel,dt,n_steps,initial_vx,initial_vy,final_vx,final_vy)
    
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
   
    
    cvx_end
end