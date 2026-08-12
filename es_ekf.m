classdef es_ekf
 methods (Static)
     function [p_next, v_next, q_next, P_next] = perdict(p , v, q, P,imu_f,imu_w, var_imu_f,var_imu_w ,g ,dt)
     C_ns = rotation.quat2rot(q);
     A_cc = (C_ns * imu_f) +  g;
     v_next = v + A_cc *dt;
     p_next = p + v*dt + ((1/2) * A_cc *(dt ^2));

     detheta = imu_w * dt;
     angle = norm(detheta);
     axis = detheta / angle;
     if angle < 1e-12
      q_step = [1 ; 0.5 *detheta];
     else
         q_step = [cos(angle / 2) ; axis * sin(angle /2)];
     
     end
     q_next = rotation.quat_mult(q , q_step);
       
     F = eye(9);
     F((1 : 3) ,(4 : 6)) = eye(3) * dt;
     F((4:6),(7:9)) = -(rotation.skew_symmetric(C_ns * imu_f)) * dt;
      
     Q = zeros(9);
     Q((4:6),(4:6)) = eye(3) * ((var_imu_f)*(dt ^ 2));
     Q((7:9),(7:9)) = eye(3) * ((var_imu_w)*(dt^2));
     P_next = (F * P * F') +Q;

     end
     function [p_up , v_up , q_up , P_up] = update(p_pred, v_pred, q_pred, P_pred,Z_sensor ,R_sensor)
    x_pred = [p_pred ; v_pred ; zeros(3,1)];
    H = [eye(3) , zeros(3) , zeros(3)];
    y = Z_sensor - (H * x_pred);
    S = (H * P_pred * H') + R_sensor;
    K = (P_pred * H') / S;
    dx = K *y;
    dp = dx(1:3);
    dv = dx(4:6);
    dtheta = dx(7:9);
    p_up = p_pred + dp;
    v_up = v_pred + dv;
    angle = norm(dtheta);
    axis = dtheta / angle;
    if angle < 1e-12
       q_error = [1 ; 0.5*dtheta];
    else
        q_error = [cos(angle /2) ; axis * sin(angle /2)];
    end
    q_up = rotation.quat_mult(q_pred , q_error);
  
      I = eye(9);
      P_up = (I - (K * H)) *P_pred;
     end
 end
end