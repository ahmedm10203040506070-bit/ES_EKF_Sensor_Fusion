classdef rotation
  methods (Static)
      function C = quat2rot(q)
      q = q(:);
      q = q / norm(q);
      w = q(1);
      x = q(2); 
      y = q(3);
      z = q(4);
      C =[1-(2*(x^2  + y ^2)), 2*((x*y) - (w*z)), 2 *((x*z) +(w*y)); 
          2*((x* y) +(w*z)) , 1-2*(x^2 +z^2),2*((y* z)- (w * x));
          2 *((x*z) -(w*y)),2*((y*z)+(w*x)),1-2*(x^2 +y^2)];
      end
      function Q = euler2quat(roll,pitch,yaw)
      cr = cos(roll /2);
      cp = cos(pitch/2);
      cy = cos(yaw/2);
      sr = sin(roll/2);
      sp = sin(pitch / 2);
      sy = sin(yaw/2);
      w = (cr*cp *cy) + (sr *sp *sy);
      x = (sr * cp *cy) -(cr *sp *sy);
      y = (cr * sp *cy) +(sr * cp *sy);
      z = (cr*cp*sy) -(sr *sp*cy);
      Q = [w; x;y;z];
      Q = Q /norm(Q);
      end
      function [Roll , Pitch , Yaw] = quat2euler(Q)
            Q = Q(:);
            Q = Q / norm(Q);
            w = Q(1);
            x = Q(2);
            y = Q(3);
            z = Q(4);
          Roll = atan2(2*(y*z + w*x) ,1-2*(x^2 + y^2));
          Pitch = asin(2*(w*y - x*z));
          Yaw = atan2(2*(x*y + w*z), 1-2*(y^2 + z^2));
      end
      function Q3 = quat_mult(Q1 , Q2)
      Q1 = Q1(:);
      Q2 =Q2(:);
      Q1 = Q1 / norm(Q1);
      Q2 = Q2 / norm(Q2);
      w1 = Q1(1);
      x1 = Q1(2);
      y1 = Q1(3);
      z1 = Q1(4);
      w2 = Q2(1);
      x2 = Q2(2);
      y2 = Q2(3);
      z2 = Q2(4);
    w3 = (w1 * w2) - (x1 *x2) -(y1*y2) -(z1 * z2);
    x3 = (w1 *x2) + (w2 * x1) + (y1 * z2) - (y2 * z1);
    y3 = (w1 * y2) +(w2 * y1) -(x1 * z2) +(x2 * z1);
    z3 = (w1 * z2) +(w2 *z1) + (x1 * y2) - (x2 * y1);
    Q3 = [w3 ; x3 ; y3 ;z3];
    Q3 = Q3 / norm(Q3);
      end
      function ang = angle_normalize(ang)
       ang = mod(ang + pi , 2*pi) - pi;
      end 
      function v_skew = skew_symmetric(v)
      v_skew = [0, -v(3),   v(2);
                v(3),  0,  -v(1);
               -v(2),  v(1),  0];   
      end
  end
end