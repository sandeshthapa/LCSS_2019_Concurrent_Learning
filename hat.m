function khat = hat(k)
% function khat = hat(k) --> gives skew symmetric matrix R^3 --. so(3)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  