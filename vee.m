function k = vee(K)
% function k = vee(K) :inverse of crossmat.m
%
k=[-K(2,3);K(1,3);-K(1,2)];
