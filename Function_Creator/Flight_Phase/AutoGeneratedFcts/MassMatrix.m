function M = MassMatrix(x1,y1,theta,s,phi,g,k,L_sp0,L_m1,m1,I1,m2)
%MASSMATRIX
%    M = MASSMATRIX(X1,Y1,THETA,S,PHI,G,K,L_SP0,L_M1,M1,I1,M2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    09-Jan-2017 01:15:15

t2 = phi+theta;
t3 = cos(t2);
t4 = m2.*s.*t3;
t5 = m1+m2;
t6 = sin(t2);
t7 = m2.*s.*t6;
t8 = s.^2;
t9 = m2.*t8;
t10 = m2.*t6;
M = reshape([t5,0.0,t4,t10,t4,0.0,t5,t7,-m2.*t3,t7,t4,t7,I1+t9,0.0,t9,t10,-m2.*t3,0.0,m2,0.0,t4,t7,t9,0.0,t9],[5,5]);
