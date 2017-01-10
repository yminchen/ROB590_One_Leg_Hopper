function invM = InvMassMatrix_st(s,phi,theta,g,k,L_sp0,L_m1,m1,I1)
%INVMASSMATRIX
%    INVM = INVMASSMATRIX(S,PHI,THETA,X2,Y2,G,K,L_SP0,L_M1,M1,I1)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    08-Jan-2017 15:39:39

t2 = 1.0./m1;
t3 = 1.0./s.^2;
t4 = t2.*t3;
invM = reshape([t2,0.0,0.0,0.0,t4,t4,0.0,t4,(t2.*t3.*(I1+m1.*s.^2))./I1],[3,3]);
