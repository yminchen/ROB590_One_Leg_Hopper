function BodyR = BodyR(x1,y1,theta,s,phi,g,k,L_sp0,L_m1,m1,I1,m2)
%BODYR
%    BODYR = BODYR(X1,Y1,THETA,S,PHI,G,K,L_SP0,L_M1,M1,I1,M2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    09-Jan-2017 01:15:16

BodyR = [x1+L_m1.*cos(theta).*(1.0./2.0);y1+L_m1.*sin(theta).*(1.0./2.0)];
