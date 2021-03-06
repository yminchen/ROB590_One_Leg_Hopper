function f_cg = F_CoriGrav(s,phi,theta,ds,dphi,dtheta,x2,y2,g,k,L_sp0,L_m1,m1,I1)
%F_CORIGRAV
%    F_CG = F_CORIGRAV(S,PHI,THETA,DS,DPHI,DTHETA,X2,Y2,G,K,L_SP0,L_M1,M1,I1)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    08-Jan-2017 23:58:14

f_cg = [k.*(L_sp0.*2.0-s.*2.0).*(1.0./2.0)+dphi.^2.*m1.*s-g.*m1.*cos(phi);-m1.*s.*(dphi.*ds.*2.0-g.*sin(phi));0.0];
