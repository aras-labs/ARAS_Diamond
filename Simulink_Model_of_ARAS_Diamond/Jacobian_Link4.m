function [J4] = untitled2(alpha,beta,gamma,A,B,a,b,c,d)


    K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
    Q=((cos(gamma)*sin(A)+sin(gamma)*K*cos(A)))/((cos(B)*sin(beta)));
    
    
        %Bdot=gamma_dot*Q;
%     theta__1_dot=phi_dot+K*gamma_dot
%     
%     Omega4=(phi_dot+K*gamma_dot)*a+gamma_dot*Q*c;
    
J4=[a,K*a+Q*c];

end

