function [J] = jac(alpha,beta,gamma,A)

K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
J=[1,K;1,-K];

end

