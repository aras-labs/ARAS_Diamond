function [J] = untitled2(a,b,c,d,gamma,beta)


ad=cross(a,d)/(sin(gamma));

J_x=[dot(ad,a),dot(ad,ad);dot(ad,a),dot(ad,ad)];
J_q=[-dot(ad,c),0;0,-dot(ad,b)];

J=inv(J_q)*(J_x);


end

