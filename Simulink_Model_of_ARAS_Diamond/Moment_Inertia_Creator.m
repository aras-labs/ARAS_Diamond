function [I] = untitled4(I_bar)

I_xx=I_bar(1);
I_xy=I_bar(2);
I_xz=I_bar(3);
I_yy=I_bar(4);
I_zy=I_bar(5);
I_zz=I_bar(6);


I=[I_xx,I_xy,I_xz;I_xy,I_yy,I_zy;I_xz,I_zy,I_zz];


end

