function [Ibar] = untitled3(I)

I_xx=I(1,1);
I_xy=I(1,2);
I_xz=I(1,3);
I_yy=I(2,2);
I_zy=I(2,3);
I_zz=I(3,3);

Ibar=[I_xx;I_xy;I_xz;I_yy;I_zy;I_zz];


end

