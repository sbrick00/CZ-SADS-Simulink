function out = skew(v)
% skew-symmetric cross product matrix of vector v

out = [0     -v(3)  v(2);
       v(3)  0      -v(1);
       -v(2)  v(1)  0 ];
end

