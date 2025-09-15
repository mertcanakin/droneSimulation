function out = sat(in, boundary)
    if abs(in) <= boundary
        out = in/boundary;
    else
        out = sign(in);
    end
end