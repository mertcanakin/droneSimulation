function rho = FcnPlane_compute_density(h, params)
    g = params.g;
    if h < params.h_limit
        T_air = params.T0 - params.L*h;
        rho = params.rho0 * (T_air/params.T0)^((g/(params.R*params.L)) - 1);
    else
        T_air = params.T0 - params.L*params.h_limit;
        rho = params.rho0 * (T_air/params.T0)^((g/(params.R*params.L)) - 1);
    end
end