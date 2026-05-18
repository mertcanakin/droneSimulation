function wind = FcnPlane_wind_model(t, h)
    u_w = 2 + 0.001*h + 0.5*sin(0.3*t);
    w_w = 1 + 0.3*sin(0.7*t);
    wind = [u_w; w_w];
end