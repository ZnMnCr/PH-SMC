function k4_sat = antiSat(x, kMax)
    k4 = x(67);

    k4_sat = kMax*((k4 >= kMax) + (k4 < kMax).* (k4 / kMax));
    
end
