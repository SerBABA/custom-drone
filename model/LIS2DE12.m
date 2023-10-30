function est = LIS2DE12(val)
    MAX_G = 4;
    SENSITIVITY = 500; %mg per digit
    %there are 256 digits as data stored in
    %8-bit register
    %datasheet value is 31.2, but this seems to good to be true
    est = normrnd(val, (SENSITIVITY/(256 * 1000)) * MAX_G * 9.81 * 2);

    %clamp between -4 and 4 G
    est = min(max(est, -MAX_G*9.81), MAX_G*9.81);
end