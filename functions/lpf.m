function filtered = lpf(raw, prev_filtered, alpha)
    filtered = alpha * prev_filtered + (1 - alpha) * raw;
end