function y = myMeasurementFcn(x)
    y = zeros(6, 1);
    y(1:3) = x(1:3);      % Position measurements
    y(4:6) = x(10:12);    % Angular velocity measurements
end