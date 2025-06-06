function Y = pyArange(startValue, endValue, stepValue, dtype)
%PYARANGE returns a 1-d array at given steps from start to end.
% at::arange(const at::Scalar&, const at::Scalar&, const at::Scalar&,
% c10::optional<at::ScalarType>, c10::optional<at::Layout>, 
% c10::optional<at::Device>, c10::optional<bool>)

% Copyright 2024 The MathWorks, Inc.

import empty_model_traced_mf.ops.*

% If input is a dlarray extract data.
if isdlarray(startValue.value)
    startValue.value = extractdata(startValue.value);
end

if isdlarray(endValue.value)
    endValue.value = extractdata(endValue.value);
end


% Cast values to single to prevent type mismatch for ":" operator.
startValue = single(startValue.value);
endValue = single(endValue.value);
stepValue = single(stepValue.value);

% End value for set of points is non-inclusive.
endValue = endValue - stepValue + mod(endValue, stepValue);

% Create array over interval [startValue, endValue) with specified
% step size.
Yval = startValue: stepValue: endValue;

% Enumerators of c10 scalars "int" data type.
dtypeList = [0,1,2,3,4,12,13,14,16,17];
% If dtype is "int", step size is rounded to int using "fix".
if(ismember(dtype.value, dtypeList))
    stepValue = fix(stepValue);
    endValue = startValue + (length(Yval) - 1).*stepValue;
    Yval = startValue: stepValue: endValue;
end

Yval = dlarray(Yval,'UU');
Y = struct('value', Yval, 'rank', 1);
end