function dlOut = pyEq(Xin, Yin)
%pyEq Computes element-wise equality
%The second argument can be a number or a tensor whose shape is broadcastable with the first argument.

%inline at::Tensor at::eq(const at::Tensor &self, const at::Scalar &other)
%inline at::Tensor at::eq(const at::Tensor &self, const at::Tensor &other)

% Copyright 2022-2023 The MathWorks, Inc.

import empty_model_traced_mf.ops.*

%Inputs will be in reverse pytorch ordering
Xval = Xin.value;
Yval = Yin.value; %Yin can be a dlarray 
% or Integer/float scalar (for the two overloaded version)


Xrank = Xin.rank;

% If X is a vector, ensure it is a column vector
if Xrank==1
    Xval = [Xval(:)];
end

% "==" return dlarray when Yval is a scalar and does not equate NaNs,
% aligns with PyTorch
dlOutVal = (Xval == single(Yval));

dlOutVal = dlarray(dlOutVal, repmat('U', 1, max(2,Xrank)));
dlOut = struct('value', dlOutVal, 'rank', Xrank);
end