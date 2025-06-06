function dlOut = pyMaskedFill(Xin, maskStruct, valueStruct)
%pyMaskedFill Fills Xin with value at mask indices
%The second argument can be a number or a tensor whose shape is broadcastable with the first argument.

%inline at::Tensor at::masked_fill(const at::Tensor &self, const at::Tensor &mask, const at::Tensor &value)
%inline at::Tensor at::masked_fill(const at::Tensor &self, const at::Tensor &mask, const at::Scalar &value)

% Copyright 2022-2024 The MathWorks, Inc.

import empty_model_traced_mf.ops.*

%Inputs will be in reverse pytorch ordering
Xval = Xin.value;
maskStructVal= maskStruct.value;
value = valueStruct.value;

Xrank = Xin.rank;

% If X is a vector, ensure it is a column vector
if Xrank==1
    Xval = [Xval(:)];
    maskStructVal = [maskStructVal(:)];
end

%Broadcast indices to size of input
maskStructVal = maskStructVal + zeros(size(Xval));
Xval(logical(maskStructVal)) = single(value);


dlOutVal = dlarray(Xval, repmat('U', 1, max(2,Xrank)));
dlOut = struct('value', dlOutVal, 'rank', Xrank);
end