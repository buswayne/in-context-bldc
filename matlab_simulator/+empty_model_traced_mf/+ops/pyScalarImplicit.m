function Y = pyScalarImplicit(X)
%PYSCALARIMPLICIT Converts a single-element tensor into a scalar value
%implicitly.
%at::Tensor aten::ScalarImplicit(Tensor a)

% Copyright 2024 The MathWorks, Inc.

import empty_model_traced_mf.ops.*

% Input dlarray scalar contained in 'X' is saved into 'Y'.
Y = X;
Y.rank = 0;
end