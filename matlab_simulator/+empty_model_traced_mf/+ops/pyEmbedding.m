function embedding = pyEmbedding(weight, indices, ~, ~, ~ )
%PYEMBEDDING A simple lookup table that looks up embeddings in a fixed dictionary and size.
%inline at::Tensor at::embedding(const at::Tensor &weight, const at::Tensor &indices, 
% int64_t padding_idx = -1, bool scale_grad_by_freq = false, bool sparse = false)

% Copyright 2022-2023 The MathWorks, Inc.

import empty_model_traced_mf.ops.*

%padding_idx - not relevant in the functional form. Weights are initialized
%to zero for padded indices.

%Inputs will be in reverse pytorch ordering
Xval = indices.value; %Expects input in 1-based indexing
Wval = stripdims(weight.value);

Xrank = indices.rank;
Yrank = Xrank + 1;

% If X is a vector, ensure it is a column vector
if Xrank==1
    Xval = [Xval(:)];
    Xval = dlarray(Xval, repmat('U', 1, max(2,Xrank)));
end

embeddingVal = embed(Xval, Wval);

embeddingVal = dlarray(embeddingVal, repmat('U', 1, max(2,Yrank)));
embedding = struct('value', embeddingVal, 'rank', Yrank);
end