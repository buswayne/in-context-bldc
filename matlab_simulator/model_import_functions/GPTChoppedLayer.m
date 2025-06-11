classdef GPTChoppedLayer < nnet.layer.Layer
    properties (Learnable)
        Wte       % Linear weights for input
        Bte       % Bias for input
        Wpe       % Positional embeddings
    end
    
    properties
        NumU      % config.n_u
        NumEmb    % config.n_embd
        BlockSize % config.block_size
    end

    methods
        function layer = GPTChoppedLayer(numU, numEmb, blockSize, name)
            layer.Name = name;
            layer.NumU = numU;
            layer.NumEmb = numEmb;
            layer.BlockSize = blockSize;
            
            % Initialize learnable parameters
            layer.Wte = randn([numEmb, numU]) * 0.02;
            layer.Bte = zeros([numEmb, 1]);
            layer.Wpe = randn([numEmb, blockSize]) * 0.02;
        end
        
        function Z = predict(layer, X)
            % X: size (B, T, NumU) in MATLAB (channels, batch, sequence)
            [U, B, T] = size(X);
            % assert(T == layer.BlockSize, 'wrong_order');
            % assert(U == layer.NumU, 'wrong_order');

            % size(layer.Wte)
            % size(X)
            % X = permute(X, [1 3 2]);
            % size(X)
            % size(layer.Bte)
            % 
            % Token embedding (like nn.Linear)
            % Wte: (n_embd x n_u), X: (n_u x T x B)
            % tokEmb = pagemtimes(layer.Wte, X) + layer.Bte'; % (n_embd x T x B)
            
            % tokEmb = permute(tokEmb, [2,3,1]);
            % size(tokEmb)

            % Position embedding
            pos = 1:T;
            posEmb = layer.Wpe(pos, :); % (T x n_embd)
            Z = posEmb;
            % size(posEmb)
            % posEmb = permute(posEmb, [3,1,2]);
            % size(posEmb)
            % Z = tokEmb + posEmb;
            % 
            Z = permute(Z, [2,3,1]);
        end
    end
end
