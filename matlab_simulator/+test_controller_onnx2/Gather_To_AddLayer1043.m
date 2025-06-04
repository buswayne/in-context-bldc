classdef Gather_To_AddLayer1043 < nnet.layer.Layer & nnet.layer.Formattable
    % A custom layer auto-generated while importing an ONNX network.

    %#codegen
    %#ok<*PROPLC>
    %#ok<*NBRAK>
    %#ok<*INUSL>
    %#ok<*VARARG>

    properties (Learnable)
        onnx__MatMul_1000
        onnx__MatMul_1020
        onnx__MatMul_1021
        onnx__MatMul_1022
        onnx__MatMul_1023
        onnx__MatMul_1043
        onnx__MatMul_1044
        onnx__MatMul_1045
        onnx__MatMul_1046
        onnx__MatMul_1066
        onnx__MatMul_1067
        onnx__MatMul_1068
        onnx__MatMul_1069
        onnx__MatMul_1089
        onnx__MatMul_1090
        onnx__MatMul_1091
        onnx__MatMul_1092
        onnx__MatMul_907
        onnx__MatMul_908
        onnx__MatMul_928
        onnx__MatMul_929
        onnx__MatMul_930
        onnx__MatMul_931
        onnx__MatMul_951
        onnx__MatMul_952
        onnx__MatMul_953
        onnx__MatMul_954
        onnx__MatMul_974
        onnx__MatMul_975
        onnx__MatMul_976
        onnx__MatMul_977
        onnx__MatMul_997
        onnx__MatMul_998
        onnx__MatMul_999
        transformer_h_0_ln_1
        transformer_h_0_ln_2
        transformer_h_1_ln_1
        transformer_h_1_ln_2
        transformer_h_2_ln_1
        transformer_h_2_ln_2
        transformer_h_3_ln_1
        transformer_h_3_ln_2
        transformer_h_4_ln_1
        transformer_h_4_ln_2
        transformer_h_5_ln_1
        transformer_h_5_ln_2
        transformer_h_6_ln_1
        transformer_h_6_ln_2
        transformer_h_7_ln_1
        transformer_h_7_ln_2
        transformer_ln_f_wei
        transformer_wpe_weig
        x_h_0_attn_Consta_1
        x_h_0_attn_Consta_12
        x_h_0_attn_Consta_13
        x_h_0_attn_Consta_2
        x_h_0_ln_1_Constant_
        x_h_0_ln_2_Constant_
        x_h_1_attn_Consta_1
        x_h_1_attn_Consta_12
        x_h_1_attn_Consta_13
        x_h_1_attn_Consta_2
        x_h_1_ln_1_Constant_
        x_h_1_ln_2_Constant_
        x_h_2_attn_Consta_1
        x_h_2_attn_Consta_12
        x_h_2_attn_Consta_13
        x_h_2_attn_Consta_2
        x_h_2_ln_1_Constant_
        x_h_2_ln_2_Constant_
        x_h_3_attn_Consta_1
        x_h_3_attn_Consta_12
        x_h_3_attn_Consta_13
        x_h_3_attn_Consta_2
        x_h_3_ln_1_Constant_
        x_h_3_ln_2_Constant_
        x_h_4_attn_Consta_1
        x_h_4_attn_Consta_12
        x_h_4_attn_Consta_13
        x_h_4_attn_Consta_2
        x_h_4_ln_1_Constant_
        x_h_4_ln_2_Constant_
        x_h_5_attn_Consta_1
        x_h_5_attn_Consta_12
        x_h_5_attn_Consta_13
        x_h_5_attn_Consta_2
        x_h_5_ln_1_Constant_
        x_h_5_ln_2_Constant_
        x_h_6_attn_Consta_1
        x_h_6_attn_Consta_12
        x_h_6_attn_Consta_13
        x_h_6_attn_Consta_2
        x_h_6_ln_1_Constant_
        x_h_6_ln_2_Constant_
        x_h_7_attn_Consta_1
        x_h_7_attn_Consta_12
        x_h_7_attn_Consta_13
        x_h_7_attn_Consta_2
        x_h_7_ln_1_Constant_
        x_h_7_ln_2_Constant_
        x_ln_f_Constant_outp
    end

    properties
        ONNXParams         % An ONNXParameters object containing parameters used by this layer.
    end

    methods
        function this = Gather_To_AddLayer1043(name, onnxParams)
            this.Name = name;
            this.NumInputs = 2;
            this.OutputNames = {'output'};
            this.ONNXParams = onnxParams;
            this.onnx__MatMul_1000 = onnxParams.Learnables.onnx__MatMul_1000;
            this.onnx__MatMul_1020 = onnxParams.Learnables.onnx__MatMul_1020;
            this.onnx__MatMul_1021 = onnxParams.Learnables.onnx__MatMul_1021;
            this.onnx__MatMul_1022 = onnxParams.Learnables.onnx__MatMul_1022;
            this.onnx__MatMul_1023 = onnxParams.Learnables.onnx__MatMul_1023;
            this.onnx__MatMul_1043 = onnxParams.Learnables.onnx__MatMul_1043;
            this.onnx__MatMul_1044 = onnxParams.Learnables.onnx__MatMul_1044;
            this.onnx__MatMul_1045 = onnxParams.Learnables.onnx__MatMul_1045;
            this.onnx__MatMul_1046 = onnxParams.Learnables.onnx__MatMul_1046;
            this.onnx__MatMul_1066 = onnxParams.Learnables.onnx__MatMul_1066;
            this.onnx__MatMul_1067 = onnxParams.Learnables.onnx__MatMul_1067;
            this.onnx__MatMul_1068 = onnxParams.Learnables.onnx__MatMul_1068;
            this.onnx__MatMul_1069 = onnxParams.Learnables.onnx__MatMul_1069;
            this.onnx__MatMul_1089 = onnxParams.Learnables.onnx__MatMul_1089;
            this.onnx__MatMul_1090 = onnxParams.Learnables.onnx__MatMul_1090;
            this.onnx__MatMul_1091 = onnxParams.Learnables.onnx__MatMul_1091;
            this.onnx__MatMul_1092 = onnxParams.Learnables.onnx__MatMul_1092;
            this.onnx__MatMul_907 = onnxParams.Learnables.onnx__MatMul_907;
            this.onnx__MatMul_908 = onnxParams.Learnables.onnx__MatMul_908;
            this.onnx__MatMul_928 = onnxParams.Learnables.onnx__MatMul_928;
            this.onnx__MatMul_929 = onnxParams.Learnables.onnx__MatMul_929;
            this.onnx__MatMul_930 = onnxParams.Learnables.onnx__MatMul_930;
            this.onnx__MatMul_931 = onnxParams.Learnables.onnx__MatMul_931;
            this.onnx__MatMul_951 = onnxParams.Learnables.onnx__MatMul_951;
            this.onnx__MatMul_952 = onnxParams.Learnables.onnx__MatMul_952;
            this.onnx__MatMul_953 = onnxParams.Learnables.onnx__MatMul_953;
            this.onnx__MatMul_954 = onnxParams.Learnables.onnx__MatMul_954;
            this.onnx__MatMul_974 = onnxParams.Learnables.onnx__MatMul_974;
            this.onnx__MatMul_975 = onnxParams.Learnables.onnx__MatMul_975;
            this.onnx__MatMul_976 = onnxParams.Learnables.onnx__MatMul_976;
            this.onnx__MatMul_977 = onnxParams.Learnables.onnx__MatMul_977;
            this.onnx__MatMul_997 = onnxParams.Learnables.onnx__MatMul_997;
            this.onnx__MatMul_998 = onnxParams.Learnables.onnx__MatMul_998;
            this.onnx__MatMul_999 = onnxParams.Learnables.onnx__MatMul_999;
            this.transformer_h_0_ln_1 = onnxParams.Learnables.transformer_h_0_ln_1;
            this.transformer_h_0_ln_2 = onnxParams.Learnables.transformer_h_0_ln_2;
            this.transformer_h_1_ln_1 = onnxParams.Learnables.transformer_h_1_ln_1;
            this.transformer_h_1_ln_2 = onnxParams.Learnables.transformer_h_1_ln_2;
            this.transformer_h_2_ln_1 = onnxParams.Learnables.transformer_h_2_ln_1;
            this.transformer_h_2_ln_2 = onnxParams.Learnables.transformer_h_2_ln_2;
            this.transformer_h_3_ln_1 = onnxParams.Learnables.transformer_h_3_ln_1;
            this.transformer_h_3_ln_2 = onnxParams.Learnables.transformer_h_3_ln_2;
            this.transformer_h_4_ln_1 = onnxParams.Learnables.transformer_h_4_ln_1;
            this.transformer_h_4_ln_2 = onnxParams.Learnables.transformer_h_4_ln_2;
            this.transformer_h_5_ln_1 = onnxParams.Learnables.transformer_h_5_ln_1;
            this.transformer_h_5_ln_2 = onnxParams.Learnables.transformer_h_5_ln_2;
            this.transformer_h_6_ln_1 = onnxParams.Learnables.transformer_h_6_ln_1;
            this.transformer_h_6_ln_2 = onnxParams.Learnables.transformer_h_6_ln_2;
            this.transformer_h_7_ln_1 = onnxParams.Learnables.transformer_h_7_ln_1;
            this.transformer_h_7_ln_2 = onnxParams.Learnables.transformer_h_7_ln_2;
            this.transformer_ln_f_wei = onnxParams.Learnables.transformer_ln_f_wei;
            this.transformer_wpe_weig = onnxParams.Learnables.transformer_wpe_weig;
            this.x_h_0_attn_Consta_1 = onnxParams.Learnables.x_h_0_attn_Consta_1;
            this.x_h_0_attn_Consta_12 = onnxParams.Learnables.x_h_0_attn_Consta_12;
            this.x_h_0_attn_Consta_13 = onnxParams.Learnables.x_h_0_attn_Consta_13;
            this.x_h_0_attn_Consta_2 = onnxParams.Learnables.x_h_0_attn_Consta_2;
            this.x_h_0_ln_1_Constant_ = onnxParams.Learnables.x_h_0_ln_1_Constant_;
            this.x_h_0_ln_2_Constant_ = onnxParams.Learnables.x_h_0_ln_2_Constant_;
            this.x_h_1_attn_Consta_1 = onnxParams.Learnables.x_h_1_attn_Consta_1;
            this.x_h_1_attn_Consta_12 = onnxParams.Learnables.x_h_1_attn_Consta_12;
            this.x_h_1_attn_Consta_13 = onnxParams.Learnables.x_h_1_attn_Consta_13;
            this.x_h_1_attn_Consta_2 = onnxParams.Learnables.x_h_1_attn_Consta_2;
            this.x_h_1_ln_1_Constant_ = onnxParams.Learnables.x_h_1_ln_1_Constant_;
            this.x_h_1_ln_2_Constant_ = onnxParams.Learnables.x_h_1_ln_2_Constant_;
            this.x_h_2_attn_Consta_1 = onnxParams.Learnables.x_h_2_attn_Consta_1;
            this.x_h_2_attn_Consta_12 = onnxParams.Learnables.x_h_2_attn_Consta_12;
            this.x_h_2_attn_Consta_13 = onnxParams.Learnables.x_h_2_attn_Consta_13;
            this.x_h_2_attn_Consta_2 = onnxParams.Learnables.x_h_2_attn_Consta_2;
            this.x_h_2_ln_1_Constant_ = onnxParams.Learnables.x_h_2_ln_1_Constant_;
            this.x_h_2_ln_2_Constant_ = onnxParams.Learnables.x_h_2_ln_2_Constant_;
            this.x_h_3_attn_Consta_1 = onnxParams.Learnables.x_h_3_attn_Consta_1;
            this.x_h_3_attn_Consta_12 = onnxParams.Learnables.x_h_3_attn_Consta_12;
            this.x_h_3_attn_Consta_13 = onnxParams.Learnables.x_h_3_attn_Consta_13;
            this.x_h_3_attn_Consta_2 = onnxParams.Learnables.x_h_3_attn_Consta_2;
            this.x_h_3_ln_1_Constant_ = onnxParams.Learnables.x_h_3_ln_1_Constant_;
            this.x_h_3_ln_2_Constant_ = onnxParams.Learnables.x_h_3_ln_2_Constant_;
            this.x_h_4_attn_Consta_1 = onnxParams.Learnables.x_h_4_attn_Consta_1;
            this.x_h_4_attn_Consta_12 = onnxParams.Learnables.x_h_4_attn_Consta_12;
            this.x_h_4_attn_Consta_13 = onnxParams.Learnables.x_h_4_attn_Consta_13;
            this.x_h_4_attn_Consta_2 = onnxParams.Learnables.x_h_4_attn_Consta_2;
            this.x_h_4_ln_1_Constant_ = onnxParams.Learnables.x_h_4_ln_1_Constant_;
            this.x_h_4_ln_2_Constant_ = onnxParams.Learnables.x_h_4_ln_2_Constant_;
            this.x_h_5_attn_Consta_1 = onnxParams.Learnables.x_h_5_attn_Consta_1;
            this.x_h_5_attn_Consta_12 = onnxParams.Learnables.x_h_5_attn_Consta_12;
            this.x_h_5_attn_Consta_13 = onnxParams.Learnables.x_h_5_attn_Consta_13;
            this.x_h_5_attn_Consta_2 = onnxParams.Learnables.x_h_5_attn_Consta_2;
            this.x_h_5_ln_1_Constant_ = onnxParams.Learnables.x_h_5_ln_1_Constant_;
            this.x_h_5_ln_2_Constant_ = onnxParams.Learnables.x_h_5_ln_2_Constant_;
            this.x_h_6_attn_Consta_1 = onnxParams.Learnables.x_h_6_attn_Consta_1;
            this.x_h_6_attn_Consta_12 = onnxParams.Learnables.x_h_6_attn_Consta_12;
            this.x_h_6_attn_Consta_13 = onnxParams.Learnables.x_h_6_attn_Consta_13;
            this.x_h_6_attn_Consta_2 = onnxParams.Learnables.x_h_6_attn_Consta_2;
            this.x_h_6_ln_1_Constant_ = onnxParams.Learnables.x_h_6_ln_1_Constant_;
            this.x_h_6_ln_2_Constant_ = onnxParams.Learnables.x_h_6_ln_2_Constant_;
            this.x_h_7_attn_Consta_1 = onnxParams.Learnables.x_h_7_attn_Consta_1;
            this.x_h_7_attn_Consta_12 = onnxParams.Learnables.x_h_7_attn_Consta_12;
            this.x_h_7_attn_Consta_13 = onnxParams.Learnables.x_h_7_attn_Consta_13;
            this.x_h_7_attn_Consta_2 = onnxParams.Learnables.x_h_7_attn_Consta_2;
            this.x_h_7_ln_1_Constant_ = onnxParams.Learnables.x_h_7_ln_1_Constant_;
            this.x_h_7_ln_2_Constant_ = onnxParams.Learnables.x_h_7_ln_2_Constant_;
            this.x_ln_f_Constant_outp = onnxParams.Learnables.x_ln_f_Constant_outp;
        end

        function [output] = predict(this, input, inputNumDims)
            if isdlarray(input)
                input = stripdims(input);
            end
            inputNumDims = numel(inputNumDims);
            onnxParams = this.ONNXParams;
            onnxParams.Learnables.onnx__MatMul_1000 = this.onnx__MatMul_1000;
            onnxParams.Learnables.onnx__MatMul_1020 = this.onnx__MatMul_1020;
            onnxParams.Learnables.onnx__MatMul_1021 = this.onnx__MatMul_1021;
            onnxParams.Learnables.onnx__MatMul_1022 = this.onnx__MatMul_1022;
            onnxParams.Learnables.onnx__MatMul_1023 = this.onnx__MatMul_1023;
            onnxParams.Learnables.onnx__MatMul_1043 = this.onnx__MatMul_1043;
            onnxParams.Learnables.onnx__MatMul_1044 = this.onnx__MatMul_1044;
            onnxParams.Learnables.onnx__MatMul_1045 = this.onnx__MatMul_1045;
            onnxParams.Learnables.onnx__MatMul_1046 = this.onnx__MatMul_1046;
            onnxParams.Learnables.onnx__MatMul_1066 = this.onnx__MatMul_1066;
            onnxParams.Learnables.onnx__MatMul_1067 = this.onnx__MatMul_1067;
            onnxParams.Learnables.onnx__MatMul_1068 = this.onnx__MatMul_1068;
            onnxParams.Learnables.onnx__MatMul_1069 = this.onnx__MatMul_1069;
            onnxParams.Learnables.onnx__MatMul_1089 = this.onnx__MatMul_1089;
            onnxParams.Learnables.onnx__MatMul_1090 = this.onnx__MatMul_1090;
            onnxParams.Learnables.onnx__MatMul_1091 = this.onnx__MatMul_1091;
            onnxParams.Learnables.onnx__MatMul_1092 = this.onnx__MatMul_1092;
            onnxParams.Learnables.onnx__MatMul_907 = this.onnx__MatMul_907;
            onnxParams.Learnables.onnx__MatMul_908 = this.onnx__MatMul_908;
            onnxParams.Learnables.onnx__MatMul_928 = this.onnx__MatMul_928;
            onnxParams.Learnables.onnx__MatMul_929 = this.onnx__MatMul_929;
            onnxParams.Learnables.onnx__MatMul_930 = this.onnx__MatMul_930;
            onnxParams.Learnables.onnx__MatMul_931 = this.onnx__MatMul_931;
            onnxParams.Learnables.onnx__MatMul_951 = this.onnx__MatMul_951;
            onnxParams.Learnables.onnx__MatMul_952 = this.onnx__MatMul_952;
            onnxParams.Learnables.onnx__MatMul_953 = this.onnx__MatMul_953;
            onnxParams.Learnables.onnx__MatMul_954 = this.onnx__MatMul_954;
            onnxParams.Learnables.onnx__MatMul_974 = this.onnx__MatMul_974;
            onnxParams.Learnables.onnx__MatMul_975 = this.onnx__MatMul_975;
            onnxParams.Learnables.onnx__MatMul_976 = this.onnx__MatMul_976;
            onnxParams.Learnables.onnx__MatMul_977 = this.onnx__MatMul_977;
            onnxParams.Learnables.onnx__MatMul_997 = this.onnx__MatMul_997;
            onnxParams.Learnables.onnx__MatMul_998 = this.onnx__MatMul_998;
            onnxParams.Learnables.onnx__MatMul_999 = this.onnx__MatMul_999;
            onnxParams.Learnables.transformer_h_0_ln_1 = this.transformer_h_0_ln_1;
            onnxParams.Learnables.transformer_h_0_ln_2 = this.transformer_h_0_ln_2;
            onnxParams.Learnables.transformer_h_1_ln_1 = this.transformer_h_1_ln_1;
            onnxParams.Learnables.transformer_h_1_ln_2 = this.transformer_h_1_ln_2;
            onnxParams.Learnables.transformer_h_2_ln_1 = this.transformer_h_2_ln_1;
            onnxParams.Learnables.transformer_h_2_ln_2 = this.transformer_h_2_ln_2;
            onnxParams.Learnables.transformer_h_3_ln_1 = this.transformer_h_3_ln_1;
            onnxParams.Learnables.transformer_h_3_ln_2 = this.transformer_h_3_ln_2;
            onnxParams.Learnables.transformer_h_4_ln_1 = this.transformer_h_4_ln_1;
            onnxParams.Learnables.transformer_h_4_ln_2 = this.transformer_h_4_ln_2;
            onnxParams.Learnables.transformer_h_5_ln_1 = this.transformer_h_5_ln_1;
            onnxParams.Learnables.transformer_h_5_ln_2 = this.transformer_h_5_ln_2;
            onnxParams.Learnables.transformer_h_6_ln_1 = this.transformer_h_6_ln_1;
            onnxParams.Learnables.transformer_h_6_ln_2 = this.transformer_h_6_ln_2;
            onnxParams.Learnables.transformer_h_7_ln_1 = this.transformer_h_7_ln_1;
            onnxParams.Learnables.transformer_h_7_ln_2 = this.transformer_h_7_ln_2;
            onnxParams.Learnables.transformer_ln_f_wei = this.transformer_ln_f_wei;
            onnxParams.Learnables.transformer_wpe_weig = this.transformer_wpe_weig;
            onnxParams.Learnables.x_h_0_attn_Consta_1 = this.x_h_0_attn_Consta_1;
            onnxParams.Learnables.x_h_0_attn_Consta_12 = this.x_h_0_attn_Consta_12;
            onnxParams.Learnables.x_h_0_attn_Consta_13 = this.x_h_0_attn_Consta_13;
            onnxParams.Learnables.x_h_0_attn_Consta_2 = this.x_h_0_attn_Consta_2;
            onnxParams.Learnables.x_h_0_ln_1_Constant_ = this.x_h_0_ln_1_Constant_;
            onnxParams.Learnables.x_h_0_ln_2_Constant_ = this.x_h_0_ln_2_Constant_;
            onnxParams.Learnables.x_h_1_attn_Consta_1 = this.x_h_1_attn_Consta_1;
            onnxParams.Learnables.x_h_1_attn_Consta_12 = this.x_h_1_attn_Consta_12;
            onnxParams.Learnables.x_h_1_attn_Consta_13 = this.x_h_1_attn_Consta_13;
            onnxParams.Learnables.x_h_1_attn_Consta_2 = this.x_h_1_attn_Consta_2;
            onnxParams.Learnables.x_h_1_ln_1_Constant_ = this.x_h_1_ln_1_Constant_;
            onnxParams.Learnables.x_h_1_ln_2_Constant_ = this.x_h_1_ln_2_Constant_;
            onnxParams.Learnables.x_h_2_attn_Consta_1 = this.x_h_2_attn_Consta_1;
            onnxParams.Learnables.x_h_2_attn_Consta_12 = this.x_h_2_attn_Consta_12;
            onnxParams.Learnables.x_h_2_attn_Consta_13 = this.x_h_2_attn_Consta_13;
            onnxParams.Learnables.x_h_2_attn_Consta_2 = this.x_h_2_attn_Consta_2;
            onnxParams.Learnables.x_h_2_ln_1_Constant_ = this.x_h_2_ln_1_Constant_;
            onnxParams.Learnables.x_h_2_ln_2_Constant_ = this.x_h_2_ln_2_Constant_;
            onnxParams.Learnables.x_h_3_attn_Consta_1 = this.x_h_3_attn_Consta_1;
            onnxParams.Learnables.x_h_3_attn_Consta_12 = this.x_h_3_attn_Consta_12;
            onnxParams.Learnables.x_h_3_attn_Consta_13 = this.x_h_3_attn_Consta_13;
            onnxParams.Learnables.x_h_3_attn_Consta_2 = this.x_h_3_attn_Consta_2;
            onnxParams.Learnables.x_h_3_ln_1_Constant_ = this.x_h_3_ln_1_Constant_;
            onnxParams.Learnables.x_h_3_ln_2_Constant_ = this.x_h_3_ln_2_Constant_;
            onnxParams.Learnables.x_h_4_attn_Consta_1 = this.x_h_4_attn_Consta_1;
            onnxParams.Learnables.x_h_4_attn_Consta_12 = this.x_h_4_attn_Consta_12;
            onnxParams.Learnables.x_h_4_attn_Consta_13 = this.x_h_4_attn_Consta_13;
            onnxParams.Learnables.x_h_4_attn_Consta_2 = this.x_h_4_attn_Consta_2;
            onnxParams.Learnables.x_h_4_ln_1_Constant_ = this.x_h_4_ln_1_Constant_;
            onnxParams.Learnables.x_h_4_ln_2_Constant_ = this.x_h_4_ln_2_Constant_;
            onnxParams.Learnables.x_h_5_attn_Consta_1 = this.x_h_5_attn_Consta_1;
            onnxParams.Learnables.x_h_5_attn_Consta_12 = this.x_h_5_attn_Consta_12;
            onnxParams.Learnables.x_h_5_attn_Consta_13 = this.x_h_5_attn_Consta_13;
            onnxParams.Learnables.x_h_5_attn_Consta_2 = this.x_h_5_attn_Consta_2;
            onnxParams.Learnables.x_h_5_ln_1_Constant_ = this.x_h_5_ln_1_Constant_;
            onnxParams.Learnables.x_h_5_ln_2_Constant_ = this.x_h_5_ln_2_Constant_;
            onnxParams.Learnables.x_h_6_attn_Consta_1 = this.x_h_6_attn_Consta_1;
            onnxParams.Learnables.x_h_6_attn_Consta_12 = this.x_h_6_attn_Consta_12;
            onnxParams.Learnables.x_h_6_attn_Consta_13 = this.x_h_6_attn_Consta_13;
            onnxParams.Learnables.x_h_6_attn_Consta_2 = this.x_h_6_attn_Consta_2;
            onnxParams.Learnables.x_h_6_ln_1_Constant_ = this.x_h_6_ln_1_Constant_;
            onnxParams.Learnables.x_h_6_ln_2_Constant_ = this.x_h_6_ln_2_Constant_;
            onnxParams.Learnables.x_h_7_attn_Consta_1 = this.x_h_7_attn_Consta_1;
            onnxParams.Learnables.x_h_7_attn_Consta_12 = this.x_h_7_attn_Consta_12;
            onnxParams.Learnables.x_h_7_attn_Consta_13 = this.x_h_7_attn_Consta_13;
            onnxParams.Learnables.x_h_7_attn_Consta_2 = this.x_h_7_attn_Consta_2;
            onnxParams.Learnables.x_h_7_ln_1_Constant_ = this.x_h_7_ln_1_Constant_;
            onnxParams.Learnables.x_h_7_ln_2_Constant_ = this.x_h_7_ln_2_Constant_;
            onnxParams.Learnables.x_ln_f_Constant_outp = this.x_ln_f_Constant_outp;
            [output, outputNumDims] = Gather_To_AddFcn(input, inputNumDims, onnxParams, 'Training', false, ...
                'InputDataPermutation', {['as-is'], ['as-is']}, ...
                'OutputDataPermutation', {['as-is'], ['as-is']});
            if any(cellfun(@(A)~isnumeric(A) && ~islogical(A), {output}))
                fprintf('Runtime error in network. At least one output of custom layer ''%s'' is a non-numeric, non-logical value.\n', 'Gather_To_AddLayer1043');
                error(message('nnet_cnn_onnx:onnx:BadCustomLayerRuntimeOutput', 'Gather_To_AddLayer1043'));
            end
            output = dlarray(single(output), repmat('U', 1, max(2, outputNumDims)));
            if ~coder.target('MATLAB')
                output = extractdata(output);
            end
        end

        function [output] = forward(this, input, inputNumDims)
            if isdlarray(input)
                input = stripdims(input);
            end
            inputNumDims = numel(inputNumDims);
            onnxParams = this.ONNXParams;
            onnxParams.Learnables.onnx__MatMul_1000 = this.onnx__MatMul_1000;
            onnxParams.Learnables.onnx__MatMul_1020 = this.onnx__MatMul_1020;
            onnxParams.Learnables.onnx__MatMul_1021 = this.onnx__MatMul_1021;
            onnxParams.Learnables.onnx__MatMul_1022 = this.onnx__MatMul_1022;
            onnxParams.Learnables.onnx__MatMul_1023 = this.onnx__MatMul_1023;
            onnxParams.Learnables.onnx__MatMul_1043 = this.onnx__MatMul_1043;
            onnxParams.Learnables.onnx__MatMul_1044 = this.onnx__MatMul_1044;
            onnxParams.Learnables.onnx__MatMul_1045 = this.onnx__MatMul_1045;
            onnxParams.Learnables.onnx__MatMul_1046 = this.onnx__MatMul_1046;
            onnxParams.Learnables.onnx__MatMul_1066 = this.onnx__MatMul_1066;
            onnxParams.Learnables.onnx__MatMul_1067 = this.onnx__MatMul_1067;
            onnxParams.Learnables.onnx__MatMul_1068 = this.onnx__MatMul_1068;
            onnxParams.Learnables.onnx__MatMul_1069 = this.onnx__MatMul_1069;
            onnxParams.Learnables.onnx__MatMul_1089 = this.onnx__MatMul_1089;
            onnxParams.Learnables.onnx__MatMul_1090 = this.onnx__MatMul_1090;
            onnxParams.Learnables.onnx__MatMul_1091 = this.onnx__MatMul_1091;
            onnxParams.Learnables.onnx__MatMul_1092 = this.onnx__MatMul_1092;
            onnxParams.Learnables.onnx__MatMul_907 = this.onnx__MatMul_907;
            onnxParams.Learnables.onnx__MatMul_908 = this.onnx__MatMul_908;
            onnxParams.Learnables.onnx__MatMul_928 = this.onnx__MatMul_928;
            onnxParams.Learnables.onnx__MatMul_929 = this.onnx__MatMul_929;
            onnxParams.Learnables.onnx__MatMul_930 = this.onnx__MatMul_930;
            onnxParams.Learnables.onnx__MatMul_931 = this.onnx__MatMul_931;
            onnxParams.Learnables.onnx__MatMul_951 = this.onnx__MatMul_951;
            onnxParams.Learnables.onnx__MatMul_952 = this.onnx__MatMul_952;
            onnxParams.Learnables.onnx__MatMul_953 = this.onnx__MatMul_953;
            onnxParams.Learnables.onnx__MatMul_954 = this.onnx__MatMul_954;
            onnxParams.Learnables.onnx__MatMul_974 = this.onnx__MatMul_974;
            onnxParams.Learnables.onnx__MatMul_975 = this.onnx__MatMul_975;
            onnxParams.Learnables.onnx__MatMul_976 = this.onnx__MatMul_976;
            onnxParams.Learnables.onnx__MatMul_977 = this.onnx__MatMul_977;
            onnxParams.Learnables.onnx__MatMul_997 = this.onnx__MatMul_997;
            onnxParams.Learnables.onnx__MatMul_998 = this.onnx__MatMul_998;
            onnxParams.Learnables.onnx__MatMul_999 = this.onnx__MatMul_999;
            onnxParams.Learnables.transformer_h_0_ln_1 = this.transformer_h_0_ln_1;
            onnxParams.Learnables.transformer_h_0_ln_2 = this.transformer_h_0_ln_2;
            onnxParams.Learnables.transformer_h_1_ln_1 = this.transformer_h_1_ln_1;
            onnxParams.Learnables.transformer_h_1_ln_2 = this.transformer_h_1_ln_2;
            onnxParams.Learnables.transformer_h_2_ln_1 = this.transformer_h_2_ln_1;
            onnxParams.Learnables.transformer_h_2_ln_2 = this.transformer_h_2_ln_2;
            onnxParams.Learnables.transformer_h_3_ln_1 = this.transformer_h_3_ln_1;
            onnxParams.Learnables.transformer_h_3_ln_2 = this.transformer_h_3_ln_2;
            onnxParams.Learnables.transformer_h_4_ln_1 = this.transformer_h_4_ln_1;
            onnxParams.Learnables.transformer_h_4_ln_2 = this.transformer_h_4_ln_2;
            onnxParams.Learnables.transformer_h_5_ln_1 = this.transformer_h_5_ln_1;
            onnxParams.Learnables.transformer_h_5_ln_2 = this.transformer_h_5_ln_2;
            onnxParams.Learnables.transformer_h_6_ln_1 = this.transformer_h_6_ln_1;
            onnxParams.Learnables.transformer_h_6_ln_2 = this.transformer_h_6_ln_2;
            onnxParams.Learnables.transformer_h_7_ln_1 = this.transformer_h_7_ln_1;
            onnxParams.Learnables.transformer_h_7_ln_2 = this.transformer_h_7_ln_2;
            onnxParams.Learnables.transformer_ln_f_wei = this.transformer_ln_f_wei;
            onnxParams.Learnables.transformer_wpe_weig = this.transformer_wpe_weig;
            onnxParams.Learnables.x_h_0_attn_Consta_1 = this.x_h_0_attn_Consta_1;
            onnxParams.Learnables.x_h_0_attn_Consta_12 = this.x_h_0_attn_Consta_12;
            onnxParams.Learnables.x_h_0_attn_Consta_13 = this.x_h_0_attn_Consta_13;
            onnxParams.Learnables.x_h_0_attn_Consta_2 = this.x_h_0_attn_Consta_2;
            onnxParams.Learnables.x_h_0_ln_1_Constant_ = this.x_h_0_ln_1_Constant_;
            onnxParams.Learnables.x_h_0_ln_2_Constant_ = this.x_h_0_ln_2_Constant_;
            onnxParams.Learnables.x_h_1_attn_Consta_1 = this.x_h_1_attn_Consta_1;
            onnxParams.Learnables.x_h_1_attn_Consta_12 = this.x_h_1_attn_Consta_12;
            onnxParams.Learnables.x_h_1_attn_Consta_13 = this.x_h_1_attn_Consta_13;
            onnxParams.Learnables.x_h_1_attn_Consta_2 = this.x_h_1_attn_Consta_2;
            onnxParams.Learnables.x_h_1_ln_1_Constant_ = this.x_h_1_ln_1_Constant_;
            onnxParams.Learnables.x_h_1_ln_2_Constant_ = this.x_h_1_ln_2_Constant_;
            onnxParams.Learnables.x_h_2_attn_Consta_1 = this.x_h_2_attn_Consta_1;
            onnxParams.Learnables.x_h_2_attn_Consta_12 = this.x_h_2_attn_Consta_12;
            onnxParams.Learnables.x_h_2_attn_Consta_13 = this.x_h_2_attn_Consta_13;
            onnxParams.Learnables.x_h_2_attn_Consta_2 = this.x_h_2_attn_Consta_2;
            onnxParams.Learnables.x_h_2_ln_1_Constant_ = this.x_h_2_ln_1_Constant_;
            onnxParams.Learnables.x_h_2_ln_2_Constant_ = this.x_h_2_ln_2_Constant_;
            onnxParams.Learnables.x_h_3_attn_Consta_1 = this.x_h_3_attn_Consta_1;
            onnxParams.Learnables.x_h_3_attn_Consta_12 = this.x_h_3_attn_Consta_12;
            onnxParams.Learnables.x_h_3_attn_Consta_13 = this.x_h_3_attn_Consta_13;
            onnxParams.Learnables.x_h_3_attn_Consta_2 = this.x_h_3_attn_Consta_2;
            onnxParams.Learnables.x_h_3_ln_1_Constant_ = this.x_h_3_ln_1_Constant_;
            onnxParams.Learnables.x_h_3_ln_2_Constant_ = this.x_h_3_ln_2_Constant_;
            onnxParams.Learnables.x_h_4_attn_Consta_1 = this.x_h_4_attn_Consta_1;
            onnxParams.Learnables.x_h_4_attn_Consta_12 = this.x_h_4_attn_Consta_12;
            onnxParams.Learnables.x_h_4_attn_Consta_13 = this.x_h_4_attn_Consta_13;
            onnxParams.Learnables.x_h_4_attn_Consta_2 = this.x_h_4_attn_Consta_2;
            onnxParams.Learnables.x_h_4_ln_1_Constant_ = this.x_h_4_ln_1_Constant_;
            onnxParams.Learnables.x_h_4_ln_2_Constant_ = this.x_h_4_ln_2_Constant_;
            onnxParams.Learnables.x_h_5_attn_Consta_1 = this.x_h_5_attn_Consta_1;
            onnxParams.Learnables.x_h_5_attn_Consta_12 = this.x_h_5_attn_Consta_12;
            onnxParams.Learnables.x_h_5_attn_Consta_13 = this.x_h_5_attn_Consta_13;
            onnxParams.Learnables.x_h_5_attn_Consta_2 = this.x_h_5_attn_Consta_2;
            onnxParams.Learnables.x_h_5_ln_1_Constant_ = this.x_h_5_ln_1_Constant_;
            onnxParams.Learnables.x_h_5_ln_2_Constant_ = this.x_h_5_ln_2_Constant_;
            onnxParams.Learnables.x_h_6_attn_Consta_1 = this.x_h_6_attn_Consta_1;
            onnxParams.Learnables.x_h_6_attn_Consta_12 = this.x_h_6_attn_Consta_12;
            onnxParams.Learnables.x_h_6_attn_Consta_13 = this.x_h_6_attn_Consta_13;
            onnxParams.Learnables.x_h_6_attn_Consta_2 = this.x_h_6_attn_Consta_2;
            onnxParams.Learnables.x_h_6_ln_1_Constant_ = this.x_h_6_ln_1_Constant_;
            onnxParams.Learnables.x_h_6_ln_2_Constant_ = this.x_h_6_ln_2_Constant_;
            onnxParams.Learnables.x_h_7_attn_Consta_1 = this.x_h_7_attn_Consta_1;
            onnxParams.Learnables.x_h_7_attn_Consta_12 = this.x_h_7_attn_Consta_12;
            onnxParams.Learnables.x_h_7_attn_Consta_13 = this.x_h_7_attn_Consta_13;
            onnxParams.Learnables.x_h_7_attn_Consta_2 = this.x_h_7_attn_Consta_2;
            onnxParams.Learnables.x_h_7_ln_1_Constant_ = this.x_h_7_ln_1_Constant_;
            onnxParams.Learnables.x_h_7_ln_2_Constant_ = this.x_h_7_ln_2_Constant_;
            onnxParams.Learnables.x_ln_f_Constant_outp = this.x_ln_f_Constant_outp;
            [output, outputNumDims] = Gather_To_AddFcn(input, inputNumDims, onnxParams, 'Training', true, ...
                'InputDataPermutation', {['as-is'], ['as-is']}, ...
                'OutputDataPermutation', {['as-is'], ['as-is']});
            if any(cellfun(@(A)~isnumeric(A) && ~islogical(A), {output}))
                fprintf('Runtime error in network. At least one output of custom layer ''%s'' is a non-numeric, non-logical value.\n', 'Gather_To_AddLayer1043');
                error(message('nnet_cnn_onnx:onnx:BadCustomLayerRuntimeOutput', 'Gather_To_AddLayer1043'));
            end
            output = dlarray(single(output), repmat('U', 1, max(2, outputNumDims)));
            if ~coder.target('MATLAB')
                output = extractdata(output);
            end
        end
    end
end

function [output, outputNumDims, state] = Gather_To_AddFcn(input, inputNumDims, params, varargin)
%GATHER_TO_ADDFCN Function implementing an imported ONNX network.
%
% THIS FILE WAS AUTO-GENERATED BY importONNXFunction.
% ONNX Operator Set Version: 17
%
% Variable names in this function are taken from the original ONNX file.
%
% [OUTPUT] = Gather_To_AddFcn(INPUT, PARAMS)
%			- Evaluates the imported ONNX network GATHER_TO_ADDFCN with input(s)
%			INPUT and the imported network parameters in PARAMS. Returns
%			network output(s) in OUTPUT.
%
% [OUTPUT, STATE] = Gather_To_AddFcn(INPUT, PARAMS)
%			- Additionally returns state variables in STATE. When training,
%			use this form and set TRAINING to true.
%
% [__] = Gather_To_AddFcn(INPUT, PARAMS, 'NAME1', VAL1, 'NAME2', VAL2, ...)
%			- Specifies additional name-value pairs described below:
%
% 'Training'
% 			Boolean indicating whether the network is being evaluated for
%			prediction or training. If TRAINING is true, state variables
%			will be updated.
%
% 'InputDataPermutation'
%			'auto' - Automatically attempt to determine the permutation
%			 between the dimensions of the input data and the dimensions of
%			the ONNX model input. For example, the permutation from HWCN
%			(MATLAB standard) to NCHW (ONNX standard) uses the vector
%			[4 3 1 2]. See the documentation for IMPORTONNXFUNCTION for
%			more information about automatic permutation.
%
%			'none' - Input(s) are passed in the ONNX model format. See 'Inputs'.
%
%			numeric vector - The permutation vector describing the
%			transformation between input data dimensions and the expected
%			ONNX input dimensions.%
%			cell array - If the network has multiple inputs, each cell
%			contains 'auto', 'none', or a numeric vector.
%
% 'OutputDataPermutation'
%			'auto' - Automatically attempt to determine the permutation
%			between the dimensions of the output and a conventional MATLAB
%			dimension ordering. For example, the permutation from NC (ONNX
%			standard) to CN (MATLAB standard) uses the vector [2 1]. See
%			the documentation for IMPORTONNXFUNCTION for more information
%			about automatic permutation.
%
%			'none' - Return output(s) as given by the ONNX model. See 'Outputs'.
%
%			numeric vector - The permutation vector describing the
%			transformation between the ONNX output dimensions and the
%			desired output dimensions.%
%			cell array - If the network has multiple outputs, each cell
%			contains 'auto', 'none' or a numeric vector.
%
% Inputs:
% -------
% INPUT
%			- Input(s) to the ONNX network.
%			  The input size(s) expected by the ONNX file are:
%				  INPUT:		[1, 10, 8]				Type: FLOAT
%			  By default, the function will try to permute the input(s)
%			  into this dimension ordering. If the default is incorrect,
%			  use the 'InputDataPermutation' argument to control the
%			  permutation.
%
%
% PARAMS	- Network parameters returned by 'importONNXFunction'.
%
%
% Outputs:
% --------
% OUTPUT
%			- Output(s) of the ONNX network.
%			  Without permutation, the size(s) of the outputs are:
%				  OUTPUT:		[1, 10, 1]				Type: FLOAT
%			  By default, the function will try to permute the output(s)
%			  from this dimension ordering into a conventional MATLAB
%			  ordering. If the default is incorrect, use the
%			  'OutputDataPermutation' argument to control the permutation.
%
% STATE		- (Optional) State variables. When TRAINING is true, these will
% 			  have been updated from the original values in PARAMS.State.
%
%
%  See also importONNXFunction

% Preprocess the input data and arguments:
[input, Training, outputDataPerms, anyDlarrayInputs] = preprocessInput(input, params, varargin{:});
% Put all variables into a single struct to implement dynamic scoping:
[Vars, NumDims] = packageVariables(params, {'input'}, {input}, [inputNumDims]);
% Call the top-level graph function:
[output, outputNumDims, state] = Gather_To_AddGraph1000(input, NumDims.input, Vars, NumDims, Training, params.State);
% Postprocess the output data
[output] = postprocessOutput(output, outputDataPerms, anyDlarrayInputs, Training, varargin{:});
end

function [output, outputNumDims1042, state] = Gather_To_AddGraph1000(input, inputNumDims1041, Vars, NumDims, Training, state)
% Function implementing the graph 'Gather_To_AddGraph1000'
% Update Vars and NumDims from the graph's formal input parameters. Note that state variables are already in Vars.
Vars.input = input;
NumDims.input = inputNumDims1041;

% Execute the operators:
% MatMul:
[Vars.x_wte_MatMul_output_, NumDims.x_wte_MatMul_output_] = onnxMatMul(Vars.input, Vars.onnx__MatMul_907, NumDims.input, NumDims.onnx__MatMul_907);

% Add:
Vars.x_wte_Add_output_0 = Vars.transformer_wte_bias + Vars.x_wte_MatMul_output_;
NumDims.x_wte_Add_output_0 = max(NumDims.transformer_wte_bias, NumDims.x_wte_MatMul_output_);

% Gather:
[Vars.x_wpe_Gather_output_, NumDims.x_wpe_Gather_output_] = onnxGather(Vars.transformer_wpe_weig, Vars.x_Constant_output_0, 0, NumDims.transformer_wpe_weig, NumDims.x_Constant_output_0);

% Add:
Vars.x_Add_output_0 = Vars.x_wte_Add_output_0 + Vars.x_wpe_Gather_output_;
NumDims.x_Add_output_0 = max(NumDims.x_wte_Add_output_0, NumDims.x_wpe_Gather_output_);

% LayerNormalization:
[Vars.x_h_0_ln_1_LayerNorm, NumDims.x_h_0_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_Add_output_0, Vars.transformer_h_0_ln_1, Vars.x_h_0_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_Add_output_0);

% MatMul:
[Vars.x_h_0_attn_c_attn_Ma, NumDims.x_h_0_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_0_ln_1_LayerNorm, Vars.onnx__MatMul_908, NumDims.x_h_0_ln_1_LayerNorm, NumDims.onnx__MatMul_908);

% Split:
[Vars.x_h_0_attn_Split_out, Vars.x_h_0_attn_Split_o_1, Vars.x_h_0_attn_Split_o_2, NumDims.x_h_0_attn_Split_out, NumDims.x_h_0_attn_Split_o_1, NumDims.x_h_0_attn_Split_o_2] = onnxSplit13(Vars.x_h_0_attn_c_attn_Ma, 2, Vars.x_h_0_attn_Consta_13, 3, NumDims.x_h_0_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_0_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_0_attn_Split_o_1, Vars.x_h_0_attn_Consta_4, NumDims.x_h_0_attn_Split_o_1, 0);
Vars.x_h_0_attn_Reshape_o = reshape(Vars.x_h_0_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_0_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1001, NumDims.x_h_0_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_0_attn_Transpo_4 = permute(Vars.x_h_0_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_0_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_0_attn_Split_out, Vars.x_h_0_attn_Consta_5, NumDims.x_h_0_attn_Split_out, 0);
Vars.x_h_0_attn_Reshape_1 = reshape(Vars.x_h_0_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_0_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1002, NumDims.x_h_0_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_0_attn_Transpose = permute(Vars.x_h_0_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_0_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_0_attn_Split_o_2, Vars.x_h_0_attn_Consta_6, NumDims.x_h_0_attn_Split_o_2, 0);
Vars.x_h_0_attn_Reshape_2 = reshape(Vars.x_h_0_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_0_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1003, NumDims.x_h_0_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_0_attn_Transpo_1 = permute(Vars.x_h_0_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_0_attn_Shape_out, NumDims.x_h_0_attn_Shape_out] = onnxShape(Vars.x_h_0_attn_Transpose, NumDims.x_h_0_attn_Transpose, 0, NumDims.x_h_0_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_0_attn_Slice_out] = prepareSliceArgs(Vars.x_h_0_attn_Shape_out, Vars.x_h_0_attn_Consta_7, Vars.x_h_0_attn_Consta_8, '', '', NumDims.x_h_0_attn_Shape_out);
Vars.x_h_0_attn_Slice_out = subsref(Vars.x_h_0_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_0_attn_Slice_out)
    Vars.x_h_0_attn_Slice_out = single(Vars.x_h_0_attn_Slice_out);
end
Vars.x_h_0_attn_Cast_outp = single(Vars.x_h_0_attn_Slice_out);
NumDims.x_h_0_attn_Cast_outp = NumDims.x_h_0_attn_Slice_out;

% Sqrt:
Vars.x_h_0_attn_Sqrt_outp = sqrt(Vars.x_h_0_attn_Cast_outp);
NumDims.x_h_0_attn_Sqrt_outp = NumDims.x_h_0_attn_Cast_outp;

% Div:
Vars.x_h_0_attn_Div_outpu = Vars.x_h_0_attn_Consta_9 ./ Vars.x_h_0_attn_Sqrt_outp;
NumDims.x_h_0_attn_Div_outpu = max(NumDims.x_h_0_attn_Consta_9, NumDims.x_h_0_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_0_attn_Div_outpu)
    Vars.x_h_0_attn_Div_outpu = single(Vars.x_h_0_attn_Div_outpu);
end
Vars.x_h_0_attn_Cast_1_ou = single(Vars.x_h_0_attn_Div_outpu);
NumDims.x_h_0_attn_Cast_1_ou = NumDims.x_h_0_attn_Div_outpu;

% Shape:
[Vars.x_h_0_attn_Shape_1_o, NumDims.x_h_0_attn_Shape_1_o] = onnxShape(Vars.x_h_0_attn_Transpose, NumDims.x_h_0_attn_Transpose, 0, NumDims.x_h_0_attn_Transpose+1);

% Shape:
[Vars.x_h_0_attn_Shape_2_o, NumDims.x_h_0_attn_Shape_2_o] = onnxShape(Vars.x_h_0_attn_Transpo_4, NumDims.x_h_0_attn_Transpo_4, 0, NumDims.x_h_0_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_0_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_0_attn_Shape_1_o, Vars.x_h_0_attn_Consta_11, Vars.x_h_0_attn_Consta_10, '', '', NumDims.x_h_0_attn_Shape_1_o);
Vars.x_h_0_attn_Slice_1_o = subsref(Vars.x_h_0_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_0_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_0_attn_Shape_2_o, Vars.x_h_0_attn_Consta_11, Vars.x_h_0_attn_Consta_10, '', '', NumDims.x_h_0_attn_Shape_2_o);
Vars.x_h_0_attn_Slice_2_o = subsref(Vars.x_h_0_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_0_attn_Concat_ou, NumDims.x_h_0_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_0_attn_Slice_1_o, Vars.x_h_0_attn_Slice_2_o}, [NumDims.x_h_0_attn_Slice_1_o, NumDims.x_h_0_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_0_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_0_attn_Concat_ou);
Vars.x_h_0_attn_Expand_ou = Vars.x_h_0_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_0_attn_Trilu_out, NumDims.x_h_0_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_0_attn_Expand_ou);

% Equal:
Vars.x_h_0_attn_Equal_out = Vars.x_h_0_attn_Trilu_out == Vars.x_h_0_attn_Constant_;
NumDims.x_h_0_attn_Equal_out = max(NumDims.x_h_0_attn_Trilu_out, NumDims.x_h_0_attn_Constant_);

% Where:
[Vars.x_h_0_attn_Where_out, NumDims.x_h_0_attn_Where_out] = onnxWhere(Vars.x_h_0_attn_Equal_out, Vars.x_h_0_attn_Consta_1, Vars.x_h_0_attn_Consta_2, NumDims.x_h_0_attn_Equal_out, NumDims.x_h_0_attn_Consta_1, NumDims.x_h_0_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_0_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1004, NumDims.x_h_0_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_0_attn_Transpo_2 = permute(Vars.x_h_0_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_0_attn_Sqrt_1_ou = sqrt(Vars.x_h_0_attn_Cast_1_ou);
NumDims.x_h_0_attn_Sqrt_1_ou = NumDims.x_h_0_attn_Cast_1_ou;

% Mul:
Vars.x_h_0_attn_Mul_outpu = Vars.x_h_0_attn_Transpose .* Vars.x_h_0_attn_Sqrt_1_ou;
NumDims.x_h_0_attn_Mul_outpu = max(NumDims.x_h_0_attn_Transpose, NumDims.x_h_0_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_0_attn_Sqrt_2_ou = sqrt(Vars.x_h_0_attn_Cast_1_ou);
NumDims.x_h_0_attn_Sqrt_2_ou = NumDims.x_h_0_attn_Cast_1_ou;

% Mul:
Vars.x_h_0_attn_Mul_1_out = Vars.x_h_0_attn_Transpo_2 .* Vars.x_h_0_attn_Sqrt_2_ou;
NumDims.x_h_0_attn_Mul_1_out = max(NumDims.x_h_0_attn_Transpo_2, NumDims.x_h_0_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_0_attn_MatMul_ou, NumDims.x_h_0_attn_MatMul_ou] = onnxMatMul(Vars.x_h_0_attn_Mul_outpu, Vars.x_h_0_attn_Mul_1_out, NumDims.x_h_0_attn_Mul_outpu, NumDims.x_h_0_attn_Mul_1_out);

% Add:
Vars.x_h_0_attn_Add_outpu = Vars.x_h_0_attn_MatMul_ou + Vars.x_h_0_attn_Where_out;
NumDims.x_h_0_attn_Add_outpu = max(NumDims.x_h_0_attn_MatMul_ou, NumDims.x_h_0_attn_Where_out);

% Softmax:
[Vars.x_h_0_attn_Softmax_o, NumDims.x_h_0_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_0_attn_Add_outpu, -1, NumDims.x_h_0_attn_Add_outpu);

% MatMul:
[Vars.x_h_0_attn_MatMul_1_, NumDims.x_h_0_attn_MatMul_1_] = onnxMatMul(Vars.x_h_0_attn_Softmax_o, Vars.x_h_0_attn_Transpo_1, NumDims.x_h_0_attn_Softmax_o, NumDims.x_h_0_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_0_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1005, NumDims.x_h_0_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_0_attn_Transpo_3 = permute(Vars.x_h_0_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_0_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_0_attn_Transpo_3, Vars.x_h_0_attn_Consta_3, NumDims.x_h_0_attn_Transpo_3, 0);
Vars.x_h_0_attn_Reshape_3 = reshape(Vars.x_h_0_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_0_attn_c_proj_Ma, NumDims.x_h_0_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_0_attn_Reshape_3, Vars.onnx__MatMul_928, NumDims.x_h_0_attn_Reshape_3, NumDims.onnx__MatMul_928);

% Add:
Vars.x_h_0_Add_output_0 = Vars.x_Add_output_0 + Vars.x_h_0_attn_c_proj_Ma;
NumDims.x_h_0_Add_output_0 = max(NumDims.x_Add_output_0, NumDims.x_h_0_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_0_ln_2_LayerNorm, NumDims.x_h_0_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_0_Add_output_0, Vars.transformer_h_0_ln_2, Vars.x_h_0_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_0_Add_output_0);

% MatMul:
[Vars.x_h_0_mlp_c_fc_MatMu, NumDims.x_h_0_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_0_ln_2_LayerNorm, Vars.onnx__MatMul_929, NumDims.x_h_0_ln_2_LayerNorm, NumDims.onnx__MatMul_929);

% Div:
Vars.x_h_0_mlp_gelu_Div_o = Vars.x_h_0_mlp_c_fc_MatMu ./ Vars.x_h_0_mlp_gelu_Con_2;
NumDims.x_h_0_mlp_gelu_Div_o = max(NumDims.x_h_0_mlp_c_fc_MatMu, NumDims.x_h_0_mlp_gelu_Con_2);

% Erf:
Vars.x_h_0_mlp_gelu_Erf_o = erf(Vars.x_h_0_mlp_gelu_Div_o);
NumDims.x_h_0_mlp_gelu_Erf_o = NumDims.x_h_0_mlp_gelu_Div_o;

% Add:
Vars.x_h_0_mlp_gelu_Add_o = Vars.x_h_0_mlp_gelu_Erf_o + Vars.x_h_0_mlp_gelu_Const;
NumDims.x_h_0_mlp_gelu_Add_o = max(NumDims.x_h_0_mlp_gelu_Erf_o, NumDims.x_h_0_mlp_gelu_Const);

% Mul:
Vars.x_h_0_mlp_gelu_Mul_o = Vars.x_h_0_mlp_c_fc_MatMu .* Vars.x_h_0_mlp_gelu_Add_o;
NumDims.x_h_0_mlp_gelu_Mul_o = max(NumDims.x_h_0_mlp_c_fc_MatMu, NumDims.x_h_0_mlp_gelu_Add_o);

% Mul:
Vars.x_h_0_mlp_gelu_Mul_1 = Vars.x_h_0_mlp_gelu_Mul_o .* Vars.x_h_0_mlp_gelu_Con_1;
NumDims.x_h_0_mlp_gelu_Mul_1 = max(NumDims.x_h_0_mlp_gelu_Mul_o, NumDims.x_h_0_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_0_mlp_c_proj_Mat, NumDims.x_h_0_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_0_mlp_gelu_Mul_1, Vars.onnx__MatMul_930, NumDims.x_h_0_mlp_gelu_Mul_1, NumDims.onnx__MatMul_930);

% Add:
Vars.x_h_0_Add_1_output_0 = Vars.x_h_0_Add_output_0 + Vars.x_h_0_mlp_c_proj_Mat;
NumDims.x_h_0_Add_1_output_0 = max(NumDims.x_h_0_Add_output_0, NumDims.x_h_0_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_h_1_ln_1_LayerNorm, NumDims.x_h_1_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_h_0_Add_1_output_0, Vars.transformer_h_1_ln_1, Vars.x_h_1_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_h_0_Add_1_output_0);

% MatMul:
[Vars.x_h_1_attn_c_attn_Ma, NumDims.x_h_1_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_1_ln_1_LayerNorm, Vars.onnx__MatMul_931, NumDims.x_h_1_ln_1_LayerNorm, NumDims.onnx__MatMul_931);

% Split:
[Vars.x_h_1_attn_Split_out, Vars.x_h_1_attn_Split_o_1, Vars.x_h_1_attn_Split_o_2, NumDims.x_h_1_attn_Split_out, NumDims.x_h_1_attn_Split_o_1, NumDims.x_h_1_attn_Split_o_2] = onnxSplit13(Vars.x_h_1_attn_c_attn_Ma, 2, Vars.x_h_1_attn_Consta_13, 3, NumDims.x_h_1_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_1_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_1_attn_Split_o_1, Vars.x_h_1_attn_Consta_4, NumDims.x_h_1_attn_Split_o_1, 0);
Vars.x_h_1_attn_Reshape_o = reshape(Vars.x_h_1_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_1_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1006, NumDims.x_h_1_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_1_attn_Transpo_4 = permute(Vars.x_h_1_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_1_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_1_attn_Split_out, Vars.x_h_1_attn_Consta_5, NumDims.x_h_1_attn_Split_out, 0);
Vars.x_h_1_attn_Reshape_1 = reshape(Vars.x_h_1_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_1_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1007, NumDims.x_h_1_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_1_attn_Transpose = permute(Vars.x_h_1_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_1_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_1_attn_Split_o_2, Vars.x_h_1_attn_Consta_6, NumDims.x_h_1_attn_Split_o_2, 0);
Vars.x_h_1_attn_Reshape_2 = reshape(Vars.x_h_1_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_1_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1008, NumDims.x_h_1_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_1_attn_Transpo_1 = permute(Vars.x_h_1_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_1_attn_Shape_out, NumDims.x_h_1_attn_Shape_out] = onnxShape(Vars.x_h_1_attn_Transpose, NumDims.x_h_1_attn_Transpose, 0, NumDims.x_h_1_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_1_attn_Slice_out] = prepareSliceArgs(Vars.x_h_1_attn_Shape_out, Vars.x_h_1_attn_Consta_7, Vars.x_h_1_attn_Consta_8, '', '', NumDims.x_h_1_attn_Shape_out);
Vars.x_h_1_attn_Slice_out = subsref(Vars.x_h_1_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_1_attn_Slice_out)
    Vars.x_h_1_attn_Slice_out = single(Vars.x_h_1_attn_Slice_out);
end
Vars.x_h_1_attn_Cast_outp = single(Vars.x_h_1_attn_Slice_out);
NumDims.x_h_1_attn_Cast_outp = NumDims.x_h_1_attn_Slice_out;

% Sqrt:
Vars.x_h_1_attn_Sqrt_outp = sqrt(Vars.x_h_1_attn_Cast_outp);
NumDims.x_h_1_attn_Sqrt_outp = NumDims.x_h_1_attn_Cast_outp;

% Div:
Vars.x_h_1_attn_Div_outpu = Vars.x_h_1_attn_Consta_9 ./ Vars.x_h_1_attn_Sqrt_outp;
NumDims.x_h_1_attn_Div_outpu = max(NumDims.x_h_1_attn_Consta_9, NumDims.x_h_1_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_1_attn_Div_outpu)
    Vars.x_h_1_attn_Div_outpu = single(Vars.x_h_1_attn_Div_outpu);
end
Vars.x_h_1_attn_Cast_1_ou = single(Vars.x_h_1_attn_Div_outpu);
NumDims.x_h_1_attn_Cast_1_ou = NumDims.x_h_1_attn_Div_outpu;

% Shape:
[Vars.x_h_1_attn_Shape_1_o, NumDims.x_h_1_attn_Shape_1_o] = onnxShape(Vars.x_h_1_attn_Transpose, NumDims.x_h_1_attn_Transpose, 0, NumDims.x_h_1_attn_Transpose+1);

% Shape:
[Vars.x_h_1_attn_Shape_2_o, NumDims.x_h_1_attn_Shape_2_o] = onnxShape(Vars.x_h_1_attn_Transpo_4, NumDims.x_h_1_attn_Transpo_4, 0, NumDims.x_h_1_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_1_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_1_attn_Shape_1_o, Vars.x_h_1_attn_Consta_11, Vars.x_h_1_attn_Consta_10, '', '', NumDims.x_h_1_attn_Shape_1_o);
Vars.x_h_1_attn_Slice_1_o = subsref(Vars.x_h_1_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_1_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_1_attn_Shape_2_o, Vars.x_h_1_attn_Consta_11, Vars.x_h_1_attn_Consta_10, '', '', NumDims.x_h_1_attn_Shape_2_o);
Vars.x_h_1_attn_Slice_2_o = subsref(Vars.x_h_1_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_1_attn_Concat_ou, NumDims.x_h_1_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_1_attn_Slice_1_o, Vars.x_h_1_attn_Slice_2_o}, [NumDims.x_h_1_attn_Slice_1_o, NumDims.x_h_1_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_1_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_1_attn_Concat_ou);
Vars.x_h_1_attn_Expand_ou = Vars.x_h_1_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_1_attn_Trilu_out, NumDims.x_h_1_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_1_attn_Expand_ou);

% Equal:
Vars.x_h_1_attn_Equal_out = Vars.x_h_1_attn_Trilu_out == Vars.x_h_1_attn_Constant_;
NumDims.x_h_1_attn_Equal_out = max(NumDims.x_h_1_attn_Trilu_out, NumDims.x_h_1_attn_Constant_);

% Where:
[Vars.x_h_1_attn_Where_out, NumDims.x_h_1_attn_Where_out] = onnxWhere(Vars.x_h_1_attn_Equal_out, Vars.x_h_1_attn_Consta_1, Vars.x_h_1_attn_Consta_2, NumDims.x_h_1_attn_Equal_out, NumDims.x_h_1_attn_Consta_1, NumDims.x_h_1_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_1_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1009, NumDims.x_h_1_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_1_attn_Transpo_2 = permute(Vars.x_h_1_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_1_attn_Sqrt_1_ou = sqrt(Vars.x_h_1_attn_Cast_1_ou);
NumDims.x_h_1_attn_Sqrt_1_ou = NumDims.x_h_1_attn_Cast_1_ou;

% Mul:
Vars.x_h_1_attn_Mul_outpu = Vars.x_h_1_attn_Transpose .* Vars.x_h_1_attn_Sqrt_1_ou;
NumDims.x_h_1_attn_Mul_outpu = max(NumDims.x_h_1_attn_Transpose, NumDims.x_h_1_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_1_attn_Sqrt_2_ou = sqrt(Vars.x_h_1_attn_Cast_1_ou);
NumDims.x_h_1_attn_Sqrt_2_ou = NumDims.x_h_1_attn_Cast_1_ou;

% Mul:
Vars.x_h_1_attn_Mul_1_out = Vars.x_h_1_attn_Transpo_2 .* Vars.x_h_1_attn_Sqrt_2_ou;
NumDims.x_h_1_attn_Mul_1_out = max(NumDims.x_h_1_attn_Transpo_2, NumDims.x_h_1_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_1_attn_MatMul_ou, NumDims.x_h_1_attn_MatMul_ou] = onnxMatMul(Vars.x_h_1_attn_Mul_outpu, Vars.x_h_1_attn_Mul_1_out, NumDims.x_h_1_attn_Mul_outpu, NumDims.x_h_1_attn_Mul_1_out);

% Add:
Vars.x_h_1_attn_Add_outpu = Vars.x_h_1_attn_MatMul_ou + Vars.x_h_1_attn_Where_out;
NumDims.x_h_1_attn_Add_outpu = max(NumDims.x_h_1_attn_MatMul_ou, NumDims.x_h_1_attn_Where_out);

% Softmax:
[Vars.x_h_1_attn_Softmax_o, NumDims.x_h_1_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_1_attn_Add_outpu, -1, NumDims.x_h_1_attn_Add_outpu);

% MatMul:
[Vars.x_h_1_attn_MatMul_1_, NumDims.x_h_1_attn_MatMul_1_] = onnxMatMul(Vars.x_h_1_attn_Softmax_o, Vars.x_h_1_attn_Transpo_1, NumDims.x_h_1_attn_Softmax_o, NumDims.x_h_1_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_1_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1010, NumDims.x_h_1_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_1_attn_Transpo_3 = permute(Vars.x_h_1_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_1_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_1_attn_Transpo_3, Vars.x_h_1_attn_Consta_3, NumDims.x_h_1_attn_Transpo_3, 0);
Vars.x_h_1_attn_Reshape_3 = reshape(Vars.x_h_1_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_1_attn_c_proj_Ma, NumDims.x_h_1_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_1_attn_Reshape_3, Vars.onnx__MatMul_951, NumDims.x_h_1_attn_Reshape_3, NumDims.onnx__MatMul_951);

% Add:
Vars.x_h_1_Add_output_0 = Vars.x_h_0_Add_1_output_0 + Vars.x_h_1_attn_c_proj_Ma;
NumDims.x_h_1_Add_output_0 = max(NumDims.x_h_0_Add_1_output_0, NumDims.x_h_1_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_1_ln_2_LayerNorm, NumDims.x_h_1_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_1_Add_output_0, Vars.transformer_h_1_ln_2, Vars.x_h_1_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_1_Add_output_0);

% MatMul:
[Vars.x_h_1_mlp_c_fc_MatMu, NumDims.x_h_1_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_1_ln_2_LayerNorm, Vars.onnx__MatMul_952, NumDims.x_h_1_ln_2_LayerNorm, NumDims.onnx__MatMul_952);

% Div:
Vars.x_h_1_mlp_gelu_Div_o = Vars.x_h_1_mlp_c_fc_MatMu ./ Vars.x_h_1_mlp_gelu_Con_2;
NumDims.x_h_1_mlp_gelu_Div_o = max(NumDims.x_h_1_mlp_c_fc_MatMu, NumDims.x_h_1_mlp_gelu_Con_2);

% Erf:
Vars.x_h_1_mlp_gelu_Erf_o = erf(Vars.x_h_1_mlp_gelu_Div_o);
NumDims.x_h_1_mlp_gelu_Erf_o = NumDims.x_h_1_mlp_gelu_Div_o;

% Add:
Vars.x_h_1_mlp_gelu_Add_o = Vars.x_h_1_mlp_gelu_Erf_o + Vars.x_h_1_mlp_gelu_Const;
NumDims.x_h_1_mlp_gelu_Add_o = max(NumDims.x_h_1_mlp_gelu_Erf_o, NumDims.x_h_1_mlp_gelu_Const);

% Mul:
Vars.x_h_1_mlp_gelu_Mul_o = Vars.x_h_1_mlp_c_fc_MatMu .* Vars.x_h_1_mlp_gelu_Add_o;
NumDims.x_h_1_mlp_gelu_Mul_o = max(NumDims.x_h_1_mlp_c_fc_MatMu, NumDims.x_h_1_mlp_gelu_Add_o);

% Mul:
Vars.x_h_1_mlp_gelu_Mul_1 = Vars.x_h_1_mlp_gelu_Mul_o .* Vars.x_h_1_mlp_gelu_Con_1;
NumDims.x_h_1_mlp_gelu_Mul_1 = max(NumDims.x_h_1_mlp_gelu_Mul_o, NumDims.x_h_1_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_1_mlp_c_proj_Mat, NumDims.x_h_1_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_1_mlp_gelu_Mul_1, Vars.onnx__MatMul_953, NumDims.x_h_1_mlp_gelu_Mul_1, NumDims.onnx__MatMul_953);

% Add:
Vars.x_h_1_Add_1_output_0 = Vars.x_h_1_Add_output_0 + Vars.x_h_1_mlp_c_proj_Mat;
NumDims.x_h_1_Add_1_output_0 = max(NumDims.x_h_1_Add_output_0, NumDims.x_h_1_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_h_2_ln_1_LayerNorm, NumDims.x_h_2_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_h_1_Add_1_output_0, Vars.transformer_h_2_ln_1, Vars.x_h_2_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_h_1_Add_1_output_0);

% MatMul:
[Vars.x_h_2_attn_c_attn_Ma, NumDims.x_h_2_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_2_ln_1_LayerNorm, Vars.onnx__MatMul_954, NumDims.x_h_2_ln_1_LayerNorm, NumDims.onnx__MatMul_954);

% Split:
[Vars.x_h_2_attn_Split_out, Vars.x_h_2_attn_Split_o_1, Vars.x_h_2_attn_Split_o_2, NumDims.x_h_2_attn_Split_out, NumDims.x_h_2_attn_Split_o_1, NumDims.x_h_2_attn_Split_o_2] = onnxSplit13(Vars.x_h_2_attn_c_attn_Ma, 2, Vars.x_h_2_attn_Consta_13, 3, NumDims.x_h_2_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_2_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_2_attn_Split_o_1, Vars.x_h_2_attn_Consta_4, NumDims.x_h_2_attn_Split_o_1, 0);
Vars.x_h_2_attn_Reshape_o = reshape(Vars.x_h_2_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_2_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1011, NumDims.x_h_2_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_2_attn_Transpo_4 = permute(Vars.x_h_2_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_2_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_2_attn_Split_out, Vars.x_h_2_attn_Consta_5, NumDims.x_h_2_attn_Split_out, 0);
Vars.x_h_2_attn_Reshape_1 = reshape(Vars.x_h_2_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_2_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1012, NumDims.x_h_2_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_2_attn_Transpose = permute(Vars.x_h_2_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_2_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_2_attn_Split_o_2, Vars.x_h_2_attn_Consta_6, NumDims.x_h_2_attn_Split_o_2, 0);
Vars.x_h_2_attn_Reshape_2 = reshape(Vars.x_h_2_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_2_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1013, NumDims.x_h_2_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_2_attn_Transpo_1 = permute(Vars.x_h_2_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_2_attn_Shape_out, NumDims.x_h_2_attn_Shape_out] = onnxShape(Vars.x_h_2_attn_Transpose, NumDims.x_h_2_attn_Transpose, 0, NumDims.x_h_2_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_2_attn_Slice_out] = prepareSliceArgs(Vars.x_h_2_attn_Shape_out, Vars.x_h_2_attn_Consta_7, Vars.x_h_2_attn_Consta_8, '', '', NumDims.x_h_2_attn_Shape_out);
Vars.x_h_2_attn_Slice_out = subsref(Vars.x_h_2_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_2_attn_Slice_out)
    Vars.x_h_2_attn_Slice_out = single(Vars.x_h_2_attn_Slice_out);
end
Vars.x_h_2_attn_Cast_outp = single(Vars.x_h_2_attn_Slice_out);
NumDims.x_h_2_attn_Cast_outp = NumDims.x_h_2_attn_Slice_out;

% Sqrt:
Vars.x_h_2_attn_Sqrt_outp = sqrt(Vars.x_h_2_attn_Cast_outp);
NumDims.x_h_2_attn_Sqrt_outp = NumDims.x_h_2_attn_Cast_outp;

% Div:
Vars.x_h_2_attn_Div_outpu = Vars.x_h_2_attn_Consta_9 ./ Vars.x_h_2_attn_Sqrt_outp;
NumDims.x_h_2_attn_Div_outpu = max(NumDims.x_h_2_attn_Consta_9, NumDims.x_h_2_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_2_attn_Div_outpu)
    Vars.x_h_2_attn_Div_outpu = single(Vars.x_h_2_attn_Div_outpu);
end
Vars.x_h_2_attn_Cast_1_ou = single(Vars.x_h_2_attn_Div_outpu);
NumDims.x_h_2_attn_Cast_1_ou = NumDims.x_h_2_attn_Div_outpu;

% Shape:
[Vars.x_h_2_attn_Shape_1_o, NumDims.x_h_2_attn_Shape_1_o] = onnxShape(Vars.x_h_2_attn_Transpose, NumDims.x_h_2_attn_Transpose, 0, NumDims.x_h_2_attn_Transpose+1);

% Shape:
[Vars.x_h_2_attn_Shape_2_o, NumDims.x_h_2_attn_Shape_2_o] = onnxShape(Vars.x_h_2_attn_Transpo_4, NumDims.x_h_2_attn_Transpo_4, 0, NumDims.x_h_2_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_2_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_2_attn_Shape_1_o, Vars.x_h_2_attn_Consta_11, Vars.x_h_2_attn_Consta_10, '', '', NumDims.x_h_2_attn_Shape_1_o);
Vars.x_h_2_attn_Slice_1_o = subsref(Vars.x_h_2_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_2_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_2_attn_Shape_2_o, Vars.x_h_2_attn_Consta_11, Vars.x_h_2_attn_Consta_10, '', '', NumDims.x_h_2_attn_Shape_2_o);
Vars.x_h_2_attn_Slice_2_o = subsref(Vars.x_h_2_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_2_attn_Concat_ou, NumDims.x_h_2_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_2_attn_Slice_1_o, Vars.x_h_2_attn_Slice_2_o}, [NumDims.x_h_2_attn_Slice_1_o, NumDims.x_h_2_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_2_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_2_attn_Concat_ou);
Vars.x_h_2_attn_Expand_ou = Vars.x_h_2_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_2_attn_Trilu_out, NumDims.x_h_2_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_2_attn_Expand_ou);

% Equal:
Vars.x_h_2_attn_Equal_out = Vars.x_h_2_attn_Trilu_out == Vars.x_h_2_attn_Constant_;
NumDims.x_h_2_attn_Equal_out = max(NumDims.x_h_2_attn_Trilu_out, NumDims.x_h_2_attn_Constant_);

% Where:
[Vars.x_h_2_attn_Where_out, NumDims.x_h_2_attn_Where_out] = onnxWhere(Vars.x_h_2_attn_Equal_out, Vars.x_h_2_attn_Consta_1, Vars.x_h_2_attn_Consta_2, NumDims.x_h_2_attn_Equal_out, NumDims.x_h_2_attn_Consta_1, NumDims.x_h_2_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_2_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1014, NumDims.x_h_2_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_2_attn_Transpo_2 = permute(Vars.x_h_2_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_2_attn_Sqrt_1_ou = sqrt(Vars.x_h_2_attn_Cast_1_ou);
NumDims.x_h_2_attn_Sqrt_1_ou = NumDims.x_h_2_attn_Cast_1_ou;

% Mul:
Vars.x_h_2_attn_Mul_outpu = Vars.x_h_2_attn_Transpose .* Vars.x_h_2_attn_Sqrt_1_ou;
NumDims.x_h_2_attn_Mul_outpu = max(NumDims.x_h_2_attn_Transpose, NumDims.x_h_2_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_2_attn_Sqrt_2_ou = sqrt(Vars.x_h_2_attn_Cast_1_ou);
NumDims.x_h_2_attn_Sqrt_2_ou = NumDims.x_h_2_attn_Cast_1_ou;

% Mul:
Vars.x_h_2_attn_Mul_1_out = Vars.x_h_2_attn_Transpo_2 .* Vars.x_h_2_attn_Sqrt_2_ou;
NumDims.x_h_2_attn_Mul_1_out = max(NumDims.x_h_2_attn_Transpo_2, NumDims.x_h_2_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_2_attn_MatMul_ou, NumDims.x_h_2_attn_MatMul_ou] = onnxMatMul(Vars.x_h_2_attn_Mul_outpu, Vars.x_h_2_attn_Mul_1_out, NumDims.x_h_2_attn_Mul_outpu, NumDims.x_h_2_attn_Mul_1_out);

% Add:
Vars.x_h_2_attn_Add_outpu = Vars.x_h_2_attn_MatMul_ou + Vars.x_h_2_attn_Where_out;
NumDims.x_h_2_attn_Add_outpu = max(NumDims.x_h_2_attn_MatMul_ou, NumDims.x_h_2_attn_Where_out);

% Softmax:
[Vars.x_h_2_attn_Softmax_o, NumDims.x_h_2_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_2_attn_Add_outpu, -1, NumDims.x_h_2_attn_Add_outpu);

% MatMul:
[Vars.x_h_2_attn_MatMul_1_, NumDims.x_h_2_attn_MatMul_1_] = onnxMatMul(Vars.x_h_2_attn_Softmax_o, Vars.x_h_2_attn_Transpo_1, NumDims.x_h_2_attn_Softmax_o, NumDims.x_h_2_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_2_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1015, NumDims.x_h_2_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_2_attn_Transpo_3 = permute(Vars.x_h_2_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_2_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_2_attn_Transpo_3, Vars.x_h_2_attn_Consta_3, NumDims.x_h_2_attn_Transpo_3, 0);
Vars.x_h_2_attn_Reshape_3 = reshape(Vars.x_h_2_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_2_attn_c_proj_Ma, NumDims.x_h_2_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_2_attn_Reshape_3, Vars.onnx__MatMul_974, NumDims.x_h_2_attn_Reshape_3, NumDims.onnx__MatMul_974);

% Add:
Vars.x_h_2_Add_output_0 = Vars.x_h_1_Add_1_output_0 + Vars.x_h_2_attn_c_proj_Ma;
NumDims.x_h_2_Add_output_0 = max(NumDims.x_h_1_Add_1_output_0, NumDims.x_h_2_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_2_ln_2_LayerNorm, NumDims.x_h_2_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_2_Add_output_0, Vars.transformer_h_2_ln_2, Vars.x_h_2_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_2_Add_output_0);

% MatMul:
[Vars.x_h_2_mlp_c_fc_MatMu, NumDims.x_h_2_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_2_ln_2_LayerNorm, Vars.onnx__MatMul_975, NumDims.x_h_2_ln_2_LayerNorm, NumDims.onnx__MatMul_975);

% Div:
Vars.x_h_2_mlp_gelu_Div_o = Vars.x_h_2_mlp_c_fc_MatMu ./ Vars.x_h_2_mlp_gelu_Con_2;
NumDims.x_h_2_mlp_gelu_Div_o = max(NumDims.x_h_2_mlp_c_fc_MatMu, NumDims.x_h_2_mlp_gelu_Con_2);

% Erf:
Vars.x_h_2_mlp_gelu_Erf_o = erf(Vars.x_h_2_mlp_gelu_Div_o);
NumDims.x_h_2_mlp_gelu_Erf_o = NumDims.x_h_2_mlp_gelu_Div_o;

% Add:
Vars.x_h_2_mlp_gelu_Add_o = Vars.x_h_2_mlp_gelu_Erf_o + Vars.x_h_2_mlp_gelu_Const;
NumDims.x_h_2_mlp_gelu_Add_o = max(NumDims.x_h_2_mlp_gelu_Erf_o, NumDims.x_h_2_mlp_gelu_Const);

% Mul:
Vars.x_h_2_mlp_gelu_Mul_o = Vars.x_h_2_mlp_c_fc_MatMu .* Vars.x_h_2_mlp_gelu_Add_o;
NumDims.x_h_2_mlp_gelu_Mul_o = max(NumDims.x_h_2_mlp_c_fc_MatMu, NumDims.x_h_2_mlp_gelu_Add_o);

% Mul:
Vars.x_h_2_mlp_gelu_Mul_1 = Vars.x_h_2_mlp_gelu_Mul_o .* Vars.x_h_2_mlp_gelu_Con_1;
NumDims.x_h_2_mlp_gelu_Mul_1 = max(NumDims.x_h_2_mlp_gelu_Mul_o, NumDims.x_h_2_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_2_mlp_c_proj_Mat, NumDims.x_h_2_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_2_mlp_gelu_Mul_1, Vars.onnx__MatMul_976, NumDims.x_h_2_mlp_gelu_Mul_1, NumDims.onnx__MatMul_976);

% Add:
Vars.x_h_2_Add_1_output_0 = Vars.x_h_2_Add_output_0 + Vars.x_h_2_mlp_c_proj_Mat;
NumDims.x_h_2_Add_1_output_0 = max(NumDims.x_h_2_Add_output_0, NumDims.x_h_2_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_h_3_ln_1_LayerNorm, NumDims.x_h_3_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_h_2_Add_1_output_0, Vars.transformer_h_3_ln_1, Vars.x_h_3_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_h_2_Add_1_output_0);

% MatMul:
[Vars.x_h_3_attn_c_attn_Ma, NumDims.x_h_3_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_3_ln_1_LayerNorm, Vars.onnx__MatMul_977, NumDims.x_h_3_ln_1_LayerNorm, NumDims.onnx__MatMul_977);

% Split:
[Vars.x_h_3_attn_Split_out, Vars.x_h_3_attn_Split_o_1, Vars.x_h_3_attn_Split_o_2, NumDims.x_h_3_attn_Split_out, NumDims.x_h_3_attn_Split_o_1, NumDims.x_h_3_attn_Split_o_2] = onnxSplit13(Vars.x_h_3_attn_c_attn_Ma, 2, Vars.x_h_3_attn_Consta_13, 3, NumDims.x_h_3_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_3_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_3_attn_Split_o_1, Vars.x_h_3_attn_Consta_4, NumDims.x_h_3_attn_Split_o_1, 0);
Vars.x_h_3_attn_Reshape_o = reshape(Vars.x_h_3_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_3_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1016, NumDims.x_h_3_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_3_attn_Transpo_4 = permute(Vars.x_h_3_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_3_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_3_attn_Split_out, Vars.x_h_3_attn_Consta_5, NumDims.x_h_3_attn_Split_out, 0);
Vars.x_h_3_attn_Reshape_1 = reshape(Vars.x_h_3_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_3_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1017, NumDims.x_h_3_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_3_attn_Transpose = permute(Vars.x_h_3_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_3_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_3_attn_Split_o_2, Vars.x_h_3_attn_Consta_6, NumDims.x_h_3_attn_Split_o_2, 0);
Vars.x_h_3_attn_Reshape_2 = reshape(Vars.x_h_3_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_3_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1018, NumDims.x_h_3_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_3_attn_Transpo_1 = permute(Vars.x_h_3_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_3_attn_Shape_out, NumDims.x_h_3_attn_Shape_out] = onnxShape(Vars.x_h_3_attn_Transpose, NumDims.x_h_3_attn_Transpose, 0, NumDims.x_h_3_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_3_attn_Slice_out] = prepareSliceArgs(Vars.x_h_3_attn_Shape_out, Vars.x_h_3_attn_Consta_7, Vars.x_h_3_attn_Consta_8, '', '', NumDims.x_h_3_attn_Shape_out);
Vars.x_h_3_attn_Slice_out = subsref(Vars.x_h_3_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_3_attn_Slice_out)
    Vars.x_h_3_attn_Slice_out = single(Vars.x_h_3_attn_Slice_out);
end
Vars.x_h_3_attn_Cast_outp = single(Vars.x_h_3_attn_Slice_out);
NumDims.x_h_3_attn_Cast_outp = NumDims.x_h_3_attn_Slice_out;

% Sqrt:
Vars.x_h_3_attn_Sqrt_outp = sqrt(Vars.x_h_3_attn_Cast_outp);
NumDims.x_h_3_attn_Sqrt_outp = NumDims.x_h_3_attn_Cast_outp;

% Div:
Vars.x_h_3_attn_Div_outpu = Vars.x_h_3_attn_Consta_9 ./ Vars.x_h_3_attn_Sqrt_outp;
NumDims.x_h_3_attn_Div_outpu = max(NumDims.x_h_3_attn_Consta_9, NumDims.x_h_3_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_3_attn_Div_outpu)
    Vars.x_h_3_attn_Div_outpu = single(Vars.x_h_3_attn_Div_outpu);
end
Vars.x_h_3_attn_Cast_1_ou = single(Vars.x_h_3_attn_Div_outpu);
NumDims.x_h_3_attn_Cast_1_ou = NumDims.x_h_3_attn_Div_outpu;

% Shape:
[Vars.x_h_3_attn_Shape_1_o, NumDims.x_h_3_attn_Shape_1_o] = onnxShape(Vars.x_h_3_attn_Transpose, NumDims.x_h_3_attn_Transpose, 0, NumDims.x_h_3_attn_Transpose+1);

% Shape:
[Vars.x_h_3_attn_Shape_2_o, NumDims.x_h_3_attn_Shape_2_o] = onnxShape(Vars.x_h_3_attn_Transpo_4, NumDims.x_h_3_attn_Transpo_4, 0, NumDims.x_h_3_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_3_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_3_attn_Shape_1_o, Vars.x_h_3_attn_Consta_11, Vars.x_h_3_attn_Consta_10, '', '', NumDims.x_h_3_attn_Shape_1_o);
Vars.x_h_3_attn_Slice_1_o = subsref(Vars.x_h_3_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_3_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_3_attn_Shape_2_o, Vars.x_h_3_attn_Consta_11, Vars.x_h_3_attn_Consta_10, '', '', NumDims.x_h_3_attn_Shape_2_o);
Vars.x_h_3_attn_Slice_2_o = subsref(Vars.x_h_3_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_3_attn_Concat_ou, NumDims.x_h_3_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_3_attn_Slice_1_o, Vars.x_h_3_attn_Slice_2_o}, [NumDims.x_h_3_attn_Slice_1_o, NumDims.x_h_3_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_3_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_3_attn_Concat_ou);
Vars.x_h_3_attn_Expand_ou = Vars.x_h_3_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_3_attn_Trilu_out, NumDims.x_h_3_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_3_attn_Expand_ou);

% Equal:
Vars.x_h_3_attn_Equal_out = Vars.x_h_3_attn_Trilu_out == Vars.x_h_3_attn_Constant_;
NumDims.x_h_3_attn_Equal_out = max(NumDims.x_h_3_attn_Trilu_out, NumDims.x_h_3_attn_Constant_);

% Where:
[Vars.x_h_3_attn_Where_out, NumDims.x_h_3_attn_Where_out] = onnxWhere(Vars.x_h_3_attn_Equal_out, Vars.x_h_3_attn_Consta_1, Vars.x_h_3_attn_Consta_2, NumDims.x_h_3_attn_Equal_out, NumDims.x_h_3_attn_Consta_1, NumDims.x_h_3_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_3_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1019, NumDims.x_h_3_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_3_attn_Transpo_2 = permute(Vars.x_h_3_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_3_attn_Sqrt_1_ou = sqrt(Vars.x_h_3_attn_Cast_1_ou);
NumDims.x_h_3_attn_Sqrt_1_ou = NumDims.x_h_3_attn_Cast_1_ou;

% Mul:
Vars.x_h_3_attn_Mul_outpu = Vars.x_h_3_attn_Transpose .* Vars.x_h_3_attn_Sqrt_1_ou;
NumDims.x_h_3_attn_Mul_outpu = max(NumDims.x_h_3_attn_Transpose, NumDims.x_h_3_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_3_attn_Sqrt_2_ou = sqrt(Vars.x_h_3_attn_Cast_1_ou);
NumDims.x_h_3_attn_Sqrt_2_ou = NumDims.x_h_3_attn_Cast_1_ou;

% Mul:
Vars.x_h_3_attn_Mul_1_out = Vars.x_h_3_attn_Transpo_2 .* Vars.x_h_3_attn_Sqrt_2_ou;
NumDims.x_h_3_attn_Mul_1_out = max(NumDims.x_h_3_attn_Transpo_2, NumDims.x_h_3_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_3_attn_MatMul_ou, NumDims.x_h_3_attn_MatMul_ou] = onnxMatMul(Vars.x_h_3_attn_Mul_outpu, Vars.x_h_3_attn_Mul_1_out, NumDims.x_h_3_attn_Mul_outpu, NumDims.x_h_3_attn_Mul_1_out);

% Add:
Vars.x_h_3_attn_Add_outpu = Vars.x_h_3_attn_MatMul_ou + Vars.x_h_3_attn_Where_out;
NumDims.x_h_3_attn_Add_outpu = max(NumDims.x_h_3_attn_MatMul_ou, NumDims.x_h_3_attn_Where_out);

% Softmax:
[Vars.x_h_3_attn_Softmax_o, NumDims.x_h_3_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_3_attn_Add_outpu, -1, NumDims.x_h_3_attn_Add_outpu);

% MatMul:
[Vars.x_h_3_attn_MatMul_1_, NumDims.x_h_3_attn_MatMul_1_] = onnxMatMul(Vars.x_h_3_attn_Softmax_o, Vars.x_h_3_attn_Transpo_1, NumDims.x_h_3_attn_Softmax_o, NumDims.x_h_3_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_3_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1020, NumDims.x_h_3_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_3_attn_Transpo_3 = permute(Vars.x_h_3_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_3_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_3_attn_Transpo_3, Vars.x_h_3_attn_Consta_3, NumDims.x_h_3_attn_Transpo_3, 0);
Vars.x_h_3_attn_Reshape_3 = reshape(Vars.x_h_3_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_3_attn_c_proj_Ma, NumDims.x_h_3_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_3_attn_Reshape_3, Vars.onnx__MatMul_997, NumDims.x_h_3_attn_Reshape_3, NumDims.onnx__MatMul_997);

% Add:
Vars.x_h_3_Add_output_0 = Vars.x_h_2_Add_1_output_0 + Vars.x_h_3_attn_c_proj_Ma;
NumDims.x_h_3_Add_output_0 = max(NumDims.x_h_2_Add_1_output_0, NumDims.x_h_3_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_3_ln_2_LayerNorm, NumDims.x_h_3_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_3_Add_output_0, Vars.transformer_h_3_ln_2, Vars.x_h_3_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_3_Add_output_0);

% MatMul:
[Vars.x_h_3_mlp_c_fc_MatMu, NumDims.x_h_3_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_3_ln_2_LayerNorm, Vars.onnx__MatMul_998, NumDims.x_h_3_ln_2_LayerNorm, NumDims.onnx__MatMul_998);

% Div:
Vars.x_h_3_mlp_gelu_Div_o = Vars.x_h_3_mlp_c_fc_MatMu ./ Vars.x_h_3_mlp_gelu_Con_2;
NumDims.x_h_3_mlp_gelu_Div_o = max(NumDims.x_h_3_mlp_c_fc_MatMu, NumDims.x_h_3_mlp_gelu_Con_2);

% Erf:
Vars.x_h_3_mlp_gelu_Erf_o = erf(Vars.x_h_3_mlp_gelu_Div_o);
NumDims.x_h_3_mlp_gelu_Erf_o = NumDims.x_h_3_mlp_gelu_Div_o;

% Add:
Vars.x_h_3_mlp_gelu_Add_o = Vars.x_h_3_mlp_gelu_Erf_o + Vars.x_h_3_mlp_gelu_Const;
NumDims.x_h_3_mlp_gelu_Add_o = max(NumDims.x_h_3_mlp_gelu_Erf_o, NumDims.x_h_3_mlp_gelu_Const);

% Mul:
Vars.x_h_3_mlp_gelu_Mul_o = Vars.x_h_3_mlp_c_fc_MatMu .* Vars.x_h_3_mlp_gelu_Add_o;
NumDims.x_h_3_mlp_gelu_Mul_o = max(NumDims.x_h_3_mlp_c_fc_MatMu, NumDims.x_h_3_mlp_gelu_Add_o);

% Mul:
Vars.x_h_3_mlp_gelu_Mul_1 = Vars.x_h_3_mlp_gelu_Mul_o .* Vars.x_h_3_mlp_gelu_Con_1;
NumDims.x_h_3_mlp_gelu_Mul_1 = max(NumDims.x_h_3_mlp_gelu_Mul_o, NumDims.x_h_3_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_3_mlp_c_proj_Mat, NumDims.x_h_3_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_3_mlp_gelu_Mul_1, Vars.onnx__MatMul_999, NumDims.x_h_3_mlp_gelu_Mul_1, NumDims.onnx__MatMul_999);

% Add:
Vars.x_h_3_Add_1_output_0 = Vars.x_h_3_Add_output_0 + Vars.x_h_3_mlp_c_proj_Mat;
NumDims.x_h_3_Add_1_output_0 = max(NumDims.x_h_3_Add_output_0, NumDims.x_h_3_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_h_4_ln_1_LayerNorm, NumDims.x_h_4_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_h_3_Add_1_output_0, Vars.transformer_h_4_ln_1, Vars.x_h_4_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_h_3_Add_1_output_0);

% MatMul:
[Vars.x_h_4_attn_c_attn_Ma, NumDims.x_h_4_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_4_ln_1_LayerNorm, Vars.onnx__MatMul_1000, NumDims.x_h_4_ln_1_LayerNorm, NumDims.onnx__MatMul_1000);

% Split:
[Vars.x_h_4_attn_Split_out, Vars.x_h_4_attn_Split_o_1, Vars.x_h_4_attn_Split_o_2, NumDims.x_h_4_attn_Split_out, NumDims.x_h_4_attn_Split_o_1, NumDims.x_h_4_attn_Split_o_2] = onnxSplit13(Vars.x_h_4_attn_c_attn_Ma, 2, Vars.x_h_4_attn_Consta_13, 3, NumDims.x_h_4_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_4_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_4_attn_Split_o_1, Vars.x_h_4_attn_Consta_4, NumDims.x_h_4_attn_Split_o_1, 0);
Vars.x_h_4_attn_Reshape_o = reshape(Vars.x_h_4_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_4_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1021, NumDims.x_h_4_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_4_attn_Transpo_4 = permute(Vars.x_h_4_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_4_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_4_attn_Split_out, Vars.x_h_4_attn_Consta_5, NumDims.x_h_4_attn_Split_out, 0);
Vars.x_h_4_attn_Reshape_1 = reshape(Vars.x_h_4_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_4_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1022, NumDims.x_h_4_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_4_attn_Transpose = permute(Vars.x_h_4_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_4_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_4_attn_Split_o_2, Vars.x_h_4_attn_Consta_6, NumDims.x_h_4_attn_Split_o_2, 0);
Vars.x_h_4_attn_Reshape_2 = reshape(Vars.x_h_4_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_4_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1023, NumDims.x_h_4_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_4_attn_Transpo_1 = permute(Vars.x_h_4_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_4_attn_Shape_out, NumDims.x_h_4_attn_Shape_out] = onnxShape(Vars.x_h_4_attn_Transpose, NumDims.x_h_4_attn_Transpose, 0, NumDims.x_h_4_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_4_attn_Slice_out] = prepareSliceArgs(Vars.x_h_4_attn_Shape_out, Vars.x_h_4_attn_Consta_7, Vars.x_h_4_attn_Consta_8, '', '', NumDims.x_h_4_attn_Shape_out);
Vars.x_h_4_attn_Slice_out = subsref(Vars.x_h_4_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_4_attn_Slice_out)
    Vars.x_h_4_attn_Slice_out = single(Vars.x_h_4_attn_Slice_out);
end
Vars.x_h_4_attn_Cast_outp = single(Vars.x_h_4_attn_Slice_out);
NumDims.x_h_4_attn_Cast_outp = NumDims.x_h_4_attn_Slice_out;

% Sqrt:
Vars.x_h_4_attn_Sqrt_outp = sqrt(Vars.x_h_4_attn_Cast_outp);
NumDims.x_h_4_attn_Sqrt_outp = NumDims.x_h_4_attn_Cast_outp;

% Div:
Vars.x_h_4_attn_Div_outpu = Vars.x_h_4_attn_Consta_9 ./ Vars.x_h_4_attn_Sqrt_outp;
NumDims.x_h_4_attn_Div_outpu = max(NumDims.x_h_4_attn_Consta_9, NumDims.x_h_4_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_4_attn_Div_outpu)
    Vars.x_h_4_attn_Div_outpu = single(Vars.x_h_4_attn_Div_outpu);
end
Vars.x_h_4_attn_Cast_1_ou = single(Vars.x_h_4_attn_Div_outpu);
NumDims.x_h_4_attn_Cast_1_ou = NumDims.x_h_4_attn_Div_outpu;

% Shape:
[Vars.x_h_4_attn_Shape_1_o, NumDims.x_h_4_attn_Shape_1_o] = onnxShape(Vars.x_h_4_attn_Transpose, NumDims.x_h_4_attn_Transpose, 0, NumDims.x_h_4_attn_Transpose+1);

% Shape:
[Vars.x_h_4_attn_Shape_2_o, NumDims.x_h_4_attn_Shape_2_o] = onnxShape(Vars.x_h_4_attn_Transpo_4, NumDims.x_h_4_attn_Transpo_4, 0, NumDims.x_h_4_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_4_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_4_attn_Shape_1_o, Vars.x_h_4_attn_Consta_11, Vars.x_h_4_attn_Consta_10, '', '', NumDims.x_h_4_attn_Shape_1_o);
Vars.x_h_4_attn_Slice_1_o = subsref(Vars.x_h_4_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_4_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_4_attn_Shape_2_o, Vars.x_h_4_attn_Consta_11, Vars.x_h_4_attn_Consta_10, '', '', NumDims.x_h_4_attn_Shape_2_o);
Vars.x_h_4_attn_Slice_2_o = subsref(Vars.x_h_4_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_4_attn_Concat_ou, NumDims.x_h_4_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_4_attn_Slice_1_o, Vars.x_h_4_attn_Slice_2_o}, [NumDims.x_h_4_attn_Slice_1_o, NumDims.x_h_4_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_4_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_4_attn_Concat_ou);
Vars.x_h_4_attn_Expand_ou = Vars.x_h_4_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_4_attn_Trilu_out, NumDims.x_h_4_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_4_attn_Expand_ou);

% Equal:
Vars.x_h_4_attn_Equal_out = Vars.x_h_4_attn_Trilu_out == Vars.x_h_4_attn_Constant_;
NumDims.x_h_4_attn_Equal_out = max(NumDims.x_h_4_attn_Trilu_out, NumDims.x_h_4_attn_Constant_);

% Where:
[Vars.x_h_4_attn_Where_out, NumDims.x_h_4_attn_Where_out] = onnxWhere(Vars.x_h_4_attn_Equal_out, Vars.x_h_4_attn_Consta_1, Vars.x_h_4_attn_Consta_2, NumDims.x_h_4_attn_Equal_out, NumDims.x_h_4_attn_Consta_1, NumDims.x_h_4_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_4_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1024, NumDims.x_h_4_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_4_attn_Transpo_2 = permute(Vars.x_h_4_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_4_attn_Sqrt_1_ou = sqrt(Vars.x_h_4_attn_Cast_1_ou);
NumDims.x_h_4_attn_Sqrt_1_ou = NumDims.x_h_4_attn_Cast_1_ou;

% Mul:
Vars.x_h_4_attn_Mul_outpu = Vars.x_h_4_attn_Transpose .* Vars.x_h_4_attn_Sqrt_1_ou;
NumDims.x_h_4_attn_Mul_outpu = max(NumDims.x_h_4_attn_Transpose, NumDims.x_h_4_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_4_attn_Sqrt_2_ou = sqrt(Vars.x_h_4_attn_Cast_1_ou);
NumDims.x_h_4_attn_Sqrt_2_ou = NumDims.x_h_4_attn_Cast_1_ou;

% Mul:
Vars.x_h_4_attn_Mul_1_out = Vars.x_h_4_attn_Transpo_2 .* Vars.x_h_4_attn_Sqrt_2_ou;
NumDims.x_h_4_attn_Mul_1_out = max(NumDims.x_h_4_attn_Transpo_2, NumDims.x_h_4_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_4_attn_MatMul_ou, NumDims.x_h_4_attn_MatMul_ou] = onnxMatMul(Vars.x_h_4_attn_Mul_outpu, Vars.x_h_4_attn_Mul_1_out, NumDims.x_h_4_attn_Mul_outpu, NumDims.x_h_4_attn_Mul_1_out);

% Add:
Vars.x_h_4_attn_Add_outpu = Vars.x_h_4_attn_MatMul_ou + Vars.x_h_4_attn_Where_out;
NumDims.x_h_4_attn_Add_outpu = max(NumDims.x_h_4_attn_MatMul_ou, NumDims.x_h_4_attn_Where_out);

% Softmax:
[Vars.x_h_4_attn_Softmax_o, NumDims.x_h_4_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_4_attn_Add_outpu, -1, NumDims.x_h_4_attn_Add_outpu);

% MatMul:
[Vars.x_h_4_attn_MatMul_1_, NumDims.x_h_4_attn_MatMul_1_] = onnxMatMul(Vars.x_h_4_attn_Softmax_o, Vars.x_h_4_attn_Transpo_1, NumDims.x_h_4_attn_Softmax_o, NumDims.x_h_4_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_4_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1025, NumDims.x_h_4_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_4_attn_Transpo_3 = permute(Vars.x_h_4_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_4_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_4_attn_Transpo_3, Vars.x_h_4_attn_Consta_3, NumDims.x_h_4_attn_Transpo_3, 0);
Vars.x_h_4_attn_Reshape_3 = reshape(Vars.x_h_4_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_4_attn_c_proj_Ma, NumDims.x_h_4_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_4_attn_Reshape_3, Vars.onnx__MatMul_1020, NumDims.x_h_4_attn_Reshape_3, NumDims.onnx__MatMul_1020);

% Add:
Vars.x_h_4_Add_output_0 = Vars.x_h_3_Add_1_output_0 + Vars.x_h_4_attn_c_proj_Ma;
NumDims.x_h_4_Add_output_0 = max(NumDims.x_h_3_Add_1_output_0, NumDims.x_h_4_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_4_ln_2_LayerNorm, NumDims.x_h_4_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_4_Add_output_0, Vars.transformer_h_4_ln_2, Vars.x_h_4_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_4_Add_output_0);

% MatMul:
[Vars.x_h_4_mlp_c_fc_MatMu, NumDims.x_h_4_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_4_ln_2_LayerNorm, Vars.onnx__MatMul_1021, NumDims.x_h_4_ln_2_LayerNorm, NumDims.onnx__MatMul_1021);

% Div:
Vars.x_h_4_mlp_gelu_Div_o = Vars.x_h_4_mlp_c_fc_MatMu ./ Vars.x_h_4_mlp_gelu_Con_2;
NumDims.x_h_4_mlp_gelu_Div_o = max(NumDims.x_h_4_mlp_c_fc_MatMu, NumDims.x_h_4_mlp_gelu_Con_2);

% Erf:
Vars.x_h_4_mlp_gelu_Erf_o = erf(Vars.x_h_4_mlp_gelu_Div_o);
NumDims.x_h_4_mlp_gelu_Erf_o = NumDims.x_h_4_mlp_gelu_Div_o;

% Add:
Vars.x_h_4_mlp_gelu_Add_o = Vars.x_h_4_mlp_gelu_Erf_o + Vars.x_h_4_mlp_gelu_Const;
NumDims.x_h_4_mlp_gelu_Add_o = max(NumDims.x_h_4_mlp_gelu_Erf_o, NumDims.x_h_4_mlp_gelu_Const);

% Mul:
Vars.x_h_4_mlp_gelu_Mul_o = Vars.x_h_4_mlp_c_fc_MatMu .* Vars.x_h_4_mlp_gelu_Add_o;
NumDims.x_h_4_mlp_gelu_Mul_o = max(NumDims.x_h_4_mlp_c_fc_MatMu, NumDims.x_h_4_mlp_gelu_Add_o);

% Mul:
Vars.x_h_4_mlp_gelu_Mul_1 = Vars.x_h_4_mlp_gelu_Mul_o .* Vars.x_h_4_mlp_gelu_Con_1;
NumDims.x_h_4_mlp_gelu_Mul_1 = max(NumDims.x_h_4_mlp_gelu_Mul_o, NumDims.x_h_4_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_4_mlp_c_proj_Mat, NumDims.x_h_4_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_4_mlp_gelu_Mul_1, Vars.onnx__MatMul_1022, NumDims.x_h_4_mlp_gelu_Mul_1, NumDims.onnx__MatMul_1022);

% Add:
Vars.x_h_4_Add_1_output_0 = Vars.x_h_4_Add_output_0 + Vars.x_h_4_mlp_c_proj_Mat;
NumDims.x_h_4_Add_1_output_0 = max(NumDims.x_h_4_Add_output_0, NumDims.x_h_4_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_h_5_ln_1_LayerNorm, NumDims.x_h_5_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_h_4_Add_1_output_0, Vars.transformer_h_5_ln_1, Vars.x_h_5_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_h_4_Add_1_output_0);

% MatMul:
[Vars.x_h_5_attn_c_attn_Ma, NumDims.x_h_5_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_5_ln_1_LayerNorm, Vars.onnx__MatMul_1023, NumDims.x_h_5_ln_1_LayerNorm, NumDims.onnx__MatMul_1023);

% Split:
[Vars.x_h_5_attn_Split_out, Vars.x_h_5_attn_Split_o_1, Vars.x_h_5_attn_Split_o_2, NumDims.x_h_5_attn_Split_out, NumDims.x_h_5_attn_Split_o_1, NumDims.x_h_5_attn_Split_o_2] = onnxSplit13(Vars.x_h_5_attn_c_attn_Ma, 2, Vars.x_h_5_attn_Consta_13, 3, NumDims.x_h_5_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_5_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_5_attn_Split_o_1, Vars.x_h_5_attn_Consta_4, NumDims.x_h_5_attn_Split_o_1, 0);
Vars.x_h_5_attn_Reshape_o = reshape(Vars.x_h_5_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_5_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1026, NumDims.x_h_5_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_5_attn_Transpo_4 = permute(Vars.x_h_5_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_5_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_5_attn_Split_out, Vars.x_h_5_attn_Consta_5, NumDims.x_h_5_attn_Split_out, 0);
Vars.x_h_5_attn_Reshape_1 = reshape(Vars.x_h_5_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_5_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1027, NumDims.x_h_5_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_5_attn_Transpose = permute(Vars.x_h_5_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_5_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_5_attn_Split_o_2, Vars.x_h_5_attn_Consta_6, NumDims.x_h_5_attn_Split_o_2, 0);
Vars.x_h_5_attn_Reshape_2 = reshape(Vars.x_h_5_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_5_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1028, NumDims.x_h_5_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_5_attn_Transpo_1 = permute(Vars.x_h_5_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_5_attn_Shape_out, NumDims.x_h_5_attn_Shape_out] = onnxShape(Vars.x_h_5_attn_Transpose, NumDims.x_h_5_attn_Transpose, 0, NumDims.x_h_5_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_5_attn_Slice_out] = prepareSliceArgs(Vars.x_h_5_attn_Shape_out, Vars.x_h_5_attn_Consta_7, Vars.x_h_5_attn_Consta_8, '', '', NumDims.x_h_5_attn_Shape_out);
Vars.x_h_5_attn_Slice_out = subsref(Vars.x_h_5_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_5_attn_Slice_out)
    Vars.x_h_5_attn_Slice_out = single(Vars.x_h_5_attn_Slice_out);
end
Vars.x_h_5_attn_Cast_outp = single(Vars.x_h_5_attn_Slice_out);
NumDims.x_h_5_attn_Cast_outp = NumDims.x_h_5_attn_Slice_out;

% Sqrt:
Vars.x_h_5_attn_Sqrt_outp = sqrt(Vars.x_h_5_attn_Cast_outp);
NumDims.x_h_5_attn_Sqrt_outp = NumDims.x_h_5_attn_Cast_outp;

% Div:
Vars.x_h_5_attn_Div_outpu = Vars.x_h_5_attn_Consta_9 ./ Vars.x_h_5_attn_Sqrt_outp;
NumDims.x_h_5_attn_Div_outpu = max(NumDims.x_h_5_attn_Consta_9, NumDims.x_h_5_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_5_attn_Div_outpu)
    Vars.x_h_5_attn_Div_outpu = single(Vars.x_h_5_attn_Div_outpu);
end
Vars.x_h_5_attn_Cast_1_ou = single(Vars.x_h_5_attn_Div_outpu);
NumDims.x_h_5_attn_Cast_1_ou = NumDims.x_h_5_attn_Div_outpu;

% Shape:
[Vars.x_h_5_attn_Shape_1_o, NumDims.x_h_5_attn_Shape_1_o] = onnxShape(Vars.x_h_5_attn_Transpose, NumDims.x_h_5_attn_Transpose, 0, NumDims.x_h_5_attn_Transpose+1);

% Shape:
[Vars.x_h_5_attn_Shape_2_o, NumDims.x_h_5_attn_Shape_2_o] = onnxShape(Vars.x_h_5_attn_Transpo_4, NumDims.x_h_5_attn_Transpo_4, 0, NumDims.x_h_5_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_5_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_5_attn_Shape_1_o, Vars.x_h_5_attn_Consta_11, Vars.x_h_5_attn_Consta_10, '', '', NumDims.x_h_5_attn_Shape_1_o);
Vars.x_h_5_attn_Slice_1_o = subsref(Vars.x_h_5_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_5_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_5_attn_Shape_2_o, Vars.x_h_5_attn_Consta_11, Vars.x_h_5_attn_Consta_10, '', '', NumDims.x_h_5_attn_Shape_2_o);
Vars.x_h_5_attn_Slice_2_o = subsref(Vars.x_h_5_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_5_attn_Concat_ou, NumDims.x_h_5_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_5_attn_Slice_1_o, Vars.x_h_5_attn_Slice_2_o}, [NumDims.x_h_5_attn_Slice_1_o, NumDims.x_h_5_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_5_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_5_attn_Concat_ou);
Vars.x_h_5_attn_Expand_ou = Vars.x_h_5_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_5_attn_Trilu_out, NumDims.x_h_5_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_5_attn_Expand_ou);

% Equal:
Vars.x_h_5_attn_Equal_out = Vars.x_h_5_attn_Trilu_out == Vars.x_h_5_attn_Constant_;
NumDims.x_h_5_attn_Equal_out = max(NumDims.x_h_5_attn_Trilu_out, NumDims.x_h_5_attn_Constant_);

% Where:
[Vars.x_h_5_attn_Where_out, NumDims.x_h_5_attn_Where_out] = onnxWhere(Vars.x_h_5_attn_Equal_out, Vars.x_h_5_attn_Consta_1, Vars.x_h_5_attn_Consta_2, NumDims.x_h_5_attn_Equal_out, NumDims.x_h_5_attn_Consta_1, NumDims.x_h_5_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_5_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1029, NumDims.x_h_5_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_5_attn_Transpo_2 = permute(Vars.x_h_5_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_5_attn_Sqrt_1_ou = sqrt(Vars.x_h_5_attn_Cast_1_ou);
NumDims.x_h_5_attn_Sqrt_1_ou = NumDims.x_h_5_attn_Cast_1_ou;

% Mul:
Vars.x_h_5_attn_Mul_outpu = Vars.x_h_5_attn_Transpose .* Vars.x_h_5_attn_Sqrt_1_ou;
NumDims.x_h_5_attn_Mul_outpu = max(NumDims.x_h_5_attn_Transpose, NumDims.x_h_5_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_5_attn_Sqrt_2_ou = sqrt(Vars.x_h_5_attn_Cast_1_ou);
NumDims.x_h_5_attn_Sqrt_2_ou = NumDims.x_h_5_attn_Cast_1_ou;

% Mul:
Vars.x_h_5_attn_Mul_1_out = Vars.x_h_5_attn_Transpo_2 .* Vars.x_h_5_attn_Sqrt_2_ou;
NumDims.x_h_5_attn_Mul_1_out = max(NumDims.x_h_5_attn_Transpo_2, NumDims.x_h_5_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_5_attn_MatMul_ou, NumDims.x_h_5_attn_MatMul_ou] = onnxMatMul(Vars.x_h_5_attn_Mul_outpu, Vars.x_h_5_attn_Mul_1_out, NumDims.x_h_5_attn_Mul_outpu, NumDims.x_h_5_attn_Mul_1_out);

% Add:
Vars.x_h_5_attn_Add_outpu = Vars.x_h_5_attn_MatMul_ou + Vars.x_h_5_attn_Where_out;
NumDims.x_h_5_attn_Add_outpu = max(NumDims.x_h_5_attn_MatMul_ou, NumDims.x_h_5_attn_Where_out);

% Softmax:
[Vars.x_h_5_attn_Softmax_o, NumDims.x_h_5_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_5_attn_Add_outpu, -1, NumDims.x_h_5_attn_Add_outpu);

% MatMul:
[Vars.x_h_5_attn_MatMul_1_, NumDims.x_h_5_attn_MatMul_1_] = onnxMatMul(Vars.x_h_5_attn_Softmax_o, Vars.x_h_5_attn_Transpo_1, NumDims.x_h_5_attn_Softmax_o, NumDims.x_h_5_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_5_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1030, NumDims.x_h_5_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_5_attn_Transpo_3 = permute(Vars.x_h_5_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_5_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_5_attn_Transpo_3, Vars.x_h_5_attn_Consta_3, NumDims.x_h_5_attn_Transpo_3, 0);
Vars.x_h_5_attn_Reshape_3 = reshape(Vars.x_h_5_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_5_attn_c_proj_Ma, NumDims.x_h_5_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_5_attn_Reshape_3, Vars.onnx__MatMul_1043, NumDims.x_h_5_attn_Reshape_3, NumDims.onnx__MatMul_1043);

% Add:
Vars.x_h_5_Add_output_0 = Vars.x_h_4_Add_1_output_0 + Vars.x_h_5_attn_c_proj_Ma;
NumDims.x_h_5_Add_output_0 = max(NumDims.x_h_4_Add_1_output_0, NumDims.x_h_5_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_5_ln_2_LayerNorm, NumDims.x_h_5_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_5_Add_output_0, Vars.transformer_h_5_ln_2, Vars.x_h_5_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_5_Add_output_0);

% MatMul:
[Vars.x_h_5_mlp_c_fc_MatMu, NumDims.x_h_5_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_5_ln_2_LayerNorm, Vars.onnx__MatMul_1044, NumDims.x_h_5_ln_2_LayerNorm, NumDims.onnx__MatMul_1044);

% Div:
Vars.x_h_5_mlp_gelu_Div_o = Vars.x_h_5_mlp_c_fc_MatMu ./ Vars.x_h_5_mlp_gelu_Con_2;
NumDims.x_h_5_mlp_gelu_Div_o = max(NumDims.x_h_5_mlp_c_fc_MatMu, NumDims.x_h_5_mlp_gelu_Con_2);

% Erf:
Vars.x_h_5_mlp_gelu_Erf_o = erf(Vars.x_h_5_mlp_gelu_Div_o);
NumDims.x_h_5_mlp_gelu_Erf_o = NumDims.x_h_5_mlp_gelu_Div_o;

% Add:
Vars.x_h_5_mlp_gelu_Add_o = Vars.x_h_5_mlp_gelu_Erf_o + Vars.x_h_5_mlp_gelu_Const;
NumDims.x_h_5_mlp_gelu_Add_o = max(NumDims.x_h_5_mlp_gelu_Erf_o, NumDims.x_h_5_mlp_gelu_Const);

% Mul:
Vars.x_h_5_mlp_gelu_Mul_o = Vars.x_h_5_mlp_c_fc_MatMu .* Vars.x_h_5_mlp_gelu_Add_o;
NumDims.x_h_5_mlp_gelu_Mul_o = max(NumDims.x_h_5_mlp_c_fc_MatMu, NumDims.x_h_5_mlp_gelu_Add_o);

% Mul:
Vars.x_h_5_mlp_gelu_Mul_1 = Vars.x_h_5_mlp_gelu_Mul_o .* Vars.x_h_5_mlp_gelu_Con_1;
NumDims.x_h_5_mlp_gelu_Mul_1 = max(NumDims.x_h_5_mlp_gelu_Mul_o, NumDims.x_h_5_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_5_mlp_c_proj_Mat, NumDims.x_h_5_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_5_mlp_gelu_Mul_1, Vars.onnx__MatMul_1045, NumDims.x_h_5_mlp_gelu_Mul_1, NumDims.onnx__MatMul_1045);

% Add:
Vars.x_h_5_Add_1_output_0 = Vars.x_h_5_Add_output_0 + Vars.x_h_5_mlp_c_proj_Mat;
NumDims.x_h_5_Add_1_output_0 = max(NumDims.x_h_5_Add_output_0, NumDims.x_h_5_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_h_6_ln_1_LayerNorm, NumDims.x_h_6_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_h_5_Add_1_output_0, Vars.transformer_h_6_ln_1, Vars.x_h_6_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_h_5_Add_1_output_0);

% MatMul:
[Vars.x_h_6_attn_c_attn_Ma, NumDims.x_h_6_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_6_ln_1_LayerNorm, Vars.onnx__MatMul_1046, NumDims.x_h_6_ln_1_LayerNorm, NumDims.onnx__MatMul_1046);

% Split:
[Vars.x_h_6_attn_Split_out, Vars.x_h_6_attn_Split_o_1, Vars.x_h_6_attn_Split_o_2, NumDims.x_h_6_attn_Split_out, NumDims.x_h_6_attn_Split_o_1, NumDims.x_h_6_attn_Split_o_2] = onnxSplit13(Vars.x_h_6_attn_c_attn_Ma, 2, Vars.x_h_6_attn_Consta_13, 3, NumDims.x_h_6_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_6_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_6_attn_Split_o_1, Vars.x_h_6_attn_Consta_4, NumDims.x_h_6_attn_Split_o_1, 0);
Vars.x_h_6_attn_Reshape_o = reshape(Vars.x_h_6_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_6_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1031, NumDims.x_h_6_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_6_attn_Transpo_4 = permute(Vars.x_h_6_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_6_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_6_attn_Split_out, Vars.x_h_6_attn_Consta_5, NumDims.x_h_6_attn_Split_out, 0);
Vars.x_h_6_attn_Reshape_1 = reshape(Vars.x_h_6_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_6_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1032, NumDims.x_h_6_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_6_attn_Transpose = permute(Vars.x_h_6_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_6_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_6_attn_Split_o_2, Vars.x_h_6_attn_Consta_6, NumDims.x_h_6_attn_Split_o_2, 0);
Vars.x_h_6_attn_Reshape_2 = reshape(Vars.x_h_6_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_6_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1033, NumDims.x_h_6_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_6_attn_Transpo_1 = permute(Vars.x_h_6_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_6_attn_Shape_out, NumDims.x_h_6_attn_Shape_out] = onnxShape(Vars.x_h_6_attn_Transpose, NumDims.x_h_6_attn_Transpose, 0, NumDims.x_h_6_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_6_attn_Slice_out] = prepareSliceArgs(Vars.x_h_6_attn_Shape_out, Vars.x_h_6_attn_Consta_7, Vars.x_h_6_attn_Consta_8, '', '', NumDims.x_h_6_attn_Shape_out);
Vars.x_h_6_attn_Slice_out = subsref(Vars.x_h_6_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_6_attn_Slice_out)
    Vars.x_h_6_attn_Slice_out = single(Vars.x_h_6_attn_Slice_out);
end
Vars.x_h_6_attn_Cast_outp = single(Vars.x_h_6_attn_Slice_out);
NumDims.x_h_6_attn_Cast_outp = NumDims.x_h_6_attn_Slice_out;

% Sqrt:
Vars.x_h_6_attn_Sqrt_outp = sqrt(Vars.x_h_6_attn_Cast_outp);
NumDims.x_h_6_attn_Sqrt_outp = NumDims.x_h_6_attn_Cast_outp;

% Div:
Vars.x_h_6_attn_Div_outpu = Vars.x_h_6_attn_Consta_9 ./ Vars.x_h_6_attn_Sqrt_outp;
NumDims.x_h_6_attn_Div_outpu = max(NumDims.x_h_6_attn_Consta_9, NumDims.x_h_6_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_6_attn_Div_outpu)
    Vars.x_h_6_attn_Div_outpu = single(Vars.x_h_6_attn_Div_outpu);
end
Vars.x_h_6_attn_Cast_1_ou = single(Vars.x_h_6_attn_Div_outpu);
NumDims.x_h_6_attn_Cast_1_ou = NumDims.x_h_6_attn_Div_outpu;

% Shape:
[Vars.x_h_6_attn_Shape_1_o, NumDims.x_h_6_attn_Shape_1_o] = onnxShape(Vars.x_h_6_attn_Transpose, NumDims.x_h_6_attn_Transpose, 0, NumDims.x_h_6_attn_Transpose+1);

% Shape:
[Vars.x_h_6_attn_Shape_2_o, NumDims.x_h_6_attn_Shape_2_o] = onnxShape(Vars.x_h_6_attn_Transpo_4, NumDims.x_h_6_attn_Transpo_4, 0, NumDims.x_h_6_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_6_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_6_attn_Shape_1_o, Vars.x_h_6_attn_Consta_11, Vars.x_h_6_attn_Consta_10, '', '', NumDims.x_h_6_attn_Shape_1_o);
Vars.x_h_6_attn_Slice_1_o = subsref(Vars.x_h_6_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_6_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_6_attn_Shape_2_o, Vars.x_h_6_attn_Consta_11, Vars.x_h_6_attn_Consta_10, '', '', NumDims.x_h_6_attn_Shape_2_o);
Vars.x_h_6_attn_Slice_2_o = subsref(Vars.x_h_6_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_6_attn_Concat_ou, NumDims.x_h_6_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_6_attn_Slice_1_o, Vars.x_h_6_attn_Slice_2_o}, [NumDims.x_h_6_attn_Slice_1_o, NumDims.x_h_6_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_6_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_6_attn_Concat_ou);
Vars.x_h_6_attn_Expand_ou = Vars.x_h_6_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_6_attn_Trilu_out, NumDims.x_h_6_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_6_attn_Expand_ou);

% Equal:
Vars.x_h_6_attn_Equal_out = Vars.x_h_6_attn_Trilu_out == Vars.x_h_6_attn_Constant_;
NumDims.x_h_6_attn_Equal_out = max(NumDims.x_h_6_attn_Trilu_out, NumDims.x_h_6_attn_Constant_);

% Where:
[Vars.x_h_6_attn_Where_out, NumDims.x_h_6_attn_Where_out] = onnxWhere(Vars.x_h_6_attn_Equal_out, Vars.x_h_6_attn_Consta_1, Vars.x_h_6_attn_Consta_2, NumDims.x_h_6_attn_Equal_out, NumDims.x_h_6_attn_Consta_1, NumDims.x_h_6_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_6_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1034, NumDims.x_h_6_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_6_attn_Transpo_2 = permute(Vars.x_h_6_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_6_attn_Sqrt_1_ou = sqrt(Vars.x_h_6_attn_Cast_1_ou);
NumDims.x_h_6_attn_Sqrt_1_ou = NumDims.x_h_6_attn_Cast_1_ou;

% Mul:
Vars.x_h_6_attn_Mul_outpu = Vars.x_h_6_attn_Transpose .* Vars.x_h_6_attn_Sqrt_1_ou;
NumDims.x_h_6_attn_Mul_outpu = max(NumDims.x_h_6_attn_Transpose, NumDims.x_h_6_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_6_attn_Sqrt_2_ou = sqrt(Vars.x_h_6_attn_Cast_1_ou);
NumDims.x_h_6_attn_Sqrt_2_ou = NumDims.x_h_6_attn_Cast_1_ou;

% Mul:
Vars.x_h_6_attn_Mul_1_out = Vars.x_h_6_attn_Transpo_2 .* Vars.x_h_6_attn_Sqrt_2_ou;
NumDims.x_h_6_attn_Mul_1_out = max(NumDims.x_h_6_attn_Transpo_2, NumDims.x_h_6_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_6_attn_MatMul_ou, NumDims.x_h_6_attn_MatMul_ou] = onnxMatMul(Vars.x_h_6_attn_Mul_outpu, Vars.x_h_6_attn_Mul_1_out, NumDims.x_h_6_attn_Mul_outpu, NumDims.x_h_6_attn_Mul_1_out);

% Add:
Vars.x_h_6_attn_Add_outpu = Vars.x_h_6_attn_MatMul_ou + Vars.x_h_6_attn_Where_out;
NumDims.x_h_6_attn_Add_outpu = max(NumDims.x_h_6_attn_MatMul_ou, NumDims.x_h_6_attn_Where_out);

% Softmax:
[Vars.x_h_6_attn_Softmax_o, NumDims.x_h_6_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_6_attn_Add_outpu, -1, NumDims.x_h_6_attn_Add_outpu);

% MatMul:
[Vars.x_h_6_attn_MatMul_1_, NumDims.x_h_6_attn_MatMul_1_] = onnxMatMul(Vars.x_h_6_attn_Softmax_o, Vars.x_h_6_attn_Transpo_1, NumDims.x_h_6_attn_Softmax_o, NumDims.x_h_6_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_6_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1035, NumDims.x_h_6_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_6_attn_Transpo_3 = permute(Vars.x_h_6_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_6_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_6_attn_Transpo_3, Vars.x_h_6_attn_Consta_3, NumDims.x_h_6_attn_Transpo_3, 0);
Vars.x_h_6_attn_Reshape_3 = reshape(Vars.x_h_6_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_6_attn_c_proj_Ma, NumDims.x_h_6_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_6_attn_Reshape_3, Vars.onnx__MatMul_1066, NumDims.x_h_6_attn_Reshape_3, NumDims.onnx__MatMul_1066);

% Add:
Vars.x_h_6_Add_output_0 = Vars.x_h_5_Add_1_output_0 + Vars.x_h_6_attn_c_proj_Ma;
NumDims.x_h_6_Add_output_0 = max(NumDims.x_h_5_Add_1_output_0, NumDims.x_h_6_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_6_ln_2_LayerNorm, NumDims.x_h_6_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_6_Add_output_0, Vars.transformer_h_6_ln_2, Vars.x_h_6_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_6_Add_output_0);

% MatMul:
[Vars.x_h_6_mlp_c_fc_MatMu, NumDims.x_h_6_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_6_ln_2_LayerNorm, Vars.onnx__MatMul_1067, NumDims.x_h_6_ln_2_LayerNorm, NumDims.onnx__MatMul_1067);

% Div:
Vars.x_h_6_mlp_gelu_Div_o = Vars.x_h_6_mlp_c_fc_MatMu ./ Vars.x_h_6_mlp_gelu_Con_2;
NumDims.x_h_6_mlp_gelu_Div_o = max(NumDims.x_h_6_mlp_c_fc_MatMu, NumDims.x_h_6_mlp_gelu_Con_2);

% Erf:
Vars.x_h_6_mlp_gelu_Erf_o = erf(Vars.x_h_6_mlp_gelu_Div_o);
NumDims.x_h_6_mlp_gelu_Erf_o = NumDims.x_h_6_mlp_gelu_Div_o;

% Add:
Vars.x_h_6_mlp_gelu_Add_o = Vars.x_h_6_mlp_gelu_Erf_o + Vars.x_h_6_mlp_gelu_Const;
NumDims.x_h_6_mlp_gelu_Add_o = max(NumDims.x_h_6_mlp_gelu_Erf_o, NumDims.x_h_6_mlp_gelu_Const);

% Mul:
Vars.x_h_6_mlp_gelu_Mul_o = Vars.x_h_6_mlp_c_fc_MatMu .* Vars.x_h_6_mlp_gelu_Add_o;
NumDims.x_h_6_mlp_gelu_Mul_o = max(NumDims.x_h_6_mlp_c_fc_MatMu, NumDims.x_h_6_mlp_gelu_Add_o);

% Mul:
Vars.x_h_6_mlp_gelu_Mul_1 = Vars.x_h_6_mlp_gelu_Mul_o .* Vars.x_h_6_mlp_gelu_Con_1;
NumDims.x_h_6_mlp_gelu_Mul_1 = max(NumDims.x_h_6_mlp_gelu_Mul_o, NumDims.x_h_6_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_6_mlp_c_proj_Mat, NumDims.x_h_6_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_6_mlp_gelu_Mul_1, Vars.onnx__MatMul_1068, NumDims.x_h_6_mlp_gelu_Mul_1, NumDims.onnx__MatMul_1068);

% Add:
Vars.x_h_6_Add_1_output_0 = Vars.x_h_6_Add_output_0 + Vars.x_h_6_mlp_c_proj_Mat;
NumDims.x_h_6_Add_1_output_0 = max(NumDims.x_h_6_Add_output_0, NumDims.x_h_6_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_h_7_ln_1_LayerNorm, NumDims.x_h_7_ln_1_LayerNorm] = onnxLayerNormalization(Vars.x_h_6_Add_1_output_0, Vars.transformer_h_7_ln_1, Vars.x_h_7_ln_1_Constant_, -1, 1.000000e-05, NumDims.x_h_6_Add_1_output_0);

% MatMul:
[Vars.x_h_7_attn_c_attn_Ma, NumDims.x_h_7_attn_c_attn_Ma] = onnxMatMul(Vars.x_h_7_ln_1_LayerNorm, Vars.onnx__MatMul_1069, NumDims.x_h_7_ln_1_LayerNorm, NumDims.onnx__MatMul_1069);

% Split:
[Vars.x_h_7_attn_Split_out, Vars.x_h_7_attn_Split_o_1, Vars.x_h_7_attn_Split_o_2, NumDims.x_h_7_attn_Split_out, NumDims.x_h_7_attn_Split_o_1, NumDims.x_h_7_attn_Split_o_2] = onnxSplit13(Vars.x_h_7_attn_c_attn_Ma, 2, Vars.x_h_7_attn_Consta_13, 3, NumDims.x_h_7_attn_c_attn_Ma);

% Reshape:
[shape, NumDims.x_h_7_attn_Reshape_o] = prepareReshapeArgs(Vars.x_h_7_attn_Split_o_1, Vars.x_h_7_attn_Consta_4, NumDims.x_h_7_attn_Split_o_1, 0);
Vars.x_h_7_attn_Reshape_o = reshape(Vars.x_h_7_attn_Split_o_1, shape{:});

% Transpose:
[perm, NumDims.x_h_7_attn_Transpo_4] = prepareTransposeArgs(Vars.TransposePerm1036, NumDims.x_h_7_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_7_attn_Transpo_4 = permute(Vars.x_h_7_attn_Reshape_o, perm);
end

% Reshape:
[shape, NumDims.x_h_7_attn_Reshape_1] = prepareReshapeArgs(Vars.x_h_7_attn_Split_out, Vars.x_h_7_attn_Consta_5, NumDims.x_h_7_attn_Split_out, 0);
Vars.x_h_7_attn_Reshape_1 = reshape(Vars.x_h_7_attn_Split_out, shape{:});

% Transpose:
[perm, NumDims.x_h_7_attn_Transpose] = prepareTransposeArgs(Vars.TransposePerm1037, NumDims.x_h_7_attn_Reshape_1);
if ~isempty(perm)
    Vars.x_h_7_attn_Transpose = permute(Vars.x_h_7_attn_Reshape_1, perm);
end

% Reshape:
[shape, NumDims.x_h_7_attn_Reshape_2] = prepareReshapeArgs(Vars.x_h_7_attn_Split_o_2, Vars.x_h_7_attn_Consta_6, NumDims.x_h_7_attn_Split_o_2, 0);
Vars.x_h_7_attn_Reshape_2 = reshape(Vars.x_h_7_attn_Split_o_2, shape{:});

% Transpose:
[perm, NumDims.x_h_7_attn_Transpo_1] = prepareTransposeArgs(Vars.TransposePerm1038, NumDims.x_h_7_attn_Reshape_2);
if ~isempty(perm)
    Vars.x_h_7_attn_Transpo_1 = permute(Vars.x_h_7_attn_Reshape_2, perm);
end

% Shape:
[Vars.x_h_7_attn_Shape_out, NumDims.x_h_7_attn_Shape_out] = onnxShape(Vars.x_h_7_attn_Transpose, NumDims.x_h_7_attn_Transpose, 0, NumDims.x_h_7_attn_Transpose+1);

% Slice:
[Indices, NumDims.x_h_7_attn_Slice_out] = prepareSliceArgs(Vars.x_h_7_attn_Shape_out, Vars.x_h_7_attn_Consta_7, Vars.x_h_7_attn_Consta_8, '', '', NumDims.x_h_7_attn_Shape_out);
Vars.x_h_7_attn_Slice_out = subsref(Vars.x_h_7_attn_Shape_out, Indices);

% Cast:
if islogical(Vars.x_h_7_attn_Slice_out)
    Vars.x_h_7_attn_Slice_out = single(Vars.x_h_7_attn_Slice_out);
end
Vars.x_h_7_attn_Cast_outp = single(Vars.x_h_7_attn_Slice_out);
NumDims.x_h_7_attn_Cast_outp = NumDims.x_h_7_attn_Slice_out;

% Sqrt:
Vars.x_h_7_attn_Sqrt_outp = sqrt(Vars.x_h_7_attn_Cast_outp);
NumDims.x_h_7_attn_Sqrt_outp = NumDims.x_h_7_attn_Cast_outp;

% Div:
Vars.x_h_7_attn_Div_outpu = Vars.x_h_7_attn_Consta_9 ./ Vars.x_h_7_attn_Sqrt_outp;
NumDims.x_h_7_attn_Div_outpu = max(NumDims.x_h_7_attn_Consta_9, NumDims.x_h_7_attn_Sqrt_outp);

% Cast:
if islogical(Vars.x_h_7_attn_Div_outpu)
    Vars.x_h_7_attn_Div_outpu = single(Vars.x_h_7_attn_Div_outpu);
end
Vars.x_h_7_attn_Cast_1_ou = single(Vars.x_h_7_attn_Div_outpu);
NumDims.x_h_7_attn_Cast_1_ou = NumDims.x_h_7_attn_Div_outpu;

% Shape:
[Vars.x_h_7_attn_Shape_1_o, NumDims.x_h_7_attn_Shape_1_o] = onnxShape(Vars.x_h_7_attn_Transpose, NumDims.x_h_7_attn_Transpose, 0, NumDims.x_h_7_attn_Transpose+1);

% Shape:
[Vars.x_h_7_attn_Shape_2_o, NumDims.x_h_7_attn_Shape_2_o] = onnxShape(Vars.x_h_7_attn_Transpo_4, NumDims.x_h_7_attn_Transpo_4, 0, NumDims.x_h_7_attn_Transpo_4+1);

% Slice:
[Indices, NumDims.x_h_7_attn_Slice_1_o] = prepareSliceArgs(Vars.x_h_7_attn_Shape_1_o, Vars.x_h_7_attn_Consta_11, Vars.x_h_7_attn_Consta_10, '', '', NumDims.x_h_7_attn_Shape_1_o);
Vars.x_h_7_attn_Slice_1_o = subsref(Vars.x_h_7_attn_Shape_1_o, Indices);

% Slice:
[Indices, NumDims.x_h_7_attn_Slice_2_o] = prepareSliceArgs(Vars.x_h_7_attn_Shape_2_o, Vars.x_h_7_attn_Consta_11, Vars.x_h_7_attn_Consta_10, '', '', NumDims.x_h_7_attn_Shape_2_o);
Vars.x_h_7_attn_Slice_2_o = subsref(Vars.x_h_7_attn_Shape_2_o, Indices);

% Concat:
[Vars.x_h_7_attn_Concat_ou, NumDims.x_h_7_attn_Concat_ou] = onnxConcat(0, {Vars.x_h_7_attn_Slice_1_o, Vars.x_h_7_attn_Slice_2_o}, [NumDims.x_h_7_attn_Slice_1_o, NumDims.x_h_7_attn_Slice_2_o]);

% Expand:
[shape, NumDims.x_h_7_attn_Expand_ou] = prepareExpandArgs(Vars.x_h_7_attn_Concat_ou);
Vars.x_h_7_attn_Expand_ou = Vars.x_h_7_attn_Consta_12 + zeros(shape);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Trilu):
[Vars.x_h_7_attn_Trilu_out, NumDims.x_h_7_attn_Trilu_out] = PLACEHOLDER(Vars.x_h_7_attn_Expand_ou);

% Equal:
Vars.x_h_7_attn_Equal_out = Vars.x_h_7_attn_Trilu_out == Vars.x_h_7_attn_Constant_;
NumDims.x_h_7_attn_Equal_out = max(NumDims.x_h_7_attn_Trilu_out, NumDims.x_h_7_attn_Constant_);

% Where:
[Vars.x_h_7_attn_Where_out, NumDims.x_h_7_attn_Where_out] = onnxWhere(Vars.x_h_7_attn_Equal_out, Vars.x_h_7_attn_Consta_1, Vars.x_h_7_attn_Consta_2, NumDims.x_h_7_attn_Equal_out, NumDims.x_h_7_attn_Consta_1, NumDims.x_h_7_attn_Consta_2);

% Transpose:
[perm, NumDims.x_h_7_attn_Transpo_2] = prepareTransposeArgs(Vars.TransposePerm1039, NumDims.x_h_7_attn_Reshape_o);
if ~isempty(perm)
    Vars.x_h_7_attn_Transpo_2 = permute(Vars.x_h_7_attn_Reshape_o, perm);
end

% Sqrt:
Vars.x_h_7_attn_Sqrt_1_ou = sqrt(Vars.x_h_7_attn_Cast_1_ou);
NumDims.x_h_7_attn_Sqrt_1_ou = NumDims.x_h_7_attn_Cast_1_ou;

% Mul:
Vars.x_h_7_attn_Mul_outpu = Vars.x_h_7_attn_Transpose .* Vars.x_h_7_attn_Sqrt_1_ou;
NumDims.x_h_7_attn_Mul_outpu = max(NumDims.x_h_7_attn_Transpose, NumDims.x_h_7_attn_Sqrt_1_ou);

% Sqrt:
Vars.x_h_7_attn_Sqrt_2_ou = sqrt(Vars.x_h_7_attn_Cast_1_ou);
NumDims.x_h_7_attn_Sqrt_2_ou = NumDims.x_h_7_attn_Cast_1_ou;

% Mul:
Vars.x_h_7_attn_Mul_1_out = Vars.x_h_7_attn_Transpo_2 .* Vars.x_h_7_attn_Sqrt_2_ou;
NumDims.x_h_7_attn_Mul_1_out = max(NumDims.x_h_7_attn_Transpo_2, NumDims.x_h_7_attn_Sqrt_2_ou);

% MatMul:
[Vars.x_h_7_attn_MatMul_ou, NumDims.x_h_7_attn_MatMul_ou] = onnxMatMul(Vars.x_h_7_attn_Mul_outpu, Vars.x_h_7_attn_Mul_1_out, NumDims.x_h_7_attn_Mul_outpu, NumDims.x_h_7_attn_Mul_1_out);

% Add:
Vars.x_h_7_attn_Add_outpu = Vars.x_h_7_attn_MatMul_ou + Vars.x_h_7_attn_Where_out;
NumDims.x_h_7_attn_Add_outpu = max(NumDims.x_h_7_attn_MatMul_ou, NumDims.x_h_7_attn_Where_out);

% Softmax:
[Vars.x_h_7_attn_Softmax_o, NumDims.x_h_7_attn_Softmax_o] = onnxSoftmax13(Vars.x_h_7_attn_Add_outpu, -1, NumDims.x_h_7_attn_Add_outpu);

% MatMul:
[Vars.x_h_7_attn_MatMul_1_, NumDims.x_h_7_attn_MatMul_1_] = onnxMatMul(Vars.x_h_7_attn_Softmax_o, Vars.x_h_7_attn_Transpo_1, NumDims.x_h_7_attn_Softmax_o, NumDims.x_h_7_attn_Transpo_1);

% Transpose:
[perm, NumDims.x_h_7_attn_Transpo_3] = prepareTransposeArgs(Vars.TransposePerm1040, NumDims.x_h_7_attn_MatMul_1_);
if ~isempty(perm)
    Vars.x_h_7_attn_Transpo_3 = permute(Vars.x_h_7_attn_MatMul_1_, perm);
end

% Reshape:
[shape, NumDims.x_h_7_attn_Reshape_3] = prepareReshapeArgs(Vars.x_h_7_attn_Transpo_3, Vars.x_h_7_attn_Consta_3, NumDims.x_h_7_attn_Transpo_3, 0);
Vars.x_h_7_attn_Reshape_3 = reshape(Vars.x_h_7_attn_Transpo_3, shape{:});

% MatMul:
[Vars.x_h_7_attn_c_proj_Ma, NumDims.x_h_7_attn_c_proj_Ma] = onnxMatMul(Vars.x_h_7_attn_Reshape_3, Vars.onnx__MatMul_1089, NumDims.x_h_7_attn_Reshape_3, NumDims.onnx__MatMul_1089);

% Add:
Vars.x_h_7_Add_output_0 = Vars.x_h_6_Add_1_output_0 + Vars.x_h_7_attn_c_proj_Ma;
NumDims.x_h_7_Add_output_0 = max(NumDims.x_h_6_Add_1_output_0, NumDims.x_h_7_attn_c_proj_Ma);

% LayerNormalization:
[Vars.x_h_7_ln_2_LayerNorm, NumDims.x_h_7_ln_2_LayerNorm] = onnxLayerNormalization(Vars.x_h_7_Add_output_0, Vars.transformer_h_7_ln_2, Vars.x_h_7_ln_2_Constant_, -1, 1.000000e-05, NumDims.x_h_7_Add_output_0);

% MatMul:
[Vars.x_h_7_mlp_c_fc_MatMu, NumDims.x_h_7_mlp_c_fc_MatMu] = onnxMatMul(Vars.x_h_7_ln_2_LayerNorm, Vars.onnx__MatMul_1090, NumDims.x_h_7_ln_2_LayerNorm, NumDims.onnx__MatMul_1090);

% Div:
Vars.x_h_7_mlp_gelu_Div_o = Vars.x_h_7_mlp_c_fc_MatMu ./ Vars.x_h_7_mlp_gelu_Con_2;
NumDims.x_h_7_mlp_gelu_Div_o = max(NumDims.x_h_7_mlp_c_fc_MatMu, NumDims.x_h_7_mlp_gelu_Con_2);

% Erf:
Vars.x_h_7_mlp_gelu_Erf_o = erf(Vars.x_h_7_mlp_gelu_Div_o);
NumDims.x_h_7_mlp_gelu_Erf_o = NumDims.x_h_7_mlp_gelu_Div_o;

% Add:
Vars.x_h_7_mlp_gelu_Add_o = Vars.x_h_7_mlp_gelu_Erf_o + Vars.x_h_7_mlp_gelu_Const;
NumDims.x_h_7_mlp_gelu_Add_o = max(NumDims.x_h_7_mlp_gelu_Erf_o, NumDims.x_h_7_mlp_gelu_Const);

% Mul:
Vars.x_h_7_mlp_gelu_Mul_o = Vars.x_h_7_mlp_c_fc_MatMu .* Vars.x_h_7_mlp_gelu_Add_o;
NumDims.x_h_7_mlp_gelu_Mul_o = max(NumDims.x_h_7_mlp_c_fc_MatMu, NumDims.x_h_7_mlp_gelu_Add_o);

% Mul:
Vars.x_h_7_mlp_gelu_Mul_1 = Vars.x_h_7_mlp_gelu_Mul_o .* Vars.x_h_7_mlp_gelu_Con_1;
NumDims.x_h_7_mlp_gelu_Mul_1 = max(NumDims.x_h_7_mlp_gelu_Mul_o, NumDims.x_h_7_mlp_gelu_Con_1);

% MatMul:
[Vars.x_h_7_mlp_c_proj_Mat, NumDims.x_h_7_mlp_c_proj_Mat] = onnxMatMul(Vars.x_h_7_mlp_gelu_Mul_1, Vars.onnx__MatMul_1091, NumDims.x_h_7_mlp_gelu_Mul_1, NumDims.onnx__MatMul_1091);

% Add:
Vars.x_h_7_Add_1_output_0 = Vars.x_h_7_Add_output_0 + Vars.x_h_7_mlp_c_proj_Mat;
NumDims.x_h_7_Add_1_output_0 = max(NumDims.x_h_7_Add_output_0, NumDims.x_h_7_mlp_c_proj_Mat);

% LayerNormalization:
[Vars.x_ln_f_LayerNormaliz, NumDims.x_ln_f_LayerNormaliz] = onnxLayerNormalization(Vars.x_h_7_Add_1_output_0, Vars.transformer_ln_f_wei, Vars.x_ln_f_Constant_outp, -1, 1.000000e-05, NumDims.x_h_7_Add_1_output_0);

% MatMul:
[Vars.x_lm_head_MatMul_out, NumDims.x_lm_head_MatMul_out] = onnxMatMul(Vars.x_ln_f_LayerNormaliz, Vars.onnx__MatMul_1092, NumDims.x_ln_f_LayerNormaliz, NumDims.onnx__MatMul_1092);

% Add:
Vars.output = Vars.lm_head_bias + Vars.x_lm_head_MatMul_out;
NumDims.output = max(NumDims.lm_head_bias, NumDims.x_lm_head_MatMul_out);

% Set graph output arguments from Vars and NumDims:
output = Vars.output;
outputNumDims1042 = NumDims.output;
% Set output state from Vars:
state = updateStruct(state, Vars);
end

function [inputDataPerms, outputDataPerms, Training] = parseInputs(input, numDataOutputs, params, varargin)
% Function to validate inputs to Gather_To_AddFcn:
p = inputParser;
isValidArrayInput = @(x)isnumeric(x) || isstring(x);
isValidONNXParameters = @(x)isa(x, 'ONNXParameters');
addRequired(p, 'input', isValidArrayInput);
addRequired(p, 'params', isValidONNXParameters);
addParameter(p, 'InputDataPermutation', 'auto');
addParameter(p, 'OutputDataPermutation', 'auto');
addParameter(p, 'Training', false);
parse(p, input, params, varargin{:});
inputDataPerms = p.Results.InputDataPermutation;
outputDataPerms = p.Results.OutputDataPermutation;
Training = p.Results.Training;
if isnumeric(inputDataPerms)
    inputDataPerms = {inputDataPerms};
end
if isstring(inputDataPerms) && isscalar(inputDataPerms) || ischar(inputDataPerms)
    inputDataPerms = repmat({inputDataPerms},1,1);
end
if isnumeric(outputDataPerms)
    outputDataPerms = {outputDataPerms};
end
if isstring(outputDataPerms) && isscalar(outputDataPerms) || ischar(outputDataPerms)
    outputDataPerms = repmat({outputDataPerms},1,numDataOutputs);
end
end

function [input, Training, outputDataPerms, anyDlarrayInputs] = preprocessInput(input, params, varargin)
% Parse input arguments
[inputDataPerms, outputDataPerms, Training] = parseInputs(input, 1, params, varargin{:});
anyDlarrayInputs = any(cellfun(@(x)isa(x, 'dlarray'), {input}));
% Make the input variables into unlabelled dlarrays:
input = makeUnlabeledDlarray(input);
% Permute inputs if requested:
input = permuteInputVar(input, inputDataPerms{1}, 3);
end

function [output] = postprocessOutput(output, outputDataPerms, anyDlarrayInputs, Training, varargin)
% Set output type:
if ~anyDlarrayInputs && ~Training
    if isdlarray(output)
        output = extractdata(output);
    end
end
% Permute outputs if requested:
output = permuteOutputVar(output, outputDataPerms{1}, 3);
end


%% dlarray functions implementing ONNX operators:

function [Y, numDimsY] = onnxConcat(ONNXAxis, XCell, numDimsXArray)
% Concatentation that treats all empties the same. Necessary because
% dlarray.cat does not allow, for example, cat(1, 1x1, 1x0) because the
% second dimension sizes do not match.

% Copyright 2021 The MathWorks, Inc.

numDimsY = numDimsXArray(1);
XCell(cellfun(@isempty, XCell)) = [];
if isempty(XCell)
    Y = dlarray([]);
else
    if ONNXAxis<0
        ONNXAxis = ONNXAxis + numDimsY;
    end
    DLTAxis = numDimsY - ONNXAxis;
    Y = cat(DLTAxis, XCell{:});
end
end

function [Y, numDimsY] = onnxGather(X, ONNXIdx, ONNXAxis, numDimsX, numDimsIdx)
% Function implementing the ONNX Gather operator

% In ONNX, 'Gather' first indexes into dimension ONNXAxis of data, using
% the contents of ONNXIdx as the indices. Then, it reshapes the ONNXAxis
% into the shape of ONNXIdx.
%   Example 1:
% Suppose data has shape [2 3 4 5], ONNXIdx has shape [6 7], and axis=1.
% The result has shape [2 6 7 4 5].
%   Example 2:
% Suppose data has shape [2 3 4 5], ONNXIdx has shape [6], and axis=1.
% The result has shape [2 6 4 5].
%   Example 3:
% Suppose data has shape [2 3 4 5], ONNXIdx has shape [] (a scalar), and axis=1.
% The result has shape [2 4 5].
%
% Since we're using reverse indexing relative to ONNX, in this function
% data and ONNXIdx both have reversed dimension ordering.

% Copyright 2020-2021 The MathWorks, Inc.

numDimsY = numDimsIdx + (numDimsX - 1);
if isempty(X)
    Y = X;
    return;
end
% (1) First, do the subsref part of Gather
if ONNXAxis<0
    ONNXAxis = ONNXAxis + numDimsX;                                 % Axis can be negative. Convert it to its positive equivalent.
end
dltAxis = numDimsX - ONNXAxis;                                      % Convert axis to DLT. ONNXAxis is origin 0 and we index from the end
ONNXIdx(ONNXIdx<0) = ONNXIdx(ONNXIdx<0) + size(X, dltAxis);         % ONNXIdx can have negative components. Make them positive.
dltIdx  = extractdata(ONNXIdx) + 1;                                 % ONNXIdx is origin-0 in ONNX, so add 1 to get dltIdx
% Use subsref to index into data
Indices.subs = repmat({':'}, 1, numDimsX);
Indices.subs{dltAxis} = dltIdx(:);                                  % Index as a column to ensure the output is 1-D in the indexed dimension (for now).
Indices.type = '()';
Y = subsref(X, Indices);
% (2) Now do the reshaping part of Gather
shape = size(Y, 1:numDimsX);
if numDimsIdx == 0
    % Delete the indexed dimension
    shape(dltAxis) = [];
elseif numDimsIdx > 1
    % Reshape the indexed dimension into the shape of ONNXIdx
    shape = [shape(1:dltAxis-1) size(ONNXIdx, 1:numDimsIdx) shape(dltAxis+1:end)];
end
% Extend the shape to 2D so it's valid MATLAB
if numel(shape) < 2
    shape = [shape ones(1,2-numel(shape))];
end
Y = reshape(Y, shape);
end

function [Y, numDimsY] = onnxLayerNormalization(X, scale, offset, axis, epsilon, numDimsX)
%onnxLayerNormalization Normalizes mini-batches of data over the normalized shape.

%Copyright 2024 The MathWorks, Inc.

% axis is in forward ONNX format. Convert axis to reverse ONNX format.
if (axis<0)
    axis = -axis;
else
    axis = numDimsX - axis;
end

epsilon = double(epsilon);
% Epsilon must be non-negative.
if(epsilon <= 0)
    warning(message("nnet_cnn_onnx:onnx:BadEpsilon"));
    epsilon = 1e-5;
end

dimension = 1: axis;

% stage 1: standarization
XMean = mean(X, dimension);
D = X - XMean;
DD = D.^2;
XVar = mean(DD, dimension);
XVarEps = XVar + epsilon;
StdDev = sqrt(XVarEps);
InvStdDev  = 1 ./ StdDev;
Normalized = D .* InvStdDev;

% stage 2
NormalizedScaled = Normalized .* scale;
if ~isempty(offset)
    Y = NormalizedScaled + offset;
else
    Y = NormalizedScaled;
end
numDimsY = numDimsX;
end
function [D, numDimsD] = onnxMatMul(A, B, numDimsA, numDimsB)
% Implements the ONNX MatMul operator.

% Copyright 2020-2023 The MathWorks, Inc.

% If B is 1-D, temporarily extend it to a row vector
if numDimsB==1
    B = B(:)';
end
maxNumDims = max(numDimsA, numDimsB);
numDimsD = maxNumDims;
if maxNumDims > 2
    % Removes dlarray formats if only one of the input dlarrays is formatted.
    if sum([isempty(dims(A)), isempty(dims(B))]) == 1
        D = pagemtimes(stripdims(B), stripdims(A));
    else
        %computes matrix product of corresponding pages of input arrays A and
        %B.
        D = pagemtimes(B, A);
    end
else
    D = B * A;
    if numDimsA==1 || numDimsB==1
        D = D(:);
        numDimsD = 1;
    end
end
end

function [Y, numDimsY] = onnxShape(X, numDimsX, startAxis, endAxis)
% Implements the ONNX Shape operator
% Return the reverse ONNX shape as a 1D column vector

% Copyright 2020-2024 The MathWorks, Inc.

switch numDimsX
    case 0
        if isempty(X)
            Y = dlarray(0);
        else
            Y = dlarray(1);
        end
    case 1
        if isempty(X)
            Y = dlarray(0);
        else
            Y = dlarray(size(X,1));
        end
    otherwise
        if(endAxis<0)
            %  If the endAxis is smaller than 0 after converting it positive,
            % the endAxis is 0
            endAxis = max(0, numDimsX + endAxis);
        end
        if(startAxis<0)
            %  If the startAxis is smaller than 0 after converting it positive,
            % the startAxis is 0
            startAxis = max(0, numDimsX + startAxis);
        end
        % transform startAxis and endAxis from 0 index to 1 index
        startAxis = startAxis + 1;
        endAxis = endAxis + 1;
        % if startAxis is larger than numDimsX or endAxis is larger than
        % numDimsX + 1, cramp it to the upper bound. The endAxis is exclusive,
        % transform it to MATLAB inclusive way
        endAxis = min(endAxis, numDimsX + 1) - 1;
        startAxis = min(startAxis, numDimsX);
        if endAxis < startAxis || endAxis == 0
            Y = dlarray(0);
        else
            Y = dlarray(fliplr(size(X, (numDimsX-endAxis+1):(numDimsX-startAxis+1)))');
        end
end
numDimsY = 1;
end

function [Y, numDimsY] = onnxSoftmax13(X, ONNXaxis, numDimsX)
% Implements the ONNX Softmax function:
% Softmax(input, axis) = Exp(input) / ReduceSum(Exp(input), axis=axis, keepdims=1)
% The input is constrained to floating point types.

% Copyright 2021 The MathWorks, Inc.

if ONNXaxis < 0
    ONNXaxis = ONNXaxis + numDimsX;
end
DLTaxis = numDimsX - ONNXaxis;

X = X - max(X, [], DLTaxis); % Subtract max(X) for numerical stability
expX = exp(X);
dims = prepareReduceArgs(ONNXaxis, numDimsX);
Y = expX ./ sum(expX, dims);
numDimsY = numDimsX;

end
function varargout = onnxSplit13(X, ONNXaxis, splits, numSplits, numDimsX)
% Implements the ONNX Split operator

% Copyright 2021-2024 The MathWorks, Inc.

% ONNXaxis is origin 0. splits is a vector of the lengths of each segment.
% If splits is empty, instead split into segments of equal length.
if ONNXaxis<0
    ONNXaxis = ONNXaxis + numDimsX;
end
DLTAxis = numDimsX - ONNXaxis;
if isempty(splits)
    C       = size(X, DLTAxis);
    sz      = floor(C/numSplits);
    splits	= repmat(sz, 1, numSplits);
else
    splits = extractdata(splits);
end
S      = struct;
S.type = '()';
S.subs = repmat({':'}, 1, numDimsX);        % Important to use numDimsX. ndims(X) may be too small.
splitIndices = [0 cumsum(splits(:)')];
numY = numel(splitIndices)-1;
for i = 1:numY
    from            = splitIndices(i) + 1;
    to              = splitIndices(i+1);
    S.subs{DLTAxis}	= from:to;
    % The first numY outputs are the Y's. The second numY outputs are their
    % numDims. We assume all the outputs of Split have the same numDims as
    % the input.
    varargout{i}        = subsref(X, S);
    varargout{i + numY} = numDimsX;
end
end

function [output, numDimsOutput] = onnxWhere(condition, X, Y, numDimsCondition, numDimsX, numDimsY)

% Copyright 2020 The MathWorks, Inc.

bigz = zeros(size(condition + X + Y));      % broadcast
condition = condition + bigz;
X = X + bigz;
output = Y + bigz;
output(condition==1) = X(condition==1);
numDimsOutput = max([numDimsCondition, numDimsX, numDimsY]);
end

function [shape, numDimsY] = prepareExpandArgs(ONNXShape)
% Prepares arguments for implementing the ONNX Expand operator

%   Copyright 2020 The MathWorks, Inc.

% Broadcast X to ONNXShape. The shape of X must be compatible with ONNXShape.
ONNXShape = extractdata(ONNXShape);
shape = fliplr(ONNXShape(:)');
if numel(shape) < 2
    shape = [shape ones(1, 2-numel(shape))];
end
numDimsY = numel(ONNXShape);
end

function dims = prepareReduceArgs(ONNXAxes, numDimsX)
% Prepares arguments for implementing the ONNX Reduce operator

%   Copyright 2020 The MathWorks, Inc.

if isempty(ONNXAxes)
    ONNXAxes = 0:numDimsX-1;   % All axes
end
ONNXAxes(ONNXAxes<0) = ONNXAxes(ONNXAxes<0) + numDimsX;
dims = numDimsX - ONNXAxes;
end

function [DLTShape, numDimsY] = prepareReshapeArgs(X, ONNXShape, numDimsX, allowzero)
% Prepares arguments for implementing the ONNX Reshape operator

%   Copyright 2020-2024 The MathWorks, Inc.

ONNXShape = flip(extractdata(ONNXShape));            % First flip the shape to make it correspond to the dimensions of X.
% In ONNX, 0 means "unchanged" if allowzero is false, and -1 means "infer". In DLT, there is no
% "unchanged", and [] means "infer".
DLTShape = num2cell(ONNXShape);                      % Make a cell array so we can include [].
% Replace zeros with the actual size if allowzero is false
if any(ONNXShape==0) && allowzero==0
    i0 = find(ONNXShape==0);
    DLTShape(i0) = num2cell(size(X, numDimsX - numel(ONNXShape) + i0));  % right-align the shape vector and dims
end
if any(ONNXShape == -1)
    % Replace -1 with []
    i = ONNXShape == -1;
    DLTShape{i} = [];
end
if numel(DLTShape)==1
    DLTShape = [DLTShape 1];
end
numDimsY = numel(ONNXShape);
end

function [S, numDimsY] = prepareSliceArgs(X, Starts, Ends, Axes, Steps, numDimsX)
% Prepares arguments for implementing the ONNX Slice operator

%   Copyright 2020 The MathWorks, Inc.

% Starts, Ends and Axes are all origin 0. Axes refer to the ONNX dimension
% ordering, but X uses the reverse, DLT ordering. Starts, Ends, Axes, and
% Steps correspond positionally. Axes and Steps may be omitted, with
% defaults described in the ONNX spec.

% Set default Axes and Steps if not supplied
if isempty(Axes)
    Axes = 0:numDimsX-1;   % All axes
end
Axes(Axes<0) = Axes(Axes<0) + numDimsX; % Handle negative Axes.
if isempty(Steps)
    Steps = ones(1, numel(Starts));
end
% Init all dims to :
S.subs = repmat({':'}, 1, numDimsX);
S.type = '()';
% Set Starts and Ends for each axis
for i = 1:numel(Axes)
    DLTDim = numDimsX - Axes(i);                                               % The DLT dim is the reverse of the ONNX dim.
    % "If a negative value is passed for any of the start or end indices,
    % it represents number of elements before the end of that dimension."
    if Starts(i) < 0
        Starts(i) = size(X,DLTDim) + Starts(i);
    end
    if Ends(i) < 0
        Ends(i) = max(-1, size(X,DLTDim) + Ends(i));                        % The -1 case is when we're slicing backward and want to include 0.
    end
    % "If the value passed to start or end is larger than the n (the number
    % of elements in this dimension), it represents n."
    if Starts(i) > size(X,DLTDim)
        Starts(i) = size(X,DLTDim);
    end
    if Ends(i) > size(X,DLTDim)
        Ends(i) = size(X,DLTDim);
    end
    if Steps(i) > 0
        S.subs{DLTDim} = 1 + (Starts(i) : Steps(i) : Ends(i)-1);            % 1 + (Origin 0 indexing with end index excluded)
    else
        S.subs{DLTDim} = 1 + (Starts(i) : Steps(i) : Ends(i)+1);            % 1 + (Origin 0 indexing with end index excluded)
    end
end
numDimsY = numDimsX;
end

function [perm, numDimsA] = prepareTransposeArgs(ONNXPerm, numDimsA)
% Prepares arguments for implementing the ONNX Transpose operator

%   Copyright 2020 The MathWorks, Inc.

if numDimsA <= 1        % Tensors of numDims 0 or 1 are unchanged by ONNX Transpose.
    perm = [];
else
    if isempty(ONNXPerm)        % Empty ONNXPerm means reverse the dimensions.
        perm = numDimsA:-1:1;
    else
        perm = numDimsA-flip(ONNXPerm);
    end
end
end

%% Utility functions:

function s = appendStructs(varargin)
% s = appendStructs(s1, s2,...). Assign all fields in s1, s2,... into s.

%   Copyright 2020 The MathWorks, Inc.

if isempty(varargin)
    s = struct;
else
    s = varargin{1};
    for i = 2:numel(varargin)
        fromstr = varargin{i};
        fs = fieldnames(fromstr);
        for j = 1:numel(fs)
            s.(fs{j}) = fromstr.(fs{j});
        end
    end
end
end

function checkInputSize(inputShape, expectedShape, inputName)

%   Copyright 2020-2021 The MathWorks, Inc.

if numel(expectedShape)==0
    % The input is a scalar
    if ~isequal(inputShape, [1 1])
        inputSizeStr = makeSizeString(inputShape);
        error(message('nnet_cnn_onnx:onnx:InputNeedsResize',inputName, "[1,1]", inputSizeStr));
    end
elseif numel(expectedShape)==1
    % The input is a vector
    if ~shapeIsColumnVector(inputShape) || ~iSizesMatch({inputShape(1)}, expectedShape)
        expectedShape{2} = 1;
        expectedSizeStr = makeSizeString(expectedShape);
        inputSizeStr = makeSizeString(inputShape);
        error(message('nnet_cnn_onnx:onnx:InputNeedsResize',inputName, expectedSizeStr, inputSizeStr));
    end
else
    % The input has 2 dimensions or more

    % The input dimensions have been reversed; flip them back to compare to the
    % expected ONNX shape.
    inputShape = fliplr(inputShape);

    % If the expected shape has fewer dims than the input shape, error.
    if numel(expectedShape) < numel(inputShape)
        expectedSizeStr = strjoin(["[", strjoin(string(expectedShape), ","), "]"], "");
        error(message('nnet_cnn_onnx:onnx:InputHasGreaterNDims', inputName, expectedSizeStr));
    end

    % Prepad the input shape with trailing ones up to the number of elements in
    % expectedShape
    inputShape = num2cell([ones(1, numel(expectedShape) - length(inputShape)) inputShape]);

    % Find the number of variable size dimensions in the expected shape
    numVariableInputs = sum(cellfun(@(x) isa(x, 'char') || isa(x, 'string'), expectedShape));

    % Find the number of input dimensions that are not in the expected shape
    % and cannot be represented by a variable dimension
    nonMatchingInputDims = setdiff(string(inputShape), string(expectedShape));
    numNonMatchingInputDims  = numel(nonMatchingInputDims) - numVariableInputs;

    expectedSizeStr = makeSizeString(expectedShape);
    inputSizeStr = makeSizeString(inputShape);
    if numNonMatchingInputDims == 0 && ~iSizesMatch(inputShape, expectedShape)
        % The actual and expected input dimensions match, but in
        % a different order. The input needs to be permuted.
        error(message('nnet_cnn_onnx:onnx:InputNeedsPermute',inputName, expectedSizeStr, inputSizeStr));
    elseif numNonMatchingInputDims > 0
        % The actual and expected input sizes do not match.
        error(message('nnet_cnn_onnx:onnx:InputNeedsResize',inputName, expectedSizeStr, inputSizeStr));
    end
end
end

function doesMatch = iSizesMatch(inputShape, expectedShape)
% Check whether the input and expected shapes match, in order.
% Size elements match if (1) the elements are equal, or (2) the expected
% size element is a variable (represented by a character vector or string)
doesMatch = true;
for i=1:numel(inputShape)
    if ~(isequal(inputShape{i},expectedShape{i}) || ischar(expectedShape{i}) || isstring(expectedShape{i}))
        doesMatch = false;
        return
    end
end
end

function sizeStr = makeSizeString(shape)
sizeStr = strjoin(["[", strjoin(string(shape), ","), "]"], "");
end

function isVec = shapeIsColumnVector(shape)
if numel(shape) == 2 && shape(2) == 1
    isVec = true;
else
    isVec = false;
end
end
function X = makeUnlabeledDlarray(X)
% Make numeric X into an unlabelled dlarray

%   Copyright 2020-2021 The MathWorks, Inc.

if isa(X, 'dlarray')
    X = stripdims(X);
elseif isnumeric(X)
    if isinteger(X)
        % Make ints double so they can combine with anything without
        % reducing precision
        X = double(X);
    end
    X = dlarray(X);
end
end

function [Vars, NumDims] = packageVariables(params, inputNames, inputValues, inputNumDims)

%   Copyright 2020 The MathWorks, Inc.

% inputNames, inputValues are cell arrays. inputRanks is a numeric vector.
Vars = appendStructs(params.Learnables, params.Nonlearnables, params.State);
NumDims = params.NumDimensions;
% Add graph inputs
for i = 1:numel(inputNames)
    Vars.(inputNames{i}) = inputValues{i};
    NumDims.(inputNames{i}) = inputNumDims(i);
end
end

function X = permuteInputVar(X, userDataPerm, onnxNDims)

%   Copyright 2020-2021 The MathWorks, Inc.
% Returns reverse-ONNX ordering
if onnxNDims == 0
    return;
elseif onnxNDims == 1 && isvector(X)
    X = X(:);
    return;
elseif isnumeric(userDataPerm)
    % Permute into reverse ONNX ordering
    if numel(userDataPerm) ~= onnxNDims
        error(message('nnet_cnn_onnx:onnx:InputPermutationSize', numel(userDataPerm), onnxNDims));
    end
    perm = fliplr(userDataPerm);
elseif isequal(userDataPerm, 'auto') && onnxNDims == 4
    % Permute MATLAB HWCN to reverse onnx (WHCN)
    perm = [2 1 3 4];
elseif isequal(userDataPerm, 'as-is')
    % Do not permute the input
    perm = 1:ndims(X);
else
    % userDataPerm is either 'none' or 'auto' with no default, which means
    % it's already in onnx ordering, so just make it reverse onnx
    perm = max(2,onnxNDims):-1:1;
end
X = permute(X, perm);
end

function Y = permuteOutputVar(Y, userDataPerm, onnxNDims)

%   Copyright 2020-2021 The MathWorks, Inc.
switch onnxNDims
    case 0
        perm = [];
    case 1
        if isnumeric(userDataPerm)
            % Use the user's permutation because Y is a column vector which
            % already matches ONNX.
            perm = userDataPerm;
        elseif isequal(userDataPerm, 'auto')
            % Treat the 1D onnx vector as a 2D column and transpose it
            perm = [2 1];
        else
            % userDataPerm is 'none'. Leave Y alone because it already
            % matches onnx.
            perm = [];
        end
    otherwise
        % ndims >= 2
        if isnumeric(userDataPerm)
            % Use the inverse of the user's permutation. This is not just the
            % flip of the permutation vector.
            perm = onnxNDims + 1 - userDataPerm;
        elseif isequal(userDataPerm, 'auto')
            if onnxNDims == 2
                % Permute reverse ONNX CN to DLT CN (do nothing)
                perm = [];
            elseif onnxNDims == 4
                % Permute reverse onnx (WHCN) to MATLAB HWCN
                perm = [2 1 3 4];
            else
                % User wants the output in ONNX ordering, so just reverse it from
                % reverse onnx
                perm = onnxNDims:-1:1;
            end
        elseif isequal(userDataPerm, 'as-is')
            % Do not permute the input
            perm = 1:ndims(Y);
        else
            % userDataPerm is 'none', so just make it reverse onnx
            perm = onnxNDims:-1:1;
        end
end
if ~isempty(perm)
    Y = permute(Y, perm);
end
end

function s = updateStruct(s, t)
% Set all existing fields in s from fields in t, ignoring extra fields in
% t.
%   Copyright 2020 The MathWorks, Inc.

for name = transpose(fieldnames(s))
    s.(name{1}) = t.(name{1});
end
end
