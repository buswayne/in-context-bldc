clear
close all


function_name = "fun_onnx";

params = importONNXFunction("C:\Users\39340\Documents\GitHub\in-context-bldc\speed_controller\test_controller.onnx", function_name);
input = zeros(1,10,8);

function_file = function_name + ".m";

f_id = fopen(function_file, "a+");

full_text = [newline, newline, 'function [Y, NumdimY] = PLACEHOLDER(X)', ...
             newline, '[rows_number, cols_number] = size(X);', ...
             newline, 'Y = 1:rows_number <= (1:cols_number)'';', ...
             newline, 'NumdimY = 2;', ...
             newline, 'end'];

fwrite(f_id, full_text);

fclose(f_id);

fun_onnx(input, params)




% function [Y, NumdimY] = PLACEHOLDER(X)
% 
%     [rows_number, cols_number] = size(X);
%     rows = 1:rows_number;
%     cols = 1:cols_number;
%     Y = rows <= cols';
%     % Y = Y + 0;
%     NumdimY = 2;
% 
% end
