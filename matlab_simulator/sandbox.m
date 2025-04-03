function [error] = sandbox()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
error = 0;
mdl = 'BLDC_simulator';
sim(mdl)
end