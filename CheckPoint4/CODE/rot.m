%Author: Won Dong Shin
%Email: shin123@illinois.edu

function [rtn] = rot(axis, th)
%% Instruction
%Parameters====================================================================================
%Inputs:
%   axis: Rotation axis (Options: 'x', 'y', 'z')(Not case sensitive)
%   th: Rotation angle in radian
%Output:
%   rtn: 4X4 Homogenous transformation(rotation) matrix
%==============================================================================================

%Note================================================================= 
%  Make sure you put single quotation mark arround the variable 'axis' 
%  when you call this function
%===================================================================== 

%Example==================================
% rot('x',pi)
% 
% ans =
% 
%     1.0000         0         0         0
%          0   -1.0000   -0.0000         0
%          0    0.0000   -1.0000         0
%          0         0         0    1.0000
%=========================================

%% Calculation
    %Initialization
    rtn = sym(eye(4));
    
    if(axis == 'x' || axis == 'X')
        rtn(1:3,1:3) = rot3x3('x',th); 
    end
    
    if(axis == 'y' || axis == 'Y')
        rtn(1:3,1:3) = rot3x3('y',th);
    end
    
    if(axis == 'z' || axis == 'Z')
        rtn(1:3,1:3) = rot3x3('z',th);
    end
    
end