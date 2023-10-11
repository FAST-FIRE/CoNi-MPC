%End the current ACADO problem. 
% This function can only be called when a problem is active (and thus
% BEGIN_ACADO is called before). Upon calling this function the current
% problem will be transfered to a C++ file and will be compiled. 
%
%  Example:
%    >> END_ACADO
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    Author: David Ariens, Rien Quirynen
%    Date: 2009-2012
% 

GEN_ACADO;

warning off all

% % Remove the ACADO package from the path
% rmpath(genpath([ACADO_.pwd filesep 'acado']));  
% addpath([ACADO_.pwd filesep 'acado']);   
% addpath([ACADO_.pwd filesep 'acado/functions']);
% addpath( genpath([ACADO_.pwd filesep 'acado' filesep 'packages']) );

% Clear global
ACADO_ = {};
clear ACADO_;

warning on all