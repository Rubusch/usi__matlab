% ASSIGNMENT07
% Lothar Rubusch
% 2013-Apr-29

% cleanup
clc; close all;

% Implement a potential field navigation strategy with round round obstacles and
% the repulsing and attractive functions given in the previous lecture. The
% environment has the size 200 x 300 cm, the obstacles a diameter of 15cm.
%
% -> repulsing potential field
% -> attractive potential field
% -> field: 200 x 300cm
% -> obstacles (round): 15cm
%
%
% Define a scenario with 9 obstacles, a starting position and a goal position.
% Let a holonomic robot (with a diameter of 7cm) navigate from the start to the
% goal position by moving down the gradient of the potential field. Choose an
% adequate step size for the gradient descent. Plot the paths of the robot and
% the potential fields for different configurations.
%
% -> 9 obstacles
% -> robot: 7cm diameter
% -> stepsize, e.g. 5cm
%
%
% Use a point for the robot and extend the diameter of the obstacles by the
% diameter of the robot.
%
% -> method: extend the diameter of the obstacles
%
%
% It is not possible to reach the goal in each configuration. Setup a situation
% where the robot cannot reach the goal with the potential field method.
%
% -> setup where the robot can reach the goal with the potential field method
% -> setup where the robot cannot reach the goal with the potential field method
%
%
% To validade that the robot is not colliding with the obstacles visualize the
% obstacles in the path plot. You can use 'pdcirc' in a 2D plot and 'patch' in a
% 3D plot to visualize the obstacle in MATLAB.
%
% -> visualize in 2D and/or in 3D using MATLAB
%
%
%
%% Formulae
%
% potential field
% U_att(q) = 1/2 * k(att) rho_goal^2(q)
%
% attracting force converges linearily towards 0 (goal)
% F_att(q) = -nabla(U_att( q ))
%          = -k_att * rho_goal( q ) * nabla( rho_goal( q ) )
%          = -k_att * (q - q_goal)
%
% repulsing potential field
%             / 1/2 * k_rep * ( 1 / rho(q) - 1/rho_0 )^2 ;  if rho(q) <  rho_0
% U_rep(q) = {
%             \ 0                                        ;  if rho(q) >= rho_0
%
%            ; p(q) is the minimum distance between q and the object
%            ; k_rep = k_att
%
% F_rep(q) = -nabla( U_rep(q) )
%
%              / k_rep( 1/rho(q) - 1/rho_0 ) * 1/(rho( p ))^2 * (q - q_obstacle) / rho( q )
%             /                                          ; if rho( q ) < rho_0
%          = {
%             \ 0                                        ; if rho( q ) >= rho_0
%


% TODO
% -> field: 200 x 300cm
% -> obstacles (round): 15cm
% -> 9 obstacles
% -> robot: 7cm diameter
% -> stepsize, e.g. 5cm

% 1a)
% -> repulsing potential field
% -> visualize in 2D and/or in 3D using MATLAB
%
% 1b)
% -> attractive potential field
% -> visualize in 2D and/or in 3D using MATLAB
%
% 2a)
% -> method: extend the diameter of the obstacles
% -> repulsing potential field
% -> visualize in 2D and/or in 3D using MATLAB
%
% 2b)
% -> method: extend the diameter of the obstacles
% -> attractive potential field
% -> visualize in 2D and/or in 3D using MATLAB
%
% 3a)
% -> setup where the robot cannot reach the goal with the potential field method
% -> repulsing potential field
% -> visualize in 2D and/or in 3D using MATLAB
%
% 3b)
% -> setup where the robot cannot reach the goal with the potential field method
% -> attractive potential field
% -> visualize in 2D and/or in 3D using MATLAB

    


printf( "READY.\n" );
