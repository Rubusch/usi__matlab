% ASSIGNMENT07
% Lothar Rubusch
% 2013-Apr-29
printf( "\n\nASSIGNMENT07 - Robotics\n" );
printf( "Lothar Rubusch\n" );
printf( "2013-Apr-29\n\n" );


% cleanup
clear; clc; close all;

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
% U_att(q) = 1/2 * k(att) * rho_goal^2(q)
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

function db_print( M, YMAX, XMAX )
  for yval=1:YMAX
    printf( "\n" );
    for xval=1:XMAX
      val = M(XMAX * (yval-1) + xval , 3);
      printf( "%2.1f\t", val );
    endfor
  endfor
endfunction


%% set up field
XMAX=60
YMAX=40
dx=0; dy=0;

%% goal
goal = [ 10 10 ];

%% obstacle
#obst1 = [ 5 5 ; 6 5; 7 5 ];
obst1 = [ 5 10 ];  

%% method
k_att = 0.5;    

M=[];
for yval=1:YMAX
  for xval=1:XMAX

    %% base potential
    dx = xval - goal(1);
    dy = yval - goal(2);
    pot = sqrt(dx^2 + dy^2);

    M(xval,yval) = pot;


    %% attraction
%    pot = -k_att * pot;

    %% operations...
    % TODO    

    %% model obstacles
% TODO
%    for xobst=1:length(obst1)
%      for yobst=1:length(obst1)
%        dx = xval - xobst;
%        dy = yval - yobst;
%        
%        pot += sqrt(dx^2 + dy^2);     
%      endfor
%    endfor
%
    %% round up
%    M = [M ; [xval yval pot]];

  endfor
endfor

%% DEBUGGING
%db_print( M, YMAX, XMAX );
%M
#plot3(M(:,1), M(:,3), M(:,2), "b*")
#plot3(M(:,3), M(:,1), M(:,2), "b.")
%num2str( M, "%2.1f " )   
%tx = ty = linspace(1,XMAX,XMAX);

%% DEBUG
tx = [1:XMAX];
ty = [1:YMAX];
[xx,yy] = meshgrid( tx, ty );
mesh( tx, ty, M' );



%mesh(M(:,1), M(:,3), M(:,2))

%% find way through it

printf( "READY.\n" );
