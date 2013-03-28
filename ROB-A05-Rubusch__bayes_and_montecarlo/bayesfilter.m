% ASSIGNMENT05
% Lothar Rubusch
% 2013-mar-25


close all;



% test model of robot
%
function [new_robot] = mv_left( robot, worldsize )
  % move left in circle
  new_robot = mod( robot+1, worldsize );
endfunction

function [new_robot] = mv_right( robot, worldsize )
  % move right in circle, add 10 and modulo for 0->9
  new_robot = mod( (10 + (robot-1)), worldsize );
endfunction

function [sense] = sensing( robot, worldmap )
  sense = worldmap( 1 + robot );
endfunction

% discrete bayes localization
function [posterior_worldmap] = bayeslocalization( prior_worldmap, isdoorsensed )
  posterior_worldmap = prior_worldmap;
%  for row=1:2
    % reset
  n_door = sum( prior_worldmap(1,:));
  n_nodoor = length( prior_worldmap ) - n_door;
  p_door = n_door / n_nodoor;
  p_nodoor = 1 - p_door;

  % probabilities
  p_see = 0.8;                % P( see door | door )
  p_see_err = 1-p_see;        % P( see door | no door )
  p_notsee_err = 0.4;         % P( don't see door | door )
  p_notsee = 1-p_notsee_err;  % P( don't see door | no door )

  p_sense = 0; 
  p_moving = +1; 
  for col = 1:length( prior_worldmap )
    posterior_worldmap(1,col) = prior_worldmap(1, col);

% TODO sensing
% TODO movement

    if 1 == prior_worldmap(1, col)
      %% map has a door
      if 1 == isdoorsensed
% TODO check this
        p_sense = p_see;
      else
        p_sense = p_notsee_err;
      end

%          posterior_worldmap(2,col) = 1 / n_door * p_door * p_see / (p_door * p_see + p_nodoor * p_notsee_err);
      % may sum up to more than 1
      posterior_worldmap(2,col) = 1 / n_door   * p_door    * p_see      * (p_sense * p_moving);

      
    else
      %% map has NO door
      if 1 == isdoorsensed
% TODO check this
        p_sense = p_see_err;
      else
        p_sense = p_notsee;
      end
%          posterior_worldmap(2,col) = 1 / n_nodoor *  p_nodoor * p_notsee / (p_nodoor * p_notsee + p_door * p_see_err);
      % may sum up to more than 1
      posterior_worldmap(2,col) = 1 / n_nodoor *  p_nodoor * p_notsee   * (p_sense * p_moving);
    endif
  endfor

  nu = sum( posterior_worldmap(2,:) );


      nu

%  endfor
  
% for k=1; k<N do
%     p_hat(k,t) = 0;
%     for i=1; i<N do
%         p_hat(k,t) = p_hat(k,t) + p(X(t) = x(k) | u(k), X(t-1) = x(i)) p(i,t-1) % motion model
%     endfor
%     p(k,t) = p(y(t) | X(t) = x(k)) p_hat(k,t) % sensor model
% endfor
% nu = 0;
% for k=1; k<N do
%     nu=nu+p(k,t) % calculate normalization factor
% endfor
% for k=1; k<N do
%     p(k,t) = inv( nu ) p(k,t) % normalize
% endfor
% return p(k,t);
  

% TODO
endfunction


function particle_filter()
% for i=1; i<k do % normally done online
%     for j=1; j<N do % for all particles
%         compute a new state x by sampling according to P(x|u(i-1,x(j)));
%         x_hat(j) = x;
%     endfor
%     nu = 0;
%     for j=1; j<N do % for all particles
%         omega(j) = P(y(i)|x(j)); % sensor model
%         nu = nu + omega(j) % calculate normalization factor
%     endfor
%     for j=1; j<N do % for all particles
%         omega(j) = inv(nu) * omega(j); % normalize
%     endfor
%     M = resampling(M);
% endfor

% TODO
endfunction


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START                                                                        

%% simulation workd
worldmap = [ 1 0 0 1 0 0 1 0 0 0 ];
worldsize = 10;

% robot position in world, unknown to the robot!
robot = mod(abs(int8(randn * 100)), worldsize);
robot_sensing = -1;


robot_sensing = sensing( robot, worldmap )
worldmap = bayeslocalization( worldmap, robot_sensing );
% TODO filter

robot = mv_left( robot, worldsize);
robot = mv_left( robot, worldsize);
robot = mv_left( robot, worldsize);
robot_sensing = sensing( robot, worldmap )
% TODO filter

robot = mv_left( robot, worldsize);
robot = mv_left( robot, worldsize);
robot = mv_left( robot, worldsize);
robot = mv_left( robot, worldsize);
robot_sensing = sensing( robot, worldmap )
% TODO filter



% door or wall?
if 1 == sensing( robot, worldmap )
  printf("DOOR\n");
else
  printf("WALL\n");
end



printf( "READY.\n" );