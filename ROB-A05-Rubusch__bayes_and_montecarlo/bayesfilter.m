% ASSIGNMENT05
% Lothar Rubusch
% 2013-mar-25


close all;



% test model of robot
%
function [new_robot] = mv_left( steps, robot, worldsize )
  % move left in circle
  new_robot = mod( robot+1, worldsize );
endfunction

function [new_robot] = mv_right( steps, robot, worldsize )
  % move right in circle, add 10 and modulo for 0->9
  new_robot = mod( (10 + (robot-1)), worldsize );
endfunction

function [sense] = sensing( robot, worldmap )
  sense = worldmap( 1 + robot );
endfunction

% discrete bayes localization
function [posterior_worldmap] = bayeslocalization( prior_worldmap, is_sensing, moves )
  posterior_worldmap = prior_worldmap;
%  for row=1:2
    % reset
  n_door = sum( prior_worldmap(1,:));
  n_nodoor = length( prior_worldmap ) - n_door;
  p_door = n_door / n_nodoor;
  p_nodoor = 1 - p_door;

  % probabilities
  p_see = 0.8;                % P( see door       | door )
  p_see_err = 1-p_see;        % P( see door       | no door )
  p_notsee_err = 0.4;         % P( don't see door | door )
  p_notsee = 1-p_notsee_err;  % P( don't see door | no door )

  % models
  P_sensor = zeros( lenght( prior_worldmap ) );
  P_motion = zeros( length( prior_worldmap ) );
  P_prior = prior_worldmap;


  for col = 1:length( prior_worldmap )
    %% motion model

    % get correct moving index
    if 0 < moves
      col_ng = mv_left( moves, col, length( prior_worldmap ) );
    else if 0 > moves
      col_ng = mv_right( abs(moves), col, length( prior_worldmap ) );
    else
      col_ng = col;
    endif
    % take old map, take P( col+move )
    P_motion(col) = prior_worldmap( col_ng );



    %% sensor model

    if 1 == prior_worldmap(1, col)
      % map shows a door, and...
      if 1 == is_sensing
        % ...a door sensed
        P_sensor(col) = p_see;
      else
        % ...no door sensed but measure incorrect
        P_sensor(col) = p_notsee_err;
      end
    else
      % map has NO door, and...
      if 1 == is_sensing
        % ...no door sensed
        P_sensor(col) = p_see_err;
      else
        % ...a door sensed, but incorrect
        P_sensor(col) = p_notsee;
      end
    endif
  endfor


  % prediction - sum of motion model(k) times prior(k)
  prediction = 0;
  for col = 1:length( prior_worldmap )
    prediction += P_motion( col ) * P_prior( col );
  endfor


  % normalization
  nu = 0;
  for col = 1:length( prior_worldmap )
    nu += P_sensor( col ) * P_motion( col );
  endfor
  nu = 1/nu;


  % formula
  for col = 1:length( prior_worldmap )
    posterior_worldmap(2, col) = nu * P_sensor(col) *  prediction;
  endfor




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