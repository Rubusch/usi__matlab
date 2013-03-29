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
function [posterior_worldmap] = bayeslocalization( worldmap, P_prior, sees_door, moves )
  worldsize = length( worldmap );

  % reset
  n_door = sum( worldmap );
  n_nodoor = worldsize - n_door;

  % probabilities, map
  p_door = n_door / n_nodoor;
  p_nodoor = 1 - p_door;

  % probabilities, sensor
  p_see = 0.8;                % P( see door       | door )
  p_see_err = 1-p_see;        % P( see door       | no door )
  p_notsee_err = 0.4;         % P( don't see door | door )
  p_notsee = 1-p_notsee_err;  % P( don't see door | no door )

  % models
  P_sensor = zeros( 1, worldsize )
  P_motion = zeros( 1, worldsize )

%  for k=1:worldsize
%    %% motion model
%    % get correct moving index
%    k_ng = -1;
%    if 0 < moves
%      k_ng = mv_left( moves, k, worldsize );
%    elseif 0 > moves
%      k_ng = mv_right( abs(moves), k, worldsize );
%    else
%      k_ng = k;
%    end
%    % take old map, take P( k+move )
%    P_motion(k) = worldmap( k_ng );
%
%    %% sensor model
%    if 1 == worldmap(k)
%      % map shows a door, and...
%      if 1 == sees_door
%        % ...a door sensed
%        P_sensor(k) = p_see;
%      else
%        % ...no door sensed but measure incorrect
%        P_sensor(k) = p_notsee_err;
%      end
%    else
%      % map has NO door, and...
%      if 1 == sees_door
%        % ...no door sensed
%        P_sensor(k) = p_see_err;
%      else
%        % ...a door sensed, but incorrect
%        P_sensor(k) = p_notsee;
%      end
%    end
%  end


  
  prediction = zeros(1,worldsize);
  for k=1:worldsize
    for i=1:worldsize
      %% motion model
      % get correct moving index
      i_ng = -1;
      if 0 < moves
        i_ng = mv_left( moves, i, worldsize );
      elseif 0 > moves
        i_ng = mv_right( abs(moves), i, worldsize );
      else
        i_ng = i;
      end
      % take old map, take P( i + move )
      P_motion(i) = worldmap( i_ng );
      prediction(k) += P_motion(i) * P_prior(i);
    end


    %% sensor model
    if 1 == worldmap(k)
      % map shows a door, and...
      if 1 == sees_door
        % ...a door sensed
        P_sensor(k) = p_see;
      else
        % ...no door sensed but measure incorrect
        P_sensor(k) = p_notsee_err;
      end
    else
      % map has NO door, and...
      if 1 == sees_door
        % ...no door sensed
        P_sensor(k) = p_see_err;
      else
        % ...a door sensed, but incorrect
        P_sensor(k) = p_notsee;
      end
    end
    prediction(k) = P_sensor(k) * prediction(k);
  end
    

  % prediction - sum of motion model(k) times prior(k)
%  prediction = 0;
%  prediction_arr = zeros(1,worldsize);;
%  for col = 1:worldsize
%    prediction += P_motion(col) * P_prior(col);
%    prediction_arr(col) = P_motion(col) * P_prior(col);
%  end
%  prediction = sum( prediction_arr );  



  % normalization
  nu = 0;
  for k = 1:worldsize
%    nu += P_sensor( col ) * prediction;
    nu += prediction( k );
  end
  nu = 1/nu;

  % formula
  for k = 1:worldsize
%    posterior_worldmap(col) = nu * P_sensor(col) *  prediction;
    posterior_worldmap(k) = nu * P_sensor(k) * prediction(k);
  end

  printf("######################################################################\n");
  [ P_motion ; P_sensor ; prediction ]                      
%  [ prediction ; nu ; sum( posterior_worldmap ) ]      
%  [ nu ; sum( posterior_worldmap ) ]      
  printf("######################################################################\n");



%% algorithm
%
% for k=1; k<N do
%     p_hat(k,t) = 0;
%     for i=1; i<N do
%         p_hat(k,t) = p_hat(k,t) + p(X(t) = x(k) | u(k), X(t-1) = x(i)) p(i,t-1) % motion model
%     endfor
%     p(k,t) = p(y(t) | X(t) = x(k)) p_hat(k,t) % sensor model
% endfor
%
% nu = 0;
% for k=1; k<N do
%     nu=nu+p(k,t) % calculate normalization factor
% endfor
%
% for k=1; k<N do
%     p(k,t) = inv( nu ) p(k,t) % normalize
% endfor
%
% return p(k,t);
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
worldmap    = [ 1 0 0 1 0 0 1 0 0 0 ];
probability = [ 1 1 1 1 1 1 1 1 1 1 ];
worldsize = length( worldmap );

% robot position in world, unknown to the robot!
%robot = mod(abs(int8(randn * 100)), worldsize)
robot = 3;
robot_sensing = -1;

% 1st iteration
moves = 0;
robot_sensing = sensing( robot, worldmap );
probability = bayeslocalization( worldmap, probability, robot_sensing, moves );
% door or wall?
printf("1. robot sees a ");
if 1 == sensing( robot, worldmap )
  printf("DOOR\n");
else
  printf("WALL\n");
end
probability


%return;      


% 2eme iteration
moves = 3;
robot = mv_left( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
probability = sensing( worldmap, probability, robot_sensing, moves );
% door or wall?
printf("2. robot sees a ");
if 1 == sensing( robot, worldmap )
  printf("DOOR\n");
else
  printf("WALL\n");
end
probability


return;       


% 3eme iteration
moves = 4;
robot = mv_left( moves, robot, worldsize);
probability = sensing( worldmap, probability, robot_sensing, moves );
% door or wall?
printf("3. robot sees a ");
if 1 == sensing( robot, worldmap )
  printf("DOOR\n");
else
  printf("WALL\n");
end
probability


printf( "READY.\n" );