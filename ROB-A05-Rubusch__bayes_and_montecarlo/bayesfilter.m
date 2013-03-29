% ASSIGNMENT05
% Lothar Rubusch
% 2013-mar-25


close all;



% test model of robot
%
function [new_position] = move( steps, position, worldsize )
  if 0 == (new_position = mod( 11+ position + steps, worldsize+1 ))
    if 0 > steps
      new_position = 10;
    elseif 0 < steps
      new_position = 1;
    else
      printf("ERROR: position input was 0 and no movements\n");
      new_position = -1;
    endif
  endif
endfunction

function [sense] = sensing( robot, worldmap )
  sense = worldmap( robot );
endfunction


% discrete bayes localization
function [posterior_worldmap] = bayeslocalization( worldmap, P_prior, sees_door, moves )

%  printf("### INPUT ###########################################################\n");
%  worldmap
%  P_prior
%  sees_door
%  moves
%  printf("#####################################################################\n");


  worldsize = length( worldmap );

  % reset
  n_door = sum( worldmap );
  n_nodoor = worldsize - n_door;

  % probabilities, map
  p_door = n_door / n_nodoor;
  p_nodoor = 1 - p_door;

  % probabilities, sensor
  p_see = 0.8;                % P( see door       | door )
  p_see_err = 1-p_see;        % P( don't see door | door )
  p_notsee_err = 0.4;         % P( see door       | no door )
  p_notsee = 1-p_notsee_err;  % P( don't see door | no door )

  % models
  P_sensor = zeros( 1, worldsize );
  P_motion = zeros( 1, worldsize );
  prediction = zeros(1,worldsize);

  for k=1:worldsize

    k_before = move( -moves, k, worldsize );
    for i=1:worldsize
      %% motion model
      % get correct moving index
      i_ng = -1;
%      i_ng = move( -moves, i, worldsize );
      i_before = move( -moves, i, worldsize );
%      prediction(k) += P_prior( i ) * P_prior(i_before);
      prediction(k) += P_prior( i ) * P_prior(k_before);
    end


    %% sensor model
    if 1 == worldmap(k)
      % map shows a door, and...
      if 1 == sees_door
        % ...a door sensed
        P_sensor(k) = p_see * p_door;
      else
        % ...no door sensed (incorrect)
        P_sensor(k) = p_see_err * p_door;
      end
    else
      % map has NO door, and...
      if 1 == sees_door
        % ...no door sensed
        P_sensor(k) = p_notsee * p_nodoor;
      else
        % ...a door sensed (incorrect)
        P_sensor(k) = p_notsee_err * p_nodoor;
      end
    end
    prediction(k) = P_sensor(k) * prediction(k);
  end


  % normalization
  nu = 0;
  for k = 1:worldsize
    nu += prediction( k );
  end
  nu = 1/nu;

  % formula
  for k = 1:worldsize
    posterior_worldmap(k) = nu * prediction(k);
  end

%  printf("######################################################################\n");
%  [P_sensor; prediction]
%  printf( "DB: sum of all probabilitys %f\n", sum(posterior_worldmap));
%  printf("######################################################################\n");



%% algorithm
%
% for k=1; k<N do
%     p_hat(k,t) = 0;
%     for i=1; i<N do
%         p_hat(k,t) = p_hat(k,t) + p(X(t) = x(k) | u(k), X(t-1) = x(i)) * p(i,t-1) % motion model
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

function report( idx, moves, sensing, probability )
  printf("%d. robot moved %d and sees a ", idx, moves);
  if 1 == sensing
    printf("DOOR\n");
  else
    printf("WALL\n");
  end
  probability
endfunction



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START                                                                        

%% init
worldmap    = [ 1 0 0 1 0 0 1 0 0 0 ];
probability = [ 1 1 1 1 1 1 1 1 1 1 ];
worldsize = length( worldmap );
robot = 3;
robot_sensing = -1;
moves = 0;
% robot position in world, unknown to the robot!
%robot = mod(abs(int8(randn * 100)), worldsize)

printf("########################################################################\n");
printf("DEBUG: robot on position %d\n", robot);



%printf("\n");
%pos = move( 1, robot, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%pos = move( 1, pos, worldsize )
%               
%printf("\n");
%pos = move( -1, robot, worldsize )
%sensing( robot, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%pos = move( -1, pos, worldsize )
%sensing( pos, worldmap )
%printf("\n");

%return;        
worldmap


% 1st iteration
robot_sensing = sensing( robot, worldmap );
probability = bayeslocalization( worldmap, probability, robot_sensing, moves );
report( 1, moves, robot_sensing, probability );


%return;      


% 2eme iteration
moves = 3;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
probability = bayeslocalization( worldmap, probability, robot_sensing, moves );
report( 2, moves, robot_sensing, probability );


%return;       


% 3eme iteration
moves = 4;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
probability = bayeslocalization( worldmap, probability, robot_sensing, moves );
report( 3, moves, robot_sensing, probability );



printf( "READY.\n" );
