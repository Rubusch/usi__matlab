% ASSIGNMENT05
% Lothar Rubusch
% 2013-mar-25

% cleanup
close all;


% auxiliary movement function (for simulation)
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


% auxiliary sensor function (for simulation)
function [sense] = sensing( robot, worldmap )
  sense = worldmap( robot );
endfunction


% discrete bayes localization
function [posterior_worldmap] = bayeslocalization( worldmap, P_prior, sees_landmark, moves )
  % init
  worldsize = length( worldmap );
  P_sensor = zeros( 1, worldsize );
  P_motion = zeros( 1, worldsize );
  prediction = zeros(1,worldsize);

  % probabilities, sensor
  p_see = 0.8;                % P( see landmark       | landmark )
  p_see_err = 1-p_see;        % P( don't see landmark | landmark )
  p_notsee_err = 0.4;         % P( see landmark       | no landmark )
  p_notsee = 1-p_notsee_err;  % P( don't see landmark | no landmark )

  for k=1:worldsize
    prediction(k) = 0;

    %% motion model
    k_before = move( -moves, k, worldsize );
    for i=1:worldsize
      i_before = move( -moves, i, worldsize );
%      prediction(k) += P_prior( k ) * P_prior( k_before ) * P_prior( i_before );  % TODO ???
      prediction(k) += P_prior( k_before ) * P_prior( i_before );
    end

    %% sensor model
    if 1 == worldmap(k)
      % map shows a landmark, and...
      if 1 == sees_landmark
        % ...a landmark sensed
        P_sensor(k) = p_see;
      else
        % ...no landmark sensed (incorrect)
        P_sensor(k) = p_see_err;
      end
    else
      % map has NO landmark, and...
      if 1 != sees_landmark
        % ...no landmark sensed
        P_sensor(k) = p_notsee;
      else
        % ...a landmark sensed (incorrect)
        P_sensor(k) = p_notsee_err;
      end
    end
    prediction(k) = P_sensor(k) * prediction(k);
  end

  %% normalization factor
  nu = 0;
  for k = 1:worldsize
    nu += prediction( k );
  end
  nu = 1/nu;

  %% normalization
  for k = 1:worldsize
    posterior_worldmap(k) = nu * prediction(k);
  end
endfunction


function report( idx, moves, sensing, probability )
  printf("%d. robot moved %d and senses ", idx, moves);
  if 1 == sensing
    printf("a LANDMARK\n");
  else
    printf("NOTHING\n");
  end
  printf("position - probability\n");
  [ 0:9; probability]
endfunction



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START                                                                        

%% init
worldmap    = [ 1 0 0 1 0 0 1 0 0 0 ];
probability = [ 1 1 1 1 1 1 1 1 1 1 ];
worldsize = length( worldmap );
robot = 1;
robot_sensing = -1;
moves = 0;

% test robot with random position in world
%robot = mod(abs(int8(randn * 100)), worldsize)

printf("########################################################################\n");
printf("DEBUG: robot on position %d\n", robot);


% print world
[ 0:9; worldmap ]


% 1st iteration
robot_sensing = sensing( robot, worldmap );
probability = bayeslocalization( worldmap, probability, robot_sensing, moves );
report( 1, moves, robot_sensing, probability );
figure
hold on;
grid("minor");
xlabel( "Positions [steps]" );
xlim = 10;
ylabel( "Probability [fraction of one]" );
title( "Localization with discrete Bayes Filter" );
plot(0:9, probability, "c*;1st result;");


% 2nd iteration
moves = 3;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
probability = bayeslocalization( worldmap, probability, robot_sensing, moves );
report( 2, moves, robot_sensing, probability );
plot(0:9, probability, "b*;2nd result;")


% 3nd iteration
moves = 4;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
probability = bayeslocalization( worldmap, probability, robot_sensing, moves );
report( 3, moves, robot_sensing, probability );
plot(0:9, probability, "r*;final result;")
hold off;


printf( "READY.\n" );
