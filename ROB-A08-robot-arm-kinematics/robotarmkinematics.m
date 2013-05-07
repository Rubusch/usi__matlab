% ASSIGNMENT08
% Lothar Rubusch
% 2013-May-06
printf( "ASSIGNMENT08 - Robotics\n" );
printf( "Lothar Rubusch\n" );
printf( "2013-May-06\n\n" );

% cleanup
clear; clc; close all;


% function
function robotarmkinematics( theta )
  theta
% TODO
endfunction


% a)
printf( "a)\n" );
theta = [ 0 ; 0 ; 0 ; 0 ; 0 ]
robotarmkinematics( theta )

% b)
printf( "b)\n" );
theta = [ 0    ; pi/4 ; -pi/4 ; -pi/4+pi/4)+pi ; 0 ];
robotarmkinematics( theta );

% c)
printf( "c)\n" );
theta = [ pi/4 ; pi/4 ; -pi/4 ; -pi/4+pi/4+pi ; 0 ];
robotarmkinematics( theta );

% d)
printf( "d)\n" );
theta = [ 0    ; pi/2 ; -pi/2 ; -pi/2+pi/2+pi ; 0 ];
robotarmkinematics( theta );

% e)
printf( "d)\n" );
theta = [ pi/4 ; pi/2 ; -pi/2 ; -pi/2+pi/2+pi ; 0 ];
robotarmkinematics( theta );



printf( "READY.\n" );


