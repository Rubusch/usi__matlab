%% ASSIGNMENT08
%% Lothar Rubusch
%% 2013-May-06
printf( "ASSIGNMENT08 - Robotics\n" );
printf( "Lothar Rubusch\n" );
printf( "2013-May-06\n\n" );
clear; clc; close all;


function [ T ] = dhframe( theta, alpha, a, d )
  T = [ cos( theta )               -sin( theta )              0              a               ;...
        sin( theta )*cos( alpha )  cos( theta )*cos( alpha )  -sin( alpha )  -sin( alpha )*d ;...
        sin( theta )*sin( alpha )  cos( theta )*sin( alpha )  cos( alpha )   cos( alpha )*d  ;...
        0                          0                          0              1 ];
end

function kinematics( thetas )
  T = 1;
  T = T * dhframe( thetas(1),     0,   0, 0 );
  T = T * dhframe( thetas(2),  pi/2,   0, 0 );
  T = T * dhframe( thetas(3),     0, 190, 0 );
  T = T * dhframe( thetas(4),     0, 139, 0 );
  T = T * dhframe( thetas(5), -pi/2,   0, 147.3+166 );

  printf("X = %.2f\nY = %.2f\nZ = %.2f\n", T(1,4), T(2,4), T(3,4) );
endfunction


% a)
printf( "\nexercise a)\n" );
thetas = [ 0    ; 0   ; 0     ; 0              ; 0 ];
kinematics( thetas );

%% b)
printf( "\nexercise b)\n" );
thetas = [ 0    ; pi/4 ; -pi/4 ; -pi/4+pi/4+pi ; 0 ];
kinematics( thetas );

%% c)
printf( "\nexercise c)\n" );
thetas = [ pi/4 ; pi/4 ; -pi/4 ; -pi/4+pi/4+pi ; 0 ];
kinematics( thetas );

%% d)
printf( "\nexercise d)\n" );
thetas = [ 0    ; pi/2 ; -pi/2 ; -pi/2+pi/2+pi ; 0 ];
kinematics( thetas );

%% e)
printf( "\nexercise d)\n" );
thetas = [ pi/4 ; pi/2 ; -pi/2 ; -pi/2+pi/2+pi ; 0 ];
kinematics( thetas );

printf( "\nREADY.\n\n" );

