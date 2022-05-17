clear;
clc;

A = [1 0.1; 0 2];
B = [0; 0.5];
N = 3;
x_k = [5; 5];
Q = [1 0; 0 1];
R = 0.1;
F = [2 0; 0 2];

[M, C, Q_bar, R_bar, G, E, H, U_k] = MPC_Zero_Ref(A, B, N, x_k, Q, R, F);
disp('M=');
disp(M);
disp('C=');
disp(C);
disp('Q_bar=');
disp(Q_bar);
disp('R_bar=');
disp(R_bar);
disp('G=');
disp(G);
disp('E=');
disp(E);
disp('H=');
disp(H);

disp('N=3:');
disp(U_k);

N= 10;
[M, C, Q_bar, R_bar, G, E, H, U_k] = MPC_Zero_Ref(A, B, N, x_k, Q, R, F);
disp('N=10:');
disp(U_k);