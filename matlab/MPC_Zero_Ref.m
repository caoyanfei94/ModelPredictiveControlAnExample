function [M, C, Q_bar, R_bar, G, E, H, U_k] = MPC_Zero_Ref(A, B, N, x_k, Q, R, F)

% find the dimensions of A and B
n = size(A, 1);           % A is a nxn matrix
p = size(B, 2);           % B is a nxp matrix

% initialize C, it is a (N)
% the upper part is a nxn I, the lower part is firstly defined as 0
M = [eye(n); zeros(N * n, n)];

% initialize C, a (N+1)n x Np matrix firstly with 0
C = zeros((N + 1) * n, N * p);

% define M and C
tmp = eye(n);
for i = 1: N
    % define current row, begin with i * n, and have n rows in total
    rows = i * n + (1: n);
    % fill C
    C(rows, :) = [tmp * B, C(rows-n, 1: end-p)];
    % Each time tmp left multiplys A
    tmp = A * tmp;

    % fill M
    M(rows, :) = tmp;
end

% define Q_bar
% find the dimensions of Q and R
S_q = size(Q, 1);
S_r = size(R, 1);
% initialize Q_bar with 0
Q_bar = zeros((N + 1)*S_q, (N+1)*S_q);
% arrange Q onto the diagonal
for i = 0: N-1
    Q_bar(i * S_q + 1: (i + 1) * S_q, i*S_q + 1: (i + 1) * S_q) = Q;
end
% put F in the last place
Q_bar((N - 1) * S_q + 1: N * S_q, (N - 1) * S_q + 1: N * S_q) = F;

% define R_bar
R_bar = zeros(N * S_r, N * S_r);
for i = 0: N - 1
    R_bar(i * S_r + 1: (i + 1) * S_r, i * S_r + 1: (i + 1) * S_r) = R;
end

% solve G, E, H
G = M' * Q_bar * M;
E = M' * Q_bar * C;
H = C' * Q_bar * C + R_bar;

% optimal control
% define f
f = (x_k' * E)';
% solve the optimal U_k
U_k = quadprog(H, f);

end

