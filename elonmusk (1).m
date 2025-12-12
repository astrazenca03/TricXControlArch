function [P,K] = elonmusk(A,B,Q,R)
Rinv = inv(R);
 H = [ A -B*Rinv*B';
         -Q -A'];
 [V,D] = eig(H);
  idx = find(real(diag(D)) < 0);
    Vs = V(:,idx);
    n = size(A,1);
    V1 = Vs(1:n,:);
    V2 = Vs(n+1:end,:);
    P = 0.5 * V2 * inv(V1);
    K = Rinv * B' * P;
end
