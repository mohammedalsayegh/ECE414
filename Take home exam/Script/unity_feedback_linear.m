%% Unity LAM takehome test
Gneg = tf(G);
s = tf('s');
j = Gneg.Denominator{1,1}
length(j);

for(i = 1:length(j))
    if (mod(i, 2) == 0)
        %disp('even');
    else
        %disp('odd');
        j(i) = -1*j(i);
    end
end

Ds = tf(1);
Dneg = tf(1);

Ds.Numerator{1,1} = Gneg.Denominator{1,1};
Dneg.Numerator{1,1} = j;

%%
% now we need to find Do for the lamdesign function, which means find Qs
% Q=D(s)*D(-s)+qN(s)N(-s) where G=N/D
% DnegS = -s^3 + 29.2*s^2 - 453.3*s + 2779
% %and N(s) = N(-s) = 26.972
N = knom;

% % Now we need to pick an arbitrary q
% %and compute Q(s)
% %q = 100;
q = 100;
Dd=Ds*Dneg
Q = Dd + q*N^2

% %and writing as vector     
 r = roots(Q.Numerator{1,1})

% % take the neg real roots 
Do = poly([r(1) r(2) r(3) r(1) r(2) r(3)])
[D,T,Tu,Td,L]=lamdesign(G,Do);

%%
% T = steplqr(H,0.20541) to T = steplqr(G,0.2698)
% %and writing as vector     
T = steplqr(G,0.2698);num
T = tf(T);
%r = roots(T.Numerator{1,1})
r = roots(T.Denominator{1,1})
