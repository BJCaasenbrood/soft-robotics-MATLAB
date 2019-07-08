%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -- name --
%   "computeDynamics.m"
%       created by Brandon Caasenbrood - TU/e - (14/05/19)    
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ConfigureLibrary(); 
clear; clc; close all;

PlanarDynamics      = false;
NeglectRotational   = true;
NeglectExtensible   = false;
ReduxRotational     = true;
LinearDampingFunc   = true;
LinearStiffnessFunc = false;
IncludeBaseVelocity = false;
ShowDeformability   = false;

%% variables
syms kx ky l sigma eta zeta 
syms kxe kye le kxz kyz lz 
syms dkxe dkye dle dkxz dkyz dlz 
syms m g ke kb kt Ixx Iyy Izz l0 de db kb1 kb2 kb3 beta0 dm
syms dkx dky dl r ke1 ke2 ke3
syms ddkx ddky ddl
syms eps beta 
syms v0x v0y v0z w0x w0y w0z  

print('constructed symb. variables');

vect    =  [0;0;1]; % cross vector
gv      =  [0;0;g]; % gravity vector

% options 
if ~NeglectExtensible && ~PlanarDynamics
q = [l; kx; ky];
dq = [dl; dkx; dky]; 
ddq = [ddl; ddkx; ddky];
print('generating full 3dof model');
print('q(t) = [l(t); kx(t); ky(t)]');
elseif PlanarDynamics 
q = [l; kx];
dq = [dl; dkx]; 
ddq = [ddl; ddkx];    
print('generating planar 2d model');
print('q(t) = [l(t); kx(t)]');

else
q = [kx; ky];
dq = [dkx; dky]; 
ddq = [ddkx; ddky];
print('generating inextensible model');
print('q(t) = [kx(t); ky(t)]');
end

nDOF = length(q);

%% rotation matrix 
print('constructing rotation matrix Phi(q,s)');
Phi0Sigma = RotationMatrix(kx,ky,sigma);

Phi0Eta = subs(Phi0Sigma,sigma,eta);

PhiSigmaEta = transpose(Phi0Sigma)*Phi0Eta;

%% position vector
print('constructing position vector p(q,s)');
Pos0Sigma = simplify(int(Phi0Sigma*vect,0,sigma));

Pos0Eta = subs(Pos0Sigma,sigma,eta);

REtaSigma = transpose(Phi0Sigma)*Pos0Eta - transpose(Phi0Sigma)*Pos0Sigma;

%% position vector plot
FPos0Sigma = matlabFunction(Pos0Sigma);

if ShowDeformability
print('showing backbone curve');
figure;
for i = 1
   [l_, kx_, ky_] = generateRandomDeform(q,l,kx,ky);
   plotSpatialCurve(FPos0Sigma,l_,kx_,ky_);
end
end

%% frame-rate vector 
thetaSkewMat = transpose(Phi0Sigma)*diff(Phi0Sigma,sigma);

theta = simplify([thetaSkewMat(3,2); 
                  thetaSkewMat(1,3); 
                  thetaSkewMat(2,1)]);

%% Jacobian matrix
print('constructing jacobian matrices');
h = [0;0;l-l0];

JOmega = jacobian(theta,q);
JTrans = jacobian(h,q);

J = [JTrans;JOmega];

%% physical properties of slice
print('assigning physical properties: inertia/stiffness');
if ~ReduxRotational, msl = m/l; 
else, msl = (m+dm)/l; 
end

Is = [Ixx, 0, 0;
      0, Iyy, 0;
      0, 0, Izz];
 
Ms = [msl*eye(3,3), zeros(3,3);
      zeros(3,3),          Is]; 
  
if ~LinearStiffnessFunc
LinearStiffness = ke1 + ke2*(tanh(ke3*eps)^2 - 1);
else, LinearStiffness = ke1; 
end
if ~LinearStiffnessFunc
BendingStiffness = kb1 + kb2*(tanh(kb3*beta)^2 - 1);
else, BendingStiffness = kb1; 
end
  
%% adjoint matrix
print('constructing adjoint matrix');
if ~NeglectRotational
AdjointMatrix = [PhiSigmaEta, CrossMatrix(REtaSigma)*PhiSigmaEta; ...
                  zeros(3,3),                        PhiSigmaEta];
                       
elseif NeglectRotational || ReduxRotational
    
AdjointMatrix = [PhiSigmaEta, CrossMatrix(REtaSigma)*PhiSigmaEta; ...
                  zeros(3,3),                        zeros(3,3)];
end

%% kinetic energy of total system
JManipulator = simplify(AdjointMatrix*J);

if ~IncludeBaseVelocity
Vsigma = int(JManipulator*dq,eta,0,sigma);    
else
Vbase = [v0x;v0y;v0z;w0x;w0y;w0z];
Vsigma = int(JManipulator*dq,eta,0,sigma) + Vbase;    
end

print('computing total kinetic energy T(q,dq)');
T = simplify(int(0.5*transpose(Vsigma)*Ms*Vsigma,sigma,0,l));

%% potential energy of total system
Vslice = msl*transpose(gv)*FPos0Sigma;

print('computing total potential energy V(q)');
VGrav = int(Vslice, sigma, 0, l);
VStretch = int(LinearStiffness*eps,eps,0,l-l0);
VBend = int(BendingStiffness*(beta-beta0),beta,0,l*sqrt(kx^2 + ky^2));

V = simplify(VGrav + VStretch + VBend);

%% euler lagrange
L = T - V;
print('assembling EOM');
dLddq = jacobian(L,dq);
dLdq = jacobian(L,q);
dLdt = zeros(length(q),1);
set = [q;dq]; 
dtset = [dq;ddq];

for i = 1:length(set)
    dLdt = dLdt + simplify(diff(dLddq,set(i))*dtset(i),'IgnoreAnalyticConstraints',true).';
end

EOM = dLdt - dLdq;

%% mass matrix
print('forming inertia matrix M(q)');
M = jacobian(dLdt,ddq); 
M = simplify(M,200);

%% coreolis matrix
print('forming coreolis matrix C(q,dq)');
for k = 1:nDOF
for i = 1:nDOF
for j = 1:nDOF
    Cq{i,j,k} = 0.5*simplify( diff(M(k,j),q(i)) + ...
        diff(M(k,i),q(j)) - diff(M(i,j),q(k)) ,'IgnoreAnalyticConstraints',true);
end
end
end

C = sym(zeros(nDOF,nDOF));

for k=1:nDOF
for j=1:nDOF % used to be j
for i=1:nDOF % used to be i
     C(k,j) = C(k,j) + Cq{i,j,k}*dq(i);
end
end
end

C = simplify(C);

%% potential force vector
print('forming potenial force vector N(q)');
N = transpose(jacobian(V,q)); 
N = simplify(N,'IgnoreAnalyticConstraints',true);

%% damping force vector vector
syms d1 d2 d3 dk eta
print('forming viscous force vector F(q)');
if ~LinearDampingFunc
Fd = int((d1 + d2*(tanh(d3*eta)^2 - 1))*eta,eta,0,dk);
else, Fd = 0.5*d1*dl^2 + 0.5*db*dkx^2 + 0.5*d1*dky^2;
end

if ~NeglectExtensible && ~PlanarDynamics
F = [de*dl; db*dkx; db*dky];
else
F = [db*dkx; db*dky];
end
%F = [de*dl;d1*kx];

%% write text file
print('==== DYNAMIC MODEL: M(q)ddq + C(q,dq)dq + N(q) + F(q) = u(t) ====');
%ExportMatrixToText(simplify(inv(M),200,'IgnoreAnalyticConstraints',true),'iM',[nDOF nDOF]);
ExportMatrixToText(simplify(M,200,'IgnoreAnalyticConstraints',true),'M',[nDOF nDOF]);
ExportMatrixToText(simplify(C,200,'IgnoreAnalyticConstraints',true),'C',[nDOF nDOF]);
ExportVectorToText(N,'N',nDOF);
ExportVectorToText(F,'F',nDOF);

function R = RotationMatrix(kappa_x,kappa_y,sigma)

assume(kappa_x,'real'); 
assume(kappa_y,'real');
assume(sigma,'real');

kappa = sqrt(kappa_x^2 + kappa_y^2);

assume(kappa >= 0);

alpha = kappa*sigma;

ca = cos(alpha);
sa = sin(alpha);
va = 1-ca;

kx = -(kappa_y/kappa);
ky = (kappa_x/kappa);

R = [((kx^2)*va) + ca, (kx*ky*va), (ky*sa);
     (kx*ky*va), ((ky^2)*va) + ca, -(kx*sa);
     -(ky*sa), (kx*sa),  ca];


end

function C = CrossMatrix(a)

check = size(a);

if check(1) < 3
    error('Size not correct -> [x,y,z]^T')
end

C = [0, -a(3), a(2);
     a(3), 0, -a(1);
     -a(2), a(1), 0];

end

function ExportMatrixToText(Matrix,Name,Size)
%diary([Name,'.txt'])
for i = 1:Size(1)
for j = 1:Size(2)
disp([Name,'(',num2str(i),',',num2str(j),') = ',char(Matrix(i,j)),';']);
end
end
%diary off
end

function ExportVectorToText(Vector,Name,Size)
%diary([Name,'.txt'])
for i = 1:Size(1)
disp([Name,'(',num2str(i),') = ',char(Vector(i)),';']);
end
%diary off
end

function [l_,kx_,ky_] = generateRandomDeform(q,l,kx,ky)
betaMax = pi;
if size(q,1) == 3
l_ = 1;
kx_ = (2*rand(1,1)-1)*betaMax/l_;
ky_ = (2*rand(1,1)-1)*betaMax/l_;
elseif size(q,1) == 2 && sum(jacobian(q,ky)) == 0
l_ = 1;
kx_ = (2*rand(1,1)-1)*betaMax/l_;
ky_ = 0;  
elseif size(q,1) == 2 && sum(jacobian(q,ky)) == 0
l_ = 1;
kx_ = (2*rand(1,1)-1)*betaMax/l_;
ky_ = (2*rand(1,1)-1)*betaMax/l_;
end
end

function plotSpatialCurve(func,l,kx,ky)
S = linspace(0,l,50);
%P = zeros(50,3);
for i = 1:length(S)
    P(i,:) = func(kx,ky,S(i)).';
end
hold on;
plot3(P(:,1),P(:,2),P(:,3),'Linewidth',1);
axis equal; grid on; view(-30,15); drawnow
end

function print(str)
pause(.1);
preString = '* ';
disp([preString,str]);
end

