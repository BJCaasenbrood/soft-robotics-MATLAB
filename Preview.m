function x = Preview(x,Request,Arg)

if nargin == 1, Request = 'b'; 
    if ~isempty(x('Quality')) 
        Arg = x('Quality'); 
    else, Arg = 50;
    end
end
if nargin == 2 && isa(Request,'char'), Arg = 50; 
elseif nargin == 2 && ~isa(Request,'char'), Arg = Request; end

switch(Request)
    case('b');              x = PreviewBare(x,Arg);
    case('Update');         x = UpdatePreview(x);
end 

ax = gca;
ax.Clipping = 'off';

end

%---------------------------------------------------------- UPPDATE PREVIEW
function mesh = UpdatePreview(mesh)

if isfield('Delta',mesh)
set(mesh.FigHandle,'Vertices',mesh.Node + mesh.Delta);
else
set(mesh.FigHandle,'Vertices',mesh.Node);
end

end

%----------------------------------------------------- PREVIEW MESH IN BARE
function Mesh = PreviewBare(Domain,Quality)

[Node,Element,BdBox,Norm,Colors,Cent,Mat] = PreparePreview(Domain,Quality);

axis equal; axis off; hold on; grid on; box on;

FigHandle = patch('Faces',Element,'Vertices',Node,...
    'FaceVertexCData',Colors,'linestyle','none','FaceColor','flat',...
    'EdgeColor','none','Marker','none','MarkerSize',5,'FaceAlpha',1);

view(110,20); pause(1e-6); axis equal; axis vis3d; zoom(1.1);
%A = Domain('View'); if sum(abs(A)) ~= 0; view(A(1),A(2)); end
Domain('View'); 
set(gcf,'color',[255, 255, 255]/255);
material dull;
drawnow;

Mesh.Node = Node;
Mesh.Element = Element;
Mesh.NNode = length(Node);
Mesh.NElem = length(Element);
Mesh.Normals = Norm;
Mesh.Centers = Cent;
Mesh.FigHandle = FigHandle;
Mesh.BdBox = BdBox;
Mesh.Material = Mat;

end

%-------------------------------------------------- PREPARATION FOR PREVIEW
function [Node,Element,BdBox,Norm,Col,Cent,Mat] = PreparePreview(Domain,Quality)

if nargin==1, Quality = 20; 
CallDisplay('loading preview');
if strcmp(Domain('Type'),'implicit')
if isnumeric(Domain('Quality'))
Quality = Domain('Quality'); 
end
end
end

if isfield(Domain,'Node') && isfield(Domain,'Element') 
    Node = Domain.Node;
    Element = Domain.Element;
    BdBox = Domain.BdBox;
else
    BdBox = Domain('BdBox');
        disp('* generating preview for .stl model');
        Node = double(Domain('Node')); Element = double(Domain('Element'));
        
        disp(['* number of nodes = ',num2str(length(Node))]);
        disp(['* number of faces = ',num2str(length(Element))]);
        
end

[Norm,Cent] = TriangularNormal(Node,Element); 
[Col,Mat] = Normal2RGB(Domain,Norm);
end

%----------------------------------------------- COMPUTE NORMALS OF POLYGON
function [Normal, Center] = TriangularNormal(Node,Element)
N = zeros(length(Element),3);
C = zeros(length(Element),3);

for i = 1:length(Element)
    v1 = Node(Element(i,2),:) - Node(Element(i,1),:);
    v2 = Node(Element(i,3),:) - Node(Element(i,1),:);
    
    tmp = cross(v1,v2); 
    N(i,:) = tmp/sqrt(tmp(1)^2 + tmp(2)^2 + tmp(3)^2);
    
    Atmp = 0.5*tmp;
    A(:,1) = sqrt(Atmp(:,1).^2+Atmp(:,2).^2+Atmp(:,3).^2);

    p1 = Node(Element(i,1),:);
    p2 = Node(Element(i,2),:);
    p3 = Node(Element(i,3),:);
    
    C(i,:) = (1/3).*(p1 + p2 + p3);
end

Normal = N;
Center = C;

end

%-------------------------------------------- CONVERT NORMALS TO RGB COLORS
function [N, Material] = Normal2RGB(Domain,N)    

if isempty(Domain('Material')), Material = 'Porcelain';
else, Material = Domain('Material');
end

if size(N,2) < 3, N = repmat(N,1,3); end

C1 = [1,0,0]*0.95;
C2 = [0,1,0]*0.95;
C3 = [0,0,1]*0.95;

Ntmp = zeros(length(N),3);
if strcmp(Material,'Bump')
    N(:,1) = 0.5 + 0.5*(N(:,1));
    N(:,2) = 0.5 + 0.5*(N(:,2));
    N(:,3) = 0.5 + 0.5*softabs(N(:,3));
else
N(:,1) = 0.1 + 0.6*softabs(N(:,1));
N(:,2) = 0.1 + 0.6*softabs(N(:,2));
N(:,3) = 0.3 + 0.6*(N(:,3));
end

CB = ColorMultiply(ColorMultiply(C1,C2),C3);
for ii = 1:length(N)
    tmp = AffineColorMix(C1,C2,C3,N(ii,:));
    Ntmp(ii,:) = [max(CB(1),tmp(1)),...
        max(CB(2),tmp(2)),...
        max(CB(3),tmp(3))];
end
N = Ntmp;

end

%------------------------------------------------ AFFINE COLOR COMBINATIONS
function Color = AffineColorMix(C1,C2,C3,N)
%Invert sRGB gamma compression

   %N = N/sum(N);
   a1 = N(1); a2 = N(2); a3 = N(3);

   Color1 = SrgbCompanding(C1,1);
   Color2 = SrgbCompanding(C2,1);
   Color3 = SrgbCompanding(C3,1);
   
   r = Color1(1)*a1 + Color2(1)*a2 + Color3(1)*a3;
   g = Color1(2)*a1 + Color2(2)*a2 + Color3(2)*a3;
   b = Color1(3)*a1 + Color2(3)*a2 + Color3(3)*a3;
   
   Color = [r,g,b];

   %//Reapply sRGB gamma compression
    Color = SrgbCompanding(Color,0);
end

%------------------------------------------- CONVERT/INVERT sRGB COMPANDING
function ColorNew = SrgbCompanding(Color,Inverse)
r = Color(1);
g = Color(2);
b = Color(3);

if Inverse == 1
    if (r > 0.04045), r = ((r+0.055)/1.055)^2.4; else, r = r / 12.92; end
    if (g > 0.04045), g = ((g+0.055)/1.055)^2.4; else, g = g / 12.92; end
    if (b > 0.04045), b = ((b+0.055)/1.055)^2.4; else, b = b / 12.92; end
else
    if (r > 0.0031308),r = 1.055*(r)^(1/2.4)-0.055; else, r = r * 12.92;end
    if (g > 0.0031308),g = 1.055*(g)^(1/2.4)-0.055; else, g = g * 12.92;end
    if (b > 0.0031308),b = 1.055*(b)^(1/2.4)-0.055; else, b = b * 12.92;end
end

ColorNew = [r, g, b];
end


%------------------------------------------------------ MULTIPLY RBG COLORS
function N = ColorMultiply(N1,N2)
N = zeros(size(N1,1),3);

for i = 1:length(N)
    N(1) = N1(1)*N2(1);
    N(2) = N1(2)*N2(2);
    N(3) = N1(3)*N2(3);
end

end

%------------------------------------------------- SMOOTH ABSOLUTE FUNCTION
function y = softabs(x,a)
if nargin < 2, a = 20; end
y = (a*abs(x).^3)./(1+a*x.^2);
end
