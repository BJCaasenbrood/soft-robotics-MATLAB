function x = SoftActuatorRedux(Request,Arg)
  file = 'SoftActuatorRedux';
  %CheckModel(file);
  
  BdBox = BoundingBox;
  
  switch(Request)
    case('BdBox');   x = BoundingBox;                      % call bounding box
    case('Node');    x = LoadNode(file);      % call distance function
    case('Element'); x = LoadElement(file);      % call distance function
    case('BC');      x = BoundaryCondition(Arg{:},BdBox);  % call boundary condition
    case('PFix');    x = FixedPoints(BdBox);               % call fixed points
    case('Area');    x = (BdBox(2)-BdBox(1))*...           % call bounding area
                         (BdBox(4)-BdBox(3))*...
                         (BdBox(6)-BdBox(5));
    case('Type');    x = 'stl';
    case('Scale');   x = 1e-3;
    case('Scale');   x = 1;
    case('Dim');     x = 3;
    case('Material');     x = 'Bump';
    case('View');    x = SetView(240,20);
    case('Check');   x = CheckModel(file);
    case('Help');    x = Help();
    otherwise;       x = [];
  end
  
end 

  %-------------------------------------------------- generate bounding box
function BdBox = BoundingBox()

    % set bounding box := [xmin xmax ymin ymax] zmin zmax]
    BdBox = [-19.7827, 27.9676, -25.5803, 25.5803, -1.1182e-06, 64.5];
end  

%---------------------------------------------- compute boundary conditions
function x = BoundaryCondition(Node,Element,BdBox)

  Supp = [];
  Load = [];

  x = {Supp,Load};

end

%----------------------------------------------------- specify fixed points
function PFix = FixedPoints(BdBox)
   PFix = [];
end

%----------------------------------------------- compute distance functions
function x = SetView(a,b)
view(a,b);
x = [];
zoom(1)
end