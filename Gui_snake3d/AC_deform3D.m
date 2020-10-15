function [vXnew,vYnew,vZnew] = AC_deform3D(vertex2DX,vertex2DY, vertex2DZ,alpha,beta,tau,Fext,ITER,type)
% AC_DEFORM     Deform an active contour (AC), also known as snake.
%     vertex1 = AC_DEFORM(vertex0,alpha,beta,tau,Fext,iter)
%     vertex1 = AC_DEFORM(vertex0,alpha,beta,tau,Fext,iter,type)
%   
%     Inputs
%     vertex0     position of the vertices, n-by-2 matrix, each row of 
%                 which is [x y]. n is the number of vertices.
%     alpha       AC elasticity (1st order) parameter ranges from 0 to 1.
%     beta        AC rigidity (2nd order) parameter ranges from 0 to 1.
%     tau         time step of each iteration.
%     Fext        the external force field,d1-by-d2-by-2 matrix, 
%                 the force at (x,y) is [Fext(y,x,1) Fext(y,x,2)].
%     iter        number of iterations, usually ranges from 1 to 5.
%     type        'close' - close contour (default), the last vertex and
%                           first vertex are connected 
%                 'open'  - open contour,  the last vertex and first vertex
%                           are not connected 
%               
%     Outputs
%     vertex1     position of the vertices after deformation, n-by-2 matrix
%     
%     Note that if the vertices are outside the valid range, i.e., y>d1 ||
%     y<1 || x>d2 || x<1, they will be pulled inside the valid range. 
% 
%     Example
%         See EXAMPLE_VFC, EXAMPLE_PIG.
%
%     See also AMT, AM_VFC, AM_VFK, AM_PIG, AC_INITIAL, AC_REMESH,
%     AC_DISPLAY, AM_GVF, EXAMPLE_VFC, EXAMPLE_PIG. 
% 
%     Reference
%     [1] Bing Li and Scott T. Acton, "Active contour external force using
%     vector field convolution for image segmentation," Image Processing,
%     IEEE Trans. on, vol. 16, pp. 2096-2106, 2007.  
%     [2] Bing Li and Scott T. Acton, "Automatic Active Model
%     Initialization via Poisson Inverse Gradient," Image Processing,
%     IEEE Trans. on, vol. 17, pp. 1406-1420, 2008.   
% 
% (c) Copyright Bing Li 2005 - 2009.

% Revision Log
%   11-30-2005  original 
%   01-30-2006  external force interpolation outside the image 
%   02-18-2006  add open contour codes
%   01-30-2009  minor bug fix

%% inputs check
% if ~ismember(nargin, 6:7) || ndims(Fext) ~= 3 || size(Fext,3) ~= 2,
%     error('Invalid inputs to AC_DEFORM!')    
% end
if nargin == 6
    type = 'close';
end

% N = size(vertex,1);
% if size(vertex,2) ~= 2
%     error('Invalid vertex matrix!')
% end

% if N < 3
%     return
% end

%% compute T = (I + tao*A) of equation (9) in reference [1]
% Lap = sparse(1:N, 1:N, -2) + sparse(1:N, [N 1:N-1], 1) + sparse(1:N, [2:N 1], 1);
% 
% if strcmp(type,'open'), % offset tau for boundary vertices    
%     tau = sparse(1:N, 1:N, tau);
%     tau(1) = 0;
%     tau(end) = 0;
%     offset = sparse(1:N, 1:N, 1);
%     offset(1,1)=0;      offset(N,N) = 0;
%     offset(1,2)=1;      offset(N,N-1) = 1;
%     Lap = offset*Lap;
% end
% 
% T =sparse(1:N,1:N,1)+ tau*(beta*Lap*Lap-alpha*Lap);

% T=createT(N,0,0.25,1.5);

%% Another way to compute T for close AC
% a = beta;
% b = -alpha - 4*beta;
% c = 2*alpha + 6*beta;
% 
% T = sparse(1:N,1:N,1) + tau*(sparse(1:N,1:N,c) + sparse(1:N,[N,1:N-1],b) + sparse(1:N,[2:N,1],b)...
%     + sparse(1:N,[N-1,N,1:N-2],a) + sparse(1:N,[3:N,1,2],a));

%% Deform
center = size(Fext)/2;
center = center([2,1,3]);   % *** ???
for i=1:ITER,
    IdxOut = find(vertex2DX<1 | vertex2DX>size(Fext,2) | vertex2DY<1 | vertex2DY>size(Fext,1) |...
        vertex2DZ<1 | vertex2DZ>size(Fext,3));
    IdxIn = setdiff(1:prod(size(vertex2DX)),IdxOut);
    for i=1:3,
        % interpolate the external force for vertices within the range
       % F(IdxIn,i)  = interp2(Fext(:,:,i),vertex(IdxIn,1),vertex(IdxIn,2),'*linear');
        F(IdxIn,i)  = interp3(Fext(:,:,:,i),vertex2DX(IdxIn),vertex2DY(IdxIn),vertex2DZ(IdxIn),'linear');
    end
    
%     F(IdxOut,1) = center(1)-vertex2DX(IdxOut);  % pointing to the image center
%     F(IdxOut,2) = center(2)-vertex2DY(IdxOut);
%     F(IdxOut,3) = center(3)-vertex2DZ(IdxOut);
    if ~isempty(IdxOut)
        F(IdxOut,1) = center(1)-vertex2DX(IdxOut);  % pointing to the image center
        F(IdxOut,2) = center(2)-vertex2DY(IdxOut);
        F(IdxOut,3) = center(3)-vertex2DZ(IdxOut);
        % normalize the forces outside the image
        Fmag = sqrt(sum(F(IdxOut,:).^2,2));
        F(IdxOut,1) = F(IdxOut,1)./Fmag;
        F(IdxOut,2) = F(IdxOut,2)./Fmag;
        F(IdxOut,3) = F(IdxOut,3)./Fmag;
    end       

     [N1,N2]=size(vertex2DX);

    p=beta;
    q=-alpha-4*beta;
    r=1+2*alpha+6*beta;
    F1=reshape(F(:,1),N1,N2);    % ***
    F2=reshape(F(:,2),N1,N2);    % ***
    F3=reshape(F(:,3),N1,N2);    F3=zeros(N1,N2);
    
    mat_deriv2=[-1/12,4/3,-2.5,4/3,-1/12];
    % mat_deriv2=mat_deriv2/sum(abs(mat_deriv2));
    mat_deriv4=[1,-4,6,-4,1];
    % mat_deriv4=mat_deriv4/sum(abs(mat_deriv4));
    
    vXder2=imfilter(vertex2DX,mat_deriv2','circular','same','conv'); %kata grammh
    vXder2=imfilter(vXder2,mat_deriv2,'symmetric','same','conv');    %kata sthlh
    vYder2=imfilter(vertex2DY,mat_deriv2','circular','same','conv');
    vYder2=imfilter(vYder2,mat_deriv2,'symmetric','same','conv');
    vZder2=imfilter(vertex2DZ,mat_deriv2','circular','same','conv');
    vZder2=imfilter(vZder2,mat_deriv2,'symmetric','same','conv');
    vXder2(1:2,:)=0; vXder2(end-1:end,:)=0;
    vYder2(1:2,:)=0; vYder2(end-1:end,:)=0;
    vZder2(1:2,:)=0; vZder2(end-1:end,:)=0;
    
    vXder4=imfilter(vertex2DX,mat_deriv4','circular','same','conv');
    vXder4=imfilter(vXder4,mat_deriv4,'symmetric','same','conv');
    vYder4=imfilter(vertex2DY,mat_deriv4','circular','same','conv');
    vYder4=imfilter(vYder4,mat_deriv4,'symmetric','same','conv');
    vZder4=imfilter(vertex2DZ,mat_deriv4','circular','same','conv');
    vZder4=imfilter(vZder4,mat_deriv4,'symmetric','same','conv');
%     vXder4(1:2,:)=0; vXder4(end-1:end,:)=0;
%     vYder4(1:2,:)=0; vYder4(end-1:end,:)=0;
%     vZder4(1:2,:)=0; vZder4(end-1:end,:)=0;
    [max(vXder2(:)),max(vYder2(:)),max(vZder2(:))];
    [min(vXder2(:)),min(vYder2(:)),min(vZder2(:))];
    
    vXnew=vertex2DX+tau*(alpha*vXder2 - beta*vXder4 + F1);
    vYnew=vertex2DY+tau*(alpha*vYder2 - beta*vYder4 + F2);
    vZnew=vertex2DZ+tau*(alpha*vZder2 - beta*vZder4 + F3);
    
    % smotthing the surface after iteration
    % along the closed contour it should be 'circular'
    % along the meridian it should be 'symmetric'
    mat_smooth=[1,4,1]/6;   % ones(1,3)/3;  ones(3,1)/3; 
    vXnew=imfilter(vXnew,mat_smooth,'symmetric','same','conv');
    vXnew=imfilter(vXnew,mat_smooth','circular','same','conv');
    
    vYnew=imfilter(vYnew,mat_smooth,'symmetric','same','conv');
    vYnew=imfilter(vYnew,mat_smooth','circular','same','conv');
    
    vZnew=imfilter(vZnew,mat_smooth,'symmetric','same','conv');
    vZnew=imfilter(vZnew,mat_smooth','circular','same','conv');
    
%    vZnew=imfilter(vZnew,ones(1,3)/3,'symmetric','same','conv');
%    vZnew=imfilter(vZnew,ones(3,1)/3,'circular','same','conv');
    
    
%     mat_deriv2=[0,1/4,0;...
%         1/4,-1,1/4;...
%         0,1/4,0];
%     mat_deriv4=[0,1/4,0;...
%         1/4,-1,1/4;...
%         0,1/4,0];
    
%     T1=vertex2DX + tau*F1';  T1=[zeros(2,N2); T1; zeros(2,N2)];
%     T2=vertex2DY + tau*F2';  T2=[zeros(2,N2); T2; zeros(2,N2)]; 
%     T3=vertex2DZ + tau*F3';  T3=[zeros(2,N2); T3; zeros(2,N2)];
%     T1new=imfilter(T1,pinakas,'circular','same','conv');
%     T2new=imfilter(T2,pinakas,'circular','same','conv');
%     T3new=imfilter(T3,pinakas,'circular','same','conv');
%     vertex=[T1new(:),T2new(:),T3new(:)];
   % vertex = invT*(vertex+tau*double(F));
   % vertex = T \(vertex+tau*double(F));   % equation (9) in reference [1]
    
end