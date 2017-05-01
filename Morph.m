close all
clear all
im1 = imread('woman.png');
im2 = imread('cheetah.png');

[m1,n1,p1]=size(im1);
[m2,n2,p2]=size(im2);

[pts1,pts2]=cpselect(im1,im2,[1 1;1 m1;n1 1;n1 m1],[1 1;1 m2;n2 1;n2 m2],'Wait',true);
tri= delaunayTriangulation(pts1);
noTri=length(tri.ConnectivityList);
figure
for t=0:0.01:1;
    tic
    clear imout1 imout2
    pts=pts1*(1-t)+pts2*t;
%     trimesh(tri.ConnectivityList,pts(:,1), pts(:,2),zeros(size(pts(:,1))))
    imout1=uint8(zeros(floor(m1*(1-t)+m2*t),floor(n1*(1-t)+n2*t),3));
    imout2=imout1;
    for i=1:noTri
        index=tri.ConnectivityList(i,:);
        T1=maketform('affine',pts1(index,:),pts(index,:));
        T2=maketform('affine',pts2(index,:),pts(index,:));

        xv=pts(index,1);
        yv=pts(index,2);
        [x,y]=meshgrid(floor(min(xv)):floor(max(xv)),floor(min(yv)):floor(max(yv)));
        in=inpolygon(x,y,xv,yv);

        [xm1, ym1] = tforminv(T1, x,y);
        [xm2, ym2] = tforminv(T2, x,y);

        xmp1=round(xm1);
        ymp1=round(ym1);
        xmp2=round(xm2);
        ymp2=round(ym2);

        for j=1:length(x(:,1))
            for k=1:length(x(1,:))
                if in(j,k)==1
                    imout1(y(j,k),x(j,k),:)=im1(ymp1(j,k),xmp1(j,k),:);
                    imout2(y(j,k),x(j,k),:)=im2(ymp2(j,k),xmp2(j,k),:);
                end
            end
        end
    end
    

    imout=imout1*(1-t)+imout2*t;
    imwrite(imout,strcat('morphed', num2str(t*100),'.jpg'));
    imshow(imout)
    pause(0.001)
    toc
end

% figure
% imshow(imout1)
% figure
% imshow(imout2)