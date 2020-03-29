clc;close all;clear;

%%???J????????*********************************************
%% left camera calibration parameter
%{
u0 = 1050.18724;
v0 = 776.90152;
fu = 1765.39009;
fv = 1764.93199;
kc = [0.00119, 0.00070, -0.00101, -0.00003, 0.00000] ;
%}

%% right camera parameter

u0 = 1067.43219;
v0 = 770.87706;
fu = 1762.49155;
fv = 1761.41036;
kc = [-0.00413, 0.01595, -0.00143, 0.00127, 0.00000] ;

%% origin calibration parameter
%{
u0=  300.38138;
v0=  256.52766;
fu=  822.34438;
fv=  822.61701;
kc= [ 0.07930  0.09615   0.00155  0.00222  0.00000 ] ;
%}
%% ?{??
%%???????o**************************************************
x = 2048;
y = 1536;
%x=640;
%y=480;
%******************************************************************
size=[0,x,0,y];

figure(1);

k=1;
%{
for i=10:10:(x-10)
        IIx(k)=i;
        k=k+1;
end
%}
for i=32:32:(x-32)
        IIx(k)=i;
        k=k+1;
end
k=1;
%{
for j=10:10:(y-10)
  IIy(k)=j;
  k=k+1;
end
%}
for j=32:32:(y-32)
  IIy(k)=j;
  k=k+1;
end

num=1;
%for i=1:(x/10-1)
for i=1:(x/32-1)
    %for j=1:(y/10-1)
    for j=1:(y/32-1)
        Ix(num)=IIx(i);
        Iy(num)=IIy(j);
        plot(Ix(num),Iy(num),'b.');hold on;
        num=num+1;
    end
end
title('Ideal pattern','fontsize',14,'fontname','Times New Roman');
xlabel('Ix(pixel)','fontsize',14,'fontname','Times New Roman');
ylabel('Iy(pixel)','fontsize',14,'fontname','Times New Roman');
axis(size);
axis ij;

%%*************************************************************************
figure(2);



%for num=1:((x/10-1)*(y/10-1))
for num=1:((x/32-1)*(y/32-1))

        fhxz(num)=(Ix(num)-u0);
        fhyz(num)=(Iy(num)-v0);
        
        hxz(num)=fhxz(num)/fu;
        hyz(num)=fhyz(num)/fv;
        r(num)=sqrt(hxz(num)^2+hyz(num)^2);
        
        hcxz(num)=(1 + kc(1)*(r(num)^2) + kc(2)*(r(num)^4) + kc(5)*(r(num)^6))*hxz(num) + 2*kc(3)*hxz(num)*hyz(num) + kc(4)*((r(num)^2) + 2*(hxz(num)^2));
        hcyz(num)=(1 + kc(1)*(r(num)^2) + kc(2)*(r(num)^4) + kc(5)*(r(num)^6))*hyz(num) + kc(3)*((r(num)^2) + 2*(hyz(num)^2)) + 2*kc(4)*hxz(num)*hyz(num);
        
        fhcxz(num)=fu*hcxz(num);
        fhcyz(num)=fv*hcyz(num);
        
        Ix_dist(num)=u0 + fhcxz(num);
        Iy_dist(num)=v0 + fhcyz(num);

        plot(Ix_dist(num),Iy_dist(num),'b.');hold on;
      
end
title('Distortion pattern','fontsize',14,'fontname','Times New Roman');
xlabel('Ix(pixel)','fontsize',14,'fontname','Times New Roman');
ylabel('Iy(pixel)','fontsize',14,'fontname','Times New Roman');
axis(size);
axis ij;
%%*************************************************************************

%for num=1:((x/10-1)*(y/10-1))
for num=1:((x/32-1)*(y/32-1))
        
        r_dist(num)=sqrt(hcxz(num)^2+hcyz(num)^2);
        
        u=[-hcxz(num)*(r_dist(num)^2), -hcxz(num)*(r_dist(num)^4), -2*hcxz(num)*hcyz(num), -(r_dist(num)^2+2*hcxz(num)^2), 4*(hxz(num)-hcxz(num))*r_dist(num)^2, 6*(hxz(num)-hcxz(num))*r_dist(num)^4, 8*(hxz(num)-hcxz(num))*hcyz(num), 8*(hxz(num)-hcxz(num))*hcxz(num)];
        
        v=[-hcyz(num)*(r_dist(num)^2), -hcyz(num)*(r_dist(num)^4), -(r_dist(num)^2+2*hcyz(num)^2), -2*hcxz(num)*hcyz(num), 4*(hyz(num)-hcyz(num))*r_dist(num)^2, 6*(hyz(num)-hcyz(num))*r_dist(num)^4, 8*(hyz(num)-hcyz(num))*hcyz(num), 8*(hyz(num)-hcyz(num))*hcxz(num)];
        k=1;
        for k=1:8
            T(2*num-1,k)=u(k);
            T(2*num,k)=v(k);
            e(2*num-1,1)=hcxz(num)-hxz(num);
            e(2*num,1)=hcyz(num)-hyz(num);
        end
end

p=inv(T'*T)*T'*e;

%%*************************************************************************
figure(3);

%for num=1:((x/10-1)*(y/10-1))
for num=1:((x/32-1)*(y/32-1))
        G(num)= 4*p(5)*r_dist(num)^2 + 6*p(6)*r_dist(num)^4 + 8*p(7)*hcyz(num) + 8*p(8)*hcxz(num) + 1;
        
        new_hxz(num) =  hcxz(num) + (hcxz(num)*(p(1)*r_dist(num)^2 + p(2)*r_dist(num)^4) + 2*p(3)*hcxz(num)*hcyz(num) + p(4)*(r_dist(num)^2 + 2*hcxz(num)^2) )/G(num);
        new_hyz(num) =  hcyz(num) + (hcyz(num)*(p(1)*r_dist(num)^2 + p(2)*r_dist(num)^4) + p(3)*(r_dist(num)^2 + 2*hcyz(num)^2) + 2*p(4)*hcxz(num)*hcyz(num) )/G(num);
        
        new_Ix(num) = u0 + fu*new_hxz(num);
        new_Iy(num) = v0 + fv*new_hyz(num);

        plot(new_Ix(num),new_Iy(num),'b.');hold on;
        
        dis_error(num) = sqrt( (new_Ix(num)-Ix(num) )^2 + (new_Iy(num)-Iy(num))^2 );
        
        dis_error2(num) = sqrt( (new_Ix(num)-Ix_dist(num) )^2 + (new_Iy(num)-Iy_dist(num))^2 );
end
title('Correction pattern','fontsize',14,'fontname','Times New Roman');
xlabel('Ix(pixel)','fontsize',14,'fontname','Times New Roman');
ylabel('Iy(pixel)','fontsize',14,'fontname','Times New Roman');
axis(size);
axis ij;
%%*************************************************************************
figure(4);
hist(dis_error);
title('Error histogram after correction','fontsize',14,'fontname','Times New Roman');
xlabel('pixels','fontsize',14,'fontname','Times New Roman');
ylabel('number of features','fontsize',14,'fontname','??????');

%%*************************************************************************
figure(5);
hist(dis_error2);
title('Error histogram before correction','fontsize',14,'fontname','Times New Roman');
xlabel('pixels','fontsize',14,'fontname','Times New Roman');
ylabel('number of features','fontsize',14,'fontname','??????');




%**************************************************************************
% fid = fopen('IxIy_position.txt','w');
% 
%         fprintf(fid,'Ix\t\t\t');
%         fprintf(fid,'Iy\t\t\t');
%         fprintf(fid,'Ix_dist\t\t');
%         fprintf(fid,'Iy_dist\t\t');
%         fprintf(fid,'Ix_correction\t');
%         fprintf(fid,'Iy_correction\t');
%         fprintf(fid,'\n');
%         
% for num=1:2961
%     
%         fprintf(fid,'%f\t',Ix(num));
%         fprintf(fid,'%f\t',Iy(num));
%         fprintf(fid,'%f\t',Ix_dist(num));
%         fprintf(fid,'%f\t',Iy_dist(num));
%         fprintf(fid,'%f\t',new_Ix(num));
%         fprintf(fid,'%f\t',new_Iy(num));
%         fprintf(fid,'\n');
% end
% 
% fclose(fid);
