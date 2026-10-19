function export_inner()
% Export all final transfer functions in numerical ZPK form for the report.
folder=fileparts(mfilename('fullpath'));D=load(fullfile(folder,'controller.mat'));
names={'Gq','Pq','Kq','Fq','Lq','Tq','Sq','Uq'};rows=struct([]);k=0;
fid=fopen(fullfile(folder,'zpk.txt'),'w');
fprintf(fid,'Units: rad, s. e=qc-q_measured; physical elevator=trim+Kq*e.\n');
fprintf(fid,'Tq=q/qc=Lq/(1+Lq); Sq=1/(1+Lq); Uq=Kq*Sq.\n');
fprintf(fid,'Only Tq is a stable minimal pitch-rate tracking channel. Uq has an origin pole.\n\n');
for j=1:numel(names)
 [z,p,g]=zpkdata(zpk(D.(names{j})),'v');
 fprintf(fid,'%s(s) = gain * prod(s-zero)/prod(s-pole)\nGain = %.12g\n',names{j},g);
 fprintf(fid,'Zeros:\n');for a=z.',fprintf(fid,'  %.12g %+.12gi\n',real(a),imag(a));end
 fprintf(fid,'Poles:\n');for a=p.',fprintf(fid,'  %.12g %+.12gi\n',real(a),imag(a));end
 fprintf(fid,'\n');
 for a=z.'
  k=k+1;r=struct('Transfer',string(names{j}),'Kind',"zero",'Real',real(a),'Imaginary',imag(a));
  if k==1,rows=r;else,rows(k)=r;end
 end
 for a=p.'
  k=k+1;r=struct('Transfer',string(names{j}),'Kind',"pole",'Real',real(a),'Imaginary',imag(a));rows(k)=r;
 end
end
fclose(fid);writetable(struct2table(rows),fullfile(folder,'zpk.csv'));
fprintf('ZPK_EXPORT_COMPLETE\n');
end
