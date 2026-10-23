function report=sync_uart_session(folder)
% Import the actual 40Hz retained records; control remains 200Hz.
% Preserve gain revisions and exclude snapshots spanning online changes.
catalog=jsondecode(fileread(fullfile(folder,'catalog.json')));
all=table;summary=[];
for k=1:numel(catalog)
 c=catalog(k);file=fullfile(folder,sprintf('capture_%04d_r%d.csv',c.capture,c.revision));
 T=readtable(file,'VariableNamingRule','preserve');
 T.revision=repmat(c.revision,height(T),1);
 T.transition_window=repmat(c.transition_window,height(T),1);
 g=c.gains;fields=fieldnames(g);
 for j=1:numel(fields),T.(fields{j})=repmat(g.(fields{j}),height(T),1);end
 all=[all;T]; %#ok<AGROW>
 if ~c.transition_window
  summary(end+1,:)=[c.capture c.revision c.pitch_mean_deg c.pitch_rms_deg c.pitch_std_deg c.speed_rms_m_s c.command_rms_A(:)' c.saturation_fraction]; %#ok<AGROW>
 end
end
% Prefer a settled snapshot over an overlapping transition snapshot.
all=sortrows(all,{'transition_window','t_us'});
[~,index]=unique(all.t_us,'stable');observed=sortrows(all(index,:),'t_us');
observed.time_s=(observed.t_us-observed.t_us(1))/1e6;
observed.forward_sum_iq_A=-observed.iq_a+observed.iq_b;
observed.differential_iq_A=observed.iq_b+observed.iq_a;
report=struct('unique_samples',height(observed),'revisions',unique(observed.revision)',...
 'control_hz',200,'recorded_hz',40,'source','Actual serial records, no interpolation',...
 'limits','40Hz cannot identify millisecond FOC delay. Mechanical parameters are not changed by this import.');
save(fullfile(folder,'measured_observations.mat'),'observed','catalog','report');
metrics=array2table(summary,'VariableNames',{'capture','revision','pitch_mean_deg','pitch_rms_deg','pitch_std_deg','speed_rms_m_s','command_a_rms_A','command_b_rms_A','saturation_fraction'});
writetable(metrics,fullfile(folder,'matlab_metrics.csv'));
fid=fopen(fullfile(folder,'matlab_sync.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
disp(metrics(max(1,height(metrics)-5):end,:));disp(report);
end
