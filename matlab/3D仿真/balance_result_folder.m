function folder=balance_result_folder(resultsRoot,controller,actuatorGain,profile)
% 执行增益误差结果单独保存，不覆盖标称工况。
if isscalar(actuatorGain),actuatorGain=[actuatorGain actuatorGain];end
if nargin<4,profile='normal';end
motion=balance_motion_profile(profile);folder=fullfile(resultsRoot,'suite','y_forward');
if motion.id>0,folder=fullfile(folder,motion.name);end
if any(actuatorGain~=1)
 label=sprintf('%s_gain_%.4g_%.4g',lower(controller),actuatorGain(1),actuatorGain(2));
 folder=fullfile(folder,'robustness',strrep(label,'.','p'));
end
end
