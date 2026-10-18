function summary=run_slope_comparison
% 两种控制器、同一12度坡和停车轨迹；保留控制失败，不以失败为脚本异常。
addpath(fileparts(fileparts(mfilename('fullpath'))));paths=balance_setup;
rows=cell(4,1);k=0;
for controller={'lqr','pid'}
 for scenario={'slope','slope_hold'}
  k=k+1;[~,rows{k}]=run_balance_case(scenario{1},false,[],controller{1},1,'slope');
 end
end
summary=struct2table(vertcat(rows{:}));
writetable(summary,fullfile(paths.suite_results,'slope','all_tests.csv'));
compare_slope_controllers;disp(summary(:,{'scenario','controller','pass','hold_mean_y_error_m','hold_stability_pass','hold_tracking_pass'}));
end
