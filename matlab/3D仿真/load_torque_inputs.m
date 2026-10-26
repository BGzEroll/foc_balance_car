function t=load_torque_inputs(t,folder)
% Means cover all FOC samples in the explicitly timestamped preceding window.
q=readtable(fullfile(folder,'torque.csv'));q=q(q.count_a>0 & q.count_b>0,:);
ta=(q.time_us-(q.first_age_a_us+q.last_age_a_us)/2)*1e-6;tb=(q.time_us-(q.first_age_b_us+q.last_age_b_us)/2)*1e-6;
[ta,a]=unique(ta);[tb,b]=unique(tb);
t.mean_net_iq_a=interp1(ta,q.mean_net_iq_a(a),t.sample_us*1e-6,'linear','extrap');
t.mean_net_iq_b=interp1(tb,q.mean_net_iq_b(b),t.sample_us*1e-6,'linear','extrap');
end
