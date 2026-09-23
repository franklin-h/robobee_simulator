function results = analyze_yaw_spin34()
% Analyze existing recordings only; does not modify or run either model.
% Reconstruct controller gains from logged heading, geometric angular rate,
% reference and torque. The current map is independently checked against
% its calibration sweep, not treated as measured in-flight torque.
thisDir = fileparts(mfilename('fullpath'));
repo = fileparts(fileparts(thisDir));
simDir = fullfile(repo,'simulink');
fit = load(fullfile(simDir,'system id','popts_fit_20260918_005015.mat'));
results = struct([]);
f = figure('Visible','off','Position',[50 50 1450 1450]);
tiledlayout(5,2,'TileSpacing','compact','Padding','compact');
for j = 1:2
    run = j+2;
    saved = load(fullfile(simDir,'virtualBee flight logs',sprintf('yaw_spin%d.mat',run)));
    ds = saved.data;
    [t,U] = signal(ds,'/u',1);
    [~,R] = signal(ds,'/Observer for averaged system1',6);
    [~,om] = signal(ds,'/Observer for averaged system1',7);
    [~,ref] = signal(ds,'/MATLAB Function',6);
    [~,cmd] = signal(ds,'/yaw_torque_desired',1);
    cmd = cmd*1e6;
    psi = unwrap(atan2(R(:,4),R(:,1))); % row-major rotation packing
    pref = unwrap(ref(:,1));
    err = atan2(sin(pref-psi),cos(pref-psi));
    tilt = acos(max(-1,min(1,R(:,9))))*180/pi;
    dt = median(diff(t));
    W = mapWrench(U,fit.popts);
    causalCmd = filter(ones(32,1)/32,1,cmd); % actual WLQP request filter
    cutoff = t(find(t>.6 & tilt>5,1));
    good = t>.15 & t<min(cutoff,1.8) & abs(cmd)<.79;
    regressors = [err ref(:,2)-om(:,3) cumsum(err)*dt ones(size(t))];
    gains = regressors(good,:)\cmd(good);
    fitRms = sqrt(mean((regressors(good,:)*gains-cmd(good)).^2));
    early = t>.65 & t<1.8;
    compare = abs(t-1.5)<.025;
    du = [zeros(1,4);diff(U)];
    results(j).run = run;
    results(j).inferred_gains = gains;
    results(j).gain_fit_rms_uNm = fitRms;
    results(j).error_at_1p5_deg = mean(err(compare))*180/pi;
    results(j).map_request_rms_uNm = sqrt(mean((W(early,6)-causalCmd(early)).^2));
    results(j).h2_range_early = [min(U(early,4)) max(U(early,4))];
    results(j).h2_slew_pct_early = 100*mean(abs(du(early,4))>=.001998);
    results(j).first_roll_lower_bound_s = t(find(t>.6 & U(:,3)<=-.12,1));
    results(j).first_tilt_30_s = t(find(t>.6 & tilt>30,1));
    fprintf('\nyaw_spin%d\n',run);
    disp(results(j));
    for ti = [.75 1.5 2 3 3.5]
        m = abs(t-ti)<.025;
        if any(m)
            fprintf('t=%.2f: reference %.2f deg, heading %.2f deg, error %.2f deg; request %.4f, map %.4f uNm; h2 %.4f\n',...
                ti,mean(pref(m))*180/pi,mean(psi(m))*180/pi,mean(err(m))*180/pi,mean(cmd(m)),mean(W(m,6)),mean(U(m,4)));
        end
    end
    win = round((20/155)/dt); % suppress wingbeat ripple for rate display
    nexttile(j); plot(t,pref*180/pi,'k--',t,psi*180/pi,'LineWidth',1.2);
    ylabel('Heading (deg)'); title(sprintf('yaw spin %d: Kp=.1, Kd=.009, Ki=%.1f',run,gains(3)));
    legend('Logged reference','Measured','Location','northwest'); ylim([-10 300]); style(t,cutoff);
    nexttile(2+j); plot(t,ref(:,2),'k--',t,movmean(om(:,3),win),'LineWidth',1.2);
    ylabel('Yaw rate (rad/s)'); ylim([-.3 3]); style(t,cutoff);
    legend('Reference','Geometric rate, 20-cycle average','Location','northwest');
    nexttile(4+j); plot(t,causalCmd,'k-','LineWidth',1.5); hold on;
    plot(t,W(:,6),'--','Color',[.05 .5 .7],'LineWidth',1);
    ylabel('Yaw torque (uN m)'); ylim([-.05 .85]); yline(.8,':','Controller clamp'); style(t,cutoff);
    legend('Filtered request','Static map at applied u','Location','northwest');
    nexttile(6+j); plot(t,U(:,4),t,U(:,3),'LineWidth',1);
    ylabel('Actuator input'); ylim([-.15 .15]); yline(-.12,':','Roll lower limit'); style(t,cutoff);
    legend('Yaw harmonic h2','Roll split udiff','Location','northwest');
    nexttile(8+j); plot(t,tilt,'LineWidth',1.2); ylabel('Tilt (deg)'); ylim([0 90]);
    xlabel('Time (s)'); style(t,cutoff);
end
exportgraphics(f,fullfile(thisDir,'yaw_spin34_diagnosis.png'),'Resolution',140);
close(f);
save(fullfile(thisDir,'yaw_spin34_metrics.mat'),'results');
% Check the actual static calibration slices near the flight voltage range.
sweep = load(fullfile(simDir,'system id','popts_id_sweep_results_20260918_002907.mat'),...
    'U_fit','W_template_fit','campaign_fit');
U = sweep.U_fit; W = sweep.W_template_fit;
Wfit = mapWrench(U,fit.popts);
for v = [130 150]
    m = sweep.campaign_fit==3 & U(:,1)==v & abs(U(:,4))<.101;
    fprintf('\nStatic yaw calibration at %g V: h2, measured, fitted [uNm]\n',v);
    disp([U(m,4),W(m,6),Wfit(m,6)]);
end
end

function [t,x] = signal(ds,suffix,port)
for k = 1:ds.numElements
    el = ds.getElement(k);
    bp = el.BlockPath;
    if bp.getLength>0 && endsWith(bp.getBlock(bp.getLength),suffix) && el.PortIndex==port
        t = el.Values.Time(:);
        x = squeeze(double(el.Values.Data));
        if size(x,1)~=numel(t), x=x'; end
        assert(size(x,1)==numel(t),'Unexpected signal shape');
        return
    end
end
error('Missing signal: %s, port %d',suffix,port);
end

function W = mapWrench(U,popts)
C = reshape(double(popts(:)),15,6)';
W = zeros(size(U,1),6);
for i = 1:6
    A = zeros(4); k = 6;
    for r = 1:4
        for c = r:4
            A(r,c)=C(i,k); A(c,r)=C(i,k); k=k+1;
        end
    end
    W(:,i)=C(i,1)+U*C(i,2:5)'+.5*sum((U*A).*U,2);
end
end

function style(t,cutoff)
grid on; xlim([0 t(end)]);
xline(cutoff,':','Tilt >5 deg','HandleVisibility','off');
end
