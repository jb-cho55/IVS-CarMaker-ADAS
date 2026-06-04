function R = fp_metrics(ergfile, gx, gy, gdeg)
%FP_METRICS  Read a CarMaker erg and compute final parking precision.
%   Reads rear-bumper pose (Car.Fr1) + speed; returns struct R:
%     .x .y .rz .vx (full traces, rear-bumper) ; .t if available
%     .err    final position error [m]   = hypot(x_end-gx, y_end-gy)
%     .yawerr final heading error [deg]
%     .nrev   # reverse samples (vx<-0.1)
%     .xf .yf .rzf  final pose (deg)
d = cmread(ergfile);
x = d.Car_Fr1_tx.data(:); y = d.Car_Fr1_ty.data(:); rz = d.Car_Fr1_rz.data(:);
try, vx = d.Car_Gen_vx_1.data(:); catch, vx = zeros(size(x)); end
try, t  = d.Time.data(:);        catch, t  = (1:numel(x))'; end
gth = gdeg*pi/180;
R = struct();
R.x = x; R.y = y; R.rz = rz; R.vx = vx; R.t = t;
R.err    = hypot(x(end)-gx, y(end)-gy);
R.yawerr = abs(mod(rz(end)-gth+pi, 2*pi) - pi) * 180/pi;
R.nrev   = sum(vx < -0.1);
R.xf = x(end); R.yf = y(end); R.rzf = rz(end)*180/pi;
fprintf('final(%.2f,%.2f,%.1fdeg) err=%.3fm yaw=%.2fdeg rev=%d\n', R.xf,R.yf,R.rzf, R.err,R.yawerr,R.nrev);
end
