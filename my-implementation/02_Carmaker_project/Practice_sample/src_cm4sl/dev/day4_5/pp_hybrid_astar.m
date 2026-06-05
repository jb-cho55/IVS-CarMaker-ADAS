function [px, py, pth, pdir, np, ok] = pp_hybrid_astar(xs, ys, ths, xg, yg, thg, occ_map)
%#codegen
% PP_HYBRID_ASTAR  Hybrid A* parking planner with Reeds-Shepp (REAR-AXLE frame).
%   In : start (xs,ys,ths), goal (xg,yg,thg) [rad], occ_map (n x n, 1=occupied).
%   Out: px,py,pth (max_path x1); pdir (+1 fwd / -1 rev per point); np count; ok.
%   Bicycle fwd/rev primitives + footprint collision; open=min-heap on f=g+h;
%   closed=best-g per (cell,heading). Heuristic = Reeds-Shepp length near goal
%   (Euclidean far). RS one-shot connects to the EXACT goal pose when close.

cfg = pp_cfg();
b = cfg.bounds; n = cfg.n;
plan_res = cfg.plan_res; nxc = cfg.nxc; nyc = cfg.nyc; nth = cfg.nth;
ds = cfg.ds; Lwb = cfg.wheelbase; dmax = cfg.delta_max; Rmin = cfg.Rmin;
ahead = cfg.foot_ahead; behind = cfg.foot_behind; halfw = cfg.veh_halfW;
MAXN = cfg.max_nodes; MAXP = cfg.max_path; ns = cfg.n_steer; hw = cfg.h_weight;
rs_shot = cfg.rs_shot_dist; rs_heur = cfg.rs_heur_dist;
dth_bin = 2*pi/nth;

% steering samples (symmetric about 0)
steer = zeros(1, ns);
for i = 1:ns
    steer(i) = -dmax + (i-1)*(2*dmax/(ns-1));
end

% node + heap + closed storage
NX = zeros(MAXN,1); NY = zeros(MAXN,1); NT = zeros(MAXN,1);
NG = inf(MAXN,1);   NPar = zeros(MAXN,1); NDir = zeros(MAXN,1);
HID = zeros(MAXN,1); HF = inf(MAXN,1); hn = 0;
gbest = inf(nxc, nyc, nth);

% outputs + RS one-shot buffer
px = zeros(MAXP,1); py = zeros(MAXP,1); pth = zeros(MAXP,1); pdir = zeros(MAXP,1);
np = 0; ok = false;
rsx = zeros(MAXP,1); rsy = zeros(MAXP,1); rsth = zeros(MAXP,1); rsdir = zeros(MAXP,1);
rsn = 0; viars = false;

% seed start node
ncount = 1;
NX(1) = xs; NY(1) = ys; NT(1) = ths; NG(1) = 0; NPar(1) = 0; NDir(1) = 0;
six = min(max(floor((xs-b(1))/plan_res)+1,1),nxc);
siy = min(max(floor((ys-b(3))/plan_res)+1,1),nyc);
sit = mod(floor(mod(ths,2*pi)/dth_bin),nth)+1;
gbest(six,siy,sit) = 0;
hn = hn + 1; HID(hn) = 1;
HF(hn) = hw*heur_(xs, ys, ths, xg, yg, thg, Rmin, rs_heur);

goalid = 0; iters = 0;
while hn > 0 && iters < MAXN
    iters = iters + 1;

    % --- heap pop (min f) ---
    cur = HID(1);
    HID(1) = HID(hn); HF(1) = HF(hn); hn = hn - 1;
    hc = 1;
    while true
        hl = 2*hc; hr = 2*hc+1; sm = hc;
        if hl <= hn && HF(hl) < HF(sm), sm = hl; end
        if hr <= hn && HF(hr) < HF(sm), sm = hr; end
        if sm == hc, break; end
        tf = HF(sm); HF(sm) = HF(hc); HF(hc) = tf;
        tk = HID(sm); HID(sm) = HID(hc); HID(hc) = tk;
        hc = sm;
    end

    cx = NX(cur); cy = NY(cur); cth = NT(cur); cg = NG(cur); cdir = NDir(cur);

    % --- tol goal test (fallback) ---
    if abs(cx-xg) < cfg.pos_tol && abs(cy-yg) < cfg.pos_tol && ...
            abs(pp_angdiff(cth,thg)) < cfg.yaw_tol
        goalid = cur; ok = true; break;
    end

    % --- Reeds-Shepp one-shot to exact goal (when close) ---
    if hypot(cx-xg, cy-yg) < rs_shot
        [Tg, segLg, ~, okg] = pp_reedsshepp(cx, cy, cth, xg, yg, thg, Rmin);
        if okg
            [tx, ty, tth, tdir, tn] = pp_rs_sample(cx, cy, cth, Tg, segLg, Rmin, ds, MAXP);
            free = true;
            for sidx = 1:tn
                if pp_collision(occ_map, tx(sidx),ty(sidx),tth(sidx), ahead,behind,halfw, b, n)
                    free = false; break;
                end
            end
            if free
                goalid = cur; ok = true; viars = true; rsn = tn;
                for sidx = 1:tn
                    rsx(sidx)=tx(sidx); rsy(sidx)=ty(sidx); rsth(sidx)=tth(sidx); rsdir(sidx)=tdir(sidx);
                end
                break;
            end
        end
    end

    % --- expand successors (2 directions x ns steers) ---
    for di = 1:2
        if di == 1, dir = 1.0; else, dir = -1.0; end
        sgn = dir*ds;
        for si = 1:ns
            st = steer(si);
            if abs(st) < 1e-6
                nthp = cth;
                nxp = cx + sgn*cos(cth);  nyp = cy + sgn*sin(cth);
                mthp = cth;
                mxp = cx + 0.5*sgn*cos(cth);  myp = cy + 0.5*sgn*sin(cth);
            else
                R = Lwb/tan(st);
                nthp = cth + sgn/R;
                nxp = cx + R*(sin(nthp)-sin(cth));
                nyp = cy - R*(cos(nthp)-cos(cth));
                mthp = cth + 0.5*sgn/R;
                mxp = cx + R*(sin(mthp)-sin(cth));
                myp = cy - R*(cos(mthp)-cos(cth));
            end

            if pp_collision(occ_map, mxp,myp,mthp, ahead,behind,halfw, b, n), continue; end
            if pp_collision(occ_map, nxp,nyp,nthp, ahead,behind,halfw, b, n), continue; end

            stepc = ds;
            if dir < 0, stepc = stepc + ds*(cfg.w_rev-1); end
            if cdir ~= 0 && dir ~= cdir, stepc = stepc + cfg.w_switch; end
            stepc = stepc + cfg.w_steer*abs(st);
            ng = cg + stepc;

            kix = min(max(floor((nxp-b(1))/plan_res)+1,1),nxc);
            kiy = min(max(floor((nyp-b(3))/plan_res)+1,1),nyc);
            kit = mod(floor(mod(nthp,2*pi)/dth_bin),nth)+1;

            if ng < gbest(kix,kiy,kit) && ncount < MAXN
                gbest(kix,kiy,kit) = ng;
                ncount = ncount + 1;
                NX(ncount)=nxp; NY(ncount)=nyp; NT(ncount)=nthp;
                NG(ncount)=ng;  NPar(ncount)=cur; NDir(ncount)=dir;
                fval = ng + hw*heur_(nxp, nyp, nthp, xg, yg, thg, Rmin, rs_heur);
                hn = hn + 1; HID(hn) = ncount; HF(hn) = fval;
                hc = hn;
                while hc > 1
                    hp = floor(hc/2);
                    if HF(hp) <= HF(hc), break; end
                    tf = HF(hp); HF(hp) = HF(hc); HF(hc) = tf;
                    tk = HID(hp); HID(hp) = HID(hc); HID(hc) = tk;
                    hc = hp;
                end
            end
        end
    end
end

% --- reconstruct: Hybrid path (start -> goalid), then append RS one-shot ---
if ok && goalid > 0
    tmpx = zeros(MAXP,1); tmpy = zeros(MAXP,1); tmpt = zeros(MAXP,1); tmpd = zeros(MAXP,1);
    cntr = 0; node = goalid;
    while node ~= 0 && cntr < MAXP
        cntr = cntr + 1;
        tmpx(cntr)=NX(node); tmpy(cntr)=NY(node); tmpt(cntr)=NT(node); tmpd(cntr)=NDir(node);
        node = NPar(node);
    end
    np = cntr;
    for i = 1:cntr
        j = cntr - i + 1;
        px(i)=tmpx(j); py(i)=tmpy(j); pth(i)=tmpt(j); pdir(i)=tmpd(j);
    end
    if cntr >= 2, pdir(1) = pdir(2); end
    if viars
        for sidx = 2:rsn                       % skip RS sample 1 (== node, already added)
            if np >= MAXP, break; end
            np = np + 1;
            px(np)=rsx(sidx); py(np)=rsy(sidx); pth(np)=rsth(sidx); pdir(np)=rsdir(sidx);
        end
    end
end
end

% ===== heuristic: RS length near goal, Euclidean far ======================
function h = heur_(x, y, th, xg, yg, thg, Rmin, rs_heur)
%#codegen
eu = hypot(x-xg, y-yg);
if eu < rs_heur
    [~, ~, Lr, okr] = pp_reedsshepp(x, y, th, xg, yg, thg, Rmin);
    if okr && Lr > eu, h = Lr; else, h = eu; end
else
    h = eu;
end
end
