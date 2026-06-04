function fp_setgoal(m, gx, gy, gdeg)
%FP_SETGOAL  Set the FP parking goal on generic_IVS (rear-bumper pose).
%   Parking_Info/Constant32 = goal x, Constant33 = goal y (rear-bumper [m]);
%   Parking_system/Constant23 = goal yaw [rad].  gdeg in degrees.
%   Requires the project open (so subsystem links resolve under generic_IVS).
pinfo = find_system(m,'LookUnderMasks','all','FollowLinks','on','BlockType','SubSystem','Name','Parking_Info');
psys  = find_system(m,'LookUnderMasks','all','FollowLinks','on','BlockType','SubSystem','Name','Parking_system');
assert(~isempty(pinfo) && ~isempty(psys), 'Parking_Info/Parking_system not found — is the project open?');
c32 = find_system(pinfo{1},'SearchDepth',1,'BlockType','Constant','Name','Constant32');
c33 = find_system(pinfo{1},'SearchDepth',1,'BlockType','Constant','Name','Constant33');
c23 = find_system(psys{1}, 'SearchDepth',1,'BlockType','Constant','Name','Constant23');
set_param(c32{1},'Value',num2str(gx));
set_param(c33{1},'Value',num2str(gy));
set_param(c23{1},'Value',num2str(gdeg*pi/180));
fprintf('goal set: x=%.2f y=%.2f yaw=%.0fdeg(%.4frad)\n', gx, gy, gdeg, gdeg*pi/180);
end
