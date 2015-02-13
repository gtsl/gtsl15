%% Generate Mass, Thrust Data
function generateMotorData(rocketMotor)
fh = fopen(sprintf('%s%s%s','MotorData/Raw/',rocketMotor,'.txt'));

timeTmp = [];
massTmp = [];
thrustTmp = [];

cnt = true;
while cnt == true
    line = fgetl(fh);
    if line==-1
        cnt = false;
    end
    if line~=-1
        [~,rest1] = strtok(line,'=');
        [t,~] = strtok(rest1,'=''"');
        t = str2double(t);
        timeTmp=[timeTmp;t];
        [~,rest2] = strtok(rest1,'='); 
        [f,~] = strtok(rest2,'=''"');
        f = str2double(f);
        thrustTmp = [thrustTmp;f];
        [~,rest3] = strtok(rest2,'='); 
        [m,~] = strtok(rest3,'=''"');
        m = str2double(m);
        massTmp = [massTmp;m];
    end
end
%%
% Resample to higher resolutiion, units of Newtons, Kg, s
time = linspace(0,max(timeTmp),1000);
mass = interp1(timeTmp,massTmp,time,'spline') / 1000;
thrust = interp1(timeTmp,thrustTmp,time,'spline');
dt_data = time(2)-time(1);
%%
% Change mass to be a change in mass, so mass(t) = m0 + deltaMass
deltaMass = mass - mass(1);
m0=7.6370;
%%
% Save to .mat
save(sprintf('%s%s','MotorData/MAT/',rocketMotor),'time',...
    'deltaMass','thrust','dt_data','m0')
end