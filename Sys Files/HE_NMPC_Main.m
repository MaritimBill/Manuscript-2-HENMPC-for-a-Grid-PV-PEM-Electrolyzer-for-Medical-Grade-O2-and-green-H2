function varargout = HE_NMPC_Main(action, varargin)
% HE_NMPC_Main — Manuscript 2 MATLAB/Simulink Serial HIL plant bridge.
% MATLAB owns the Arduino USB COM port; MQTT remains on the GIGA/Python
% supervisory and telemetry path, not on the Simulink plant request/response.
% HIL plant exchanges are fixed at hil_dt_s (1 s in m2_config.json); the
% supervisory economic layer remains on its separate 300 s period.

requestedAction = '';
if nargin >= 1 && ~isempty(action)
    requestedAction = lower(char(string(action)));
end

clc; close all force; rng(42,'twister');
savePath = fileparts(mfilename('fullpath'));
if isempty(savePath), savePath = pwd; end
addpath(genpath(savePath));

cfgDefaults = struct( ...
    'hospitalDemand', 48, 'pvCapacity', 250000, 'gridMax', 200000, ...
    'serialPort', 'COM5', 'serialBaud', 115200, 'sampleTime', 1, 'dt_batch_s', 300, 'hil_dt_s', 1, ...
    'minH2Rate', 1.012e-3, 'minPower', 1000, 'maxPower', 300000, ...
    'minVoltage', 1.3, 'maxVoltage', 2.5, ...
    'minCurrent', 100, 'maxCurrent', 1400, ...
    'minTemp', 50, 'maxTemp', 80, 'minPressure', 1e5, 'maxPressure', 30e5, ...
    'faradayEff', 0.95, 'areaCm2', 641, 'nCells', 80, ...
    'temperatureC', 65, 'pressurePa', 15e5, 'Jmin', 1000, 'Jmax', 20000);

buildCache = struct('mdl','','failed',false);
lastBest = [];
plantSerial = [];
simTimeS = 0;

% Dispatch command-line actions only after the nested-function state is ready.
switch requestedAction
    case 'build'
        cfg = safeCfg(); buildDynamicModel(cfg); disp('SLX built.'); return;
    case 'validate'
        validateDynamicModel(); disp('SLX validated.'); return;
    case 'start'
        cfg = safeCfg(); buildDynamicModel(cfg); disp('SLX ready.'); return;
end

fig = figure('Name','HE-NMPC — Manuscript 2 (Serial HIL bridge)','NumberTitle','off', ...
    'MenuBar','none','ToolBar','none','Color',[0 0 0], ...
    'Units','pixels','Position',[120 80 1100 640],'CloseRequestFcn',@onClose);

uicontrol(fig,'Style','text','String','Hospital Demand (L/min)', ...
    'ForegroundColor',[.9 .9 .95],'BackgroundColor',[0 0 0], ...
    'FontSize',11,'FontWeight','bold','Position',[20 580 220 22]);
demandEdit = uicontrol(fig,'Style','edit','String','48', ...
    'Position',[20 552 220 28],'BackgroundColor',[1 1 1]);

uicontrol(fig,'Style','text','String','PV (kW)', ...
    'ForegroundColor',[.9 .9 .95],'BackgroundColor',[0 0 0], ...
    'FontSize',11,'FontWeight','bold','Position',[20 515 220 22]);
pvEdit = uicontrol(fig,'Style','edit','String','250', ...
    'Position',[20 487 220 28],'BackgroundColor',[1 1 1]);

uicontrol(fig,'Style','text','String','Grid (kW)', ...
    'ForegroundColor',[.9 .9 .95],'BackgroundColor',[0 0 0], ...
    'FontSize',11,'FontWeight','bold','Position',[20 450 220 22]);
gridEdit = uicontrol(fig,'Style','edit','String','200', ...
    'Position',[20 422 220 28],'BackgroundColor',[1 1 1]);

uicontrol(fig,'Style','text','String','Arduino USB Serial Port (MATLAB owns this port)', ...
    'ForegroundColor',[.9 .9 .95],'BackgroundColor',[0 0 0], ...
    'FontSize',9,'Position',[20 385 260 22]);
serialEdit = uicontrol(fig,'Style','edit','String','COM5', ...
    'Position',[20 355 220 26],'BackgroundColor',[1 1 1]);

uicontrol(fig,'Style','pushbutton','String','RUN OPTIMIZATION / PREPARE SLX', ...
    'Position',[20 5 230 42],'FontSize',12,'FontWeight','bold', ...
    'BackgroundColor',[.16 .5 .77],'ForegroundColor',[1 1 1],'Callback',@onRun);
uicontrol(fig,'Style','pushbutton','String','▶ Start Serial Plant', ...
    'Position',[20 300 220 34],'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[.2 .6 .3],'ForegroundColor',[1 1 1],'Callback',@onStartSerial);
uicontrol(fig,'Style','pushbutton','String','⏹ Stop Serial Plant', ...
    'Position',[20 260 220 34],'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[.6 .2 .2],'ForegroundColor',[1 1 1],'Callback',@onStopSerial);

logBox = uicontrol(fig,'Style','edit','Max',2,'HorizontalAlignment','left', ...
    'Enable','inactive','BackgroundColor',[0 0 0],'ForegroundColor',[.8 .95 1], ...
    'FontName','Consolas','FontSize',10,'Position',[260 60 820 540]);
statusText = uicontrol(fig,'Style','text','String','Status: Ready', ...
    'ForegroundColor',[.85 .9 1],'BackgroundColor',[0 0 0], ...
    'FontSize',11,'HorizontalAlignment','left','Position',[260 15 820 30]);

appendLog('HE-NMPC GUI (Serial plant bridge). Click RUN OPTIMIZATION, connect Arduino USB, then Start Serial Plant.');

% ==================== MAIN WORKFLOW ====================
function onRun(~,~)
    cfg = readConfig();
    set(logBox,'String',{});
    fprintf('\n[HE-NMPC] ================================================\n');
    fprintf('[HE-NMPC] WORKFLOW START\n');
    appendLog('--- WORKFLOW START ---');
    try
        staleFiles = {'M2_PEM_Step.slx'};
        for k = 1:numel(staleFiles)
            mdl = regexprep(staleFiles{k}, '\.slx$','');
            try, if bdIsLoaded(mdl), close_system(mdl,0); end, catch, end
            if isfile(staleFiles{k}), try, delete(staleFiles{k}); catch, end, end
        end
        buildCache.mdl = ''; buildCache.failed = false; simTimeS = 0;

        appendLog('PHASE 1/4: Reading config.');
        msg = sprintf('  Demand=%.1f L/min | PV=%.0f kW | Grid=%.0f kW | HIL dt=%.1f s | SIL dt=%.0f s', ...
            cfg.hospitalDemand, cfg.pvCapacity/1000, cfg.gridMax/1000, cfg.hil_dt_s, cfg.dt_batch_s);
        fprintf('[HE-NMPC]%s\n', msg); appendLog(msg);

        appendLog('PHASE 2/4: Design optimization.');
        best = optimizeDesign(cfg);
        lastBest = best;
        msg = sprintf('  Design: A=%.1f cm², N=%d, J=%.0f A/m², T=%.1f °C, P=%.0f Pa', ...
            best.areaCm2, best.nCells, best.J, best.temperatureC, best.pressurePa);
        fprintf('[HE-NMPC]%s\n', msg); appendLog(msg);

        appendLog('PHASE 3/4: Building M2_PEM_Step.slx at the 1 s HIL exchange step.');
        plantCfg = safeCfg();
        plantCfg.hospital_demand_Lpm = cfg.hospitalDemand;
        plantCfg.pvCapacity = cfg.pvCapacity;
        plantCfg.P_grid_max_W = cfg.gridMax;
        buildDynamicModel(plantCfg);
        validateDynamicModel();
        fprintf('[HE-NMPC]   SLX built and validated.\n');
        appendLog('  SLX built and validated.');

        appendLog('PHASE 4/4: Warm-up simulation.');
        warmUpDynamicPlant(plantCfg);
        appendLog('--- READY. Connect GIGA USB and click Start Serial Plant. ---');
        set(statusText,'String','Status: SLX ready. Start Serial Plant.');
    catch ME
        msg = ['ERROR: ' ME.message];
        fprintf(2,'[HE-NMPC] %s\n', msg);
        if ~isempty(ME.stack)
            fprintf(2,'[HE-NMPC]   at %s:%d\n', ME.stack(1).name, ME.stack(1).line);
        end
        try, fprintf(2,'[HE-NMPC] --- extended ---\n%s\n', getReport(ME,'extended','hyperlinks','off')); catch, end
        appendLog(msg);
        set(statusText,'String','Workflow failed');
    end
end

% ==================== USB SERIAL PLANT BRIDGE ====================
function onStartSerial(~,~)
    try
        if ~isempty(plantSerial)
            try, configureCallback(plantSerial,"off"); catch, end

            plantSerial = [];
        end
        netCfg = safeCfg();
        portName = strtrim(char(get(serialEdit,'String')));
        if isempty(portName), portName = char(string(netCfg.serialPort)); end
        baudRate = double(netCfg.serialBaud);
        if isempty(buildCache.mdl), buildDynamicModel(netCfg); validateDynamicModel(); end
        plantSerial = serialport(portName, baudRate, 'Timeout', 10);
        configureTerminator(plantSerial,"LF");
        flush(plantSerial);
        configureCallback(plantSerial,"terminator",@onPlantSerial);
        msg = sprintf('Serial plant bridge ready on %s at %d baud. MATLAB is the sole COM-port owner.',portName,baudRate);
        appendLog(msg); fprintf('[HE-NMPC] %s\n',msg);
        set(statusText,'String',['Serial plant bridge ready: ' portName]);
    catch ME
        appendLog(['Serial bridge failed: ' ME.message]);
        fprintf(2,'[HE-NMPC] Serial bridge failed: %s\n',ME.message);
        set(statusText,'String','Serial bridge failed');
    end
end

function onStopSerial(~,~)
    try
        if ~isempty(plantSerial)
            try, configureCallback(plantSerial,"off"); catch, end
            try, flush(plantSerial); catch, end
            plantSerial = [];
            appendLog('Serial plant bridge closed.');
            fprintf('[HE-NMPC] Serial plant bridge closed.\n');
            set(statusText,'String','Serial bridge closed');
        end
    catch ME
        fprintf(2,'[HE-NMPC] Serial close failed: %s\n',ME.message);
    end
end

function onPlantSerial(src,~)
    try
        payload = strtrim(readline(src));
        if isempty(payload), return; end
        d = jsondecode(char(payload));
        typ = '';
        try, typ = char(string(d.type)); catch, end
        if ~strcmp(typ,'step') && ~strcmp(typ,'reset'), return; end
        if strcmp(typ,'step')
            out = handleStep(d);
        else
            simTimeS = 0;
            x0 = m2_initialState(safeCfg());
            out = struct('type','reset_ack','ok',true,'state',x0(:)');
        end
        writeline(src,jsonencode(out));
    catch ME
        fprintf(2,'[HE-NMPC] Serial plant request failed: %s\n',ME.message);
        try
            seqNo = 0; reqType = '';
            if exist('d','var') && isstruct(d)
                seqNo = getf(d,'seq',0);
                reqType = char(string(getf(d,'type','')));
            end
            if strcmp(reqType,'reset')
                resp = struct('type','reset_ack','ok',false,'state',[],'error',ME.message);
            else
                resp = struct('type','slx_plant_response','ok',false,'seq',seqNo,'error',ME.message);
            end
            writeline(src,jsonencode(resp));
        catch writeError
            fprintf(2,'[HE-NMPC] Could not return Serial error response: %s\n',writeError.message);
        end
    end
end

function onClose(~,~)
    try, if ~isempty(plantSerial), configureCallback(plantSerial,"off"); plantSerial=[]; end, catch, end
    try, if bdIsLoaded('M2_PEM_Step'), set_param('M2_PEM_Step','FastRestart','off'); end, catch, end
    delete(fig);
end

% ==================== SLX STEP HANDLER ====================
function out = handleStep(d)
    cfg = safeCfg();
    if isempty(buildCache.mdl), buildDynamicModel(cfg); end
    mdl = buildCache.mdl;
    if ~bdIsLoaded(mdl), load_system([mdl '.slx']); end

    dt = double(getf(d,'dt_s',cfg.hil_dt_s));
    if abs(dt - cfg.hil_dt_s) > 1e-6
        error('HIL dt mismatch: request %.6g s, configured Simulink dt %.6g s. Set both to hil_dt_s.', dt, cfg.hil_dt_s);
    end
    if ~isfield(d,'state') || numel(d.state) ~= 6
        error('step request must contain a six-element state');
    end
    x = double(d.state(:));
    I = double(getf(d,'I_cmd_A',cfg.I_min_A));
    % Environmental disturbances arrive at the GIGA from the Python supervisor
    % over MQTT, then travel with the actuator command over USB Serial.
    pvAvailable = double(getf(d,'PV_available_W',0));
    o2Demand = double(getf(d,'O2_demand_Lpm',cfg.hospital_demand_Lpm));
    tamb = double(getf(d,'Tamb_C',25));
    h2Draw = double(getf(d,'H2_draw_kg_s',0));
    tariff = double(getf(d,'tariff_KES_kWh',cfg.tariff_mid_KES));
    leakPpm = double(getf(d,'H2_leak_ppm_inject',0));
    scenarioName = char(string(getf(d,'scenario','hil_demo')));
    faultName = char(string(getf(d,'fault_state','nominal')));

    pv = m2_configVector(cfg);
    u = [I; pvAvailable; cfg.P_grid_max_W; o2Demand; tamb; h2Draw; tariff; dt];
    assignin('base','m2_x',[0, x(:)'; dt, x(:)']);
    assignin('base','m2_u',[0, u(:)'; dt, u(:)']);
    assignin('base','m2_p',[0, pv(:)'; dt, pv(:)']);

    % The model is precompiled in Accelerator/FastRestart mode during the
    % one-time warm-up. Do not reset solver settings on every Serial request.
    tic;
    simOut = sim(mdl,'ReturnWorkspaceOutputs','on');
    wall = toc * 1000;
    [y,~] = m2_lastData(simOut.get('m2_y_log'),27);
    [xn,~] = m2_lastData(simOut.get('m2_x_log'),6);
    if isempty(y) || size(y,2) ~= 27 || isempty(xn) || size(xn,2) ~= 6
        error('Unexpected Simulink log dimensions: y=%s, x=%s',mat2str(size(y)),mat2str(size(xn)));
    end
    yE = y(end,:)'; xnE = xn(end,:)';
    simTimeS = simTimeS + dt;
    pO2Tank = yE(8) + cfg.BOP_sep_dp_bar;
    gridOver = max(0,yE(15)-cfg.P_grid_max_W);
    fprintf('[HE-NMPC] step seq=%d T=%.2f I=%.1f V=%.3f purity=%.3f grid=%.0fW\n', ...
        getf(d,'seq',0),yE(1),yE(2),yE(3),yE(13),yE(15));

    out = struct('type','slx_plant_response','ok',true, ...
        'seq',getf(d,'seq',0),'time_s',simTimeS,'xnext',xnE(:)', ...
        'O2_sensor_Lpm',yE(5),'O2_flow_Lpm',yE(5),'O2_Lpm',yE(5), ...
        'O2_prod_Lpm',yE(24),'H2_export_kg_s',yE(25),'H2_vent_kg_s',yE(26), ...
        'O2_vented_L_step',yE(27),'H2_in_O2_ppm',yE(12), ...
        'P_BoP_W',yE(18),'P_system_W',yE(19),'system_efficiency_pct',yE(20), ...
        'O2_product_water_ppm',yE(21),'H2_product_water_ppm',yE(22), ...
        'water_removed_kg_s',yE(23),'H2_kg_s',yE(6),'Efficiency_pct',yE(7), ...
        'P_stack_W',yE(4),'Deg_health_pct',100.0,'I_stack_A',yE(2), ...
        'V_cell_V',yE(3),'T_stack_C',yE(1),'O2_demand_Lpm',o2Demand, ...
        'P_O2_bar',yE(8),'P_O2_tank_bar',pO2Tank,'P_H2_bar',yE(9), ...
        'O2_buffer_Nm3',yE(10),'H2_buffer_Nm3',yE(11), ...
        'purity_model_pct',yE(13),'P_PV_W',yE(14),'PV_available_W',pvAvailable, ...
        'PV_filtered_W',xnE(5),'P_grid_W',yE(15),'grid_overlimit_W',gridOver, ...
        'tariff_KES_kWh',tariff,'stress_idx',yE(16),'plant_step_ms',wall, ...
        'missed_deadline',wall > 1000.0,'O2_unmet_L_step',yE(17), ...
        'H2_leak_ppm',leakPpm,'scenario',scenarioName,'fault',faultName);
end

function v = getf(s,f,def)
    if isstruct(s) && isfield(s,f) && ~isempty(s.(f)), v=s.(f); else, v=def; end
end

% ==================== DESIGN OPTIMIZATION ====================
function [best,history] = optimizeDesign(cfg)
    best=[]; history=[];
    platemoPath='C:\Users\Admin\Desktop\HENMPC\PlatEMO-4.7\PlatEMO';
    if exist(platemoPath,'dir')
        if ~contains([path pathsep],platemoPath), addpath(genpath(platemoPath)); end
        fprintf('[HE-NMPC] PlatEMO path added.\n');
    else
        fprintf(2,'[HE-NMPC] PlatEMO not found at: %s\n',platemoPath);
    end
    hasNSGA=exist('NSGAII_AdaptiveSearch','class')==8;
    hasProblem=exist('PEM_Problem_Scaled','class')==8;
    hasFeNSGA=exist('FeaNSGA_II','class')==8;
    fprintf('[HE-NMPC] NSGAII_AdaptiveSearch=%d, PEM_Problem_Scaled=%d, FeaNSGA_II=%d\n', ...
        hasNSGA,hasProblem,hasFeNSGA);
    if hasNSGA && hasProblem
        try
            fprintf('[HE-NMPC] Running NSGAII_AdaptiveSearch ...\n');
            problem=PEM_Problem_Scaled('N',30,'maxFE',600,'parameter',cfg);
            alg=NSGAII_AdaptiveSearch('maxRuntime',10,'save',0,'outputFcn',@(~,~)[]);
            alg.Solve(problem);
            best=extractBestFromResult(alg.result);
            if ~isempty(best)
                fprintf('[HE-NMPC] NSGAII_AdaptiveSearch returned a solution.\n');
                return;
            end
        catch ME
            fprintf(2,'[HE-NMPC] NSGAII_AdaptiveSearch failed: %s\n',ME.message);
        end
    end
    fprintf('[HE-NMPC] Falling back to deterministic design.\n');
    best=fallbackDesign(cfg);
end

function best = extractBestFromResult(result)
    best=[];
    if isempty(result), return; end
    pop=result{end,2};
    if isempty(pop), return; end
    scores=inf(numel(pop),1);
    for i=1:numel(pop)
        try
            o=pop(i).obj; if iscell(o), o=o{1}; end
            o=double(o(:)'); scores(i)=sum(o(1:min(3,numel(o))));
        catch, end
    end
    [~,idx]=min(scores);
    try
        d=pop(idx).dec; o=pop(idx).obj;
        if iscell(d), d=d{1}; end
        if iscell(o), o=o{1}; end
        d=double(d(:)'); o=double(o(:)');
        if numel(d)<5, return; end
        best=struct('areaCm2',d(1),'nCells',round(d(2)),'J',d(3), ...
            'temperatureC',d(4),'pressurePa',d(5),'specificEnergy',o(1), ...
            'stressIndex',o(min(3,numel(o))));
        best.totalArea=best.areaCm2*best.nCells/10000;
    catch, best=[]; end
end

function best = fallbackDesign(cfg)
    A=641; N=80; J=12000; T=65; P=15e5;
    o2PerJ=@(j,a,n) n*(j*a/10000)*cfg.faradayEff/(4*96485)*0.032/1.429*60000;
    best=struct('areaCm2',A,'nCells',N,'J',J,'temperatureC',T,'pressurePa',P);
    best.totalArea=A*N/10000;
    if o2PerJ(J,A,N)<cfg.hospitalDemand
        scale=cfg.hospitalDemand/max(o2PerJ(J,A,N),1e-9);
        best.areaCm2=A*scale; best.totalArea=best.areaCm2*N/10000;
    end
    best.o2Flow=o2PerJ(best.J,best.areaCm2,best.nCells);
end

% ==================== CONFIGURATION ====================
function cfg = safeCfg()
    cfgPath=fullfile(savePath,'m2_data','m2_config.json');
    if isfile(cfgPath), cfg=jsondecode(fileread(cfgPath)); else, cfg=struct(); end
    defaults=struct( ...
        'N_cells',80,'A_cell_cm2',641,'O2_tank_volume_m3',2,'H2_tank_volume_m3',2, ...
        'mqtt_host','broker.hivemq.com','mqtt_port',1883,'serialPort','COM5','serialBaud',115200, ...
        'O2_reserve_Nm3',15,'initial_O2_inventory_Nm3',30,'alpha',0.448938,'j0_A_cm2',0.05428292, ...
        'r_ohm_cm2',0.169475,'k_conc',4e-6,'faradayEff',0.95, ...
        'I_min_A',100,'I_max_A',1400,'pressure_max_bar',30, ...
        'P_grid_max_W',200000,'pvCapacity',250000,'initial_temperature_C',65, ...
        'tau_I_s',5,'tau_PV_s',60,'UA_cool_W_per_K',60,'C_th_J_per_K',500000, ...
        'k_cross_0',50,'k_cross_1',5,'BOP_sep_dp_bar',0.3, ...
        'BOP_dryer_power_frac',0.03,'BOP_O2_comp_kWh_kg',0.15,'BOP_H2_comp_kWh_kg',1.5, ...
        'hospital_demand_Lpm',48,'tariff_off_KES',12,'tariff_mid_KES',18,'tariff_peak_KES',24, ...
        'hil_dt_s',1,'sil_dt_s',300,'dt_batch_s',300,'max_voltage_V',2.5, ...
        'max_temperature_C',80,'purity_min_pct',99.5);
    names=fieldnames(defaults);
    for i=1:numel(names)
        if ~isfield(cfg,names{i}) || isempty(cfg.(names{i})), cfg.(names{i})=defaults.(names{i}); end
    end
    if isfield(cfg,'BOP_O2_comp_kWh_per_kg'), cfg.BOP_O2_comp_kWh_kg=cfg.BOP_O2_comp_kWh_per_kg; end
    if isfield(cfg,'BOP_H2_comp_kWh_per_kg'), cfg.BOP_H2_comp_kWh_kg=cfg.BOP_H2_comp_kWh_per_kg; end
    if isfield(cfg,'hospitalDemand'), cfg.hospital_demand_Lpm=cfg.hospitalDemand; end
    if isfield(cfg,'dt_batch_s') && ~isfield(cfg,'sil_dt_s'), cfg.sil_dt_s=cfg.dt_batch_s; end
    if isfield(cfg,'lower_dt_s') && ~isfield(cfg,'hil_dt_s'), cfg.hil_dt_s=cfg.lower_dt_s; end
    cfg.P_grid_max_W=double(cfg.P_grid_max_W);
    cfg.serialBaud=double(cfg.serialBaud);
    cfg.serialPort=char(string(cfg.serialPort));
    cfg.pvCapacity=double(cfg.pvCapacity);
    cfg.hospital_demand_Lpm=double(cfg.hospital_demand_Lpm);
    cfg.tariff_mid_KES=double(cfg.tariff_mid_KES);
    cfg.dt_batch_s=double(cfg.dt_batch_s);
    cfg.hil_dt_s=double(cfg.hil_dt_s);
    if cfg.hil_dt_s<=0, error('hil_dt_s must be > 0'); end
    % Keep the active SIL/HIL plant geometry anchored to m2_config.json.
    % The optimizer result (lastBest) is reported as a design-study result,
    % but is not silently substituted into the HIL plant: the Python
    % collector and frozen surrogate use the same configuration file.
    fn=fieldnames(cfg);
    for i=1:numel(fn)
        if isnumeric(cfg.(fn{i})) && isscalar(cfg.(fn{i})), cfg.(fn{i})=double(cfg.(fn{i})); end
    end
end

function cfg = readConfig()
    cfg=cfgDefaults;
    cfg.hospitalDemand=str2double(get(demandEdit,'String'));
    cfg.pvCapacity=str2double(get(pvEdit,'String'))*1000;
    cfg.gridMax=str2double(get(gridEdit,'String'))*1000;
    cfg.serialPort=strtrim(char(get(serialEdit,'String')));
    cfg.P_grid_max_W=cfg.gridMax;
    cfg.pvCapacity_W=cfg.pvCapacity;
    cfg.dt_batch_s=cfgDefaults.dt_batch_s;
    cfg.hil_dt_s=cfgDefaults.hil_dt_s;
end

% ==================== BUILD / WARM-UP ====================
function mdl = buildDynamicModel(cfg)
    if buildCache.failed, error('Build failed earlier.'); end
    if ~isempty(buildCache.mdl) && isfile([buildCache.mdl '.slx'])
        mdl=buildCache.mdl;
        if ~bdIsLoaded(mdl), load_system([mdl '.slx']); end
        return;
    end
    try
        fnPath=fullfile(savePath,'m2_plant_fn.m');
        fid=fopen(fnPath,'w');
        if fid<0, error('Cannot write %s',fnPath); end
        c=onCleanup(@()fclose(fid));
        fprintf(fid,'%s\n',m2_plant_function_code_body());
        clear c;
        rehash path;
        x0=m2_initialState(cfg);
        pv=m2_configVector(cfg);
        u0=[cfg.I_min_A;0;cfg.P_grid_max_W;cfg.hospital_demand_Lpm;25;0;cfg.tariff_mid_KES;cfg.hil_dt_s];
        dt=cfg.hil_dt_s;
        assignin('base','m2_x',[0,x0(:)';dt,x0(:)']);
        assignin('base','m2_u',[0,u0(:)';dt,u0(:)']);
        assignin('base','m2_p',[0,pv(:)';dt,pv(:)']);
        m2_buildOneModel('M2_PEM_Step',dt);
        if ~bdIsLoaded('M2_PEM_Step'), load_system('M2_PEM_Step.slx'); end
        buildCache.mdl='M2_PEM_Step'; mdl=buildCache.mdl;
        fprintf('[HE-NMPC] SLX built: M2_PEM_Step.slx (fixed step %.3f s)\n',dt);
    catch ME
        buildCache.failed=true;
        try, close_system('M2_PEM_Step',0); catch, end
        error('Build failed: %s',ME.message);
    end
end

function validateDynamicModel()
    if ~isfile('M2_PEM_Step.slx'), error('SLX missing'); end
    if ~bdIsLoaded('M2_PEM_Step'), load_system('M2_PEM_Step.slx'); end
    cfg=safeCfg();
    fixedStep=str2double(get_param('M2_PEM_Step','FixedStep'));
    if abs(fixedStep-cfg.hil_dt_s)>1e-9
        error('M2_PEM_Step fixed step %.6g does not match hil_dt_s %.6g',fixedStep,cfg.hil_dt_s);
    end
    try
        mode = get_param('M2_PEM_Step','SimulationMode');
        if ~strcmpi(mode,'accelerator')
            set_param('M2_PEM_Step','SimulationMode','accelerator');
        end
        set_param('M2_PEM_Step','FastRestart','on');
    catch ME
        error('Could not enable Accelerator/FastRestart: %s',ME.message);
    end
end

function warmUpDynamicPlant(cfg)
    mdl='M2_PEM_Step';
    if ~bdIsLoaded(mdl), load_system([mdl '.slx']); end
    x0=m2_initialState(cfg); pv=m2_configVector(cfg);
    u0=[cfg.I_min_A;0;cfg.P_grid_max_W;cfg.hospital_demand_Lpm;25;0;cfg.tariff_mid_KES;cfg.hil_dt_s];
    dt=cfg.hil_dt_s;
    assignin('base','m2_x',[0,x0(:)';dt,x0(:)']);
    assignin('base','m2_u',[0,u0(:)';dt,u0(:)']);
    assignin('base','m2_p',[0,pv(:)';dt,pv(:)']);
    set_param(mdl,'FixedStep',num2str(dt),'StopTime',num2str(dt));
    simOut=sim(mdl,'ReturnWorkspaceOutputs','on');
    [y,ys]=m2_lastData(simOut.get('m2_y_log'),27);
    [x,xs]=m2_lastData(simOut.get('m2_x_log'),6);
    if size(y,2)~=27 || size(x,2)~=6
        error('Warm-up output dimensions invalid: y=%s x=%s',mat2str(ys),mat2str(xs));
    end
    fprintf('[HE-NMPC] Warm-up OK: y=%s, xnext=%s\n',mat2str(ys),mat2str(xs));
end

function appendLog(msg)
    if ~ishghandle(logBox), return; end
    prev=get(logBox,'String');
    if ischar(prev), prev={prev}; end
    prev{end+1}=['[' datestr(now,'HH:MM:SS') '] ' msg];
    if numel(prev)>400, prev=prev(end-399:end); end
    set(logBox,'String',prev,'Value',numel(prev));
    drawnow limitrate;
end
end % main function

% =====================================================================
% LOCAL FUNCTIONS — plant and Simulink builder
% =====================================================================
function x0=m2_initialState(cfg)
    nO2_0=cfg.initial_O2_inventory_Nm3*1.01325e5/(8.314462618*273.15);
    x0=[cfg.initial_temperature_C;nO2_0;0;cfg.I_min_A;0;0];
end

function pv=m2_configVector(cfg)
    names={'N_cells','A_cell_cm2','O2_tank_volume_m3','H2_tank_volume_m3','O2_reserve_Nm3', ...
        'alpha','j0_A_cm2','r_ohm_cm2','k_conc','faradayEff','I_min_A','I_max_A', ...
        'pressure_max_bar','P_grid_max_W','pvCapacity','initial_temperature_C', ...
        'tau_I_s','tau_PV_s','UA_cool_W_per_K','C_th_J_per_K','k_cross_0','k_cross_1','BOP_sep_dp_bar','BOP_dryer_power_frac', ...
        'BOP_O2_comp_kWh_kg','BOP_H2_comp_kWh_kg'};
    pv=zeros(1,numel(names));
    for i=1:numel(names)
        if isfield(cfg,names{i}), pv(i)=double(cfg.(names{i})); end
    end
end

function [data,shape]=m2_lastData(logObj,ncols)
    data=[];
    try
        if isstruct(logObj) && isfield(logObj,'signals') && isfield(logObj.signals,'values')
            data=squeeze(logObj.signals.values);
        elseif isstruct(logObj) && isfield(logObj,'Data')
            data=squeeze(logObj.Data);
        else
            data=squeeze(logObj);
        end
        if isempty(data), data=zeros(0,ncols); end
        if isvector(data) && numel(data)==ncols, data=data(:)'; end
        if size(data,2)~=ncols && size(data,1)==ncols, data=data'; end
        if size(data,2)~=ncols
            error('Expected %d columns, received %s',ncols,mat2str(size(data)));
        end
        shape=size(data);
    catch ME
        fprintf(2,'[HE-NMPC] m2_lastData failed: %s\n',ME.message);
        data=zeros(0,ncols); shape=[0,ncols];
    end
end

function m2_buildOneModel(mdlName,dt_s)
    if bdIsLoaded(mdlName), close_system(mdlName,0); end
    if isfile([mdlName '.slx']), delete([mdlName '.slx']); end
    new_system(mdlName); open_system(mdlName);
    add_block('simulink/Sources/From Workspace',[mdlName '/x_in'], ...
        'VariableName','m2_x','SampleTime',num2str(dt_s),'Position',[30 80 130 120]);
    add_block('simulink/Sources/From Workspace',[mdlName '/u_in'], ...
        'VariableName','m2_u','SampleTime',num2str(dt_s),'Position',[30 160 130 200]);
    add_block('simulink/Sources/From Workspace',[mdlName '/p_in'], ...
        'VariableName','m2_p','SampleTime',num2str(dt_s),'Position',[30 240 130 280]);
    fcnPath=[mdlName '/plant'];
    add_block('simulink/User-Defined Functions/MATLAB Function',fcnPath, ...
        'Position',[220 120 460 340]);
    stub=sprintf(['function [y,xnext] = plant(x,u,p)\n' ...
        '%%#codegen\n' ...
        '[y,xnext] = m2_plant_fn(x,u,p);\n' ...
        'end']);
    rt=sfroot;
    chart=rt.find('-isa','Stateflow.EMChart','Path',fcnPath);
    if isempty(chart), error('Could not obtain MATLAB Function chart at %s',fcnPath); end
    chart.Script=stub;
    add_block('simulink/Sinks/To Workspace',[mdlName '/y_out'], ...
        'VariableName','m2_y_log','SaveFormat','Structure With Time', ...
        'SampleTime',num2str(dt_s),'Position',[520 140 620 180]);
    add_block('simulink/Sinks/To Workspace',[mdlName '/x_out'], ...
        'VariableName','m2_x_log','SaveFormat','Structure With Time', ...
        'SampleTime',num2str(dt_s),'Position',[520 240 620 280]);
    add_line(mdlName,'x_in/1','plant/1','autorouting','on');
    add_line(mdlName,'u_in/1','plant/2','autorouting','on');
    add_line(mdlName,'p_in/1','plant/3','autorouting','on');
    add_line(mdlName,'plant/1','y_out/1','autorouting','on');
    add_line(mdlName,'plant/2','x_out/1','autorouting','on');
    set_param(mdlName,'Solver','FixedStepDiscrete','FixedStep',num2str(dt_s), ...
        'StartTime','0','StopTime',num2str(dt_s),'SimulationMode','accelerator', ...
        'FastRestart','on');
    save_system(mdlName);
end

function code=m2_plant_function_code_body()
lines={ ...
'function [y,xnext] = m2_plant_fn(x,u,p)' ...
'%#codegen' ...
'R=8.314462618; F=96485.33212;' ...
'T_C=x(1); nO2=x(2); nH2=x(3); I_act=x(4); PVf=x(5); stress=x(6);' ...
'I_cmd=u(1); PV_av=u(2); grid_max=u(3); O2_dem=u(4); Tamb=u(5); H2_export=u(6); tariff=u(7); dt=u(8);' ...
'N=p(1); A=p(2); V_O2=p(3); V_H2=p(4); Rserv=p(5);' ...
'alpha=p(6); j0=p(7); r_ohm=p(8); kc=p(9); fe=p(10);' ...
'I_min=p(11); I_max=p(12); Pmax=p(13); Pgmax=p(14); pvCap=p(15);' ...
'T0=p(16); tau_I=p(17); tau_PV=p(18); UA=p(19); Cth=p(20); k0=p(21); k1=p(22);' ...
'pO2drop=p(23); dryerFrac=p(24); O2comp=p(25); H2comp=p(26);' ...
'% An explicit zero command is an immediate shutdown, bypassing the normal minimum-load clamp.' ...
'if I_cmd<=0, I_n=0; else' ...
'aI=exp(-dt/max(tau_I,1e-3)); I_c=min(max(I_cmd,I_min),I_max); I_n=aI*I_act+(1-aI)*I_c;' ...
'end' ...
'TK=max(T_C+273.15,250);' ...
'pO2=max(0.05,nO2*R*TK/max(V_O2,1e-6)/1e5);' ...
'pH2=max(0.05,nH2*R*TK/max(V_H2,1e-6)/1e5);' ...
'Jcm=I_n/max(A,1e-9);' ...
'Erev=1.42-0.0009*(T_C-75);' ...
'etaA=(R*TK/(alpha*2*F))*asinh(Jcm/(2*max(j0,1e-9)));' ...
'etaP=(R*TK/(2*F))*log(max(pH2*sqrt(pO2),1e-8));' ...
'V=max(0.5,Erev+etaA+r_ohm*Jcm+kc*Jcm*Jcm+etaP);' ...
'ndO2=N*I_n/(4*F)*fe; ndH2=N*I_n/(2*F)*fe;' ...
'mO2=ndO2*0.032; mH2=ndH2*0.002016;' ...
'Pst=N*V*I_n; Pdry=dryerFrac*Pst;' ...
'PO2c=O2comp*mO2*3600*1000; PH2c=H2comp*mH2*3600*1000;' ...
'PBoP=Pdry+PO2c+PH2c; Psys=Pst+PBoP;' ...
'sysEff=100*(ndH2*120e6)/max(Psys,1e-3);' ...
'waterOut=ndH2*18e-3+ndO2*18e-3;' ...
'O2water=50+5*I_n/max(A,1e-9); H2water=30+4*I_n/max(A,1e-9);' ...
'% O2 delivery uses initial inventory plus current-interval Faraday production.' ...
'nd=O2_dem/60/22.414; O2_available=max(0,nO2)+ndO2*dt;' ...
'O2_requested=nd*dt; O2_served=min(O2_available,O2_requested);' ...
'supply=O2_served/max(dt,1e-9)*60*22.414;' ...
'unmet=max(0,(O2_dem-supply)*dt/60); nO2_pre_cap=max(0,O2_available-O2_served);' ...
'% H2 is exported only against an explicit offtake request; the balance is stored or vented.' ...
'H2_available=max(0,nH2)*0.002016+mH2*dt;' ...
'H2_export_step=min(H2_available,max(0,H2_export)*dt);' ...
'nH2_pre_cap=max(0,H2_available-H2_export_step)/0.002016;' ...
'Ph=N*I_n*max(V-1.48,0); Pc=UA*(T_C-Tamb);' ...
'T_n=min(max(T_C+(Ph-Pc)*dt/max(Cth,1e-3),20),120);' ...
'TKn=max(T_n+273.15,250); nO2_cap=Pmax*1e5*V_O2/(R*TKn); nH2_cap=Pmax*1e5*V_H2/(R*TKn);' ...
'O2_vent_mol=max(0,nO2_pre_cap-nO2_cap); H2_vent_kg=max(0,(nH2_pre_cap-nH2_cap)*0.002016);' ...
'nO2_n=min(nO2_cap,nO2_pre_cap); nH2_n=min(nH2_cap,nH2_pre_cap);' ...
'aPV=exp(-dt/max(tau_PV,1e-3)); PV_av=min(max(PV_av,0),pvCap); PVf_n=aPV*PVf+(1-aPV)*PV_av;' ...
'Ppv=min(Psys,max(PVf_n,0)); Pgrid=max(0,Psys-Ppv);' ...
'TKnext=max(T_n+273.15,250);' ...
'pO2tank=max(0.05,nO2_n*R*TKnext/max(V_O2,1e-6)/1e5);' ...
'pH2n=max(0.05,nH2_n*R*TKnext/max(V_H2,1e-6)/1e5);' ...
'pO2n=max(0.05,pO2tank-pO2drop);' ...
'ppm=(k0+k1*pH2n)/(Jcm+0.1); purity=min(100,max(90,100-ppm/1e4));' ...
'ds=(max(0,T_n-70)/10+max(0,Jcm-1.5)+5*max(0,V-2.2))*(dt/3600)*1e-5;' ...
's_n=stress+ds;' ...
'eff=100*(ndH2*241.8e3)/max(Pst,1e-3);' ...
'O2prod=ndO2*60*22.414; H2exportRate=H2_export_step/max(dt,1e-9); H2ventRate=H2_vent_kg/max(dt,1e-9);' ...
'y=[T_n;I_n;V;Pst;supply; mH2;eff;pO2n;pH2n;' ...
'   nO2_n*R*273.15/1.01325e5;nH2_n*R*273.15/1.01325e5;ppm;purity;' ...
'   Ppv;Pgrid;s_n;unmet;PBoP;Psys;sysEff;O2water;H2water;waterOut;' ...
'   O2prod;H2exportRate;H2ventRate;O2_vent_mol*22.414];' ...
'xnext=[T_n;nO2_n;nH2_n;I_n;PVf_n;s_n];' ...
'end'};
code=strjoin(lines,newline);
end
