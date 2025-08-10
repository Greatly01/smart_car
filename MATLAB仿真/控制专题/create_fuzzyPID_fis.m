% create_fuzzyPID_fis.m
function fis = create_fuzzyPID_fis
% 生成 Mamdani‐FIS：输入 e, ec ∈ [−10,10]，输出 dKp,dKi,dKd ∈ [−1,1]

labels = {'NB','NM','NS','ZO','PS','PM','PB'};
% 切分点
c_e  = [-6.667, -3.333, 0, 3.333, 6.667];
c_d  = [-0.667, -0.333, 0, 0.333, 0.667];  % 对应 [-1,1] 区间的等距切分

if exist('mamfis','file')==2
    fis = mamfis('Name','FuzzyPID','DefuzzificationMethod','centroid');
else
    fis = newfis('FuzzyPID','mamdani','defuzzMethod','centroid');
end

%% —— 输入 e, ec
for var = {'e','ec'}
    name = var{1};
    fis  = addInput(fis,[-10 10],'Name',name);
    % NB (Z‐型)
    fis  = addMF(fis,name,'zmf',[-10 c_e(1)],'Name','NB');
    % NM,NS,ZO,PS,PM (trimf)
    for k=1:5
        fis = addMF(fis,name,'trimf',[c_e(k)-3.333 c_e(k) c_e(k)+3.333], ...
                    'Name',labels{k+1});
    end
    % PB (S‐型)
    fis = addMF(fis,name,'smf',[c_e(end) 10],'Name','PB');
end

%% —— 输出 dKp,dKi,dKd
for idx = 1:3
    oname = sprintf('dK%d',idx);
    fis   = addOutput(fis,[-1 1],'Name',oname);
    fis   = addMF(fis,oname,'zmf',[-1 c_d(1)],'Name','NB');
    for k=1:5
        fis = addMF(fis,oname,'trimf',[c_d(k)-0.333 c_d(k) c_d(k)+0.333], ...
                    'Name',labels{k+1});
    end
    fis = addMF(fis,oname,'smf',[c_d(end) 1],'Name','PB');
end

%% —— 49 条规则硬编码
ruleList = zeros(49,7);
n = 0;
for i = 1:7        % e 索引
    for j = 1:7    % ec 索引
        n = n + 1;
        % ΔKp: 越偏差越大，增益方向与 e 符号相反 → 8‐i
        dKp = 8 - i;
        % ΔKi: |e| 大→减小(1)，中→0(4)，小→增大(7)
        de_abs = abs(i-4);
        if     de_abs>=2, dKi = 1;
        elseif de_abs==1, dKi = 4;
        else              dKi = 7;
        end
        % ΔKd: |ec| 大→减小，中→0，小→增大
        dec_abs = abs(j-4);
        if     dec_abs>=2, dKd = 1;
        elseif dec_abs==1, dKd = 4;
        else               dKd = 7;
        end

        ruleList(n,:) = [i j dKp dKi dKd 1 1];
    end
end

fis = addRule(fis,ruleList);
end
