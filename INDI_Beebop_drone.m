clear; close all; clc;
try
    fclose(instrfindall);
catch
end
rosshutdown;

% Network
rosinit('192.168.0.100');
pub = rospublisher('/B1/cmd_vel','geometry_msgs/Twist');
msg = rosmessage(pub);

% Criação de objeto para o joystick em Matlab
J = vrjoystick(1);

% take off
pub_takeoff = rospublisher('/B1/takeoff','std_msgs/Empty');
msg_takeoff = rosmessage(pub_takeoff); 

% land
pub_land = rospublisher('/B1/land','std_msgs/Empty');
msg_land = rosmessage(pub_land); 

% pose do robô via optitrack (via NATNET)
pose = rossubscriber('/natnet_ros/B1/pose');

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% timers
T_exp = 30; % Tempo de experimento
t_exp = tic;
T_run = 1/30; % Período do experimento
% T_run = 1/50;
t_run = tic;
T_draw=0;
tempo = [];

%% Vetores de armazenamento
X_ant = [0;0;0]; % Vetor de posição anterior
X_dot_ant = [0;0;0]; % Vetor de velocidade anterior
X_2dot_ant = [0;0;0]; % Vetor de aceleração anterior
nuo_ant = [0;0];
psi_ant = 0;
pd = []; % Posição desejada para plot
pr = []; % Posição realizada para plot
pveld = []; % Velocidade desejada para plot
pvelr = []; % Velocidade realizada para plot
paccr = []; % Aceleração realizada para plot
paccd = []; % Aceleração realizada para plot
pu = []; % Esforço de controlador
er = []; % Erro para plot
ppsid = []; % Orientação desejada
ppsir = []; % Orientação realizada
pibks = []; % Vetor de armazenamento controle IBKS
thetai = 0;
phii = 0;
Xfo = [0;0;0];
qx = [0;0];
qy = [0;0];
vsx = [];
vsy = [];

%% Parametros 
flag = 1; % Linear = 0; INDI = 1;
alfa = .5; % Ganho do filtro de primeira ordem para derivada numerica
alfau = .05; % Ganho do filtro de primeira ordem para derivada numerica em nuo
lambda = 1;
tao = 1;
w = (2*pi)/5; % Frequência da trajetória
theta_max = deg2rad(10); % Angulo maximo desejado em Theta
phi_max = deg2rad(10); % Angulo maximo desejado em Phi
psi_max = deg2rad(100); % Angulo maximo desejado em Psi
z_max = 1; % Velocidade máxima desejada em z

%% Ganhos / Parametros
Kd = diag([9 7]); % Ganho diferencial (Em relação ao erro de velocidade)
Kp = diag([7.5 5.5]); % Ganho proporcional (Em relação ao erro de posicionamento)
Ku = 5*diag([.88 .88]); % Parametro de modelagem em relação a u
Kv = diag([0.18227 0.17095]); % Parametro de modelagem em relação ao disturbio de flapping
Kz = 1; % Ganho em z
K_psi = 5; % Ganho em psi

%% Filtro de segunda ordem

wn = 50;
zeta = 1;

A = [0 1; -wn^2 -2*zeta*wn];
B = [0; wn^2];
C = [1 0; 0 1; -wn^2 -2*zeta*wn];
D = [0; 0; wn^2];

Ad = eye(2) + A*T_run;
Bd = B*T_run;

%% Drone takeoff
send(pub_takeoff,msg_takeoff)
pause(3);

%% Loop de controle
try
while toc(t_exp) < T_exp
    if toc(t_run) > T_run
        tempo = [tempo toc(t_exp)];
        dt = toc(t_run);
        t_run = tic;
        t = toc(t_exp);
        t_corpo = tic;
        
%       %% Leitura da pose do robô via optitrack
        quat = [pose.LatestMessage.Pose.Orientation.W pose.LatestMessage.Pose.Orientation.X pose.LatestMessage.Pose.Orientation.Y pose.LatestMessage.Pose.Orientation.Z]; % Recebe as informações de orientação em quaternal
        EulZYX = quat2eul(quat); % converte quaternal para euler (em rad)
        position = [pose.LatestMessage.Pose.Position.X;pose.LatestMessage.Pose.Position.Y;pose.LatestMessage.Pose.Position.Z]; % Recebe informações de posição
        anglesXYZ = [EulZYX(3); EulZYX(2); EulZYX(1)]; % Reorganiza o vetor para theta, phi, psi

        X = position; % Vetor de posição  
        X_dot = alfa*((X - X_ant)/dt) + (1 - alfa)*X_dot_ant; % Derivação numérica da posição com filtro de primeira ordem
        X_2dot = alfa*((X_dot - X_dot_ant)/dt) + (1 - alfa)*X_2dot_ant; % Derivação numérica da velocidade com filtro de primeira ordem
        X_2dot_ant = X_2dot; % Recebe aceleração anterior
        X_dot_ant = X_dot; % Recebe velocidade anterior
        X_ant = X; % Recebe posição anterior
        psi = anglesXYZ(3); % Orientação do drone
        nuo = alfau*(anglesXYZ(1:2) - nuo_ant) + (1 - alfau)*nuo_ant; % Angulos reais do drone em theta e phi
        nuo_ant = nuo;


%         X = inv(1 - (dt/tao))*(X_ant + (dt/tao)*position);
%         psi = inv(1 - (dt/tao))*(psi_ant + (dt/tao)*anglesXYZ(3)); % Orientação do drone
%         nuo = inv(1 - (dt/tao))*(nuo_ant + (dt/tao)*anglesXYZ(1:2));
%         X_dot = (X - X_ant)/dt;
%         X_2dot = (X_dot - X_dot_ant)/dt;
%         X_ant = X;
%         X_dot_ant = X_dot;
%         X_2dot_ant = X_2dot; % Recebe aceleração anterior
%         nuo_ant = nuo;
%         psi_ant = psi;
        
%         sx = C*qx + D*position(1);
%         sy = C*qy + D*position(2);
%         qx = Ad*qx + Bd*position(1);
%         qy = Ad*qy + Bd*position(2);
% 
%         vsx = [vsx sx];
%         vsy = [vsy sy];
% 
%         X = [sx(1); sy(1); position(3)];
%         X_dot = [sx(2); sy(2); 0];
%         X_2dot = [sx(3); sy(3); 0];

        %% PLANEJADOR DE MOVIMENTO
        % % Lemniscata
        Xd = [.7*sin(w*t); .7*sin(2*w*t); 1]; % Posição desejada
        Xd_dot = [.7*cos(w*t)*w; .7*cos(2*w*t)*2*w; 0]; % Velocidade desejada
        Xd_2dot = [-.7*sin(w*t)*w^2; -.7*sin(2*w*t)*4*w^2; 0]; % Aceleração desejada

%         Xd = [0; 0; 1]; % Posição desejada
%         Xd_dot = [0; 0; 0]; % Velocidade desejada
%         Xd_2dot = [0; 0; 0]; % Aceleração desejada

%         Xd = [sin(w*t); cos(w*t); 1+0.25*sin(2*w*t)]; % Posição desejada
%         Xd_dot = [cos(w*t)*w; -sin(w*t)*w; 0.25*cos(2*w*t)*2*w]; % Velocidade desejada
%         Xd_2dot = [-sin(w*t)*w^2; -cos(w*t)*w^2; -0.25*sin(2*w*t)*4*w^2]; % Aceleração desejada

        % % Orientação
%         psid = [atan2(Xd_dot(2),Xd_dot(1)); 0]; % Orientação desejada
        psid = [0; 0];

        pd = [pd Xd(1:3)]; % Armazenamento da posição desejada
        pveld = [pveld Xd_dot(1:3)]; % Armazenamento da velocidade desejada
        paccd = [paccd Xd_2dot(1:3)]; % Armazenamento da velocidade desejada
        ppsid = [ppsid psid(1)]; % Armazenamento da orientação desejada

        %% LEI DE CONTROLE

        R = [cos(psi) sin(psi);
            sin(psi) -cos(psi)];

        X_til = Xd - X; % Erro de posicionamento
        X_dot_til = Xd_dot - X_dot; % Erro de Velocidade

        x_2dot_ref = Xd_2dot(1:2) + Kd*X_dot_til(1:2) + Kp*X_til(1:2); % Aceleração de referência

        if flag == 0
            nu = inv(R*Ku)*(x_2dot_ref + Kv*X_dot(1:2)); % Lei de controle linear

            theta = min(max(nu(1),-1),1); % Saturação de +-1 em theta Linear
            phi = min(max(nu(2),-1),1); % Saturação de +-1 em phi Linear
        else if flag == 1
                nui = nuo + lambda*inv(R*Ku)*(x_2dot_ref - X_2dot_ant(1:2)); % Lei de controle IBKS

                thetai = min(max(nui(1),-1),1); % Saturação de +-1 em theta IBKS
                phii = min(max(nui(2),-1),1); % Saturação de +-1 em phi IBKS
        end
        end

        %% Controle em z

        Z_dot_ref = Xd_dot(3) + Kz*X_til(3); % Velocidade de referência em z

        Z_dot_ref = min(max(Z_dot_ref/z_max,-1),1); % Saturação de +-1m/s em z

        %% Controle de orientação

        psi_til = psid(1) - psi; % Erro em psi

        % Filtro para otimização de caminho para orientação 
        if abs(psi_til) > pi
            psi_til = psi_til - sign(psi_til)*2*pi;
        end

        psi_dot_ref = psid(2) + K_psi*psi_til; % Velocidade de referência em psi

        psi_dot_ref = min(max(psi_dot_ref/psi_max,-1),1); % Limitador do controlador em z

        if flag == 0
            u = [theta; -phi; Z_dot_ref; psi_dot_ref]; % Vetor de comandos de controle Linear
        else if flag == 1
            u = [thetai; -phii; Z_dot_ref; psi_dot_ref]; % Vetor de comandos de controle IBKS
        end
        end

        pr = [pr X]; % Armazenamento de posição para calculo de erro
        pvelr = [pvelr X_dot]; % Armazenamento de velocidade para calculo de erro
        paccr = [paccr X_2dot]; % Armazenamento de Aceleração para calculo de erro
        ppsir = [ppsir psi]; %  Armazenamento de orientação para calculo de erro
        pu = [pu u]; % Armazenamento do sinal de controle para plot 
        
        %% Segurança
%         disp(u')        
        
%         u = [0 0 0 0]';
        
        % JOYSTICK
        Analog = axis(J); Digital = button(J);
        
        if abs(Analog(2)) > 0.1 || abs(Analog(3)) > 0.1 || abs(Analog(4)) > 0.1 || abs(Analog(5)) > 0.1
            u(1) = -Analog(5);
            u(2) = -Analog(4);
            u(3) = -Analog(2);
            u(4) = Analog(3);
        end

        % Botao de emergencia e Botao A do Joystick
        drawnow
        if btnEmergencia == 1 || Digital(1) == 1
            send(pub_land,msg_land);
            disp('Botao joystick')
            break
        end
        
        if sum(abs(X - X_ant)) == 0 %caso o robo nao seja detectado pelo optrack por mais de meio segundo
            if toc(t_corpo) > 0.5
                disp('encerrado por perdar o corpo por 500ms');
                send(pub_land,msg_land);
                break
            end
        else
            t_corpo = tic;
        end

        if abs(X(1)) > 2.5 ~= 0 % Parede virtual em X
            disp('encerrado por exceder as paredes virtuais');
            send(pub_land,msg_land);
            break
        end
        
        if abs(X(2)) > 1.5 ~= 0 % Parede virtual em Y
            disp('encerrado por exceder as paredes virtuais');
            send(pub_land,msg_land);
            break
        end
        
        if abs(X(3)) > 2.5 ~= 0 % Parede virtual em Z
            disp('encerrado por exceder as paredes virtuais');
            send(pub_land,msg_land);
            break
        end

        %% ENVIO DOS SINAIS DE CONTROLE
        
%         if toc(t_exp) > 4
%         disp(u')
        msg.Linear.X = u(1);
        msg.Linear.Y = u(2);
        msg.Linear.Z = u(3);
        msg.Angular.Z = u(4);
        
        send(pub,msg)
%         end
    end
end
catch ME
disp('Erro no codigo encontrado pela função try');
msg.Linear.X = 0;
msg.Linear.Y = 0;
msg.Linear.Z = 0;
msg.Angular.Z = 0;
disp('');
disp(ME);
disp('');
send(pub_land,msg_land);
end

%% ENVIO DE ZEROS AO SAIR DO LOOP
msg.Linear.X = 0;
msg.Linear.Y = 0;
msg.Linear.Z = 0;
msg.Angular.Z = 0;
disp('saiu do código')
send(pub,msg)

for i=1:5
send(pub_land,msg_land);
pause(0.2);
end

er = pd - pr; % Calculo de erros de posicionamento
err = ppsid - ppsir;
% d_ec=sqrt(er(1,:).^2+er(2,:).^2); % Calculo de erro euclidiano

figure('Name','Graficos de erro')
subplot(4,1,1)
plot (tempo,er(1,:),'b','LineWidth',1);
xlabel('Tempo(s)');ylabel('Erro em x(m)');
grid on
subplot(4,1,2)
plot (tempo,er(2,:),'r','LineWidth',1)
xlabel('Tempo(s)');ylabel('Erro em y(m)');
grid on
subplot(4,1,3)
plot (tempo,er(3,:),'k','LineWidth',1)
xlabel('Tempo(s)');ylabel('Erro em z(m)');
grid on
subplot(4,1,4)
plot (tempo,err,'c','LineWidth',1)
xlabel('Tempo(s)');ylabel('Erro em psi(rad)');
grid on

figure('Name','Gráficos de posição vs Tempo')
subplot(4,1,1)
plot (tempo,pr(1,:),'r','LineWidth',1);
hold on
% plot (tempo,vsx(1,:),'k','LineWidth',1);
hold on
plot (tempo,pd(1,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Posição em x (m)');
legend('Posição realizada','Posição desejada');
grid on
subplot(4,1,2)
plot (tempo,pr(2,:),'r','LineWidth',1);
hold on
% plot (tempo,vsy(1,:),'k','LineWidth',1);
hold on
plot (tempo,pd(2,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Posição em y (m)');
legend('Posição realizada','Posição desejada');
grid on
subplot(4,1,3)
plot (tempo,pr(3,:),'r','LineWidth',1);
hold on
plot (tempo,pd(3,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Posição em z (m)');
legend('Posição realizada','Posição desejada');
grid on
subplot(4,1,4)
plot (tempo,ppsir,'r','LineWidth',1);
hold on
plot (tempo,ppsid,'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Posição em psi (rad)');
legend('Posição realizada','Posição desejada');
grid on

figure('Name','Gráficos de velocidade vs Tempo')
subplot(3,1,1)
plot (tempo,pvelr(1,:),'r','LineWidth',1);
hold on
% plot (tempo,vsx(2,:),'k','LineWidth',1);
hold on
plot (tempo,pveld(1,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Velocidade em x (m/s)');
legend('Velocidade realizada','Velocidade desejada');
grid on
subplot(3,1,2)
plot (tempo,pvelr(2,:),'r','LineWidth',1);
hold on
% plot (tempo,vsy(2,:),'k','LineWidth',1);
hold on
plot (tempo,pveld(2,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Velocidade em y (m/s)');
legend('Velocidade realizada','Velocidade desejada');
grid on
subplot(3,1,3)
plot (tempo,pvelr(3,:),'r','LineWidth',1);
hold on
plot (tempo,pveld(3,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Velocidade em z (m/s)');
legend('Velocidade realizada','Velocidade desejada');
grid on

figure('Name','Gráficos de Aceleração vs Tempo')
subplot(2,1,1)
plot (tempo,paccr(1,:),'r','LineWidth',1);
hold on
% plot (tempo,vsx(3,:),'k','LineWidth',1);
hold on
plot (tempo,paccd(1,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Aceleração em x (m/s^2)');
legend('Aceleração realizada','Aceleração desejada');
grid on
subplot(2,1,2)
plot (tempo,paccr(2,:),'r','LineWidth',1);
hold on
% plot (tempo,vsy(3,:),'k','LineWidth',1);
hold on
plot (tempo,paccd(2,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Aceleração em y (m/s^2)');
legend('Aceleração realizada','Aceleração desejada');
grid on

figure('Name','Esforço do controlador')
subplot(4,1,1)
plot (tempo,pu(1,:),'b');
xlabel('Tempo(s)');ylabel('Esforço do controlador em theta');
grid on
subplot(4,1,2)
plot (tempo,pu(2,:),'b');
xlabel('Tempo(s)');ylabel('Esforço do controlador em phi');
grid on
subplot(4,1,3)
plot (tempo,pu(3,:),'b');
xlabel('Tempo(s)');ylabel('Esforço do controlador em z');
grid on
subplot(4,1,4)
plot (tempo,pu(4,:),'b');
xlabel('Tempo(s)');ylabel('Esforço do controlador em psi');
grid on
