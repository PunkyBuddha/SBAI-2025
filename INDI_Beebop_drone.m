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

% timers
T_exp = 60; % Tempo de experimento
t_exp = tic;
T_run = 1/30; % Período do experimento
t_run = tic;

T_draw=0;
tempo = [];

%% Vetores de armazenamento
pd = []; % Posição desejada para plot
pr = []; % Posição realizada para plot
pveld = []; % Velocidade desejada para plot
pvelr = []; % Velocidade realizada para plot
pu = []; % Esforço de controlador
er = []; % Erro para plot
ppsid = []; % Orientação desejada
ppsir = []; % Orientação realizada

%% Modelagem
valores_controle = [];
valores_posicao_mundo = [];
valores_velocidade_mundo = [];
valores_aceleracao_mundo = [];

%% Parametros 
X_ant = [0;0;0]; % Vetor de posição anterior
X_dot_ant = [0;0;0]; % Vetor de velocidade anterior
X_2dot_ant = [0;0;0]; % Vetor de aceleração anterior
psi_ant = 0; % Vetor de orientação anterior
psi_dot_ant = 0; % Vetor de velocidade de orientação anterior
psi_2dot_ant = 0; % Vetor de aceleração de orientação anterior
alfa = 0.5; % Ganho do filtro de primeira ordem para derivada numerica
X_dot_ref_til_ant = [0;0]; % Erro de x_dot_ref anterior
w = (2*pi)/12;
m_hat = 10; % massa estimada
g = 9.81; % aceleração gravitacional
% thrust = m*g;
theta_max = deg2rad(5);
phi_max = deg2rad(5);
psi_max = deg2rad(100);
z_max = 1;


%% Ganhos
Kd = diag([3 3]);
Kp = diag([4 4]);
Ki = diag([.01 .01]);
Kz = 2;
K_psi = 2;

% % Prepare the new file.
% vidObj = VideoWriter('Posicionamento.avi');
% open(vidObj);

%% Drone takeoff
send(pub_takeoff,msg_takeoff)
pause(3);

try
while toc(t_exp) < T_exp
    if toc(t_run) > T_run
        tempo = [tempo t0oc(t_exp)];
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
        psi_dot = alfa*((psi - psi_ant)/dt) + (1 - alfa)*psi_dot_ant; % Derivação numérica da orientação com filtro de primeira ordem
        psi_2dot = alfa*((psi_dot - psi_dot_ant)/dt) + (1 - alfa)*psi_2dot_ant; % Derivação numérica da orientação com filtro de primeira ordem
        psi_ant = psi; % Recebe orientação anterior
        psi_dot_ant = psi_dot; % Recebe velocidade de orientação anterior
        psi_2dot_ant = psi_2dot; % Recebe aceleração de orientação anterior

        %% PLANEJADOR DE MOVIMENTO
        % Lemniscata
        
        % Posicionamento
%         Xd = [0; 0; 1]; % Posição desejada
%         Xd_dot = [0; 0; 0]; % Velocidade desejada
%         Xd_2dot = [0; 0; 0]; % Aceleração desejada
        
          % % Lemniscata
%         Xd = [sin(w*t); sin(2*w*t); 1+0.25*sin(w*t)]; % Posição desejada
%         Xd_dot = [cos(w*t)*w; cos(2*w*t)*2*w; 0.25*cos(w*t)*w]; % Velocidade desejada
%         Xd_2dot = [-sin(w*t)*w^2; -sin(2*w*t)*4*w^2; -0.25*sin(w*t)*w^2]; % Aceleração desejada
        
        % Pringles
        % Xd = [sin(w*t); cos(w*t); 1+0.25*sin(2*w*t)]; % Posição desejada
        % Xd_dot = [cos(w*t)*w; -sin(w*t)*w; 0.25*cos(2*w*t)*2*w]; % Velocidade desejada
        % Xd_2dot = [-sin(w*t)*w^2; -cos(w*t)*w^2; -0.25*sin(2*w*t)*4*w^2]; % Aceleração desejada

        % % Orientação
        psid = [atan2(Xd_dot(2),Xd_dot(1)); 0]; % Orientação desejada
%         psid = [pi; 0]; % Orientação desejada

        pd = [pd Xd(1:3)]; % Armazenamento da posição desejada
        pveld = [pveld Xd_dot(1:3)]; % Armazenamento da velocidade desejada
        ppsid = [ppsid psid(1)]; % Armazenamento da orientação desejada


        %% LEI DE CONTROLE

        H = [cos(psi) sin(psi);
            sin(psi) -cos(psi)];

        R = [cos(psi) -sin(psi);
            sin(psi) cos(psi)];

        X_til = Xd - X; % Erro de posicionamento
        X_dot_til = Xd_dot - X_dot; % Erro de Velocidade

        X_dot_ref = Xd_dot(1:2) + Kp*X_til(1:2); % Velocidade de referencia

        %% Lei de controle

        x_2dot_ref = Xd_2dot(1:2) + Kd*X_dot_til(1:2) + Kp*X_til(1:2); % Aceleração de referência

        nu = inv(H)*(1/g)*(x_2dot_ref); % Lei de controle

        theta = nu(1)/theta_max;
        phi = nu(2)/phi_max;

        theta = min(max(theta,-1),1); % Saturação de +-1 em theta
        phi = min(max(phi,-1),1); % Saturação de +-1 em phi

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

        u = [theta; -phi; Z_dot_ref; psi_dot_ref]; % Vetor de comandos de controle

        pu = [pu u]; % Armazenamento dos valores de u para plot de esforço de controlador

        pr = [pr X(1:3)]; % Recebe a posição para plot de trajetoria
        pvelr = [pvelr X_dot(1:3)];
        ppsir = [ppsir psi]; % Recebe orientação para calculo de erro


        % l = 0.1;
        % T_draw=T_draw+T_run;
        % if T_draw>0.5
        %     plot3(pd(1,:),pd(2,:),pd(3,:),'b--','LineWidth',1);
        %     % xlabel('Eixo X');ylabel('Eixo Y');zlabel('Eixo Z');
        %     hold on
        %     grid on
        %     axis([-2 2 -2 2 0 2])
        %     plot3(pr(1,:),pr(2,:),pr(3,:),'g','LineWidth',1);
        %     plot3(X(1),X(2),X(3),'g*','LineWidth',1);
        %     plot3([X(1), X(1) + l*cos(psi)], [X(2), X(2) + l*sin(psi)], [X(3) X(3)],'-r','LineWidth',1);
        %     plot3(Xd(1),Xd(2),Xd(3),'b*','LineWidth',1);
        %     % legend('Trajetória desejada','Trajetória realizada')
        %     T_draw=0;
        %     hold off
        %     drawnow
        % end

        % % Write each frame to the file.
        % currFrame = getframe(gcf);
        % writeVideo(vidObj,currFrame);

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
        
%         disp(u')
        msg.Linear.X = u(1);
        msg.Linear.Y = u(2);
        msg.Linear.Z = u(3);
        msg.Angular.Z = u(4);
        
        send(pub,msg)

        pos = [X;psi];
        vel = [X_dot;psi_dot];
        acc = [X_2dot;psi_2dot];
        valores_controle = [valores_controle u];
        valores_posicao_mundo = [valores_posicao_mundo pos];
        valores_velocidade_mundo = [valores_velocidade_mundo vel];
        valores_aceleracao_mundo = [valores_aceleracao_mundo acc];
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


% % % Close the file.
% % close(vidObj);

er = pd - pr; % Calculo de erros de posicionamento
err = ppsid - ppsir;
d_ec=sqrt(er(1,:).^2+er(2,:).^2); % Calculo de erro euclidiano



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
plot (tempo,pd(1,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Posição em x (m)');
legend('Posição realizada','Posição desejada');
grid on
subplot(4,1,2)
plot (tempo,pr(2,:),'r','LineWidth',1);
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
plot (tempo,pveld(1,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Velocidade em x (m/s)');
legend('Velocidade realizada','Velocidade desejada');
grid on
subplot(3,1,2)
plot (tempo,pvelr(2,:),'r','LineWidth',1);
hold on
plot (tempo,pveld(2,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Velocidade em x (m/s)');
legend('Velocidade realizada','Velocidade desejada');
grid on
subplot(3,1,3)
plot (tempo,pvelr(3,:),'r','LineWidth',1);
hold on
plot (tempo,pveld(3,:),'b--','LineWidth',1);
xlabel('Tempo(s)');ylabel('Velocidade em x (m/s)');
legend('Velocidade realizada','Velocidade desejada');
grid on


figure('Name','Derivada do vetor Tempo')
plot(diff(tempo),'LineWidth',1);


figure('Name','Esforço do controlador')
subplot(2,1,1)
plot (tempo,pu(1,:),'b');
xlabel('Tempo(s)');ylabel('Esforço do controlador em theta');
grid on
subplot(2,1,2)
plot (tempo,pu(2,:),'b');
xlabel('Tempo(s)');ylabel('Esforço do controlador em phi');
grid on

