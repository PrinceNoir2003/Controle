% Parâmetros do motor
R = 0.179; % Resistência da armadura (Ohms)
L = 0.0455e-3; % Indutância da armadura (Henries)
kv = 797 * (2 * pi / 60); % Constante de velocidade (rad/s/V)
kt = 0.012; % Constante de torque (Nm/A)
J = 35.1e-7; % Inércia do rotor (kg·m^2)
ke = kt; % Usando kt como ke
c = 5.4096e-6; % Coeficiente de fricção viscosa (Nm/(rad/s))
Va_nominal = 10; % Voltagem nominal (V)
tau_nominal = 0.091; % Torque nominal (Nm)


% Definindo a matriz de estado A e a matriz de entrada B
A = [-R/L, -ke/L; kt/J, -c/J];
B = [1/L, 0; 0, -1/J];
C = [1, 0; 0, 1];
D = zeros(2, 2);
model = ss(A, B, C, D);

% Entrada degrau para Va e tau
Va = 10 * ones(10000, 1);
tau = 0.091 * ([zeros(5000, 1); ones(5000, 1)]);
time = linspace(0, 20, 10000)';

% Simulação
[y, t, x] = lsim(model, [Va tau], time);

% Extraindo a corrente (i) e a velocidade (w) do estado
i = x(:, 1);
w = x(:, 2);

% Cálculo das eficiências
ef_e = zeros(length(t), 1);
ef_m = zeros(length(t), 1);
for k = 1:length(t)
    [ef_e(k), ef_m(k), ef_global(k)] = calc_eficiencia(i(k), w(k), Va(k),ke, tau(k));
end

figure(1);
plot(time, tau, '-r');
grid on;
xlabel('time [t]');
ylabel('\tau [Nm]');
print('tau', '-depsc');

figure(2);
plot(time, Va, '-b');
grid on;
xlabel('time [t]');
ylabel('VA');
print('VA', '-depsc');

figure(3);
lsim(model, [Va tau], time)
print('i_w', '-depsc');

% Plotar eficiências
figure (4);
plot(t, ef_e, 'DisplayName', 'Eficiência Elétrica');
grid on;
xlabel('Tempo (s)');
ylabel('Eficiência Elétrica');
title('Eficiência Elétrica  do Motor DC');
legend;
print('ef_ele', '-depsc');

figure(5);
plot(t, ef_m, 'DisplayName', 'Eficiência Mecânica');
grid on;
xlabel('Tempo (s)');
ylabel('Eficiência Mecânica ');
ylim([0 1]);
title('Eficiência Mecânica do Motor DC');
legend;
print('efmec', '-depsc');


figure (6);
plot(t, ef_global, 'DisplayName', 'Eficiência global');
grid on;
xlabel('Tempo (s)');
ylabel('Eficiência Global');
ylim([0 1]);
title('Eficiência Global do motor dc');
legend;
print('efglob', '-depsc');

% Função para calcular eficiências
function [ef_e, ef_m, ef_global] = calc_eficiencia(i, w, Va,ke, tau)
kt = 0.012;
c = 5.4096e-6;
R = 0.179;
L = 0.0455e-3;


Tm = kt * i; % Torque mecânico gerado
Pout_mec = tau * w; %Potência de saída mecânica
Pout_ele = Va*i - (0.5*L*i^2 + R*i^2); % Potência de saída elétrica
Vb = ke * w; % Tensão de contrapressão
Pin_elec = Va * i ; % Potência de entrada elétrica
Pin_mech = Tm * w + c * w^2; % Potência total mecânica incluindo perdas por fricção
ef_e = Pout_ele / Pin_elec; % Eficiência elétrica
ef_m = Pout_mec / Pin_mech; % Eficiência mecânica
ef_global = tau*w / (Va*i) % Eficiência global
end
