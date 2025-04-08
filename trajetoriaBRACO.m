%% Declaracao do robo e Calculo da cinemática direta (a partir dos ângulos, gera a posição)
% Definicao dos links (tamanhos com cm)
basePrincipal = 7;
linkMeio = 9;
linkAteGarra = 8;

% Plotando o braco robotico
e = ETS3.Rz("q1")*ETS3.Tz(basePrincipal)*ETS3.Ry("q2")*ETS3.Tz(linkMeio)*ETS3.Ry("q3")*ETS3.Tx(linkAteGarra)
e.teach
%% ARDUINO - Configuração da porta serial

portasDisponiveis=serialportlist
device = serialport(portasDisponiveis(1),9600,"Timeout",0.1);
configureTerminator(device,"LF")

%% trajetoria
% Composição da string para envio pela serial

%cria-se um vetor de tempo
t= 0:0.02:2;

TE1 = [10 -5 15];
TE2 = [10 5 15];
TE3 = [10 -5 5];
TE4 = [10 5 5];
TE5 = [10 -5 15];

Ts = [TE1;TE2;TE3;TE4;TE5]

%Pode ser utilizar a cinemática inversa com o comando
qs=[0 0 0] % em rad -- tem q passar pra graus
i = 0;
while (1)
    i=i+1;
    if(i>size(Ts,1))
        i=1;
    end
qs=fminsearch(@(qs) norm(se3(e.fkine(qs)).trvec-Ts(i,:)),qs) %q ele pega o ang anterior

%printtform2d(e.fkine(q),unit="deg")
e.plot(qs)
%recebendo o ang
q = [0 0 0]
q(3) = qs(3)+qs(2)
q(1) = ((rad2deg(qs(1)))+90)*10
q(2) = ((rad2deg(qs(2)))+90)*10
q(3) = (90-(rad2deg(q(3))))*10


q = compose("%4.0f", q)
% compensar os valores para as laterais (e verificar a limitacao)

comando = strcat("mtr00 ",num2str(q(1))," mtr01 ",num2str(0300), " mtr02 ", num2str(q(2)), " mtr03 ", num2str(q(3)))

%comando = 'mtr00 0900 mtr01 0300 mtr02 0900 mtr03 1200'
writeline(device,comando)
% Criar um array para armazenar as respostas
data = {};  

% Loop para ler várias linhas da porta serial
while device.NumBytesAvailable > 0
    linha = readline(device);  % Lê uma linha
    data{end+1} = linha;       % Armazena no array
end

% Exibir os dados recebidos
disp(data)
pause(2)
end
%% INICIO
comando = strcat("mtr00 ",num2str(0900)," mtr01 ",num2str(0300), " mtr02 ", num2str(0900), " mtr03 ", num2str(0900))
%limitacoes dele

%comando = 'mtr00 0900 mtr01 0300 mtr02 0900 mtr03 1200'
writeline(device,comando)
% Criar um array para armazenar as respostas
data = {};  

% Loop para ler várias linhas da porta serial
while device.NumBytesAvailable > 0
    linha = readline(device);  % Lê uma linha
    data{end+1} = linha;       % Armazena no array
end

% Exibir os dados recebidos
disp(data)


