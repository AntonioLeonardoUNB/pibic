# ==============================================================================
# Script de Execução de DMP no UR3 com Gravação e Plotagem Automática
# ==============================================================================

import numpy as np
import pandas as pd
import time
import rtde_receive
import rtde_control
from movement_primitives.dmp import DMP

# Importa a função de avaliação 
from compare_dmp_results import plot_tcp_pose_misto

if __name__ == "__main__":
    # -------------------------------------------------------------------------
    # 1. Leitura e Preparação dos Dados da Demonstração
    # -------------------------------------------------------------------------
    # Utilizando caminhos relativos para facilitar a replicação do repositório
    csv_file = "./exemplos/demo_exemplo.csv"
    
    df = pd.read_csv(csv_file, sep=None, engine="python")
    df.columns = df.columns.str.strip()

    # Extração das colunas correspondentes às posições das 6 juntas (q)
    cols = [f"actual_q_{i}" for i in range(6)]
    q_data = df[cols].values

    # Extração e normalização do vetor de tempo
    t = df["timestamp"].values
    
    # Cálculo do dt médio (pode ser fixado na frequência do robô, ex: 0.008s para UR3 e-Series)
    dt = np.mean(np.diff(t))        
    execution_time = t[-1] - t[0]   # Duração total do movimento
    T = t - t[0]                    # Vetor de tempo normalizado (inicia em 0)

    # -------------------------------------------------------------------------
    # 2. Configuração e Treinamento do DMP
    # -------------------------------------------------------------------------
    # n_dims = 6 (graus de liberdade do UR3)
    # n_weights_per_dim = 50 (aumenta a capacidade de modelar não-linearidades do movimento)
    dmp = DMP(n_dims=6, execution_time=execution_time, dt=dt, n_weights_per_dim=50)
    
    print("Treinando a DMP com os dados da demonstração...")
    dmp.imitate(T, q_data)

    # -------------------------------------------------------------------------
    # 3. Estabelecendo Conexão com o Robô (URSim ou Hardware Físico)
    # -------------------------------------------------------------------------
    robotIP = '127.0.0.1' # IP local para o Docker URSim
    
    # Instanciando as interfaces de recebimento (sensores) e envio (atuadores)
    rtde_r = rtde_receive.RTDEReceiveInterface(robotIP)
    rtde_c = rtde_control.RTDEControlInterface(robotIP)

    # Move o robô suavemente para o estado inicial exato da demonstração
    init_q = q_data[0]
    print("Movendo o robô para o ponto inicial da demonstração...")
    rtde_c.moveJ(init_q, speed=0.5, acceleration=0.5)

    # Atualiza a posição atual (start) e a posição final desejada (goal) na DMP
    current_joints = np.array(rtde_r.getActualQ())
    goal_joints = q_data[-1] 
    dmp.configure(start_y=current_joints, goal_y=goal_joints)

    # -------------------------------------------------------------------------
    # 4. Loop de Controle e Gravação da Trajetória (Replicação)
    # -------------------------------------------------------------------------
    nome_arquivo_saida = "./exemplos/trajetoria_exemplo.csv"
    
    rtde_r.startFileRecording(nome_arquivo_saida)
    print(f"Gravando a trajetória do robô em: {nome_arquivo_saida}")

    last_q = current_joints 
    last_qd = np.zeros(6) # Velocidade inicial assumida como zero
    num_steps = int(execution_time / dt)

    print("Iniciando a execução do movimento...")
    for step in range(num_steps):
        # O DMP calcula o próximo ponto (posição e velocidade)
        next_q, next_qd = dmp.step(last_y=last_q, last_yd=last_qd)
        
        # Envia o comando de servo para o robô acompanhar a trajetória gerada
        rtde_c.servoJ(next_q.tolist(), 0.1, 0.1, dt, 0.1, 300)
        
        last_q = next_q
        last_qd = next_qd
        time.sleep(dt) # Aguarda o ciclo de controle do robô

    # Finalização segura da comunicação e gravação
    rtde_r.stopFileRecording()
    rtde_c.servoStop()
    print("Movimento concluído e gravação de dados encerrada.")

    # -------------------------------------------------------------------------
    # 5. Avaliação Visual (Plotagem Automática)
    # -------------------------------------------------------------------------
    print("Gerando gráficos comparativos de validação...")
    
    # Passa o CSV original e o gerado nesta execução para o avaliador
    arquivos_para_plotar = [csv_file, nome_arquivo_saida]
    
    # Chama a função de plotagem (exigirá que o usuário feche a janela do gráfico para o script terminar)
    plot_tcp_pose_misto(arquivos_para_plotar, time_col='timestamp')