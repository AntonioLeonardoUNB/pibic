# ==============================================================================
# Script de Avaliação de Trajetórias (Demonstração vs. Generalização/Replicação)
# ==============================================================================

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

def plot_tcp_pose_misto(file_paths, time_col='timestamp'):
    """
    Gera gráficos comparativos 3D (Cartesiano) e 2D (Juntas) entre uma 
    trajetória de referência (demonstração) e uma ou mais replicações.
    Calcula e exibe o erro RMSE para ambas as representações.
    """
    if not file_paths or len(file_paths) < 2:
        print("Forneça pelo menos dois arquivos (1 Referência + 1 ou mais Execuções).")
        return

    # -------------------------------------------------------------------------
    # 1. Leitura e Pré-processamento dos Dados
    # -------------------------------------------------------------------------
    # Carrega todos os arquivos CSV listados
    dfs = [pd.read_csv(f, sep=None, engine='python') for f in file_paths]
    
    # Valida e normaliza a coluna de tempo para que todas as trajetórias iniciem em t=0
    for i, df in enumerate(dfs):
        if time_col not in df.columns:
            raise ValueError(f"A coluna de tempo '{time_col}' é necessária no arquivo {i}.")
        df['t_norm'] = df[time_col] - df[time_col].iloc[0]
    
    # Configurações visuais padronizadas para publicação acadêmica/artigos
    plt.rcParams.update({
        'font.size': 12,
        'axes.titlesize': 14,
        'axes.labelsize': 12,
        'legend.fontsize': 10,
        'figure.facecolor': 'white'
    })
    
    # Paleta de cores e estilos de linha para diferenciar as múltiplas execuções
    colors = plt.get_cmap('tab10')
    linestyles = ['--', ':', '-.']
    
    # Limite inferior do eixo Z para as sombras de projeção 3D
    z_min = min([df['actual_TCP_pose_2'].min() for df in dfs]) - 0.05

    # -------------------------------------------------------------------------
    # 2. Interpolação e Cálculo de Erro Cartesiano (RMSE)
    # -------------------------------------------------------------------------
    # O primeiro arquivo (índice 0) é sempre tratado como a Referência (Demonstração)
    ref_df = dfs[0]
    t_ref = ref_df['t_norm'].values
    cartesian_cols = ['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']
    p_ref = ref_df[cartesian_cols].values
    
    metrics_text = "RMSE Cartesiano vs Demo:\n"
    
    # Compara cada generalização gerada contra a referência
    for i, df in enumerate(dfs[1:], start=1):
        t_gen = df['t_norm'].values
        
        # Define um intervalo de tempo comum para evitar erros de extrapolação matemática
        max_time = min(t_ref[-1], t_gen[-1])
        valid_idx = t_ref <= max_time
        t_eval = t_ref[valid_idx] 
        
        p_ref_valid = p_ref[valid_idx]
        p_gen_interp = np.zeros_like(p_ref_valid)
        
        # Interpola cada eixo cartesiano (X, Y, Z) separadamente para alinhar as frequências de amostragem
        for j, col in enumerate(cartesian_cols):
            f_interp = interp1d(t_gen, df[col].values, kind='linear')
            p_gen_interp[:, j] = f_interp(t_eval)

        # Cálculo do Erro Quadrático Médio (MSE) e da Raiz do Erro Quadrático Médio (RMSE)
        mse_cartesian = np.mean(np.sum((p_ref_valid - p_gen_interp)**2, axis=1))
        rmse_cartesian = np.sqrt(mse_cartesian)
        metrics_text += f"Gen {i}: {rmse_cartesian:.4f}m\n"

    # -------------------------------------------------------------------------
    # 3. Geração da Janela 1: Gráfico 3D da Trajetória no Espaço Cartesiano
    # -------------------------------------------------------------------------
    fig1 = plt.figure(figsize=(10, 8))
    ax = fig1.add_subplot(111, projection='3d')

    for i, df in enumerate(dfs):
        x, y, z = df['actual_TCP_pose_0'], df['actual_TCP_pose_1'], df['actual_TCP_pose_2']
        
        if i == 0:
            c = 'darkred'
            ls = '-'
            lw = 2.5
            label = 'Demonstração (Referência)'
            zord = 10
        else:
            c = colors((i - 1) % 10)
            ls = linestyles[(i - 1) % len(linestyles)]
            lw = 1.8
            label = 'Generalização ' + str(i)
            zord = 5

        # Plota a linha principal da trajetória no espaço 3D
        ax.plot(x, y, z, label=label, color=c, linestyle=ls, linewidth=lw, zorder=zord, alpha=0.9)
        
        # Cria uma projeção 2D (sombra) no plano XY para facilitar a visualização de profundidade
        ax.plot(x, y, zs=z_min, zdir='z', color=c, linestyle=ls, linewidth=lw*0.6, alpha=0.2)

        # Marcadores visuais destacando os pontos de Início (círculo) e Fim (quadrado)
        mfc = c if i == 0 else 'white' 
        ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], edgecolor=c, facecolor=mfc, s=60, marker='o', zorder=zord+1)
        ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], edgecolor=c, facecolor=mfc, s=60, marker='s', zorder=zord+1)

    # Configuração dos eixos cartesianos
    ax.set_xlabel('Posição X (m)', labelpad=20)
    ax.set_ylabel('Posição Y (m)', labelpad=20)
    ax.set_zlabel('Posição Z (m)', labelpad=20)
    ax.set_zlim(bottom=z_min)
    
    # Adiciona a caixa de texto com as métricas RMSE no canto da tela
    if len(dfs) > 1:
        ax.text2D(0.02, 0.98, metrics_text.strip(), transform=ax.transAxes, 
                  fontsize=10, verticalalignment='top', 
                  bbox=dict(facecolor='white', alpha=0.9, edgecolor='lightgray', boxstyle='round,pad=0.5'))

    # Ajustes finais da legenda do Gráfico 3D
    ax.plot([], [], 'o', color='gray', markerfacecolor='white', label='Início (Círculo)')
    ax.plot([], [], 's', color='gray', markerfacecolor='white', label='Fim (Quadrado)')
    ax.legend(loc='upper right', bbox_to_anchor=(1.15, 0.9), framealpha=1.0)
    fig1.tight_layout()

    # -------------------------------------------------------------------------
    # 4. Geração da Janela 2: Gráficos 2D das 6 Juntas em Função do Tempo
    # -------------------------------------------------------------------------
    fig2, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig2.suptitle('Replicação no Espaço de Juntas (vs Tempo)', fontsize=16)
    axes_flat = axes.flatten()

    for joint_idx in range(6):
        ax_joint = axes_flat[joint_idx]
        col_name = f'actual_q_{joint_idx}'
        
        # Fallback caso a nomenclatura da coluna varie ligeiramente entre as capturas
        if col_name not in ref_df.columns and f'q_{joint_idx}' in ref_df.columns:
            col_name = f'q_{joint_idx}'

        if col_name in ref_df.columns:
            rmse_texts = [] 
            
            for i, df in enumerate(dfs):
                if col_name in df.columns:
                    if i == 0:
                        ax_joint.plot(df['t_norm'], df[col_name], label='Demonstração', 
                                      color='darkred', linestyle='-', linewidth=2.0, zorder=10)
                    else:
                        ax_joint.plot(df['t_norm'], df[col_name], label=f'Gen {i}', 
                                      color=colors((i - 1) % 10), 
                                      linestyle=linestyles[(i - 1) % len(linestyles)], 
                                      linewidth=1.5, zorder=5, alpha=0.8)
                        
                        # Cálculo de RMSE isolado e específico para esta junta (em radianos)
                        t_gen = df['t_norm'].values
                        max_time = min(t_ref[-1], t_gen[-1])
                        valid_idx = t_ref <= max_time
                        t_eval = t_ref[valid_idx] 
                        
                        q_ref_valid = ref_df[col_name].values[valid_idx]
                        
                        f_interp = interp1d(t_gen, df[col_name].values, kind='linear')
                        q_gen_interp = f_interp(t_eval)
                        
                        joint_rmse = np.sqrt(np.mean((q_ref_valid - q_gen_interp)**2))
                        rmse_texts.append(f"G{i}: {joint_rmse:.4f}")
            
            # Adiciona o sumário do erro angular no título individual de cada gráfico de junta
            title_str = f'Junta {joint_idx}'
            if rmse_texts:
                title_str += '\nRMSE ' + ' | '.join(rmse_texts)
            
            ax_joint.set_title(title_str, fontsize=11)
            
        else:
            ax_joint.text(0.5, 0.5, f"Sem dados para {col_name}", ha='center', va='center')

        # Configurações dos eixos 2D
        ax_joint.set_xlabel('Tempo (segundos)')
        ax_joint.set_ylabel('Posição (rad)')
        ax_joint.grid(True, linestyle=':', alpha=0.7)
        
        # Garante que a legenda apareça apenas no primeiro gráfico para economizar espaço
        if joint_idx == 0:
            ax_joint.legend(loc='best', fontsize=9)

    fig2.tight_layout()
    # Ajusta a margem superior para acomodar títulos de múltiplas linhas
    fig2.subplots_adjust(top=0.88, hspace=0.4) 
    plt.show()

# ==============================================================================
# Execução do Script
# ==============================================================================
if __name__ == "__main__":

    plot_tcp_pose_misto(arquivos_csv, time_col='timestamp')