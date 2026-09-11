# Modelagem e aprendizado de sistemas dinâmicos a partir de demonstrações sintéticas
Este repositório contém os tutoriais, scripts e conjuntos de dados utilizados para a replicação e avaliação de resultados do projeto de Iniciação Científica (PIBIC) desenvolvido na Universidade de Brasília (UnB). O escopo principal do projeto é a generalização e o controle de trajetórias de um robô manipulador Universal Robots UR3 utilizando *Dynamic Movement Primitives* (DMP) e Sistemas Dinâmicos (DS) com Cinemática Inversa Generalizada (GIK).

* Os scripts em MATLAB para controle via LPV-DS são fundamentados na biblioteca [ds-opt](https://github.com/nbfigueroa/ds-opt).
* Os scripts em Python para controle via DMP são fundamentados na biblioteca [movement_primitives](https://github.com/dfki-ric/movement_primitives).

## 📁 Estrutura do Repositório

| Diretório | Descrição |
| :--- | :--- |
| **`exemplos/`** | Arquivos de dados. Armazena as trajetórias de demonstração (`.csv`), os logs de replicação/generalização DMP |
| **`docker/`** | Arquivos e instruções para a configuração do ambiente simulado do UR3 (URSim). |
| **`execute_dmp_ur3.py`** | Script principal de execução em python |
| **`execute_ds_ur3.m`** |  Script principal de execução em MATLAB |
|**`compare_dmp_results.py`**| Script de validação métrica e visual. Contém o avaliador de RMSE e gerador de gráficos |


## ⚙️ Pré-requisitos e Instalação

Os experimentos foram realizados com: **Python 3.11** e o **MATLAB 2026a/2025b/2025a**. É necessário a instalação do **Docker** para a simulação do robô no URSim.

**1. Clone este repositório**:

```bash
git clone https://github.com/AntonioLeonardoUNB/pibic.git
cd pibic
```

**2. Instalação das dependências Python:**

Abra o terminal e instale `requirements.txt`:

```
python -m pip install -r requirements.txt
```

**3. Instalação da biblioteca `movement_primitives`:**
Siga as intruções do repositório original, disponível em: <https://github.com/dfki-ric/movement_primitives>


**4. Instalação do toolbox `ds-opt`**
Siga as instruções do repositório original, disponível em: <https://github.com/nbfigueroa/ds-opt>.


---

## 🤖 Configurando o Simulador URSim (Docker)

Para executar os testes sem o hardware físico, utilizamos o simulador oficial da Universal Robots conteinerizado.

1. Instale o [docker desktop](https://docs.docker.com/desktop/);
2. Baixe a imagem do simulador em [hub.docker.com](https://hub.docker.com/r/universalrobots/ursim_cb3):
    ```bash
    docker pull universalrobots/ursim_cb3
    ```
3. Inicie a imagem do URSim:
    ```bash
    docker run --rm -it \
      -p 5900:5900 \
      -p 6080:6080 \
      -p 29999:29999 \
      -p 30001-30004:30001-30004 \
      -v "${HOME}/ur3/urcaps:/urcaps" \
      -v "${HOME}/ur3/programs:/ursim/programs/" \
      -e ROBOT_MODEL=UR3 \
      universalrobots/ursim_cb3;
    ```
4. Use a aba PORTS no VS Code para fazer o roteamento de portas do container do Docker para o localhost do computador;
    ![Portas VSCode](https://fir-wool-eae.notion.site/image/attachment%3A309cccfe-f1be-4504-bc83-d1f344ade763%3Aimage.png?table=block&id=2cefb16b-3ed1-8186-be9b-fa5062712ca1&spaceId=56117687-5331-4b6f-9239-adabd0956ef7&width=2000&userId=&cache=v2&imgBuildSrc=requestProxiedImageUrl)

5. (OPCIONAL, APENAS MÉTODO DS-OPT) Na pasta ${HOME}/ur3/programs adicione o arquivo `URServerScript.script` localizado na pasta `docker` deste respositório;

6. Abra o endereço <http://localhost:6080/vnc.html> no seu browser e ligue o robô no simulador;
---

## 🚀 Executando os Controladores

### Opção A: Execução via DMP (Python)
Este script lê uma demonstração em CSV, treina a DMP (utilizando 50 pesos por dimensão) e envia comandos de posição e velocidade via RTDE para o UR3.

1. Navegue até a pasta principal;
2. Execute o script principal:
    ```bash
    python execute_dmp_ur3.py
    ```
    
*  Nota 1: O script depende que o arquivo .csv esteja devidamente formatado, a coluna de tempo deve se chamar "timestamp", a de posição das juntas "actual_q_{i}", a de velocidade das juntas "actual_qd_{i}" e as poses cartesianos "actual_TCP_pose_{i}".

*  Nota 2: O script moverá o robô automaticamente para a posição inicial exata da demonstração antes de iniciar o loop de gravação.

### Opção B: Execução via Sistemas Dinâmicos e GIK (MATLAB) 

Este script utiliza um modelo de sistemas dinâmicos de parâmetros linearmente variáveis (LPV-DS) e a função [generalizedInverseKinematics](https://www.mathworks.com/help/robotics/ref/generalizedinversekinematics-system-object.html) do MATLAB para computar o comando de posição de juntas a partir da integração numérica do sistema dinâmico.
> [!WARNING]
> **Não execute este script no robô real**. Para encontrar soluções com mais frequência, o cálculo de cinemática inversa está operando com restrições reduzidas e frequentemente gera movimentos perigosos para o robô e usuários.

#### Parte 1 (Configuração no simuladorURSim)

1. Na aba <http://localhost:6080/vnc.html>, clique em `program robot`;
2. Clique em `Empty Programa` e selecione opção `empty`;
3. Clique em `Structure` e depois em `Script Code`;
4. Selecione o script criado e altere de `line` -> `file;
5. Clique em `Edit` -> `Open` e selecione `URServerScript.script` depois clique novamente em `Open`;
6. Saia dessa aba e aperte o play;

![Tutorial URSim](https://media.giphy.com/media/W1tiH1Ww3F7tdIzZA0/giphy.gif)

#### Parte 2 (Configuração no MATLAB)
1. Abra o MATLAB e instale o addon `Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators` com a opção `RTDE`;
2. Navegue até o repositório ds-opt;
3. Certifique-se de que o script `setup_dsopt_code.m` foi executado comentando a linha 7:
   ```matlab 
   7 %restoredefaultpath();
   ...
   10 %userpath('clear');
   ```
4. Navegue então até o repositório `pibic` e execute o script: 
    ```matlab
    execute_ds_ur3.m
    ```
* Nota 1: O script tem que ser executado enquanto o script `URServerScript.script` está rodando no URSim;
---

## 📊 Avaliação e Gráficos

Após a execução de `execute_dmp_ur3.py`, são gerados os arquivos `.csv` de reprodução (armazenados na pasta `exemplos/`) e o programa de plotagem executa automaticamente, onde você pode comparar o erro cinemático (RMSE) e o desvio de trajetória em relação à demonstração original.

Caso você queira, pode plotar as trajetórias de várias trajetórias ao mesmo tempo (muito útil em cenários de generalização)

1. Navegue até a pasta principal;
2. Altere o script de plotagem mista `python compare_dmp_results.py`;
3. Adicionando uma linha `arquivos_para_plotar = [endereço_da_trajetoria_1, endereço_da_trajetoria_2, ... endereço_da_trajetoria_n]`
4. E outra linha chamando a função `plot_tcp_pose_misto(arquivos_para_plotar, time_col='timestamp')`
5. Então, chamando a função no terminal:
    ```bash
    python compare_dmp_ur3
    ```

O script fará o alinhamento temporal das amostras e gerará duas janelas:
* **Gráfico 3D:** Trajetória no espaço cartesiano (posições X, Y, Z do efetuador final) com as respectivas sombras no plano.
* **Gráficos 2D:** Evolução temporal do espaço de juntas (q0 a q5), indicando o erro RMSE isolado para cada articulação.

## 🔍 Exemplos 

Os exemplos de trajetórias fornecidos neste repositório estão divididos em duas categorias principais, dependendo da forma como os dados de demonstração foram adquiridos:

### 1. Demonstrações Reais (Ensino Cinestésico)
Estas trajetórias foram coletadas fisicamente utilizando o modo *freedrive* do manipulador UR3, no qual um operador humano guiou o efetuador final para realizar tarefas específicas.

* Nota 1: O robô manipulador UR3, disponível no Laboratório de Automação e Robótica (LARA) - UnB, possui uma garra em seu efetuador final, se fazendo necessário correção do tcp na aba `⚠️Instalation`:

<img width="500" height="375" alt="image" src="https://github.com/user-attachments/assets/bf684f69-cf59-4da4-a870-a53c2882f149" />

Os valores devem ser alterados para: `x = -1,56mm; y = 0,87mm; z = 186,04mm; rx = 0,0051rad, ry = -0,0037rad, rz = -3,1408rad`;


Os exemplos incluem:

* **Tarefa de Desenho:**

<img width="500" height="375" alt="demo_drawing_replicacao" src="https://github.com/user-attachments/assets/2e2dbe03-d6ea-444c-8f3b-98dd6d18a53c" />

* **Manipulação de Garrafa:**

<img width="500" height="375" alt="demo_pouring2_replicacao" src="https://github.com/user-attachments/assets/73cdcb88-bd68-4001-8c8c-8308acc260fd" />

### 2. Demonstrações Sintéticas (Geradas via LLMs)

Trajetórias geradas automaticamente por Modelos de Linguagem de Larga Escala (LLMs), que interpretaram comandos em linguagem natural e sintetizaram os movimentos no ambiente PyBullet antes de serem exportadas[cite: 1]. Os exemplos incluem:

* Nota 2: Essas demos dispensam a mudança de TCP;
* Nota 3: Esse tipo de dado geralmente precisa de pré-processamento;

* **Perfil Senoidal:**
 
<img width="500" height="375" alt="Replicacao_sine_cartesian" src="https://github.com/user-attachments/assets/d1c4b03b-32a1-4c97-848a-9d451055df42" />


* **Semicírculo:** 

<img width="500" height="375" alt="Replicacao_semicircle_cartesian" src="https://github.com/user-attachments/assets/0c041fcc-0421-4b09-be3b-6684f7adf7c5" />


* **Pegar Objeto:**

<img width="500" height="375" alt="Replicacao_picking1_cartesian" src="https://github.com/user-attachments/assets/c45a89c0-ec2a-4017-81db-68b6c5397927" />


### Tipos de Execução Suportados
Ao rodar os scripts disponíveis na pasta `exemplos/`, você poderá testar o comportamento do robô sob dois cenários distintos:

* **Replicação:** O robô inicia o movimento na mesma posição e configuração de juntas da demonstração original, seguindo perfeitamente a estrutura geométrica da curva.
* **Generalização:** O robô parte de uma coordenada inicial arbitrária (diferente da ensinada) e o algoritmo adapta o caminho, convergindo com precisão para o destino final sem comprometer a estabilidade da coordenada final do movimento.
