# oxebots_comms

Pacote de comunicação e interface de controle de atuadores da equipe **OxeBots** (UFBA) para a categoria RoboCup Small Size League – Entry Level (SSL-EL).

Este pacote é responsável por traduzir as decisões e comandos de velocidade do software de alto nível e transmiti-los aos simuladores (**grSim** e **ER-Force/Ra**) ou diretamente aos robôs físicos via rádio **NRF24L01**.

---

## 1. Nós do Pacote

### 1.1 `grSim_controller_node`
Envia pacotes UDP em formato Protobuf (`grSim_Packet`) para o simulador grSim.
- **Tópicos Assinados:** `robot_commands` (`oxebots_interfaces/msg/RobotCmdData`).
- **Parâmetros:**
  - `simulator_ip` (string, default: `"127.0.0.1"`): Endereço IP do simulador.
  - `simulator_port` (int, default: `10301` para Azul, `10302` para Amarelo): Porta UDP do grSim.
  - `interface_ip` (string, default: `""`): IP da placa de rede local (opcional).
  - `is_yellow_team` (bool, default: `false`): Define a porta e o time alvo no simulador.
  - `command_topic` (string, default: `"robot_commands"`).

### 1.2 `ra_controller_node`
Controlador para o simulador ER-Force / Ra. Converte o vetor de velocidade global para coordenadas locais do robô (`vx`, `vy`, `w`).
- **Tópicos Assinados:**
  - `robot_commands` (`oxebots_interfaces/msg/RobotCmdData`)
  - `game_data` (`oxebots_interfaces/msg/GameData`)
- **Parâmetros:**
  - Mesmos parâmetros do `grSim_controller_node`, adicionando a conversão de referenciais.

### 1.3 `nrf24_bridge_node`
Ponte de rádio serial de baixa latência para comunicação com os robôs físicos via dongle NRF24L01 (usando `libserialport` e codificação compacta `bitproto`).
- **Modos de Operação (`command_mode`):**
  - `velocity`: Recebe comandos omnidirecionais de velocidade de `/robot_commands`.
  - `point`: Recebe pontos de setpoint de `/robot_trajectory_setpoints` para controle a bordo do microcontrolador (ESP32).
- **Tópicos:**
  - Assina: `/robot_commands` (ou `/robot_trajectory_setpoints` no modo `point`).
  - Publica: `/robot_telemetry` (`oxebots_interfaces/msg/RobotTelemetry`).
- **Parâmetros:**
  - `radio_rx_address` (string, default: `"ADMIN"`): Endereço de recepção da base (5 caracteres).
  - `radio_tx_address` (string, default: `"ESP32"`): Endereço de transmissão dos robôs.
  - `radio_frequency` (double, default: `2.476` GHz).
  - `radio_rate` (int, default: `3`): Velocidade de comunicação configurada via comandos AT.
  - `command_mode` (string, default: `"velocity"`).

---

## 2. Scripts Auxiliares (`scripts/`)

- **`Adapt.py`**: Ponte e adaptador em Python para testes rápidos de rádio e simulação.
- **`Nrf24.py`**: Módulo Python com driver de comunicação serial e codificação bitproto.
- **`ssl_robot_protocol_bp.py`**: Esquema de serialização binária bitproto para Python.

---

## 3. Pré-requisitos e Dependências

- **Sistema:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Bibliotecas do Sistema:** `libserialport-dev`, `pkg-config`, `python3-serial`, `protobuf-compiler`.
- **Dependências ROS:** `rclcpp`, `rclpy`, `oxebots_interfaces`.

---

## 4. Como Executar

Para iniciar o envio ao grSim:
```bash
ros2 run oxebots_comms grSim_controller_node --ros-args -p is_yellow_team:=false
```

Para iniciar a comunicação de rádio real via NRF24:
```bash
ros2 run oxebots_comms nrf24_bridge_node --ros-args -p command_mode:=velocity
```

---

## 5. Licença

Este pacote é disponibilizado sob a licença **GPLv3**.
