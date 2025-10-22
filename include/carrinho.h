/**
 * @file carrinho.h
 * @brief Interface do carrinho seguidor de linha com rodas omni e PID.
 * @details
 *  Este header define a classe Carrinho:
 *   - Leitura de 8 sensores via MCP23X17 (banco A)
 *   - Cálculo de erro por LUT para o seguidor de linha
 *   - Controle de 4 motores com DRV8833 usando LEDC (ESP32)
 *   - Controlador PID para gerar a velocidade angular (omega)
 *   - Máquina de estados: CALIBRACAO, ESPERANDO_LARGADA, CORRIDA
 *
 *  Convenções:
 *   - Percentuais de velocidade: -100 a 100
 *   - dt em segundos
 *   - omega limitado por omegaMax
 */

#pragma once
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

/**
 * @class Carrinho
 * @brief Classe principal de controle do carrinho.
 * @note O objeto recebe uma referência a um MCP23X17 já inicializado fora da classe.
 * @warning Este módulo não configura o MCP23X17. Faça a configuração no seu setup.
 */
class Carrinho
{
public:
  /**
   * @brief Construtor.
   * @param mcp_dev referência para um Adafruit_MCP23X17 já iniciado externamente.
   */
  explicit Carrinho(Adafruit_MCP23X17 &mcp_dev);



/**
 * @brief Define as velocidades base de deslocamento do carrinho.
 * @param vy Velocidade de avanço (frente), em percentual de 0 a 100.
 * @param vx Velocidade lateral (esquerda/direita), em percentual de -100 a 100.
 * @details
 * Este método ajusta as velocidades de referência utilizadas pelo seguidor de linha.
 * A velocidade de avanço (`vy`) controla o deslocamento para frente,
 * enquanto a velocidade lateral (`vx`) pode compensar desalinhamentos ou deslocamentos laterais.
 * Ambos os valores são automaticamente limitados aos intervalos válidos.
 * @note Alterar essas velocidades afeta o comportamento de `seguirLinhaStep()`
 * e dos modos automáticos de corrida.
 */
void setVelocidades(float vy, float vx);

/**
 * @brief Define apenas a velocidade base de avanço (eixo Y).
 * @param vy Velocidade de avanço em percentual (0 a 100).
 * @details
 * Ajusta a velocidade frontal padrão usada pelo seguidor de linha.
 * O valor é limitado entre 0 e 100 %, evitando movimento reverso.
 * @see setVelocidades()
 */
void setVy(float vy);

/**
 * @brief Define apenas a velocidade base lateral (eixo X).
 * @param vx Velocidade lateral em percentual (-100 a 100).
 * @details
 * Controla o deslocamento lateral constante aplicado ao carrinho durante o seguimento da linha.
 * Pode ser usado para corrigir pequenos desvios ou testar movimentações diagonais.
 * @note Valores positivos deslocam o carrinho para a direita, negativos para a esquerda.
 * @see setVelocidades()
 */
void setVx(float vx);
  // ========================= Ciclo de vida =========================

  /**
   * @brief Inicializa LUTs, PWM dos motores e temporizador interno.
   * @post Estado inicial é CALIBRACAO.
   */
  void begin();

  /**
   * @brief Executa um ciclo completo de atualização.
   * @details
   *  Processa comandos seriais, calcula dt, lê sensores e roda a máquina de estados.
   *  Deve ser chamada continuamente no loop principal.
   */
  void tick();

  // ========================= APIs públicas para os alunos =========================

  /**
   * @brief Entra no modo de calibração.
   * @post Zera integrador e derivada e para os motores.
   */
  void entrarCalibracao();

  /**
   * @brief Prepara o seguidor para iniciar.
   * @post Estado muda para ESPERANDO_LARGADA e zera integrador.
   */
  void iniciarSeguirLinha();

  /**
   * @brief Executa um passo de seguir linha independente da FSM.
   * @details Útil se você quiser chamar o seguidor em um loop próprio.
   */
  void seguirLinhaStep();

  /**
   * @brief Lê a máscara dos 8 sensores no banco A do MCP (A0..A7).
   * @return Máscara de 8 bits. Bit 1 indica sensor ativo conforme PRETO_BIT_1.
   */
  uint8_t lerLinhaMascara();

  /**
   * @brief Lê a máscara e devolve o erro do seguidor.
   * @return Erro em faixa aproximada [-7..7] ou ERRO_SEM_LINHA se não houver faixa.
   */
  float lerErro();

  /**
   * @brief Converte uma máscara de 8 bits em erro usando a LUT.
   * @param m máscara de 8 bits dos sensores.
   * @return Erro float ou ERRO_SEM_LINHA quando não há sensores ativos.
   */
  float calcularErroMascara(uint8_t m);

  /**
   * @brief Controla as rodas em percentuais de -100 a 100.
   * @param vy velocidade frontal em porcentagem.
   * @param vx velocidade lateral em porcentagem.
   * @param omega rotação em porcentagem.
   * @note Valores fora da faixa são limitados internamente.
   */
  void controlarRodas(float vy, float vx, float omega);

  // ========================= PID =========================

  /**
   * @brief Atualiza o controlador PID e devolve o comando angular.
   * @param erro erro do seguidor. Sinal indica lado.
   * @param dt intervalo de tempo em segundos.
   * @return omega limitado a [-omegaMax..omegaMax].
   * @details
   *  P: kp * erro
   *  I: acumula erro * dt com anti windup
   *  D: (erro - erroAnterior) / dt com proteção contra dt muito pequeno
   */
  float pidAtualizar(float erro, float dt);

  /**
   * @brief Calcula dt automaticamente a partir de micros.
   * @return dt em segundos limitado por faixa segura.
   */
  float calcularDt();

  /**
   * @brief Define os ganhos do PID garantindo não negatividade.
   * @param p ganho proporcional.
   * @param i ganho integral.
   * @param d ganho derivativo.
   */
  void setPID(float p, float i, float d);

  /**
   * @brief Verifica um bloco ancorado nas bordas com quantidade mínima de sensores ativos.
   * @param m máscara de sensores.
   * @param direita se true ancora do lado direito, senão do lado esquerdo.
   * @param qnt quantidade mínima de sensores ativos.
   * @return true quando o padrão é atendido.
   */
   bool verificaBloco(uint8_t m, bool direita, uint8_t qnt);

  /**
   * @brief Detecta se os dois sensores centrais estão ativos.
   * @param m máscara de sensores.
   * @return true se os bits centrais estão ligados.
   */
   bool detectaCentro(uint8_t m);

private:
  // ========================= Estados =========================

  /**
   * @brief Estados internos da FSM do carrinho.
   */
  enum Estado : uint8_t
  {
    CALIBRACAO = 0,       ///< mostra máscara e erro, motores parados
    ESPERANDO_LARGADA = 1,///< aguarda padrão central para iniciar
    CORRIDA = 2           ///< PID ativo com controle de omega
  };

  // ========================= Pinos dos motores =========================
  /// @name Pinos conectados ao DRV8833 por motor
  /// @{
  static constexpr int pinM0Dir = 18;
  static constexpr int pinM0Esq = 3;
  static constexpr int pinM1Dir = 10;
  static constexpr int pinM1Esq = 46;
  static constexpr int pinM2Dir = 13;
  static constexpr int pinM2Esq = 14;
  static constexpr int pinM3Dir = 12;
  static constexpr int pinM3Esq = 11;
  /// @}

  // ========================= PWM =========================
  /// Frequência do PWM em Hz (LEDC).
  static constexpr uint32_t Freq_PWM = 20000;
  /// Resolução do PWM em bits.
  static constexpr uint8_t Resol_PWM = 8;

  // ========================= Seguidor =========================
  /**
   * @brief Define se bit 1 significa preto. Ajuste conforme seu módulo.
   * @note Se seu sensor ativa em nível baixo, defina como false.
   */
  static constexpr bool PRETO_BIT_1 = true;

  /**
   * @brief Valor especial de erro que indica ausência de linha.
   * @note Usado quando nenhum sensor está ativo.
   */
  static constexpr int ERRO_SEM_LINHA = INT16_MAX;

  // ========================= Ajustes de controle =========================
  /// Inverte o sinal final de omega.
  bool INVERTER_OMEGA = true;
  /// Inverte individualmente o sentido de cada motor.
  bool invertMotor[4] = {false, false, false, false};

  // ========================= Estado e tempo =========================
  /// Estado atual da FSM.
  Estado estado = CALIBRACAO;
  /// Marca de tempo anterior em micros.
  uint32_t tPrevMicros = 0;
  /// Marca de tempo do último log em millis.
  uint32_t tPrevLog = 0;
  /// Período de log em ms.
  uint32_t LOG_MS = 100;
  /// Habilita saída de telemetria no Serial.
  bool LOG_ATIVO = true;

  // ========================= Buffers e variáveis de apoio =========================
  /// Buffer de comandos seriais.
  char cmdBuf[48] = {};
  /// Último erro válido usado para estratégia de busca quando perde a linha.
  float ultimoErroValido = 0.0f;

  // ========================= Parâmetros do PID e limites =========================
  /// Ganhos do PID.
  float kp = 4.0f, ki = 0.5f, kd = 1.2f;
  /// Velocidade frontal padrão em porcentagem.
  float vyPercent = 20.0f;
  /// Limite de módulo para omega.
  float omegaMax = 40.0f;
  /// Memória do erro anterior para a derivada.
  float erroAnterior = 0.0f;
  /// Acumulador da integral com anti windup.
  float integralAcumulada = 0.0f;
// deslocamento lateral base (-100..100)
  float vxPercent = 0.0f;   
 
  /// ========================= Filtros =========================
  /// Filtro passa-baixa para a derivada do PID.
  float dFilt = 0.0f;
  /// Constante de tempo do filtro da derivada em segundos.
  float tauD = 0.02f; 
  /// Máximo dt permitido em segundos.
  float dtMax = 0.05f; // 50 ms

  // ========================= LUTs =========================
  /// Tabela de duty para 0 a 100 por cento.
  uint8_t dutyLUT[101] = {};
  /// Duty atual aplicado em cada pino de cada motor.
  uint8_t dutyAtual[4][2] = {};
  /// Tabela de erro para cada máscara de 8 bits.
  int8_t erroLUT[256] = {};

  // ========================= Mapeamentos de canais e pinos =========================
  /// Canais LEDC por motor e lado [motor][lado].
  uint8_t chMotor[4][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};
  /// Pinos físicos conectados aos canais [motor][lado].
  uint8_t pinMotor[4][2] = {
      {pinM0Esq, pinM0Dir},
      {pinM1Esq, pinM1Dir},
      {pinM2Esq, pinM2Dir},
      {pinM3Esq, pinM3Dir}};

  // ========================= Referência ao MCP =========================
  /**
   * @brief Referência ao expansor de IO. Deve estar configurado fora da classe.
   */
  Adafruit_MCP23X17 &mcp;

  // ========================= Auxiliares gerais =========================

  /**
   * @brief Imprime ajuda com os comandos disponíveis no Serial.
   */
  void printHelp();

  /**
   * @brief Processa comandos vindos do monitor serial.
   * @details Permite ajustar vy, kp, ki, kd, omegaMax, log e inversão de omega.
   */
  void processaSerial();

  /**
   * @brief Emite telemetria periódica com estado, erro e ganhos.
   * @param erro erro atual do seguidor.
   * @param omega comando angular aplicado.
   */
  void logStatus(float erro, float omega);

  /**
   * @brief Limita dt para faixa segura.
   * @param dt tempo em segundos.
   * @return dt em [1e-6, 2e-2].
   */
 float clampDt(float dt);

  // ========================= Seguidor =========================

  /**
   * @brief Constrói a LUT de erro para todas as máscaras de 8 bits.
   * @note Deve ser chamada no begin.
   */
  void seguidorInitLUT();

  /**
   * @brief Imprime a máscara dos sensores como 8 bits.
   * @param mascara valor lido dos sensores.
   */
  void seguidorImprimir(uint8_t mascara);

  // ========================= Motores =========================

  /**
   * @brief Inicializa a tabela de duty para 0 a 100 por cento.
   */
  void initDutyLUT();

  /**
   * @brief Configura pinos, canais LEDC e zera as saídas dos 4 motores.
   */
  void motoresBegin();

  /**
   * @brief Para imediatamente todos os motores.
   */
  void motoresPararTodos();

  /**
   * @brief Aciona um motor pelo percentual de velocidade.
   * @param i índice do motor [0..3].
   * @param velPct velocidade em porcentagem [-100..100]. Sinal define direção.
   */
  void acionaMotor(int i, int velPct);

  /**
   * @brief Cinemática das rodas omni em inteiros com normalização.
   * @param vx lateral em centésimos [-10000..10000].
   * @param vy frontal em centésimos [-10000..10000].
   * @param omega rotação em centésimos [-10000..10000].
   */
  void acionaRodasOminiInt(int vx, int vy, int omega);

  /**
   * @brief Converte um float para centésimos com arredondamento.
   * @param v valor percentual em float.
   * @return valor inteiro em centésimos.
   */
  static int toCent(float v);

  /**
   * @brief Converte um percentual 0 a 100 para duty via LUT.
   * @param v percentual inteiro [0..100].
   * @return duty [0..255] pelo mapeamento pré-calculado.
   */
  inline uint8_t porcentagemPWM(uint8_t v) { return dutyLUT[v]; }

  /**
   * @brief Escreve PWM em um lado do motor somente se houver mudança.
   * @param m índice do motor [0..3].
   * @param lado 0 ou 1 conforme os dois pinos do DRV8833.
   * @param duty duty de 0 a 255.
   */
  inline void writePWM(int m, int lado, uint8_t duty);

  // ========================= Modos internos =========================

  /**
   * @brief Modo de calibração. Mostra máscara e erro e mantém motores parados.
   * @param mascara bits dos sensores.
   * @param erro erro calculado.
   */
  void modoCalibracao(uint8_t mascara, float erro);

  /**
   * @brief Modo de espera. Entra em corrida quando os sensores centrais detectam linha.
   * @param mascara bits dos sensores.
   */
  void modoEsperandoLargada(uint8_t mascara);

  /**
   * @brief Modo de corrida. Aplica PID e comanda as rodas.
   * @param mascara não usado. Mantido por consistência.
   * @param erro erro do seguidor.
   * @param dt passo de tempo em segundos.
   */
  void modoCorrida(uint8_t mascara, float erro, float dt);

  // ========================= Utilitários estáticos =========================

  /**
   * @brief Conta bits em 1 em um byte.
   * @param v byte de entrada.
   * @return quantidade de bits em 1.
   */
  static inline int popcount8(uint8_t v);
};
