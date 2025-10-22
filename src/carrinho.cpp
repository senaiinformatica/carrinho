#include "carrinho.h"
#include <Wire.h>
#include <stdlib.h>   // strtof, strtol
#include <ctype.h>    // isspace
#include <math.h>     // isfinite

// -------- Construtor --------
Carrinho::Carrinho(Adafruit_MCP23X17 &mcp_dev) : mcp(mcp_dev) {}

// -------- Tempo --------
float Carrinho::clampDt(float dt)
{
  if (dt < 1e-6f) return 1e-6f;
  if (dt > dtMax) return dtMax;
  return dt;
}

float Carrinho::calcularDt()
{
  uint32_t now = micros();
  float dt = clampDt((now - tPrevMicros) * 1e-6f);
  tPrevMicros = now;
  return dt;
}

// -------- Ajuda e Log --------
void Carrinho::printHelp()
{
  Serial.println("\nComandos:");
  Serial.println("h            ajuda");
  Serial.println("p            mostra parametros");
  Serial.println("r            iniciar corrida");
  Serial.println("s            voltar para calibracao");
  Serial.println("vy<val>      velocidade frente, ex: vy35");
  Serial.println("kp/ki/kd     ganhos, ex: kp6.5");
  Serial.println("om<val>      omegaMax, ex: om40");
  Serial.println("log0         desliga telemetria");
  Serial.println("log<ms>      periodo da telemetria, ex: log200");
  Serial.println("inv          alterna inversao do omega");
}

void Carrinho::logStatus(float erro, float omega)
{
  if (!LOG_ATIVO) return;

  uint32_t agora = millis();
  if (agora - tPrevLog < LOG_MS) return;
  tPrevLog = agora;

  const char* est =
      (estado == CALIBRACAO)        ? "CALIBRACAO" :
      (estado == ESPERANDO_LARGADA) ? "ESPERANDO_LARGADA" :
                                      "CORRIDA";

  auto finitesafe = [](float x) -> float { return isfinite(x) ? x : 0.0f; };
  erro  = finitesafe(erro);
  omega = finitesafe(omega);

  Serial.printf(
    "{\"t\":%lu,"
     "\"estado\":\"%s\","
     "\"erro\":%.3f,"
     "\"vy\":%.1f,"
     "\"omega\":%.3f,"
     "\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,"
     "\"omegaMax\":%.1f,"
     "\"logMs\":%lu,"
     "\"invOmega\":%s}\n",
    (unsigned long)agora,
    est,
    erro,
    vyPercent,
    omega,
    kp, ki, kd,
    omegaMax,
    (unsigned long)LOG_MS,
    INVERTER_OMEGA ? "true" : "false"
  );
}

void Carrinho::setPID(float p, float i, float d)
{
  kp = (p >= 0.0f) ? p : 0.0f;
  ki = (i >= 0.0f) ? i : 0.0f;
  kd = (d >= 0.0f) ? d : 0.0f;
  Serial.printf("PID atualizado: kp=%.3f ki=%.3f kd=%.3f \n", kp, ki, kd);
}

// Parser leve com strtof/strtol
void Carrinho::processaSerial()
{
  static size_t n = 0;

  auto skipWS = [](const char *s) -> const char *
  {
    while (*s && isspace((unsigned char)*s)) ++s;
    return s;
  };
  auto parseFloatAfter = [&](const char *s, size_t prefixLen, float &out) -> bool
  {
    const char *p = skipWS(s + prefixLen);
    char *end = nullptr;
    float v = strtof(p, &end);
    if (end == p) return false;
    out = v; return true;
  };
  auto parseLongAfter = [&](const char *s, size_t prefixLen, long &out) -> bool
  {
    const char *p = skipWS(s + prefixLen);
    char *end = nullptr;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    out = v; return true;
  };

  while (Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r')
    {
      cmdBuf[n] = '\0';
      n = 0;
      if (cmdBuf[0] == '\0') return;

      if (strcmp(cmdBuf, "h") == 0) {
        printHelp();
      }
      else if (strcmp(cmdBuf, "p") == 0) {
        Serial.printf("vy=%.1f kp=%.3f ki=%.3f kd=%.3f om=%.1f log=%s/%lums invOmega=%s\n",
                      vyPercent, kp, ki, kd, omegaMax,
                      LOG_ATIVO ? "on" : "off", (unsigned long)LOG_MS,
                      INVERTER_OMEGA ? "true" : "false");
      }
      else if (strcmp(cmdBuf, "r") == 0) {
        iniciarSeguirLinha();
        Serial.println("estado: ESPERANDO_LARGADA");
      }
      else if (strcmp(cmdBuf, "s") == 0) {
        entrarCalibracao();
        Serial.println("estado: CALIBRACAO");
      }
      else if (cmdBuf[0] == 'v' && cmdBuf[1] == 'y') {
        float v;
        if (parseFloatAfter(cmdBuf, 2, v)) {
          v = constrain(v, 0.0f, 100.0f);
          vyPercent = v;
          Serial.printf("vy = %.1f\n", vyPercent);
        } else Serial.println("vy invalido");
      }
      else if (cmdBuf[0] == 'k' && cmdBuf[1] == 'p') {
        float v; if (parseFloatAfter(cmdBuf, 2, v) && v > 0.0f) { kp = v; Serial.printf("kp = %.3f\n", kp); }
        else Serial.println("kp invalido");
      }
      else if (cmdBuf[0] == 'k' && cmdBuf[1] == 'i') {
        float v; if (parseFloatAfter(cmdBuf, 2, v) && v >= 0.0f) { ki = v; Serial.printf("ki = %.3f\n", ki); }
        else Serial.println("ki invalido");
      }
      else if (cmdBuf[0] == 'k' && cmdBuf[1] == 'd') {
        float v; if (parseFloatAfter(cmdBuf, 2, v) && v >= 0.0f) { kd = v; Serial.printf("kd = %.3f\n", kd); }
        else Serial.println("kd invalido");
      }
      else if (cmdBuf[0] == 'l' && cmdBuf[1] == 'o' && cmdBuf[2] == 'g') {
        if (strcmp(cmdBuf, "log0") == 0) {
          LOG_ATIVO = false; Serial.println("log off");
        } else {
          long ms;
          if (parseLongAfter(cmdBuf, 3, ms) && ms >= 10) {
            LOG_MS = (uint32_t)ms; LOG_ATIVO = true;
            Serial.printf("log every %lums\n", (unsigned long)LOG_MS);
          } else {
            LOG_ATIVO = true; Serial.println("log on");
          }
        }
      }
      else if (strcmp(cmdBuf, "inv") == 0) {
        INVERTER_OMEGA = !INVERTER_OMEGA;
        Serial.printf("INVERTER_OMEGA = %s\n", INVERTER_OMEGA ? "true" : "false");
      }
      else if (cmdBuf[0] == 'o' && cmdBuf[1] == 'm') {
        float v; if (parseFloatAfter(cmdBuf, 2, v) && v > 0.0f) {
          omegaMax = v; Serial.printf("omegaMax = %.1f\n", omegaMax);
        } else Serial.println("omegaMax invalido");
      }
      else {
        Serial.println("comando nao reconhecido. use h para ajuda.");
      }
    }
    else if (n < sizeof(cmdBuf) - 1)
    {
      cmdBuf[n++] = c; // acumula caractere
    }
  }
}

// -------- Seguidor (LUT de erro) --------
void Carrinho::seguidorInitLUT()
{
  static const int8_t PESOS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
  for (int m = 0; m < 256; m++)
  {
    uint8_t active = PRETO_BIT_1 ? (uint8_t)m : (uint8_t)(~m);
    if (active == 0) { erroLUT[m] = (int8_t)0x7F; continue; } // flag "sem linha"

    int soma = 0, cnt = 0;
    for (int i = 0; i < 8; i++)
      if (active & (1u << i)) { soma += PESOS[i]; cnt++; }

    // média arredondada para o inteiro mais próximo
    int e = (soma >= 0) ? (soma + cnt / 2) / cnt : (soma - cnt / 2) / cnt;
    erroLUT[m] = (int8_t)e;
  }
}

uint8_t Carrinho::lerLinhaMascara()
{
  uint16_t ab = mcp.readGPIOAB();
  return (uint8_t)(ab & 0x00FF); // GPIOA
}

float Carrinho::calcularErroMascara(uint8_t m)
{
  int8_t e = erroLUT[m];
  if (e == (int8_t)0x7F) return (float)ERRO_SEM_LINHA;
  return (float)e;
}

float Carrinho::lerErro()
{
  return calcularErroMascara(lerLinhaMascara());
}

void Carrinho::seguidorImprimir(uint8_t mascara)
{
  for (int i = 7; i >= 0; i--) Serial.print((mascara >> i) & 1);
  Serial.print("\t");
}

// -------- Motores / PWM --------
void Carrinho::initDutyLUT()
{
  dutyLUT[0] = 0;
  for (int v = 1; v <= 100; v++)
    dutyLUT[v] = (uint8_t)(150 + ((uint16_t)(v - 1) * 105) / 99);
}

void Carrinho::writePWM(int m, int lado, uint8_t duty)
{
  if (dutyAtual[m][lado] != duty)
  {
    dutyAtual[m][lado] = duty;
    ledcWrite(chMotor[m][lado], duty);
  }
}

void Carrinho::motoresBegin()
{
  initDutyLUT();
  for (uint8_t i = 0; i < 4; i++)
  {
    for (uint8_t j = 0; j < 2; j++)
    {
      pinMode(pinMotor[i][j], OUTPUT);
      ledcSetup(chMotor[i][j], Freq_PWM, Resol_PWM);
      ledcAttachPin(pinMotor[i][j], chMotor[i][j]);
      ledcWrite(chMotor[i][j], 0);
      dutyAtual[i][j] = 0;
    }
  }
}

void Carrinho::motoresPararTodos()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    ledcWrite(chMotor[i][0], 0);
    ledcWrite(chMotor[i][1], 0);
    dutyAtual[i][0] = dutyAtual[i][1] = 0;
  }
}

void Carrinho::acionaMotor(int i, int velPct)
{
  if (i < 0 || i > 3) return;

  if (velPct > 100) velPct = 100;
  else if (velPct < -100) velPct = -100;

  if (invertMotor[i]) velPct = -velPct;

  uint8_t duty = porcentagemPWM((uint8_t)abs(velPct));
  if (duty == 0) { writePWM(i, 0, 0); writePWM(i, 1, 0); return; }

  if (velPct >= 0) { writePWM(i, 0, duty); writePWM(i, 1, 0); }
  else             { writePWM(i, 0, 0);    writePWM(i, 1, duty); }
}

void Carrinho::acionaRodasOminiInt(int vx, int vy, int omega)
{
  int v0 = vy + vx + omega;
  int v1 = vy - vx - omega;
  int v2 = vy - vx + omega;
  int v3 = vy + vx - omega;

  int a0 = abs(v0), a1 = abs(v1), a2 = abs(v2), a3 = abs(v3);
  int m = max(max(a0, a1), max(a2, a3));
  if (m > 10000)
  {
    v0 = (int)((int64_t)v0 * 10000 / m);
    v1 = (int)((int64_t)v1 * 10000 / m);
    v2 = (int)((int64_t)v2 * 10000 / m);
    v3 = (int)((int64_t)v3 * 10000 / m);
  }

  auto iround = [](int x) -> int { return (x >= 0 ? (x + 50) / 100 : (x - 50) / 100); };
  acionaMotor(0, iround(v0));
  acionaMotor(1, iround(v1));
  acionaMotor(2, iround(v2));
  acionaMotor(3, iround(v3));
}

int Carrinho::toCent(float v)
{
  return (int)(v >= 0.0f ? v * 100.0f + 0.5f : v * 100.0f - 0.5f);
}

void Carrinho::controlarRodas(float vy, float vx, float omega)
{
  int vy_c = constrain(toCent(vy), -10000, 10000);
  int vx_c = constrain(toCent(vx), -10000, 10000);
  int om_c = constrain(toCent(omega), -10000, 10000);
  acionaRodasOminiInt(vx_c, vy_c, om_c);
}

// -------- PID --------
float Carrinho::pidAtualizar(float erro, float dt)
{
  // Integral com limitador simples (anti-windup)
  integralAcumulada += erro * dt;
  float lim = 100.0f / (ki > 0.0f ? ki : 0.001f);
  integralAcumulada = constrain(integralAcumulada, -lim, lim);

  // Derivada filtrada (RC de primeira ordem)
  float derivadaBruta = (erro - erroAnterior) / (dt > 0.0f ? dt : 0.001f);
  float alpha = dt / (tauD + dt);
  dFilt = dFilt + alpha * (derivadaBruta - dFilt);
  erroAnterior = erro;

  float omega = kp * erro + ki * integralAcumulada + kd * dFilt;
  omega = constrain(omega, -omegaMax, omegaMax);
  return omega;
}

// -------- Modos internos --------
void Carrinho::modoCalibracao(uint8_t mascara, float erro)
{
  motoresPararTodos();
  static uint32_t tPrint = 0;
  if (millis() - tPrint > 300)
  {
    tPrint = millis();
    Serial.print("CALIBRACAO - Mascara: ");
    seguidorImprimir(mascara);
    Serial.print("Erro: ");
    if (erro == ERRO_SEM_LINHA) Serial.println("sem linha");
    else                        Serial.println(erro, 2);
  }
}

void Carrinho::modoEsperandoLargada(uint8_t mascara)
{
  motoresPararTodos();
  if (mascara & 0b00011000)
  {
    estado = CORRIDA;
    erroAnterior = 0.0f;
    integralAcumulada = 0.0f;
    Serial.println("estado: CORRIDA");
  }
}

void Carrinho::modoCorrida(uint8_t, float erro, float dt)
{
  if (erro == ERRO_SEM_LINHA)
  {
    int vy_c = toCent(10.0f);
    int om_c = (ultimoErroValido >= 0.0f)
                   ? toCent(INVERTER_OMEGA ? -12.0f : +12.0f)
                   : toCent(INVERTER_OMEGA ? +12.0f : -12.0f);
    acionaRodasOminiInt(0, vy_c, om_c);
    logStatus(ERRO_SEM_LINHA, 0.0f);
    return;
  }

  ultimoErroValido = erro;
  float omega = pidAtualizar(erro, dt);
  if (INVERTER_OMEGA) omega = -omega;

  controlarRodas(vyPercent, vxPercent, omega);
  logStatus(erro, omega);
}

// -------- APIs de modo --------
void Carrinho::entrarCalibracao()
{
  estado = CALIBRACAO;
  erroAnterior = 0.0f;
  integralAcumulada = 0.0f;
  motoresPararTodos();
}

void Carrinho::iniciarSeguirLinha()
{
  estado = ESPERANDO_LARGADA;
  erroAnterior = 0.0f;
  integralAcumulada = 0.0f;
}

void Carrinho::seguirLinhaStep()
{
  static uint32_t lastMicros = micros();
  uint32_t now = micros();
  float dt = clampDt((now - lastMicros) * 1e-6f);
  lastMicros = now;

  float erro = lerErro();
  if (erro == ERRO_SEM_LINHA)
  {
    controlarRodas(10.0f, 0.0f,
                   (ultimoErroValido >= 0.0f)
                       ? (INVERTER_OMEGA ? -12.0f : +12.0f)
                       : (INVERTER_OMEGA ? +12.0f : -12.0f));
    logStatus(ERRO_SEM_LINHA, 0.0f);
    return;
  }

  ultimoErroValido = erro;
  float omega = pidAtualizar(erro, dt);
  if (INVERTER_OMEGA) omega = -omega;

  controlarRodas(vyPercent, vxPercent, omega);
  logStatus(erro, omega);
}

// -------- Ciclo de vida --------
void Carrinho::begin()
{
  seguidorInitLUT();
  motoresBegin();
  tPrevMicros = micros();
  printHelp();
  Serial.println("estado: CALIBRACAO");
}

void Carrinho::tick()
{
  processaSerial();

  uint32_t t = micros();
  float dt = clampDt((t - tPrevMicros) * 1e-6f);
  tPrevMicros = t;

  uint8_t mascara = lerLinhaMascara();
  float erro = calcularErroMascara(mascara);

  switch (estado)
  {
    case CALIBRACAO:        modoCalibracao(mascara, erro); break;
    case ESPERANDO_LARGADA: modoEsperandoLargada(mascara); break;
    case CORRIDA:           modoCorrida(mascara, erro, dt); break;
  }
}

// -------- Utilitários estáticos --------
int Carrinho::popcount8(uint8_t x)
{
  x = (x & 0x55) + ((x >> 1) & 0x55);
  x = (x & 0x33) + ((x >> 2) & 0x33);
  return (int)((x + (x >> 4)) & 0x0F);
}

bool Carrinho::verificaBloco(uint8_t m, bool direita, uint8_t qnt)
{
  return (popcount8(m) >= qnt) && (m & (direita ? 0x01 : 0x80));
}

bool Carrinho::detectaCentro(uint8_t m)
{
  return (m & 0b00011000) == 0b00011000;
}

void Carrinho::setVelocidades(float vy, float vx) {
  // avanço: 0..100 (costuma-se evitar ré na base do seguidor)
  vyPercent = constrain(vy, 0.0f, 100.0f);
  // lateral: -100..100 (esquerda/dir)
  vxPercent = constrain(vx, -100.0f, 100.0f);
  Serial.printf("vy=%.1f vx=%.1f\n", vyPercent, vxPercent);
}

void Carrinho::setVy(float vy) {
  vyPercent = constrain(vy, 0.0f, 100.0f);
  Serial.printf("vy=%.1f\n", vyPercent);
}

void Carrinho::setVx(float vx) {
  vxPercent = constrain(vx, -100.0f, 100.0f);
  Serial.printf("vx=%.1f\n", vxPercent);
}
