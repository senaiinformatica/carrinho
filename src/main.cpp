// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include "carrinho.h"

// --- Hardware ---
static int PINO_BOTAO_ENCODER = 19; // botão do encoder, ativo em LOW

Adafruit_MCP23X17 mcp;
Carrinho carrinho(mcp);

// --- Controle de partida ---
static uint32_t ATRASO_PARTIDA_MS = 300; // atraso pós-largada
static uint32_t instanteInicioAtraso = 0;

// Flags de estado (sem enum)
static bool emCorrida = false;       // está seguindo a linha?
static bool emAtrasoPartida = false; // aguardando atraso pós-largada?

void pararCarrinho();
bool botaoEncoderPressionadoBorda();
bool lerBotaoEncoderBruto();
void pararCarrinho();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }

  pinMode(PINO_BOTAO_ENCODER, INPUT_PULLUP); // ativo em LOW

  Wire.begin();
  mcp.begin_I2C();
  for (uint8_t i = 0; i < 8; ++i)
    mcp.pinMode(i, INPUT);

  carrinho.begin();
  carrinho.setPID(6.0f, 0.5f, 0.5f);    // valores PID iniciais kp, ki, kd
  carrinho.setVelocidades(25.0f, 0.0f); // velocidades vy, vx

  Serial.println("=== Modo BOTAO (ENCODER pino 19) ===");
  Serial.println("Inicia: estar sobre a faixa central + pressionar o botao.");
  Serial.println("Parar: pressionar o botao novamente.");
}

#define CALIBRACAO true

void loop()
{

  if (CALIBRACAO)
  {
    carrinho.tick();
    return;
  }

  else
  {
    const uint8_t mascaraSensores = carrinho.lerLinhaMascara();
    const bool faixaCentralDetectada = carrinho.detectaCentro(mascaraSensores);

    // Aguardando condição de largada
    if (!emCorrida && !emAtrasoPartida)
    {
      if (faixaCentralDetectada && botaoEncoderPressionadoBorda())
      {
        emAtrasoPartida = true;
        instanteInicioAtraso = millis();
        carrinho.controlarRodas(0.0f, 0.0f, 0.0f);
        Serial.println(">> LARGADA acionada, aguardando atraso...");
      }
      else
      {
        carrinho.controlarRodas(0.0f, 0.0f, 0.0f);
      }
    }

    // Atraso pós-largada (para estabilizar)
    if (emAtrasoPartida)
    {
      if (botaoEncoderPressionadoBorda())
      {
        pararCarrinho();
      }
      else if (millis() - instanteInicioAtraso >= ATRASO_PARTIDA_MS)
      {
        emAtrasoPartida = false;
        emCorrida = true;
        Serial.println(">> INICIOU corrida");
      }
      else
      {
        carrinho.controlarRodas(0.0f, 0.0f, 0.0f);
      }
    }

    // Seguimento ativo
    if (emCorrida)
    {
      if (botaoEncoderPressionadoBorda())
      {
        pararCarrinho();
      }
      else
      {
        carrinho.seguirLinhaStep(); // PID+telemetria
      }
    }
  }
}



// --- Botão: debounce e detecção de borda ---
bool lerBotaoEncoderBruto()
{
  return digitalRead(PINO_BOTAO_ENCODER) == LOW;
}

bool botaoEncoderPressionadoBorda()
{
  static bool estadoEstavel = false; // false = solto, true = pressionado
  static bool ultimoEstavel = false;
  static uint32_t instanteMudanca = 0;

  bool agora = lerBotaoEncoderBruto();
  if (agora != estadoEstavel)
  {
    if (millis() - instanteMudanca >= 30)
    { // debounce 30 ms
      ultimoEstavel = estadoEstavel;
      estadoEstavel = agora;
      instanteMudanca = millis();
      return (estadoEstavel == true && ultimoEstavel == false); // borda de pressão
    }
  }
  else
  {
    instanteMudanca = millis();
  }
  return false;
}

void pararCarrinho()
{
  emCorrida = false;
  emAtrasoPartida = false;
  carrinho.controlarRodas(0.0f, 0.0f, 0.0f);
  Serial.println(">> PAROU (botao)");
}