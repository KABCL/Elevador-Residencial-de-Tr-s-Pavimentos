#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <stdio.h>
#include <string.h>
//  PINOS
const byte botaoSet = 2;// INT0 - libera após emergência
const byte botaoEmergencia = 3;// INT1 - para tudo
const byte botaoCabine2 = 4;// botão da cabine: 2º andar
const byte botaoCabine1 = 5;// botão da cabine: 1º andar
const byte botaoCabineTerreo = 6;// botão da cabine: Térreo
const byte pinoServo = 7;// porta
const byte botaoChamadaTerreoSobe = 8;// chamada Térreo (subir)
const byte motorEsquerdo = 9;// ponte H, lado esquerdo
const byte motorDireito = 10;// ponte H, lado direito
const byte registradorDados = 11;// registrador: DI
const byte botaoChamada2Desce = 12;// chamada 2º andar (descer)
const byte registradorClock = 13;// registrador: clock
const byte botaoChamada1Desce = A0;// chamada 1º andar (descer)
const byte botaoChamada1Sobe = A1;// chamada 1º andar (subir)
const byte fimCursoA = A2;// fim de curso, bit 0
const byte fimCursoB = A3;// fim de curso, bit 1
// A4 = SDA, A5 = SCL (usados pela biblioteca Wire)
//  CONFIGURAÇÕES
const byte enderecoLcd[3] = { 62, 63, 64 };// Térreo, 1º, 2º
const bool inverterMotor = false;// true troca subir <-> descer
const byte anguloPortaAberta = 90;
const byte anguloPortaFechada = 0;
const unsigned long tempoMortoMs = 300;// motor parado antes de religar
const unsigned long tempoPortaAbertaMs = 3000;
const unsigned long tempoPortaFecharMs = 1000;
const unsigned long tempoMaxMovMs = 30000;// sem chegar a um andar = falha
const unsigned long tempoAvisoQuedaMs = 2000;// aviso após queda de energia
const unsigned long tempoSensorProntoMs = 100;// filtro do sensor estabilizar
const unsigned long tempoLcdLigarMs = 100;// espera antes de iniciar LCDs
const unsigned long debounceBotaoMs = 30;
const unsigned long debounceSensorMs = 40;
const unsigned long debounceEmergenciaMs = 50;
//  ESTADOS DO ELEVADOR
const byte estadoAvisoQueda = 0;// mostra o aviso de queda de energia
const byte estadoReferenciando = 1;// procurando um andar (ao ligar / após emergência)
const byte estadoParado = 2;// parado com a porta fechada, esperando pedidos
const byte estadoSubindo = 3;
const byte estadoDescendo = 4;
const byte estadoPortaAberta = 5;
const byte estadoPortaFechando = 6;
const byte estadoEmergencia = 7;// travado até apertar SET
//  VARIÁVEIS GLOBAIS
Servo servoPorta;
byte estado = estadoReferenciando;
int  andarAtual = 0;// último andar confirmado pelo sensor
int  motorDir = 0;// 0 parado, +1 subindo, -1 descendo
int  dirPreferida = 1;// última direção usada (para desempate)
unsigned long tBoot = 0;// instante em que o programa iniciou
unsigned long tAviso = 0;// início do aviso de queda de energia
unsigned long tInicioMov = 0;// início do movimento (controle de falha)
unsigned long tLiberaMov = 0;// instante a partir do qual o motor pode ligar
unsigned long tPorta = 0;// temporizador da porta
// Filas de pedidos (true = pedido pendente)
bool pedCab[3] = { false, false, false };// botões da cabine
bool pedSobe[3] = { false, false, false };// chamadas "subir" nos andares
bool pedDesce[3] = { false, false, false };// chamadas "descer" nos andares
byte ledsCabine = 0;// estado de Q0..Q7 do registrador
// Emergência (escritas dentro de interrupções => volatile)
volatile bool flagEmergencia = false;
volatile bool pedidoSet = false;
const char* msgEmergencia = "EMERGENCIA";
// Botões
const byte idCabineTerreo = 0;// índices dos botões nos vetores
const byte idCabine1 = 1;
const byte idCabine2 = 2;
const byte idChamadaTerreoSobe = 3;
const byte idChamada1Sobe = 4;
const byte idChamada1Desce = 5;
const byte idChamada2Desce = 6;
const byte quantidadeBotoes = 7;
const byte pinoBotao[quantidadeBotoes] = {
  botaoCabineTerreo, botaoCabine1, botaoCabine2,
  botaoChamadaTerreoSobe, botaoChamada1Sobe, botaoChamada1Desce, botaoChamada2Desce
};
bool estadoBotao[quantidadeBotoes];
unsigned long tBotao[quantidadeBotoes];
// Bits do registrador de deslocamento (saída Qn = bit n)
const byte ledSobe = 3;// Q3: cabine subindo
const byte ledDesce = 4;// Q4: cabine descendo
const byte ledAlarme = 5;// Q5: alarme (sistema travado)
const byte mascaraLedsCabine = 0x07;// Q0..Q2
// Textos
const char* nomeAndar[3] = { "TERREO", "1o ANDAR", "2o ANDAR" };
// EEPROM
const byte eepromMagic = 0;// marca de EEPROM válida
const byte eepromAndar = 1;// último andar confirmado
const byte eepromMovimento = 2;// 1 = estava em movimento
const byte valorMagico = 0xA5;
//  TEMPORIZAÇÃO SEM BLOQUEIO
void liberarMovimentoApos(unsigned long ms) {
  unsigned long t = millis() + ms;
  if ((long)(t - tLiberaMov) > 0) tLiberaMov = t;
}
//  MOTOR
void motorParar() {
  digitalWrite(motorEsquerdo, LOW);
  digitalWrite(motorDireito, LOW);
  motorDir = 0;
}
void motorMover(int dir) {
  motorParar();// garante estado seguro
  bool sobe = (dir > 0) != inverterMotor;
  noInterrupts();// a emergência não pode ser
  if (!flagEmergencia){// desfeita por esta função
    if (sobe) digitalWrite(motorEsquerdo, HIGH);// direita continua LOW
    else digitalWrite(motorDireito, HIGH);// esquerda continua LOW
    motorDir = dir;
  }
  interrupts();
}
//  REGISTRADOR DE DESLOCAMENTO
void enviarLeds() {
  shiftOut(registradorDados, registradorClock, MSBFIRST, ledsCabine);
}
void atualizarLedsStatus() {
  byte novo = ledsCabine & mascaraLedsCabine;       // mantém os da cabine
  if (motorDir > 0) novo |= (1 << ledSobe);
  if (motorDir < 0) novo |= (1 << ledDesce);
  if (estado == estadoEmergencia) novo |= (1 << ledAlarme);
  if (novo != ledsCabine) {
    ledsCabine = novo;
    enviarLeds();
  }
}
//  LCD I2C
const byte passosLcd = 6;// inicialização
const byte comandosLcd[passosLcd] = { 0x28, 0x28, 0x28, 0x0C, 0x01, 0x06 };
const byte esperasLcd[passosLcd] = {5, 1, 0, 0, 3, 0};
bool lcdPronto = false;// true quando a inicialização terminou
byte lcdPasso = 0;
unsigned long tLcd = 0;
char alvoL[2][17] = { "", "" };// texto desejado de cada linha
bool linhaSuja[2] = { false, false }; // linha precisa ser reescrita
char bufEsc[17];// cópia da linha que está sendo escrita
bool escAtivo = false;
byte escLinha = 0, escLcd = 0, escCol = 0;
void lcdComando(byte end, byte cmd) {
  Wire.beginTransmission(end);
  Wire.write((byte)0x80);
  Wire.write(cmd);
  Wire.endTransmission();
}
void lcdDado(byte end, char c) {
  Wire.beginTransmission(end);
  Wire.write((byte)0x40);
  Wire.write((byte)c);
  Wire.endTransmission();
}
// Define o texto das duas linhas (nos 3 displays). Só marca para reescrever
// as linhas que realmente mudaram; a escrita acontece em lcdServico().
void mostrar(const char* l1, const char* l2) {
  char b[2][17];
  snprintf(b[0], sizeof(b[0]), "%-16s", l1);
  snprintf(b[1], sizeof(b[1]), "%-16s", l2);
  for (byte i = 0; i < 2; i++) {
    if (strcmp(b[i], alvoL[i]) != 0) {
      strcpy(alvoL[i], b[i]);
      linhaSuja[i] = true;
    }
  }
}
// Chamado a cada volta do loop(): faz no máximo uma transmissão I2C
void lcdServico() {
  unsigned long agora = millis();
  // --- Fase 1: inicialização dos 3 displays, um comando por vez ------------
  if (!lcdPronto) {
    if ((long)(agora - tLcd) < 0) return;// ainda esperando
    if (lcdPasso >= passosLcd) { lcdPronto = true; return; }
    for (byte i = 0; i < 3; i++) lcdComando(enderecoLcd[i], comandosLcd[lcdPasso]);
    tLcd = agora + esperasLcd[lcdPasso];
    lcdPasso++;
    return;
  }
  // --- Fase 2: escrita de uma linha, caractere por caractere ---------------
  if (!escAtivo) {
    if (linhaSuja[0]) escLinha = 0;
    else if (linhaSuja[1]) escLinha = 1;
    else return;// nada a escrever
    linhaSuja[escLinha] = false;
    strcpy(bufEsc, alvoL[escLinha]);
    escAtivo = true;
    escLcd = 0;
    escCol = 0;
  }
  byte end = enderecoLcd[escLcd];
  if (escCol == 0) lcdComando(end, 0x80 | (escLinha ? 0x40 : 0x00));// cursor
  else lcdDado(end, bufEsc[escCol - 1]);
  escCol++;
  if (escCol > 16){// terminou este LCD
    escCol = 0;
    escLcd++;
    if (escLcd >= 3) escAtivo = false;// terminou os 3
  }
}
// Monta as duas linhas conforme o estado atual
void atualizarDisplay() {
  char l1[17];
  const char* l2 = "";
  snprintf(l1, sizeof(l1), "Andar: %s", nomeAndar[andarAtual]);
  switch (estado) {
    case estadoAvisoQueda:
      snprintf(l1, sizeof(l1), "QUEDA DE ENERGIA");
      l2 = "VERIFICANDO...";
      break;
    case estadoReferenciando: l2 = "REINICIANDO";break;
    case estadoParado: l2 = "PARADO";break;
    case estadoSubindo: l2 = "SUBINDO";break;
    case estadoDescendo: l2 = "DESCENDO";break;
    case estadoPortaAberta: l2 = "PORTA ABERTA";break;
    case estadoPortaFechando: l2 = "PORTA FECHANDO";break;
    case estadoEmergencia:
      snprintf(l1, sizeof(l1), "!! ATENCAO !!");
      l2 = msgEmergencia;
      break;
  }
  mostrar(l1, l2);
}
//  EEPROM  
void salvarEstado(int andar, byte emMovimento) {
  EEPROM.update(eepromAndar, (byte)andar);
  EEPROM.update(eepromMovimento, emMovimento);
}
//  SENSORES DE POSIÇÃO
int lerSensorBruto() {
  bool a = (digitalRead(fimCursoA) == LOW);
  bool b = (digitalRead(fimCursoB) == LOW);
  if (a && b) return 2;
  if (a) return 0;
  if (b) return 1;
  return -1;
}
int sensorAndar() {
  static int estavel = -1, candidato = -1;
  static unsigned long t0 = 0;
  int bruto = lerSensorBruto();
  if (bruto != candidato) {
    candidato = bruto;
    t0 = millis();
  } else if (millis() - t0 >= debounceSensorMs) {
    estavel = candidato;
  }
  return estavel;
}
//  PEDIDOS (filas)
bool haPedidoEm(int f) {
  return pedCab[f] || pedSobe[f] || pedDesce[f];
}
// Existe algum pedido nos andares à frente de f, na direção dir?
bool haPedidoAdiante(int f, int dir) {
  if (dir > 0) {
    for (int i = f + 1; i < 3; i++) if (haPedidoEm(i)) return true;
  } else {
    for (int i = f - 1; i >= 0; i--) if (haPedidoEm(i)) return true;
  }
  return false;
}
bool deveParar(int f, int dir) {
  if (pedCab[f]) return true;// alguém quer descer aqui
  if (dir > 0 && pedSobe[f]) return true;// chamada no mesmo sentido
  if (dir < 0 && pedDesce[f]) return true;
  // Nada mais à frente: atende também a chamada de sentido contrário
  if (!haPedidoAdiante(f, dir) && (pedSobe[f] || pedDesce[f])) return true;
  return false;
}
// Dá por atendidos todos os pedidos do andar f (porta aberta nele)
void atenderAndar(int f) {
  pedCab[f] = false;
  pedSobe[f] = false;
  pedDesce[f] = false;
  ledsCabine &= ~(1 << f);
  enviarLeds();
}
void limparPedidos() {
  for (byte i = 0; i < 3; i++) {
    pedCab[i] = false; pedSobe[i] = false; pedDesce[i] = false;
  }
  ledsCabine &= ~mascaraLedsCabine;// apaga só os LEDs dos botões
  enviarLeds();
}
// Escolhe a próxima direção: +1 sobe, -1 desce, 0 nenhum pedido
int escolherDirecao() {
  bool acima = false, abaixo = false;
  for (int i = andarAtual + 1; i < 3; i++) if (haPedidoEm(i)) acima = true;
  for (int i = andarAtual - 1; i >= 0; i--) if (haPedidoEm(i)) abaixo = true;
  if (dirPreferida > 0) {
    if (acima) return +1;
    if (abaixo) return -1;
  } else {
    if (abaixo) return -1;
    if (acima)  return +1;
  }
  return 0;
}
//  BOTÕES
void registrarPedido(byte b) {
  switch (b) {
    case idCabineTerreo: pedCab[0] = true; ledsCabine |= (1 << 0);break;
    case idCabine1: pedCab[1] = true; ledsCabine |= (1 << 1);break;
    case idCabine2: pedCab[2] = true; ledsCabine |= (1 << 2);break;
    case idChamadaTerreoSobe: pedSobe[0] = true;break;
    case idChamada1Sobe: pedSobe[1] = true;break;
    case idChamada1Desce: pedDesce[1] = true;break;
    case idChamada2Desce: pedDesce[2] = true;break;
  }
  enviarLeds();
}
// registrar = false: lê para manter o antirruído, mas ignora (emergência)
void lerBotoes(bool registrar) {
  for (byte i = 0; i < quantidadeBotoes; i++) {
    bool lido = digitalRead(pinoBotao[i]);
    if (lido != estadoBotao[i] && millis() - tBotao[i] > debounceBotaoMs) {
      tBotao[i] = millis();
      estadoBotao[i] = lido;
      if (lido == LOW && registrar) registrarPedido(i);
    }
  }
}
//  INTERRUPÇÕES
void isrEmergencia() {
  static unsigned long ultimo = 0;
  unsigned long agora = millis();
  if (agora - ultimo < debounceEmergenciaMs) return;
  ultimo = agora;
  // Corta o motor imediatamente, sem esperar o loop()
  digitalWrite(motorEsquerdo, LOW);
  digitalWrite(motorDireito, LOW);
  flagEmergencia = true;
}
void isrSet() {
  static unsigned long ultimo = 0;
  unsigned long agora = millis();
  if (agora - ultimo < debounceEmergenciaMs) return;
  ultimo = agora;
  pedidoSet = true;      // o loop() só considera se estiver em emergência
}
//  TRANSIÇÕES DE ESTADO
void entrarEmergencia(const char* msg) {
  motorParar();
  salvarEstado(andarAtual, 0);
  limparPedidos();
  msgEmergencia = msg;
  estado = estadoEmergencia;
  pedidoSet = false;
  atualizarDisplay();
}
void abrirPorta() {
  servoPorta.write(anguloPortaAberta);
  tPorta = millis();
  estado = estadoPortaAberta;
  atualizarDisplay();
}
void iniciarMovimento(int dir) {
  dirPreferida = dir;
  salvarEstado(andarAtual, 1);
  motorMover(dir);
  if (motorDir == 0) return;// emergência bloqueou a partida
  tInicioMov = millis();
  estado = (dir > 0) ? estadoSubindo : estadoDescendo;
  atualizarDisplay();
}
// Para a cabine ao chegar em um andar e abre a porta
void chegarNoAndar(int f) {
  motorParar();
  liberarMovimentoApos(tempoMortoMs);
  andarAtual = f;
  salvarEstado(f, 0);
  atenderAndar(f);
  abrirPorta();
}
//  SETUP
void setup() {
  // 1) Motor desligado antes de qualquer outra coisa
  pinMode(motorEsquerdo, OUTPUT);
  pinMode(motorDireito, OUTPUT);
  motorParar();
  // 2) Entradas (todas com pull-up interno; botão fecha para o GND)
  for (byte i = 0; i < quantidadeBotoes; i++) {
    pinMode(pinoBotao[i], INPUT_PULLUP);
    estadoBotao[i] = HIGH;
    tBotao[i] = 0;
  }
  pinMode(fimCursoA, INPUT_PULLUP);
  pinMode(fimCursoB, INPUT_PULLUP);
  pinMode(botaoEmergencia, INPUT_PULLUP);
  pinMode(botaoSet, INPUT_PULLUP);
  // 3) Registrador de deslocamento: tudo apagado
  pinMode(registradorDados, OUTPUT);
  pinMode(registradorClock, OUTPUT);
  enviarLeds();
  // 4) Porta fechada
  servoPorta.attach(pinoServo);
  servoPorta.write(anguloPortaFechada);
  // 5) I2C: os LCDs são iniciados aos poucos pelo lcdServico()
  Wire.begin();
  tLcd = millis() + tempoLcdLigarMs;
  // 6) Interrupções de emergência
  attachInterrupt(digitalPinToInterrupt(botaoEmergencia), isrEmergencia, FALLING);
  attachInterrupt(digitalPinToInterrupt(botaoSet), isrSet, FALLING);
  // 7) Verifica se o sistema foi desligado em pleno movimento
  bool caiuEnergia = false;
  if (EEPROM.read(eepromMagic) != valorMagico) {// primeira vez
    EEPROM.update(eepromMagic, valorMagico);
    salvarEstado(0, 0);
  } else {
    caiuEnergia = (EEPROM.read(eepromMovimento) == 1);
    byte a = EEPROM.read(eepromAndar);
    if (a <= 2) andarAtual = a;
  }
  tBoot = millis();
  liberarMovimentoApos(tempoPortaFecharMs);// dá tempo da porta fechar
  if (caiuEnergia) {
    tAviso = tBoot;
    estado = estadoAvisoQueda;
  } else {
    estado = estadoReferenciando;
  }
  atualizarDisplay();
}
//  LOOP PRINCIPAL
void loop() {
  unsigned long agora = millis();
  int s = sensorAndar();
  // --- Emergência tem prioridade sobre tudo ---------------------------------
  if (flagEmergencia) {
    flagEmergencia = false;
    entrarEmergencia("EMERGENCIA");
  }
  if (estado != estadoEmergencia) pedidoSet = false;   // SET só vale em emergência
  // --- Botões (ignorados durante a emergência) ------------------------------
  lerBotoes(estado != estadoEmergencia);
  bool podeMover = ((long)(agora - tLiberaMov) >= 0);
  switch (estado) {
    // ---- Aviso de queda de energia (o motor continua desligado) -------------
    case estadoAvisoQueda:
      if (agora - tAviso >= tempoAvisoQuedaMs) {
        estado = estadoReferenciando;
        atualizarDisplay();
      }
      break;
    // ---- Procura um andar: usado ao ligar e após a emergência ---------------
    case estadoReferenciando:
      if (agora - tBoot < tempoSensorProntoMs) break;// filtro ainda calibrando
      if (s >= 0){// já está em um andar
        motorParar();
        andarAtual = s;
        salvarEstado(s, 0);
        liberarMovimentoApos(tempoMortoMs);
        estado = estadoParado;
        atualizarDisplay();
      } else if (motorDir == 0 && podeMover){// entre andares: desce
        motorMover(-1);
        tInicioMov = agora;
      } else if (motorDir != 0 && agora - tInicioMov > tempoMaxMovMs) {
        entrarEmergencia("FALHA SENSOR");
      }
      break;
    // ---- Parado: decide o que fazer -----------------------------------------
    case estadoParado:
      if (haPedidoEm(andarAtual)){// pedido no andar atual
        atenderAndar(andarAtual);
        abrirPorta();
      } else if (podeMover) {
        int d = escolherDirecao();
        if (d != 0) iniciarMovimento(d);
      }
      break;
    // ---- Em movimento -------------------------------------------------------
    case estadoSubindo:
    case estadoDescendo: {
      int d = (estado == estadoSubindo) ? +1 : -1;
      // Só vale a leitura do andar vizinho, na direção do movimento. Isso
      // ignora leituras impossíveis (por exemplo, duas chaves fechadas ao
      // mesmo tempo, que o circuito lê como 2º andar).
      if (s >= 0 && s == andarAtual + d){// chegou ao próximo andar
        andarAtual = s;
        tInicioMov = agora;
        bool fimDeCurso = (d > 0 && s == 2) || (d < 0 && s == 0);
        if (fimDeCurso || deveParar(s, d)) {
          chegarNoAndar(s);
        } else {
          atualizarDisplay();
        }
      } else if (agora - tInicioMov > tempoMaxMovMs) {
        entrarEmergencia("FALHA SENSOR");
      }
      break;
    }
    // ---- Porta aberta -------------------------------------------------------
    case estadoPortaAberta:
      if (haPedidoEm(andarAtual)) {// novo pedido: estende
        atenderAndar(andarAtual);
        tPorta = agora;
      }
      if (agora - tPorta >= tempoPortaAbertaMs) {
        servoPorta.write(anguloPortaFechada);
        tPorta = agora;
        estado = estadoPortaFechando;
        atualizarDisplay();
      }
      break;
    // ---- Porta fechando -----------------------------------------------------
    case estadoPortaFechando:
      if (haPedidoEm(andarAtual)){// alguém chamou: reabre
        atenderAndar(andarAtual);
        abrirPorta();
      } else if (agora - tPorta >= tempoPortaFecharMs) {
        liberarMovimentoApos(tempoMortoMs);
        estado = estadoParado;
        atualizarDisplay();
      }
      break;
    // ---- Emergência: só sai com o botão SET ---------------------------------
    case estadoEmergencia:
      if (pedidoSet) {
        pedidoSet = false;
        servoPorta.write(anguloPortaFechada);// fecha a porta
        liberarMovimentoApos(tempoPortaFecharMs); // espera a porta fechar
        estado = estadoReferenciando;
        atualizarDisplay();
      }
      break;
  }
  // --- LEDs de situação e display (no máximo 1 mensagem I2C por volta) ------
  atualizarLedsStatus();
  lcdServico();
}