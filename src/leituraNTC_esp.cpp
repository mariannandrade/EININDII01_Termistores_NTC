// This library works only with the following circuit topology
// Vcc---NTC---ADC---SERIES_RESISTOR---GND
#include <iikit.h>
#include <math.h>

#define ADC_RESOLUTION     4095   // ESP32: use 4095
#define TEMPERATURENOMINAL 25

// ---------- FILTROS ----------
// Lê N amostras, ordena rápido (N pequeno), usa mediana e
// ainda faz média do miolo (robusto a espúrios).
static int readStableADC(int pin) {
  const int N = 9;          // número de amostras (ímpar)
  const int DISCARD = 2;    // descarta 2 de cada lado
  int v[N];
  for (int i = 0; i < N; ++i) v[i] = analogRead(pin);

  // ordenação simples (N pequeno)
  for (int i = 0; i < N; ++i)
    for (int j = i + 1; j < N; ++j)
      if (v[j] < v[i]) { int t = v[i]; v[i] = v[j]; v[j] = t; }

  long sum = 0;
  for (int i = DISCARD; i < N - DISCARD; ++i) sum += v[i];
  return (int)(sum / (N - 2*DISCARD));  // média do miolo
}

// Filtro IIR simples: y += alpha * (x - y);  0<alpha<=1
static float iir(float y_prev, float x, float alpha) {
  return y_prev + alpha * (x - y_prev);
}
// ----------------------------

double getTempTermistorNTCBeta(const uint16_t analogValue,
                               const uint16_t serialResistance,
                               const uint16_t bCoefficient,
                               const uint16_t nominalResistance)
{
  float resistance, temp;
  // convert the value to resistance
  resistance = (serialResistance / ((float)analogValue)) * ADC_RESOLUTION - serialResistance;
  temp = 1.0 / ((1.0 / (TEMPERATURENOMINAL + 273.15)) +
                (1.0 / bCoefficient) * log(resistance / nominalResistance)); // 1/( (1/To)+(1/B)*ln(R/Ro) )
  return (temp - 273.15);
}

double getTempTermistorNTCSteinhart(const uint16_t analogValue,
                                    const uint16_t serialResistance,
                                    const float a, const float b, const float c)
{
  float resistance, temp;
  // convert the value to resistance
  resistance = (serialResistance / ((float)analogValue)) * ADC_RESOLUTION - serialResistance;
  resistance = log(resistance);
  temp = 1.0 / (a + b * resistance + c * resistance * resistance * resistance);
  return (temp - 273.15);
}

void setup()
{
  IIKit.setup();

  // Opcional (ESP32): melhora faixa/ruído
  analogReadResolution(12);                 // 0..4095
  analogSetPinAttenuation(def_pin_ADC1, ADC_11db); // ~0..3.3V
}

#define TIME_DELAY_MS1 1000 // Aguarda um segundo
uint64_t previousTimeMS1 = 0;

// Estados do filtro IIR (um por variável)
float tBetaFilt      = NAN;
float tSteinhartFilt = NAN;
const float IIR_ALPHA = 0.2f;  // mais baixo = mais suave

void loop()
{
  IIKit.loop();
  const uint64_t currentTimeMS = millis();

  if ((currentTimeMS - previousTimeMS1) >= TIME_DELAY_MS1)
  {
    // mantém fase estável do “relógio”
    previousTimeMS1 += TIME_DELAY_MS1;

    // 1) ADC estabilizado por mediana/média do miolo
    const uint16_t adc = (uint16_t)readStableADC(def_pin_ADC1);

    // 2) Converte para temperatura
    const float temperature1 = getTempTermistorNTCBeta(adc,     // Analog Value
                                                       10000,   // Nominal resistance at 25 ºC
                                                       3455,    // beta do NTC
                                                       10000);  // série

    const float temperature2 = getTempTermistorNTCSteinhart(adc,
                                                            10000,          // série
                                                            0.001129241f,   // a
                                                            0.0002341077f,  // b
                                                            0.00000008775468f); // c

    // 3) Suaviza com IIR
    if (isnan(tBetaFilt))      tBetaFilt      = temperature1; else tBetaFilt      = iir(tBetaFilt,      temperature1, IIR_ALPHA);
    if (isnan(tSteinhartFilt)) tSteinhartFilt = temperature2; else tSteinhartFilt = iir(tSteinhartFilt, temperature2, IIR_ALPHA);

    // 4) Envia/mostra somente as versões filtradas
    IIKit.WSerial.plot("Temp Beta",      currentTimeMS, tBetaFilt);
    IIKit.disp.setText(2, ("TB:" + String(tBetaFilt, 2)).c_str());

    IIKit.WSerial.plot("Temp Steinhart", currentTimeMS, tSteinhartFilt);
    IIKit.disp.setText(3, ("TS:" + String(tSteinhartFilt, 2)).c_str());
  }
}
