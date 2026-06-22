/*==============================================================================
  INVERSOR SPWM PUENTE H — ESP32 — ACUMULADOR DE FASE (preparado para PLL)
  ==============================================================================

  QUE CAMBIA RESPECTO AL LEGACY VALIDADO
  --------------------------------------
  Reemplaza el motor de fase de INDICE ENTERO por un ACUMULADOR DE FASE
  fraccionario (tecnica DDS / NCO). Todo lo demas se conserva igual: fault
  handler, guarda fina del GPTimer, supresion de pulso, las dos bases de tiempo.

  POR QUE (dos razones medidas en el analizador):
  1. RESOLUCION: el indice entero solo daba frecuencias discretas (sampleNum=333
     -> 60.06Hz, 334 -> 59.88Hz, sin nada en medio). El (int) que truncaba
     20000/60=333.33 a 333 clavaba la salida en ~60.06Hz, y la guarda subia a
     ~60.21Hz. El acumulador de 32 bits da resolucion de 4.66 uHz/paso.
  2. PREPARADO PARA PLL: la frecuencia la define UNA variable, phase_increment.
     Un futuro PLL solo ajusta esa variable en tiempo real para enganchar la red.
     El tiempo de guarda en los cruces lo absorbe el lazo del PLL automaticamente.

  COMO FUNCIONA EL ACUMULADOR DE FASE:
    - phase_acc (32 bits) avanza phase_increment cada muestra de portadora.
    - Los bits ALTOS de phase_acc indexan la tabla de seno (idx = acc >> 22).
    - El acumulador DESBORDA naturalmente (wrap de 32 bits) = reinicio del ciclo,
      sin truncamiento. La fase es fraccionaria y exacta.
    - frecuencia = (phase_increment / 2^32) * FREQ_CARR_HZ
    - phase_increment = (f_deseada / FREQ_CARR_HZ) * 2^32

  GANCHO PARA EL PLL:
    phase_increment es 'volatile'. Hoy se fija para 60Hz. Manana, el PLL lo
    modula: si la red adelanta, sube phase_increment; si atrasa, lo baja. El
    NCO (este acumulador) es el oscilador que el PLL pilotea.

  VALIDAR EN ANALIZADOR:
    - Frecuencia de la fundamental: debe medir ~60.0000 Hz (no 60.21).
    - PWM D0/D1: portadora 20kHz, dead-time 300ns, complementarios.
    - Cruce: durante la guarda (D7) los 4 canales en bajo.
    - Cambiar TEST_FREQ_HZ y verificar que la frecuencia sigue el valor pedido
      con precision fina (prueba del gancho del PLL).

  HARDWARE: ESP32 clasico, APB 80MHz. Core 3.x / IDF v5.x (gptimer).
 =============================================================================*/

#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/gptimer.h"
#include "soc/rtc.h"
#include "soc/gpio_sig_map.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

extern "C" void gpio_matrix_in(uint32_t gpio, uint32_t signal_idx, bool inv);

/*==============================================================================
  CAPA DE BASE DE TIEMPO (single source of truth) — sin cambios respecto al legacy
==============================================================================*/
#define APB_HZ        80000000.0f
#define PRESCALER     5
#define TICK_NS       ((1.0f / APB_HZ) * PRESCALER * 1e9f)   // 62.5 ns

#define NS_TO_TICKS(ns)  ((uint32_t)((float)(ns) / TICK_NS + 0.5f))
#define TICKS_TO_NS(t)   ((float)(t) * TICK_NS)

#define DT_TICK_NS          6.25f      // base del dead-time (160MHz), MEDIDA
#define NS_TO_TICKS_DT(ns)  ((uint32_t)((float)(ns) / DT_TICK_NS + 0.5f))
#define DT_TICKS_TO_NS(t)   ((float)(t) * DT_TICK_NS)

#define FREQ_CARR_HZ  20000.0f
#define FREQ_MOD_HZ   60.0f            // frecuencia objetivo de la fundamental
#define DEADTIME_NS   300.0f
#define AMP_PERCENT   0.90f

/* --- SUPRESION DE PULSO MINIMO: habilitacion ---
   Interruptor para activar (1) o desactivar (0) la supresion de pulso minimo.
   Util para depurar: con el analizador puedes comparar el cruce CON supresion
   (1, pulsos diminutos cerca del cruce se omiten -> salida limpia) vs SIN
   supresion (0, todos los pulsos salen, incluso los deformes mas estrechos que
   el dead-time). Ajusta segun tu carga y lo que midas.                         */
#define MIN_PULSE_ENABLE  1     // 1 = suprimir pulsos minimos ; 0 = no suprimir

#define MIN_PULSE_NS    (2.0f * DEADTIME_NS * 1.2f)
#define MIN_PULSE_TICKS ((int)(MIN_PULSE_NS / TICK_NS + 0.5f))

#define PERIOD_TICKS    ((int)( (1.0f/FREQ_CARR_HZ)/2.0f/(TICK_NS*1e-9f) - 1.0f + 0.5f ))
#define DEADTIME_TICKS  NS_TO_TICKS_DT(DEADTIME_NS)
#define AMPLITUDE_TICKS ((int)(AMP_PERCENT * PERIOD_TICKS))
#define REAL_FREQ_CARR  ( 1.0f / ((PERIOD_TICKS + 1) * TICK_NS * 1e-9f * 2.0f) )

/*==============================================================================
  ===== ACUMULADOR DE FASE (NCO) — EL CAMBIO CENTRAL =====
==============================================================================*/
#define ACC_BITS     32                       // acumulador de 32 bits
#define TABLE_BITS   10                        // tabla de 2^10 = 1024 entradas
#define TABLE_SIZE   (1 << TABLE_BITS)
#define ACC_SHIFT    (ACC_BITS - TABLE_BITS)   // = 22; bits altos -> indice tabla

/* phase_increment para una frecuencia: (f / f_carr) * 2^32.
   El +0.5 redondea. Usa la portadora REAL (medida), no la nominal.             */
#define PHASE_INC(f)  ((uint32_t)((double)(f) / (double)REAL_FREQ_CARR * 4294967296.0 + 0.5))

/* Frecuencia que el NCO genera con un incremento dado (para reportar/verificar). */
#define FREQ_OF_INC(inc)  ((double)(inc) / 4294967296.0 * (double)REAL_FREQ_CARR)

/* Frecuencia de prueba: cambia este valor y mide; verifica que la salida la
   sigue con precision fina (esto prueba el gancho del futuro PLL).             */
#define TEST_FREQ_HZ   60.0f

/* Tabla de seno CON signo, escalada a la amplitud (como el codigo original).
   idx alto del acumulador -> entrada de la tabla.                              */
int16_t sine_table[TABLE_SIZE];

/* Estado del NCO (volatile: la ISR lo avanza, el PLL/loop lo ajustaria). */
volatile uint32_t phase_acc       = 0;
volatile uint32_t phase_increment = 0;   // se fija en setup; el PLL lo modularia

/*------------------------------------------------------------------------------
  PINES (sin cambios)
------------------------------------------------------------------------------*/
const int HO1 = 23, LO1 = 22, HO2 = 21, LO2 = 19;
const int DIAG_CRUCE = 18;
#define DIAG_BIT (1 << 18)
const int FAULT_DRIVE = 5;
#define FAULT_BIT (1 << 5)
#define HO2_BIT (1 << 21)
#define LO2_BIT (1 << 19)
#define HO1_BIT (1 << 23)
#define LO1_BIT (1 << 22)

#define MCPWM_CMPR0_REG    0x3FF5E040
#define MCPWM_INT_CLR_REG  0x3FF5E11C
#define MCPWM_CLK_CFG      0x3FF5E000
#define MCPWM_TIMER0_CFG0  0x3FF5E004
#define MCPWM_GEN0_STMP    0x3FF5E03C
#define MCPWM_INT_ENA      0x3FF5E110

const int   tmrRegVal = PERIOD_TICKS;
float real_freqCarr   = REAL_FREQ_CARR;
const int   amplitude = AMPLITUDE_TICKS;

/*------------------------------------------------------------------------------
  GUARDA DEL CRUCE (sin cambios)
------------------------------------------------------------------------------*/
#define GUARD_TICKS   10
#define GUARD_CORE    1

volatile int      prevSign    = 0;
volatile int      pendingSign = 0;
volatile uint32_t guard_ticks = GUARD_TICKS;
volatile int      guard_ready = 0;

gptimer_handle_t gt = NULL;
void initGuardTimer(void* arg);

/*==============================================================================
  CALLBACK DEL GPTIMER (fin de guarda) — sin cambios
==============================================================================*/
static bool IRAM_ATTR onGuardElapsed(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx) {
  GPIO.out_w1tc = FAULT_BIT;
  if (pendingSign > 0) GPIO.out_w1ts = HO2_BIT;
  else                 GPIO.out_w1ts = LO2_BIT;
  GPIO.out_w1tc = DIAG_BIT;
  return false;
}

/*==============================================================================
  ISR DEL MCPWM (~20 kHz) — AHORA CON ACUMULADOR DE FASE
  El cambio central: en vez de i++ y sin(radVal*i), se avanza el acumulador y se
  lee la tabla. La deteccion de cruce, el fault y la supresion: IGUAL que el legacy.
==============================================================================*/
void IRAM_ATTR MCPWM_ISR(void*) {
  WRITE_PERI_REG(MCPWM_INT_CLR_REG, BIT(3));

  /* --- AVANCE DEL ACUMULADOR DE FASE (NCO) ---
     phase_acc desborda solo (wrap de 32 bits) = fin de ciclo, sin truncamiento.
     idx = bits altos del acumulador -> entrada de la tabla de seno.            */
  phase_acc += phase_increment;
  uint32_t idx = phase_acc >> ACC_SHIFT;          // 0 .. TABLE_SIZE-1
  int sineVal = sine_table[idx];                  // seno con signo, escalado
  int sign    = (sineVal > 0) ? 1 : -1;

  /* --- SUPRESION DE PULSO MINIMO (igual que el legacy) --- */
  /* --- SUPRESION DE PULSO MINIMO (respeta la bandera MIN_PULSE_ENABLE) ---
     Si MIN_PULSE_ENABLE es 0, suppress queda siempre en false y todos los pulsos
     salen (util para ver el comportamiento sin supresion en el analizador).     */
#if MIN_PULSE_ENABLE
  bool suppress = (sineVal > -MIN_PULSE_TICKS && sineVal < MIN_PULSE_TICKS);
#else
  bool suppress = false;
#endif

  /* --- DETECCION DE CRUCE POR CERO (igual que el legacy) --- */
  if (sign != prevSign) {
    GPIO.out_w1ts = FAULT_BIT;
    GPIO.out_w1tc = HO2_BIT | LO2_BIT;
    GPIO.out_w1ts = DIAG_BIT;
    WRITE_PERI_REG(MCPWM_CMPR0_REG, 0);
    pendingSign   = sign;
    uint64_t now = 0;
    gptimer_get_raw_count(gt, &now);
    gptimer_alarm_config_t al = { .alarm_count = now + guard_ticks,
                                  .reload_count = 0,
                                  .flags = { .auto_reload_on_alarm = false } };
    gptimer_set_alarm_action(gt, &al);
    prevSign = sign;
  }

  /* --- ESCRITURA DEL DUTY (igual que el legacy) --- */
  if (suppress) {
    if (sign > 0) WRITE_PERI_REG(MCPWM_CMPR0_REG, 0);
    else          WRITE_PERI_REG(MCPWM_CMPR0_REG, tmrRegVal);
  } else if (sineVal > 0) {
    WRITE_PERI_REG(MCPWM_CMPR0_REG, sineVal);
  } else {
    WRITE_PERI_REG(MCPWM_CMPR0_REG, tmrRegVal + sineVal);
  }
  /* NOTA: ya NO hay 'i++' ni 'if (i>=sampleNum) i=0'. El acumulador se encarga. */
}

/*==============================================================================
  INIT DEL GPTIMER (sin cambios)
==============================================================================*/
void initGuardTimer(void* arg) {
  gptimer_config_t tcfg = {
    .clk_src = GPTIMER_CLK_SRC_APB, .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, .flags = { .intr_shared = false },
  };
  esp_err_t err = gptimer_new_timer(&tcfg, &gt);
  if (err != ESP_OK) Serial.printf(">>> FALLO gptimer: %s\n", esp_err_to_name(err));
  gptimer_enable(gt);
  gptimer_set_raw_count(gt, 0);
  int64_t t0 = esp_timer_get_time();
  gptimer_start(gt);
  while (esp_timer_get_time() - t0 < 2000) { }
  uint64_t tk = 0; gptimer_get_raw_count(gt, &tk);
  int64_t t1 = esp_timer_get_time();
  gptimer_stop(gt);
  float res = (float)tk / ((float)(t1 - t0) * 1e-6f);
  Serial.printf("GPTimer res: %.0f Hz (%.4f us/tick) GUARD_TICKS=%u\n",
                res, 1e6f / res, guard_ticks);
  gptimer_disable(gt);
  gptimer_event_callbacks_t cbs = { .on_alarm = onGuardElapsed };
  gptimer_register_event_callbacks(gt, &cbs, NULL);
  gptimer_enable(gt);
  gptimer_set_raw_count(gt, 0);
  gptimer_start(gt);
  guard_ready = 1;
  vTaskDelete(NULL);
}

/*==============================================================================
  REPORTE — ahora incluye el NCO y la resolucion de frecuencia
==============================================================================*/
void reportar() {
  Serial.println();
  Serial.println("===== BASE DE TIEMPO + ACUMULADOR DE FASE (NCO) =====");
  Serial.printf("TIMER    : TICK = %.3f ns (APB %.0fMHz / presc %d)\n",
                TICK_NS, APB_HZ/1e6, PRESCALER);
  Serial.printf("DEAD-TIME: %.3f ns/tick (160MHz, medida)\n", DT_TICK_NS);
  Serial.printf("portadora: %.0f Hz -> period_ticks=%d -> real %.2f Hz\n",
                FREQ_CARR_HZ, PERIOD_TICKS, REAL_FREQ_CARR);
  Serial.printf("dead-time: %.0f ns -> %d ticks -> real %.1f ns\n",
                DEADTIME_NS, DEADTIME_TICKS, DT_TICKS_TO_NS(DEADTIME_TICKS));
  Serial.println("----- NCO (acumulador de fase) -----");
  Serial.printf("acumulador: %d bits, tabla %d entradas, shift %d\n",
                ACC_BITS, TABLE_SIZE, ACC_SHIFT);
  Serial.printf("phase_increment = %u\n", phase_increment);
  Serial.printf("frecuencia generada = %.6f Hz (objetivo %.4f)\n",
                FREQ_OF_INC(phase_increment), TEST_FREQ_HZ);
  Serial.printf("resolucion = %.4f uHz por paso de phase_increment\n",
                (1.0/4294967296.0)*REAL_FREQ_CARR*1e6);
  Serial.printf("supresion pulso minimo: %s (umbral %d ticks / %.0f ns)\n",
                MIN_PULSE_ENABLE ? "ACTIVADA" : "desactivada",
                MIN_PULSE_TICKS, MIN_PULSE_NS);
  Serial.println(">>> Para el PLL: ajustar phase_increment mueve la frecuencia <<<");
  Serial.println("=====================================================");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("==== INVERSOR SPWM — ACUMULADOR DE FASE (preparado para PLL) ====");

  /* Construir la tabla de seno: sin() con signo, escalado a amplitude. */
  for (int k = 0; k < TABLE_SIZE; k++) {
    float ang = 2.0f * PI * k / TABLE_SIZE;
    sine_table[k] = (int16_t)(amplitude * sinf(ang));
  }

  /* Fijar el incremento para la frecuencia objetivo. El PLL lo modularia. */
  phase_increment = PHASE_INC(TEST_FREQ_HZ);

  reportar();

  pinMode(LO2, OUTPUT); pinMode(HO2, OUTPUT);
  pinMode(DIAG_CRUCE, OUTPUT); pinMode(FAULT_DRIVE, OUTPUT);
  GPIO.out_w1tc = FAULT_BIT | HO2_BIT | LO2_BIT | DIAG_BIT;

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, HO1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, LO1);

  mcpwm_config_t pwm_config;
  pwm_config.frequency    = real_freqCarr * 2;
  pwm_config.cmpr_a       = 0;
  pwm_config.cmpr_b       = 0;
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
  pwm_config.duty_mode    = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  WRITE_PERI_REG(MCPWM_CLK_CFG, 0);
  uint32_t reg_val = ((PRESCALER - 1) & 0xFF) | ((tmrRegVal & 0xFFFF) << 8);
  WRITE_PERI_REG(MCPWM_TIMER0_CFG0, reg_val);
  WRITE_PERI_REG(MCPWM_GEN0_STMP, 2);

  esp_intr_alloc(ETS_PWM0_INTR_SOURCE, ESP_INTR_FLAG_IRAM, MCPWM_ISR, NULL, NULL);

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        DEADTIME_TICKS, DEADTIME_TICKS);

  gpio_matrix_in(FAULT_DRIVE, PWM0_F0_IN_IDX, false);
  mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0);
  mcpwm_fault_set_cyc_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0,
                           MCPWM_FORCE_MCPWMXA_LOW, MCPWM_FORCE_MCPWMXB_LOW);

  TaskHandle_t h = NULL;
  xTaskCreatePinnedToCore(initGuardTimer, "guardInit", 4096, NULL, 5, &h, GUARD_CORE);
  while (guard_ready == 0) { delay(1); }

  WRITE_PERI_REG(MCPWM_INT_ENA, BIT(3));

  Serial.println("Listo. NCO corriendo. Mide la fundamental: debe dar ~60.0000 Hz.");
}

void loop() {
  /* Aqui ira el PLL en el futuro: leer fase de la red, comparar, ajustar
     phase_increment. Por ahora, el NCO corre a frecuencia fija.
     Prueba del gancho: descomenta para barrer la frecuencia y verificar en el
     analizador que la salida la sigue con precision fina.
  */
  // static float f = 60.0f;
  // f += 0.01f; if (f > 60.5f) f = 59.5f;
  // phase_increment = PHASE_INC(f);
  // delay(500);
  delay(1000);
}
