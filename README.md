<img src="/mcpwm_06.png" width="600" alt="PPL">
<img src="/mcpwm_07.png" width="600" alt="PPL">
<img src="/mcpwm_08.png" width="600" alt="PPL">



# -esp32-spwm-phase_increment_v2_1
Migración del motor de fase SPWM de un inversor de puente H sobre ESP32, desde un índice de muestra entero a un acumulador de fase de 32 bits con tabla de seno (el patrón DDS/NCO), habilitando salida precisa de 60.000 Hz y sentando la base para rampas de frecuencia, control de modulación y un futuro PLL de grid-tie.


# Mejora del Acumulador de Fase — De Índice Entero a Arquitectura DDS/NCO

> **Documento técnico — fundamento de diseño, validación medida y hoja de ruta.**
> Migración del motor de fase SPWM de un inversor de puente H sobre ESP32, desde un
> índice de muestra entero a un **acumulador de fase** de 32 bits con **tabla de seno**
> (el patrón DDS/NCO), habilitando salida precisa de 60.000 Hz y sentando la base para
> rampas de frecuencia, control de modulación y un futuro PLL de grid-tie.

**Propósito del documento (doble):**
1. **Referencia académica** — explica el *porqué*, la evidencia medida y la teoría,
   para que un ingeniero pueda entender y reproducir la decisión de diseño.
2. **Contexto cognitivo para una IA** — estructurado para que un modelo de lenguaje
   pueda cargar este documento y continuar el trabajo con entendimiento completo de la
   arquitectura, las restricciones, el estado validado y los pasos siguientes
   pretendidos. Cada sección expone no solo qué se hizo, sino el razonamiento y la
   verdad medida que lo respalda.

**Metodología de todo el proyecto:** *medir antes de creer.* Cada afirmación de abajo
fue verificada con un analizador lógico (Saleae) en el pin físico, no asumida de la
documentación. Las afirmaciones numéricas incluyen la medición que las confirmó.

---

## 0. Contexto para un agente que continúa (leer primero)

Este inversor sintetiza una senoidal de 60 Hz mediante **SPWM unipolar, topología
totem-pole con conmutación complementaria activa y dead-time simétrico**, sobre un
ESP32 clásico (APB 80 MHz, driver MCPWM legacy). La arquitectura es de **un solo
reloj**: una sola ISR (la ISR de portadora a ~20 kHz) controla tanto la magnitud del
seno como el cruce de polaridad. Esta propiedad de un-solo-reloj es la razón de su
robustez y de que no presente deslizamiento de fase (una exploración previa de la API
moderna `mcpwm_prelude.h` forzó una arquitectura de dos relojes que se deslizaba —ver
la bitácora de migración a prelude del proyecto; el diseño legacy de un solo reloj se
retuvo como arquitectónicamente superior para este chip).

**Fundamentos validados (no re-derivar; confiar en estos valores medidos):**
- Base de tiempo del timer: 62.5 ns/tick (APB 80 MHz / prescaler 5), verificada en 3
  niveles (cálculo = registro = pin).
- Base de tiempo del dead-time: **6.25 ns/tick (160 MHz)** — un reloj SEPARADO del
  timer, MEDIDO (48 ticks = 286 ns → 6.25 ns/tick). Es un hecho de hardware no obvio.
- Portadora: 20.000 kHz (period_ticks = 399). Dead-time: 300 ns (48 ticks de dead-time).
- Cruce por cero limpio mediante fault handler + guarda con GPTimer libre (~13 µs).

**Este documento cubre solo la mejora del motor de fase.** Todo lo demás (fault
handler, guarda, supresión de pulso, las dos bases de tiempo) permanece sin cambios.

---

## 1. Planteamiento del problema

El motor de fase original usaba un **índice de muestra entero**:

```c
int sineVal = int(amplitude * sin(radVal * i));   // sin() calculado en cada ISR
i++;
if (i >= sampleNum) i = 0;                          // sampleNum = (int)(fcarr/fmod)
```

Dos defectos, uno real y uno artefacto de medición, estaban entrelazados:

**Defecto 1 (real) — cuantización de frecuencia por truncamiento.**
`sampleNum = (int)(20000 / 60) = (int)(333.33) = 333`. Con exactamente 333 muestras
por ciclo, la frecuencia real de la fundamental es `20000 / 333 = 60.060 Hz`, no
60.000. El truncamiento de la fracción 0.33 clavaba la salida ~0.06 Hz por encima.
Peor aún, el índice entero solo permite frecuencias **discretas**: 333 muestras →
60.06 Hz, 334 → 59.88 Hz, **sin nada en medio** (escalones de 0.18 Hz). Esta
resolución gruesa es incompatible con un PLL, que necesita control en milihertz o más
fino para enganchar suavemente.

**Defecto 2 (artefacto de medición) — lectura de ~60.21 Hz en el analizador.**
La vista automática "Timing" del analizador medía intervalos entre flancos de las
ramas lentas, que incluyen la permanencia de la guarda en cada cruce, produciendo
lecturas inconsistentes (60.02–60.21 Hz, saltando). Esto era en parte el efecto de la
guarda y en parte una mala lectura de *qué* se estaba midiendo. **Cuando se midió
correctamente el período completo de una rama (de flanco de subida a flanco de subida
= un ciclo de 60 Hz), dio 16.66 ms = 60.00 Hz tras la mejora.** Lección: una medición
ambigua puede exagerar un defecto; la medición correcta reveló el panorama real.

**Defecto computacional (independiente de la frecuencia):** `sin()` se evaluaba dentro
de la ISR a ~20 kHz. La evaluación trigonométrica es lenta y su tiempo de ejecución
puede variar con la entrada — indeseable en una ISR de tiempo real estricto que además
coordina el cruce.

---

## 2. Solución — Arquitectura DDS / NCO (acumulador de fase + tabla)

El arreglo reemplaza el índice entero por el patrón estándar de **Síntesis Digital
Directa (DDS)**: un acumulador de fase avanzado por un incremento de fase, indexando
una tabla de seno precalculada. Es el oscilador controlado numéricamente (NCO)
canónico.

### 2.1 El acumulador de fase

```c
#define ACC_BITS    32                       // acumulador de 32 bits
#define TABLE_BITS  10                        // tabla de 2^10 = 1024 entradas
#define TABLE_SIZE  (1 << TABLE_BITS)
#define ACC_SHIFT   (ACC_BITS - TABLE_BITS)   // = 22

volatile uint32_t phase_acc       = 0;
volatile uint32_t phase_increment = 0;        // define la frecuencia; el PLL lo modulará

// En la ISR de portadora (~20 kHz):
phase_acc += phase_increment;                 // desborda en 2^32 = reinicio de ciclo
uint32_t idx = phase_acc >> ACC_SHIFT;        // los bits altos indexan la tabla (0..1023)
int sineVal  = sine_table[idx];               // lectura de tabla — sin sin() en la ISR
```

**Relación de frecuencia (la ecuación central):**
```
f_salida = (phase_increment / 2^32) × f_portadora
phase_increment = (f_deseada / f_portadora) × 2^32
```

Para 60 Hz con portadora de 20.000 kHz:
`phase_increment = (60 / 20000) × 2^32 = 12,884,902` → genera **60.000001 Hz**.

**Resolución de frecuencia:** un LSB de `phase_increment` = `(1/2^32) × 20000 =`
**4.66 µHz**. Esta es la resolución sobre la que actúa un PLL. (Contraste: el índice
entero daba escalones de 0.18 Hz — una mejora de 38,000× en resolución.)

**Por qué el acumulador no se desliza:** el acumulador de 32 bits desborda
naturalmente (módulo 2^32), lo cual ES el reinicio del ciclo, sin truncamiento. Las
333.33 muestras fraccionarias por ciclo se preservan entre ciclos en los bits bajos
del acumulador — el 0.33 nunca se descarta, así que no puede acumularse en
deslizamiento. Esta es la propiedad de un-solo-reloj expresada numéricamente: la fase
del seno y el cruce derivan ambos del mismo acumulador, así que no pueden deslizarse
uno respecto al otro.

### 2.2 La tabla de seno

```c
int16_t sine_table[TABLE_SIZE];   // 1024 × 2 bytes = 2 KB de RAM

// Construida una sola vez al arrancar:
for (int k = 0; k < TABLE_SIZE; k++)
    sine_table[k] = (int16_t)(amplitude * sinf(2.0f*PI*k / TABLE_SIZE));
```

---

## 3. Índice de beneficios (tabla vs sin() en vivo)

| Aspecto | sin() en vivo (antes) | Tabla (ahora) | Beneficio |
|---|---|---|---|
| **Tiempo de ejecución ISR** | Variable, lento (eval. trig) | Constante, rápido (lectura mem.) | **Determinismo** — crítico en una ISR de tiempo real a 20 kHz |
| **Carga de CPU** | Alta (trig FP en cada ISR) | Despreciable (una lectura) | Libera CPU para la lógica del cruce y el futuro PLL |
| **Jitter de temporización** | El tiempo de sin() depende de la entrada | Acceso de tiempo fijo | Menor jitter en el lazo de control |
| **Costo de RAM** | ~0 | 2 KB (1024 × int16) | Intercambio barato: la RAM abunda en el ESP32 |
| **Práctica estándar** | Ad-hoc | Patrón DDS/NCO canónico | Se alinea con el diseño probado de síntesis en tiempo real |
| **Cuantización** | Ninguna (continua) | 1024 escalones angulares | A 333 muestras/ciclo sobre 1024 entradas, imperceptible; subir a 2048/4096 si hace falta |

**Neto:** la tabla gasta 2 KB de RAM barata para ganar velocidad, determinismo y menor
carga de CPU — un intercambio muy favorable para un inversor de tiempo real. La tabla
guarda la *forma* (el seno normalizado); la frecuencia y la amplitud se vuelven
perillas independientes encima.

---

## 4. La variable `phase_increment` y su potencial

`phase_increment` es la única variable de control que define la frecuencia de salida.
Su exposición como cantidad `volatile` es el **gancho** para tres capacidades futuras,
todas las cuales manipulan el mismo motor sin reescribirlo. Este es el valor
estratégico de la mejora: no se trata meramente de los 60 Hz exactos, sino de
**habilitar un árbol de funciones de control sobre una base correcta.**

### 4.1 Rampa de frecuencia (corto plazo, trivial)
Una rampa de frecuencia es un cambio gradual de `phase_increment`:
```c
if (phase_increment < target_inc) phase_increment += step;
else if (phase_increment > target_inc) phase_increment -= step;
```
A diferencia del índice entero (que saltaba 0.18 Hz entre valores adyacentes de
sampleNum), el acumulador hace rampas **continuas**, en pasos de microhertz —
arranque suave y transiciones de frecuencia sin escalones audibles ni transitorios
bruscos.

### 4.2 Control de modulación / amplitud (independiente de la frecuencia)
En DDS, frecuencia y amplitud son ortogonales. La amplitud es un factor de escala
sobre la salida de la tabla:
```c
int sineVal = (sine_table[idx] * mod_index) >> SCALE_BITS;
```
Habilita: control de tensión de salida (RMS), regulación de tensión en lazo cerrado
bajo carga, y arranque suave de amplitud — todo sin tocar la frecuencia.

### 4.3 PLL de grid-tie (largo plazo, el destino arquitectónico)
El acumulador ES el NCO que un PLL pilotea. Un lazo PLL digital:
```
1. Detectar la fase de la red (sensado aislado del cruce por cero de la red).
2. Comparar la fase de la red con phase_acc.
3. Calcular el error de fase.
4. Un controlador PI ajusta phase_increment para anular el error
   (red adelanta → subir incremento; red atrasa → bajar incremento).
5. Lazo → el inversor se engancha a la red.
```
La permanencia de la guarda en los cruces la absorbe automáticamente el lazo, ya que
el PLL corrige la fase continuamente contra la referencia de la red.

**Jerarquía de construcción (orden recomendado):** rampa de frecuencia →
amplitud/modulación → lazo cerrado de tensión → PLL. Cada una se apoya en la anterior;
todas se apoyan en este acumulador.

**Nota honesta de alcance para el PLL:** la parte difícil no es el NCO (ya hecho y
validado) sino el **sensado aislado de la red** — adquirir la fase de la red desde una
línea de alta tensión de forma segura hasta niveles del ESP32, con aislamiento
galvánico. Eso es tanto un reto de hardware (sensado aislado de red) como de software
(sintonización del lazo). Reservado para la Fase C.

---

## 5. Validación medida (verdad de referencia)

| Magnitud | Calculado | Medido (analizador) | Estado |
|---|---|---|---|
| Portadora (PWM, D0/D1) | 20.000 kHz | ~20 kHz (anchos varían por SPWM) | OK |
| Período fundamental (rama, subida a subida) | 16.6667 ms | **16.66 ms** | **CONFIRMADO 60.00 Hz** |
| `phase_increment` (60 Hz) | 12,884,902 | (reporte serial coincide) | OK |
| Frecuencia NCO (serial) | 60.000001 Hz | período rama 16.66 ms | cálculo = pin |

**Validación clave:** el serial reporta 60.000001 Hz; el período de rama medido
correctamente es 16.66 ms = 60.00 Hz. **Cálculo = pin** — el estándar de validación
del proyecto. El "60.21 Hz" anterior era la vista Timing del analizador (intervalos
entre flancos) afectada por la guarda, no el período real de la fundamental.

**Lección de medición (para el agente que continúa):** para medir la fundamental,
medir el PERÍODO COMPLETO de la rama (de flanco de subida al siguiente flanco de
subida de la misma rama lenta), NO la vista automática "Timing", que reporta
intervalos parciales afectados por la guarda y salta entre valores.

---

## 6. Estado y hoja de ruta

**Estado validado:** inversor monofásico, 60.000 Hz vía acumulador de fase + tabla de
seno de 1024 entradas, portadora 20 kHz, dead-time 300 ns (base 6.25 ns), cruce
limpio, gancho de PLL (`phase_increment`) en su lugar. Arquitectura de un solo reloj
(sin deslizamiento).

**Hoja de ruta:**
- **Fase A:** validar el cruce con carga inductiva real.
- **Fase B:** rampa de frecuencia (vía `phase_increment`); control de amplitud (vía
  `mod_index`); lazo cerrado de tensión; dead-time adaptativo por temperatura (trivial
  con el conversor de 6.25 ns); sobrecorriente por fault de hardware.
- **Fase C:** PLL de grid-tie (NCO listo; necesita sensado aislado de red +
  sintonización del lazo), paralelo en isla con droop control, anti-islanding.

**Ruta de hardware (para la etapa de potencia, decidida por separado):** gate driver
**UCC21520** (TI, aislado, dead-time de hardware programable como segunda capa de
seguridad, 2 unidades para el puente H completo), MOSFET IRFP260N (200 V, si el bus
llega a 100 V) o IRFB4110 (si el bus se queda en 48 V), bias aislado DC-DC por lado.
PCB en KiCad con validación estricta de cada pista, componente y área de cobre.

---

## 7. Referencia de código (el núcleo de la ISR modificado)

```c
// Motor de fase — el único cambio conceptual respecto al inversor legacy validado.
void IRAM_ATTR MCPWM_ISR(void*) {
  WRITE_PERI_REG(MCPWM_INT_CLR_REG, BIT(3));

  phase_acc += phase_increment;              // avance del NCO (desborda = reinicio)
  uint32_t idx = phase_acc >> ACC_SHIFT;     // bits altos → índice de tabla
  int sineVal  = sine_table[idx];            // lectura de tabla (sin sin() en la ISR)
  int sign     = (sineVal > 0) ? 1 : -1;

  // ---- detección de cruce, fault, guarda, supresión de pulso: SIN CAMBIOS ----
  // (idéntico al diseño legacy de un solo reloj; ver el archivo principal del inversor)
}
```

Todo lo que está fuera del motor de fase (fault handler, guarda con GPTimer, supresión
de pulso, las dos bases de tiempo, configuración del MCPWM a nivel de registro) es
byte por byte el diseño legacy validado. Esta mejora es quirúrgica: reemplaza solo la
generación de fase, preservando cada cimiento ganado con esfuerzo.

---

## 8. Glosario para referencia rápida

- **DDS (Direct Digital Synthesis):** técnica de síntesis de formas de onda donde una
  tabla precalculada se recorre a una velocidad definida por un incremento de fase.
- **NCO (Numerically Controlled Oscillator):** el oscilador resultante; su frecuencia
  la controla un número (`phase_increment`). Es el bloque que un PLL pilotea.
- **Acumulador de fase:** registro (aquí 32 bits) que se incrementa cada muestra; su
  desbordamiento natural marca el fin de ciclo. Sus bits altos indexan la tabla.
- **phase_increment:** el número que define la frecuencia. La perilla única de control.
- **Cuantización angular:** el "escalonado" del seno por usar una tabla finita; a más
  entradas, más fino. 1024 entradas son suficientes para 333 muestras/ciclo.
- **PLL (Phase-Locked Loop):** lazo que ajusta la frecuencia/fase del NCO para
  enganchar una referencia externa (la red). El NCO ya está; falta el lazo y el
  sensado aislado de la red.

---

*Documento preparado como referencia académica y contexto cognitivo para IA. El
acumulador de fase es el cimiento sobre el que se construirán la rampa de frecuencia,
el control de modulación y el PLL de grid-tie. La capacidad técnica, documentada y
reproducible con recursos modestos, es resiliente.*
