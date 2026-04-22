# FingerRehab 3R — Robot Planar para Rehabilitación de Dedo

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://python.org)
[![Matplotlib](https://img.shields.io/badge/Matplotlib-3.x-orange)](https://matplotlib.org)
[![UCR](https://img.shields.io/badge/UCR-II1105-green)](https://ucr.ac.cr)

> Proyecto Integrador — II1105 Automatización y Robótica Avanzada  
> Universidad de Costa Rica | I Ciclo 2026 | Prof. Carlos Morales

---

## Descripción

**FingerRehab 3R** es la simulación de un brazo robótico planar de 3 grados de libertad (3R) diseñado para asistir la rehabilitación de pacientes con fracturas o lesiones en los dedos de la mano. Las tres articulaciones del robot modelan las tres falanges (proximal, media y distal) de un dedo humano.

El proyecto implementa:
- Cinemática directa (FK) usando transformaciones homogéneas
- Cinemática inversa (IK) analítica con ley de cosenos (dos soluciones: codo+/codo-)
- Interfaz interactiva con sliders de ángulos, display de posición en tiempo real y cálculo de IK desde un punto objetivo
- Animación con interpolación cúbica entre ejercicios de rehabilitación
- Análisis del espacio de trabajo (workspace)

---

## Integrantes del equipo

| Nombre | Carnet |
|--------|--------|
| Steve Boza| C11186 |
| Salomé Wicky | C18655 |
| Sebastián González | C03363 |
| Daniel Brizuela | C11274 |

---

## Estructura del repositorio

```
FingerRehab3R/
├── cinematica_directa.py   # FK con 4 ejercicios y visualización
├── cinematica_inversa.py   # IK analítica con validación de workspace
├── interfaz.py             # Interfaz interactiva completa (requisito principal)
├── animacion.py            # Animación de trayectoria entre ejercicios (bonus)
├── README.md               # Este archivo
├── copilot_prompts.md      # Registro de prompts de GitHub Copilot
└── docs/
    └── documento_diseno.docx  # Documento de diseño completo
```

---

## Requisitos de instalación

**Python 3.8 o superior** con las siguientes librerías:

```bash
pip install numpy matplotlib
```

Verificar instalación:
```bash
python -c "import numpy, matplotlib; print('OK')"
```

---

## Cómo ejecutar cada archivo

### `interfaz.py` — Interfaz interactiva (archivo principal)

```bash
python interfaz.py
```

Abre una ventana con:
- **Sliders de ángulos** (θ1, θ2, θ3): mueven el robot en tiempo real
- **Display X/Y/φ**: posición del efector final actualizada en cada frame
- **Cinemática inversa**: ingresa X (mm), Y (mm), φ (°) y presiona "CALCULAR IK"
- **Ejercicios preset**: 4 configuraciones de rehabilitación
- **Sliders de longitud**: ajustan las dimensiones de las falanges

---

### `cinematica_directa.py` — Cinemática directa

```bash
python cinematica_directa.py
```

Solicita por consola:
1. Longitudes de falanges en mm
2. Número de ejercicio (1–4)

Muestra la posición del efector y una visualización gráfica del robot.

**Ejercicios disponibles:**
| # | Nombre | θ1 | θ2 | θ3 |
|---|--------|----|----|-----|
| 1 | Posición neutra | 0° | 0° | 0° |
| 2 | Flexión leve | 30° | 30° | 15° |
| 3 | Flexión moderada | 45° | 45° | 45° |
| 4 | Flexión profunda | 30° | 60° | 15° |

---

### `cinematica_inversa.py` — Cinemática inversa

```bash
python cinematica_inversa.py
```

Solicita por consola:
1. Longitudes de falanges en mm
2. Posición objetivo X (mm), Y (mm)
3. Orientación objetivo φ (°)

Devuelve las dos soluciones articulares (codo arriba y codo abajo), verifica con FK y muestra el robot en la pose calculada.

---

### `animacion.py` — Animación de trayectoria (bonus)

```bash
python animacion.py
```

Abre una ventana animada donde al cambiar de ejercicio con los radio buttons, el robot se mueve suavemente con interpolación cúbica. La trayectoria del efector se traza en rojo.

---

## Especificación técnica del robot

| Parámetro | Valor |
|-----------|-------|
| Tipo | Brazo planar serial 3R |
| DOF | 3 (tres articulaciones revoluta) |
| Plano de operación | XY (2D) |
| Falange proximal (L1) | 45 mm (configurable) |
| Falange media (L2) | 30 mm (configurable) |
| Falange distal (L3) | 20 mm (configurable) |
| Alcance máximo | L1 + L2 + L3 |
| Método FK | Transformaciones homogéneas 3×3 |
| Método IK | Geométrico analítico (ley de cosenos) |

---

## Ejemplo de uso rápido

Ejecutar la interfaz y probar cinemática inversa:

1. `python interfaz.py`
2. En los campos de IK, escribir: X=200, Y=150, φ=90
3. Presionar **CALCULAR IK**
4. Observar que el robot se mueve al punto objetivo
5. Alternar entre soluciones con el radio button "codo+/codo-"

---

## Verificación matemática

La cinemática inversa fue verificada con round-trip FK→IK→FK para los 4 ejercicios:

```
Error de round-trip: 0.00000000 mm (precisión de punto flotante)
```

---

## Enlace al documento de diseño

Ver `docs/documento_diseno.docx` para el documento completo con ecuaciones, diagrama de arquitectura, análisis de workspace y reflexión sobre GitHub Copilot.
