# Registro de Prompts — GitHub Copilot

> **II1105 Automatización y Robótica Avanzada — UCR I Ciclo 2026**  
> Reflexión sobre el uso de GitHub Copilot en el desarrollo de FingerRehab 3R

---

## Metodología de uso

Se utilizó GitHub Copilot en dos modos:
1. **Autocompletado en línea** (Tab): sugerencias contextuales mientras se escribe código
2. **Chat de Copilot** (Ctrl+I): prompts explícitos para generar bloques de código

A continuación se documentan los prompts más relevantes, el resultado obtenido y una reflexión crítica de cada uno.

---

## Prompts de Cinemática Directa (`cinematica_directa.py`)

---

### Prompt CD-01

**Prompt utilizado:**
```
# Compute forward kinematics for a 3-DOF planar robot using 
# 3x3 homogeneous transformation matrices.
# Parameters: theta1, theta2, theta3 (radians), l1, l2, l3 (meters)
# Returns: x, y position of end-effector and total orientation phi
def forward_kinematics(
```

**Resultado de Copilot:**
Copilot completó la función con la estructura correcta: definió subfunciones `rotation_matrix` y `translation_matrix`, las compuso como `T1 = R(theta1) @ T(l1)` y extrajo `x = T_total[0,2]`, `y = T_total[1,2]`.

**Evaluación:** ✅ Excelente. La sugerencia fue matemáticamente correcta y siguió exactamente la convención de transformaciones homogéneas 2D. Se usó directamente con mínimas modificaciones (agregar el cálculo de `orientation = theta1+theta2+theta3`).

---

### Prompt CD-02

**Prompt utilizado:**
```
# Plot the 3-link planar robot arm given joint angles and link lengths.
# Draw each link as a colored line segment, mark joints with circles,
# mark end-effector with a red square, add grid and labels.
def plot_robot(theta1, theta2, theta3, l_proximal, l_media, l_distal):
```

**Resultado de Copilot:**
Generó la función con `ax.plot` para cada segmento, `ax.scatter` para articulaciones y efector, y configuración básica de ejes. Sin embargo, usó `ax.set_xlim(0, total_length)` y `ax.set_ylim(0, total_length)`, cubriendo solo el cuadrante positivo.

**Evaluación:** ⚠️ Parcialmente útil. La estructura fue buena pero los límites de ejes estaban mal — si θ1 es negativo el robot queda fuera del frame. Se corrigió manualmente a `ax.set_xlim(-(total+margin), total+margin)`.

---

### Prompt CD-03

**Prompt utilizado:**
```
# Return joint angles in radians for rehabilitation exercise number 1-4.
# Exercise 1: neutral position (all zeros)
# Exercise 2: light flexion 30,30,15 degrees
# Exercise 3: moderate flexion 45,45,45 degrees
# Exercise 4: deep flexion 30,60,15 degrees
def get_exercise_angles(exercise_number):
```

**Resultado de Copilot:**
Generó el diccionario de ejercicios con la conversión `np.pi/4` para 45°, `np.pi/6` para 30°, etc.

**Evaluación:** ✅ Correcto. Copilot usó las constantes de `numpy` de forma apropiada. Solo fue necesario verificar que `np.pi/12 = 15°`.

---

## Prompts de Cinemática Inversa (`cinematica_inversa.py`)

---

### Prompt CI-01

**Prompt utilizado:**
```
# Compute inverse kinematics for 3R planar robot using wrist decoupling.
# Given target position (x, y) and orientation phi for end-effector,
# find joint angles (theta1, theta2, theta3).
# Use law of cosines. Return both elbow-up and elbow-down solutions.
# Return None if target is unreachable.
def inverse_kinematics(x, y, phi, l_proximal, l_media, l_distal):
```

**Resultado de Copilot:**
Generó el esquema completo: cálculo de la posición de la muñeca (`wrist_x = x - l3*cos(phi)`), la distancia `r`, la validación de workspace, la fórmula `cos_theta2 = (r^2 - l1^2 - l2^2)/(2*l1*l2)`, el `arccos` con sus dos signos, y el cálculo de `theta1` y `theta3`.

**Evaluación:** ✅ Muy bueno. La sugerencia fue matemáticamente correcta. Hubo que agregar el `np.clip(cos_theta2, -1, 1)` para evitar errores numéricos en los bordes del workspace, y la función `normalize_angle` para mantener ángulos en [-π, π].

---

### Prompt CI-02

**Prompt utilizado:**
```
# Normalize angle to range [-pi, pi]
def normalize_angle(angle):
```

**Resultado de Copilot:**
```python
return (angle + np.pi) % (2 * np.pi) - np.pi
```

**Evaluación:** ✅ Correcto y conciso. Se usó tal cual.

---

### Prompt CI-03

**Prompt utilizado:**
```
# Validate if point (x, y) is inside the robot workspace.
# The reachable workspace condition for 2R robot is:
# |l1 - l2| <= r <= l1 + l2
```

**Resultado de Copilot:**
Sugirió el condicional `if r > l1+l2 or r < abs(l1-l2): return None`.

**Evaluación:** ✅ Correcto. Esta es exactamente la condición de alcanzabilidad para el subproblema 2R (sin considerar la falange distal que se descuenta previamente).

---

## Prompts de Interfaz (`interfaz.py`)

---

### Prompt IF-01

**Prompt utilizado:**
```
# Create a matplotlib interactive interface for a 3R planar robot.
# Include sliders for joint angles theta1, theta2, theta3 (-180 to 180 degrees),
# sliders for link lengths, radio buttons for exercise selection,
# and a text box to display end-effector position in real time.
class RobotInterface:
```

**Resultado de Copilot:**
Generó el esquema de la clase con `__init__`, `_build_ui` y `_draw_robot`. Sin embargo, puso todos los sliders apilados sin separación visual y no incluyó la sección de cinemática inversa con TextBox.

**Evaluación:** ⚠️ Útil como punto de partida. La estructura de métodos fue buena pero el layout necesitó rediseño completo (coordenadas de `add_axes` ajustadas manualmente para separar las 4 secciones: ángulos, longitudes, ejercicios e IK).

---

### Prompt IF-02

**Prompt utilizado:**
```
# Add an IK section to the interface with three TextBox widgets for
# target X (mm), Y (mm), and phi (degrees), and a Button to compute IK.
# When the button is clicked, call inverse_kinematics() and update the sliders.
```

**Resultado de Copilot:**
Generó el código para crear `TextBox` widgets y el callback del botón. El error: usó `self.fig.add_subplot` en vez de `self.fig.add_axes`, lo que interfería con el layout existente.

**Evaluación:** ⚠️ La lógica del callback fue correcta, pero la creación de los axes hubo que ajustarla. Además Copilot no manejó el caso donde `inverse_kinematics` retorna `None`, que se agregó manualmente.

---

### Prompt IF-03

**Prompt utilizado:**
```
# Fix the axis limits so the robot always fills most of the plot area,
# regardless of the link lengths set by the sliders.
# The limits should update every time the robot is redrawn.
```

**Resultado de Copilot:**
Sugirió:
```python
total = self.l1 + self.l2 + self.l3
margin = total * 0.3
self.ax.set_xlim(-margin, total + margin)
```

**Evaluación:** ❌ Incompleto. Solo cubría cuadrante positivo. Se corrigió a:
```python
self.ax.set_xlim(-(total + margin), total + margin)
self.ax.set_ylim(-(total + margin), total + margin)
```
Este fue el bug de escala reportado originalmente por el compañero.

---

## Prompts de Animación (`animacion.py`)

---

### Prompt AN-01

**Prompt utilizado:**
```
# Animate smooth transition between two robot poses using cubic ease-in-out.
# Interpolate theta1, theta2, theta3 from start to target over ANIM_STEPS frames.
# Use the formula: t = progress^2 * (3 - 2*progress) for smoothing.
```

**Resultado de Copilot:**
Generó la función `_ease_in_out` y el método `_step_animation` con la interpolación lineal modulada por el factor cúbico.

**Evaluación:** ✅ Correcto. La fórmula de ease-in-out cúbico es matemáticamente estándar y Copilot la implementó bien.

---

### Prompt AN-02

**Prompt utilizado:**
```
# Draw the trajectory of the end-effector as a dashed line that grows
# as the robot moves. Store points in self.traj_x and self.traj_y lists.
```

**Resultado de Copilot:**
Generó el código para acumular puntos en las listas y actualizar `self.ln_traj.set_data()`.

**Evaluación:** ✅ Correcto. Se añadió manualmente el `self.traj_x.clear()` al cambiar longitudes con sliders, para que la trayectoria no mezcle configuraciones distintas.

---

## Resumen de evaluación

| Categoría | Prompts exitosos | Con correcciones | Fallidos |
|-----------|:---:|:---:|:---:|
| Cinemática Directa | 2 | 1 | 0 |
| Cinemática Inversa | 3 | 0 | 0 |
| Interfaz | 1 | 2 | 0 |
| Animación | 2 | 0 | 0 |
| **Total** | **8** | **3** | **0** |

---

## Conclusión

GitHub Copilot demostró ser más efectivo en:
- **Código matemático estándar** (matrices, trigonometría): sugerencias casi siempre correctas
- **Estructura de clases**: generó andamiaje útil, aunque requirió ajustes de diseño
- **Funciones cortas y bien comentadas**: mejor resultado que en bloques complejos

Tuvo limitaciones en:
- **Layout de Matplotlib**: coordenadas de widgets requirieron ajuste manual iterativo
- **Lógica de dominio específica**: el bug de escala, el `np.clip` en la IK y el `normalize_angle` no fueron sugeridos espontáneamente
- **Manejo de casos borde**: no anticipó los casos donde `inverse_kinematics` retorna `None`

La herramienta aumentó la velocidad de desarrollo estimada en un **40-50%** en las partes de código matemático, pero el diseño final requirió revisión y corrección humana en aproximadamente el 30% de las sugerencias generadas.
