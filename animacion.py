"""
animacion.py — Animación Suave entre Ejercicios
Robot Planar 3R para Rehabilitación de Dedo
Universidad de Costa Rica — Robótica Avanzada

Bonus: animación de trayectoria entre múltiples poses.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons
from matplotlib.animation import FuncAnimation


# ===========================================================================
# Cinemática
# ===========================================================================

def forward_kinematics(theta1, theta2, theta3, l1, l2, l3):
    def R(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    def T(l):
        return np.array([[1, 0, l], [0, 1, 0], [0, 0, 1]])
    Tt = R(theta1) @ T(l1) @ R(theta2) @ T(l2) @ R(theta3) @ T(l3)
    return Tt[0, 2], Tt[1, 2], theta1 + theta2 + theta3


def get_joint_positions(t1, t2, t3, l1, l2, l3):
    p0 = np.array([0.0, 0.0])
    p1 = p0 + l1 * np.array([np.cos(t1),       np.sin(t1)])
    p2 = p1 + l2 * np.array([np.cos(t1+t2),    np.sin(t1+t2)])
    p3 = p2 + l3 * np.array([np.cos(t1+t2+t3), np.sin(t1+t2+t3)])
    return p0, p1, p2, p3


# ===========================================================================
# Ejercicios (en grados, convertidos internamente)
# ===========================================================================

EXERCISES = {
    1: {'name': 'Neutra         (0, 0, 0)',      'angles': (0,  0,  0)},
    2: {'name': 'Flex. leve    (30,30,15)',       'angles': (30, 30, 15)},
    3: {'name': 'Flex. moderada(45,45,45)',       'angles': (45, 45, 45)},
    4: {'name': 'Flex. profunda(30,60,15)',       'angles': (30, 60, 15)},
}

EXERCISE_NAMES = [EXERCISES[k]['name'] for k in sorted(EXERCISES)]


def get_exercise_angles_rad(exercise_number):
    angles_deg = EXERCISES[exercise_number]['angles']
    return tuple(np.radians(a) for a in angles_deg)


# ===========================================================================
# Animador
# ===========================================================================

class RobotAnimator:

    ANIM_STEPS  = 40    # frames para interpolar entre poses
    ANIM_FPS_MS = 30    # ms entre frames (~33 fps)

    def __init__(self):
        # Geometría
        self.l1 = 0.250
        self.l2 = 0.100
        self.l3 = 0.025

        # Estado actual
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.exercise = 1

        # Animación
        self._anim_progress = 1.0
        self._src  = (0.0, 0.0, 0.0)
        self._dst  = (0.0, 0.0, 0.0)
        self._is_animating = False

        self._build_ui()
        self._draw()

    # -----------------------------------------------------------------------
    def _build_ui(self):
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.fig.patch.set_facecolor('#f0f2f5')
        self.fig.canvas.manager.set_window_title(
            'Animacion Interactiva — Robot Planar 3R')
        plt.subplots_adjust(left=0.27, bottom=0.42, top=0.94)

        self.ax.set_facecolor('#ffffff')
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.30)
        self.ax.axhline(0, color='#bbb', lw=0.8, ls='--')
        self.ax.axvline(0, color='#bbb', lw=0.8, ls='--')
        self.ax.set_xlabel('X (m)', fontsize=11)
        self.ax.set_ylabel('Y (m)', fontsize=11)
        self.ax.set_title('ANIMACION — Robot Planar 3R  |  Rehabilitacion de Dedo',
                          fontsize=13, fontweight='bold')

        # Lineas del robot
        self.ln_l1, = self.ax.plot([], [], color='#1f77b4', lw=6,
                                   solid_capstyle='round', label='Proximal')
        self.ln_l2, = self.ax.plot([], [], color='#2ca02c', lw=6,
                                   solid_capstyle='round', label='Media')
        self.ln_l3, = self.ax.plot([], [], color='#9467bd', lw=6,
                                   solid_capstyle='round', label='Distal')
        self.sc_j  = self.ax.scatter([], [], c='#ff7f0e', s=180,
                                     edgecolors='#333', lw=1.5,
                                     zorder=5, label='Articulaciones')
        self.sc_ee, = self.ax.plot([], [], 'rs', ms=13, zorder=6,
                                   label='Efector final')

        # Trayectoria del efector
        self.traj_x = []
        self.traj_y = []
        self.ln_traj, = self.ax.plot([], [], color='#d62728', lw=1,
                                     ls='--', alpha=0.5, zorder=4,
                                     label='Trayectoria')

        self.ax.legend(loc='upper right', fontsize=9)

        # Info box
        self.info_text = self.ax.text(
            0.02, 0.975, '', transform=self.ax.transAxes,
            va='top', fontsize=11, family='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#e8f4fd',
                      edgecolor='#1f77b4', lw=1.5))

        # ---- Widgets -------------------------------------------------------
        # Sliders de longitud
        plt.figtext(0.03, 0.97, 'LONGITUDES DE FALANGES',
                    fontsize=9, fontweight='bold', color='#555')
        ax_l1 = plt.axes([0.03, 0.915, 0.20, 0.025])
        ax_l2 = plt.axes([0.03, 0.870, 0.20, 0.025])
        ax_l3 = plt.axes([0.03, 0.825, 0.20, 0.025])

        self.sl_l1 = Slider(ax_l1, 'Proximal(mm)',  10, 400,
                            valinit=250, valstep=5,  color='#1f77b4')
        self.sl_l2 = Slider(ax_l2, 'Media   (mm)',  10, 200,
                            valinit=100, valstep=5,  color='#2ca02c')
        self.sl_l3 = Slider(ax_l3, 'Distal  (mm)',   5, 100,
                            valinit=25,  valstep=1,  color='#9467bd')

        for sl in (self.sl_l1, self.sl_l2, self.sl_l3):
            sl.on_changed(self._on_lengths)

        # Radio de ejercicios
        plt.figtext(0.03, 0.79, 'EJERCICIOS DE REHABILITACION',
                    fontsize=9, fontweight='bold', color='#555')
        ax_radio = plt.axes([0.03, 0.615, 0.20, 0.165])
        ax_radio.set_facecolor('#f0f2f5')
        self.radio = RadioButtons(ax_radio, EXERCISE_NAMES,
                                  activecolor='#1f77b4')
        for lbl in self.radio.labels:
            lbl.set_fontsize(8.5)
        self.radio.on_clicked(self._on_exercise)

        # Info de animacion
        plt.figtext(0.03, 0.60,
                    'Cambia de ejercicio para animar la transicion.',
                    fontsize=7.5, color='#666', style='italic')

    # -----------------------------------------------------------------------
    def _on_lengths(self, _val):
        self.l1 = self.sl_l1.val / 1000.0
        self.l2 = self.sl_l2.val / 1000.0
        self.l3 = self.sl_l3.val / 1000.0
        # Limpiar trayectoria al cambiar dimensiones
        self.traj_x.clear()
        self.traj_y.clear()
        self._draw()

    def _on_exercise(self, label):
        new_ex = next(
            k for k, v in EXERCISES.items() if v['name'] == label)
        if new_ex == self.exercise:
            return
        self.exercise = new_ex

        # Iniciar animacion
        self._src = (self.theta1, self.theta2, self.theta3)
        self._dst = get_exercise_angles_rad(self.exercise)
        self._anim_progress = 0.0
        self._is_animating  = True
        # Limpiar trayectoria previa
        self.traj_x.clear()
        self.traj_y.clear()

    # -----------------------------------------------------------------------
    def _ease_in_out(self, t):
        """Suavizado cúbico (ease in-out)."""
        return t * t * (3 - 2 * t)

    def _step_animation(self):
        if not self._is_animating:
            return
        self._anim_progress += 1.0 / self.ANIM_STEPS
        if self._anim_progress >= 1.0:
            self._anim_progress = 1.0
            self._is_animating  = False

        t  = self._ease_in_out(self._anim_progress)
        t1 = self._src[0] + (self._dst[0] - self._src[0]) * t
        t2 = self._src[1] + (self._dst[1] - self._src[1]) * t
        t3 = self._src[2] + (self._dst[2] - self._src[2]) * t

        if not self._is_animating:
            t1, t2, t3 = self._dst

        self.theta1, self.theta2, self.theta3 = t1, t2, t3

    # -----------------------------------------------------------------------
    def _draw(self):
        p0, p1, p2, p3 = get_joint_positions(
            self.theta1, self.theta2, self.theta3,
            self.l1, self.l2, self.l3)

        self.ln_l1.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        self.ln_l2.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        self.ln_l3.set_data([p2[0], p3[0]], [p2[1], p3[1]])
        self.sc_j.set_offsets(
            np.c_[[p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]]])
        self.sc_ee.set_data([p3[0]], [p3[1]])

        # Actualizar trayectoria
        self.traj_x.append(p3[0])
        self.traj_y.append(p3[1])
        self.ln_traj.set_data(self.traj_x, self.traj_y)

        # Ejes dinamicos — cubre todos los cuadrantes
        total  = self.l1 + self.l2 + self.l3
        margin = total * 0.40
        self.ax.set_xlim(-(total + margin), total + margin)
        self.ax.set_ylim(-(total + margin), total + margin)

        x, y, phi = forward_kinematics(
            self.theta1, self.theta2, self.theta3,
            self.l1, self.l2, self.l3)

        self.info_text.set_text(
            f'  Efector:  X = {x*1000:+7.2f} mm   Y = {y*1000:+7.2f} mm\n'
            f'  Angulos: th1={np.degrees(self.theta1):+6.1f}  '
            f'th2={np.degrees(self.theta2):+6.1f}  '
            f'th3={np.degrees(self.theta3):+6.1f} deg\n'
            f'  Ejercicio: {EXERCISES[self.exercise]["name"]}')

        self.fig.canvas.draw_idle()

    # -----------------------------------------------------------------------
    def _animate_loop(self, _frame):
        if self._is_animating:
            self._step_animation()
            self._draw()
        return (self.ln_l1, self.ln_l2, self.ln_l3,
                self.sc_j, self.sc_ee, self.ln_traj)

    def show(self):
        self._ani = FuncAnimation(
            self.fig, self._animate_loop,
            interval=self.ANIM_FPS_MS,
            blit=False, repeat=True)
        plt.show()


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == '__main__':
    print('\n' + '=' * 60)
    print('  ANIMACION INTERACTIVA — Robot Planar 3R')
    print('  Rehabilitacion de Dedo  |  UCR Robotica Avanzada')
    print('=' * 60)
    print()
    print('  • Cambia de ejercicio para ver la transicion animada.')
    print('  • La linea roja punteada traza la trayectoria del efector.')
    print('  • Ajusta las longitudes con los sliders.')
    print()

    anim = RobotAnimator()
    anim.show()
