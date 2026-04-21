"""
interfaz.py — Interfaz Interactiva Completa
Robot Planar 3R para Rehabilitación de Dedo
Universidad de Costa Rica — Robótica Avanzada

Funcionalidades:
    1. Sliders de angulos articulares (theta1, theta2, theta3)
    2. Posicion (x, y) del efector final en tiempo real
    3. Cinematica inversa: ingresa un punto y calcula la configuracion
    4. Sliders de longitud de falanges
    5. Ejercicios preset de rehabilitacion
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons, TextBox


# ===========================================================================
# Funciones de cinemática (self-contained para no depender de otros archivos)
# ===========================================================================

def forward_kinematics(theta1, theta2, theta3, l1, l2, l3):
    def R(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    def T(l):
        return np.array([[1, 0, l], [0, 1, 0], [0, 0, 1]])
    Tt = R(theta1) @ T(l1) @ R(theta2) @ T(l2) @ R(theta3) @ T(l3)
    return Tt[0, 2], Tt[1, 2], theta1 + theta2 + theta3


def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def inverse_kinematics(x, y, phi, l1, l2, l3):
    wx = x - l3 * np.cos(phi)
    wy = y - l3 * np.sin(phi)
    r  = np.hypot(wx, wy)
    if r > l1 + l2 or r < abs(l1 - l2):
        return None
    cos_t2 = np.clip((r**2 - l1**2 - l2**2) / (2 * l1 * l2), -1.0, 1.0)
    alpha   = np.arctan2(wy, wx)
    sols    = []
    for t2 in [np.arccos(cos_t2), -np.arccos(cos_t2)]:
        beta = np.arctan2(l2 * np.sin(t2), l1 + l2 * np.cos(t2))
        t1   = normalize_angle(alpha - beta)
        t3   = normalize_angle(phi - t1 - t2)
        sols.append((t1, normalize_angle(t2), t3))
    return sols


def get_joint_positions(t1, t2, t3, l1, l2, l3):
    p0 = np.array([0.0, 0.0])
    p1 = p0 + l1 * np.array([np.cos(t1),       np.sin(t1)])
    p2 = p1 + l2 * np.array([np.cos(t1 + t2),  np.sin(t1 + t2)])
    p3 = p2 + l3 * np.array([np.cos(t1+t2+t3), np.sin(t1+t2+t3)])
    return p0, p1, p2, p3


# ===========================================================================
# Ejercicios preset
# ===========================================================================

EJERCICIOS = {
    'Neutra         (0, 0, 0)':    (0,        0,        0),
    'Flex. leve    (30,30,15)':    (30,       30,       15),
    'Flex. moderada(45,45,45)':    (45,       45,       45),
    'Flex. profunda(30,60,15)':    (30,       60,       15),
}


# ===========================================================================
# Clase principal de la interfaz
# ===========================================================================

class RobotInterface:

    # Valores por defecto
    L1_DEFAULT = 250  # mm
    L2_DEFAULT = 100  # mm
    L3_DEFAULT =  25  # mm

    def __init__(self):
        self.l1 = self.L1_DEFAULT / 1000.0
        self.l2 = self.L2_DEFAULT / 1000.0
        self.l3 = self.L3_DEFAULT / 1000.0
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.ik_solutions  = None
        self.ik_target_pos = None
        self._build_ui()
        self._draw_robot()

    # -----------------------------------------------------------------------
    # Construcción de la UI
    # -----------------------------------------------------------------------

    def _build_ui(self):
        self.fig = plt.figure(figsize=(18, 11))
        self.fig.patch.set_facecolor('#f0f2f5')
        self.fig.canvas.manager.set_window_title(
            'Robot Planar 3R — Rehabilitacion de Dedo')

        # ---- Plot principal ------------------------------------------------
        self.ax = self.fig.add_axes([0.33, 0.10, 0.64, 0.85])
        self.ax.set_facecolor('#ffffff')
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.30, color='#cccccc')
        self.ax.axhline(0, color='#bbbbbb', lw=0.8, ls='--')
        self.ax.axvline(0, color='#bbbbbb', lw=0.8, ls='--')
        self.ax.set_xlabel('X (m)', fontsize=11)
        self.ax.set_ylabel('Y (m)', fontsize=11)
        self.ax.set_title(
            'Robot Planar 3R  —  Rehabilitacion de Dedo',
            fontsize=14, fontweight='bold', pad=10)

        # Elementos graficos del robot
        self.ln_l1, = self.ax.plot([], [], color='#1f77b4', lw=6,
                                   solid_capstyle='round',
                                   label='Falange proximal')
        self.ln_l2, = self.ax.plot([], [], color='#2ca02c', lw=6,
                                   solid_capstyle='round',
                                   label='Falange media')
        self.ln_l3, = self.ax.plot([], [], color='#9467bd', lw=6,
                                   solid_capstyle='round',
                                   label='Falange distal')
        self.sc_joints = self.ax.scatter([], [], c='#ff7f0e', s=180,
                                         edgecolors='#333', lw=1.5,
                                         zorder=5, label='Articulaciones')
        self.sc_ee, = self.ax.plot([], [], 'rs', ms=13, mew=1.5,
                                   zorder=6, label='Efector final')
        self.sc_target, = self.ax.plot([], [], 'k+', ms=18, mew=2.5,
                                       zorder=7, label='Target IK')

        self.ax.legend(loc='upper right', fontsize=9)

        # Cuadro de posición prominente
        self.pos_box = self.ax.text(
            0.02, 0.975, '',
            transform=self.ax.transAxes,
            va='top', ha='left', fontsize=13, family='monospace',
            bbox=dict(boxstyle='round,pad=0.6', facecolor='#e8f4fd',
                      edgecolor='#1f77b4', lw=2.0))

        # ---- Panel izquierdo -----------------------------------------------
        lx = 0.02   # left margin
        sw = 0.27   # slider width

        # -- Seccion: Angulos articulares --
        self.fig.text(lx + 0.005, 0.965,
                      'ANGULOS ARTICULARES',
                      fontsize=10, fontweight='bold', color='#1f77b4')

        ax_t1 = self.fig.add_axes([lx, 0.900, sw, 0.026])
        ax_t2 = self.fig.add_axes([lx, 0.845, sw, 0.026])
        ax_t3 = self.fig.add_axes([lx, 0.790, sw, 0.026])

        self.sl_t1 = Slider(ax_t1, 'th1 (deg)', -180, 180,
                            valinit=0, valstep=1, color='#1f77b4')
        self.sl_t2 = Slider(ax_t2, 'th2 (deg)', -180, 180,
                            valinit=0, valstep=1, color='#2ca02c')
        self.sl_t3 = Slider(ax_t3, 'th3 (deg)', -180, 180,
                            valinit=0, valstep=1, color='#9467bd')

        for sl in (self.sl_t1, self.sl_t2, self.sl_t3):
            sl.on_changed(self._on_angles_changed)

        # -- Seccion: Longitudes --
        self.fig.text(lx + 0.005, 0.760,
                      'LONGITUDES DE FALANGES',
                      fontsize=10, fontweight='bold', color='#555')

        ax_l1 = self.fig.add_axes([lx, 0.700, sw, 0.026])
        ax_l2 = self.fig.add_axes([lx, 0.645, sw, 0.026])
        ax_l3 = self.fig.add_axes([lx, 0.590, sw, 0.026])

        self.sl_l1 = Slider(ax_l1, 'Proximal (mm)',  10, 400,
                            valinit=self.L1_DEFAULT, valstep=5,
                            color='#1f77b4')
        self.sl_l2 = Slider(ax_l2, 'Media    (mm)',  10, 200,
                            valinit=self.L2_DEFAULT, valstep=5,
                            color='#2ca02c')
        self.sl_l3 = Slider(ax_l3, 'Distal   (mm)',   5, 100,
                            valinit=self.L3_DEFAULT, valstep=1,
                            color='#9467bd')

        for sl in (self.sl_l1, self.sl_l2, self.sl_l3):
            sl.on_changed(self._on_lengths_changed)

        # -- Seccion: Ejercicios preset --
        self.fig.text(lx + 0.005, 0.560,
                      'EJERCICIOS PRESET',
                      fontsize=10, fontweight='bold', color='#555')

        ax_radio = self.fig.add_axes([lx, 0.420, sw, 0.130])
        ax_radio.set_facecolor('#f0f2f5')
        self.radio_ex = RadioButtons(
            ax_radio, list(EJERCICIOS.keys()), activecolor='#1f77b4')
        for lbl in self.radio_ex.labels:
            lbl.set_fontsize(8.5)
        self.radio_ex.on_clicked(self._on_exercise)

        # -- Seccion: Cinematica Inversa --
        self.fig.text(lx + 0.005, 0.400,
                      'CINEMATICA INVERSA',
                      fontsize=10, fontweight='bold', color='#d62728')
        self.fig.text(lx, 0.375,
                      'Objetivo (mm) y orientacion (deg):',
                      fontsize=8.5, color='#444')

        # Labels de los campos
        self.fig.text(lx + 0.020, 0.350, 'X (mm)', fontsize=8)
        self.fig.text(lx + 0.110, 0.350, 'Y (mm)', fontsize=8)
        self.fig.text(lx + 0.210, 0.350, 'phi(deg)', fontsize=8)

        ax_xtb = self.fig.add_axes([lx,       0.315, 0.075, 0.035])
        ax_ytb = self.fig.add_axes([lx+0.100, 0.315, 0.075, 0.035])
        ax_ptb = self.fig.add_axes([lx+0.200, 0.315, 0.075, 0.035])

        self.tb_x   = TextBox(ax_xtb, '', initial='200')
        self.tb_y   = TextBox(ax_ytb, '', initial='150')
        self.tb_phi = TextBox(ax_ptb, '', initial='90')

        ax_btn = self.fig.add_axes([lx, 0.258, sw, 0.047])
        self.btn_ik = Button(ax_btn, 'CALCULAR IK',
                             color='#d62728', hovercolor='#ff4444')
        self.btn_ik.label.set_color('white')
        self.btn_ik.label.set_fontweight('bold')
        self.btn_ik.label.set_fontsize(10)
        self.btn_ik.on_clicked(self._on_compute_ik)

        # Radio para seleccionar solucion
        self.fig.text(lx + 0.005, 0.246,
                      'Seleccionar solucion:', fontsize=8.5, color='#444')

        ax_sol = self.fig.add_axes([lx, 0.155, sw, 0.085])
        ax_sol.set_facecolor('#f0f2f5')
        self.radio_sol = RadioButtons(
            ax_sol,
            ('Sol 1: codo + (theta2 > 0)',
             'Sol 2: codo - (theta2 < 0)'),
            activecolor='#d62728')
        for lbl in self.radio_sol.labels:
            lbl.set_fontsize(8.5)
        self.radio_sol.on_clicked(self._on_sol_select)

        # Texto resultado IK
        self.ik_msg = self.fig.text(
            lx, 0.125, '',
            fontsize=8.5, color='#333', family='monospace',
            va='top')

        # Boton reset
        ax_rst = self.fig.add_axes([lx, 0.060, 0.125, 0.045])
        self.btn_reset = Button(ax_rst, 'RESET', color='#555', hovercolor='#777')
        self.btn_reset.label.set_color('white')
        self.btn_reset.label.set_fontweight('bold')
        self.btn_reset.on_clicked(self._on_reset)

        # Instruccion de uso
        self.fig.text(lx, 0.020,
                      'Sliders: mueven robot en tiempo real\n'
                      'Boton IK: calcula configuracion desde punto',
                      fontsize=7.5, color='#666', style='italic')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _on_angles_changed(self, _val):
        self.theta1 = np.radians(self.sl_t1.val)
        self.theta2 = np.radians(self.sl_t2.val)
        self.theta3 = np.radians(self.sl_t3.val)
        self._draw_robot()

    def _on_lengths_changed(self, _val):
        self.l1 = self.sl_l1.val / 1000.0
        self.l2 = self.sl_l2.val / 1000.0
        self.l3 = self.sl_l3.val / 1000.0
        self._draw_robot()

    def _on_exercise(self, label):
        t1, t2, t3 = EJERCICIOS[label]
        # Desconectar temporalmente para evitar triple redraw
        self.sl_t1.set_val(t1)
        self.sl_t2.set_val(t2)
        self.sl_t3.set_val(t3)
        # set_val dispara on_changed, que llama _draw_robot

    def _on_compute_ik(self, _event):
        try:
            x   = float(self.tb_x.text)   / 1000.0
            y   = float(self.tb_y.text)    / 1000.0
            phi = np.radians(float(self.tb_phi.text))
        except ValueError:
            self.ik_msg.set_text('! Entrada invalida — revisa los campos')
            self.fig.canvas.draw_idle()
            return

        self.ik_solutions  = inverse_kinematics(x, y, phi, self.l1, self.l2, self.l3)
        self.ik_target_pos = (x, y)

        # Mostrar marcador del objetivo
        self.sc_target.set_data([x], [y])

        if self.ik_solutions is None:
            dist = np.hypot(x, y) * 1000
            rmax = (self.l1 + self.l2) * 1000
            self.ik_msg.set_text(
                f'FUERA DEL WORKSPACE\n'
                f'  dist al origen : {dist:.1f} mm\n'
                f'  alcance maximo : {rmax:.1f} mm')
        else:
            t1a, t2a, t3a = self.ik_solutions[0]
            t1b, t2b, t3b = self.ik_solutions[1]
            self.ik_msg.set_text(
                f'Sol1: {np.degrees(t1a):.0f}  {np.degrees(t2a):.0f}  {np.degrees(t3a):.0f} deg\n'
                f'Sol2: {np.degrees(t1b):.0f}  {np.degrees(t2b):.0f}  {np.degrees(t3b):.0f} deg')
            # Aplicar solucion 1 por defecto
            self.sl_t1.set_val(round(np.degrees(t1a)))
            self.sl_t2.set_val(round(np.degrees(t2a)))
            self.sl_t3.set_val(round(np.degrees(t3a)))

        self.fig.canvas.draw_idle()

    def _on_sol_select(self, label):
        if self.ik_solutions is None:
            return
        idx = 0 if '1' in label else 1
        t1, t2, t3 = self.ik_solutions[idx]
        self.sl_t1.set_val(round(np.degrees(t1)))
        self.sl_t2.set_val(round(np.degrees(t2)))
        self.sl_t3.set_val(round(np.degrees(t3)))

    def _on_reset(self, _event):
        self.sl_t1.set_val(0)
        self.sl_t2.set_val(0)
        self.sl_t3.set_val(0)
        self.ik_solutions  = None
        self.ik_target_pos = None
        self.sc_target.set_data([], [])
        self.ik_msg.set_text('')
        self.fig.canvas.draw_idle()

    # -----------------------------------------------------------------------
    # Render
    # -----------------------------------------------------------------------

    def _draw_robot(self):
        p0, p1, p2, p3 = get_joint_positions(
            self.theta1, self.theta2, self.theta3,
            self.l1, self.l2, self.l3)

        # Actualizar geometria
        self.ln_l1.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        self.ln_l2.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        self.ln_l3.set_data([p2[0], p3[0]], [p2[1], p3[1]])
        self.sc_joints.set_offsets(
            np.c_[[p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]]])
        self.sc_ee.set_data([p3[0]], [p3[1]])

        # Ejes dinamicos — cubre todos los cuadrantes
        total  = self.l1 + self.l2 + self.l3
        margin = total * 0.40
        self.ax.set_xlim(-(total + margin), total + margin)
        self.ax.set_ylim(-(total + margin), total + margin)

        # Posicion FK
        x, y, phi = forward_kinematics(
            self.theta1, self.theta2, self.theta3,
            self.l1, self.l2, self.l3)

        # Cuadro de posicion prominente
        self.pos_box.set_text(
            f'   Efector Final\n'
            f'   X  =  {x*1000:+8.2f} mm\n'
            f'   Y  =  {y*1000:+8.2f} mm\n'
            f'   phi=  {np.degrees(phi):+7.1f} deg')

        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == '__main__':
    print('\n' + '=' * 60)
    print('  INTERFAZ INTERACTIVA — Robot Planar 3R')
    print('  Rehabilitacion de Dedo  |  UCR Robotica Avanzada')
    print('=' * 60)
    print()
    print('Controles disponibles:')
    print('  [Sliders angulos]    Mueven theta1/theta2/theta3 en tiempo real')
    print('  [Sliders longitud]   Cambian el tamano de cada falange')
    print('  [Ejercicios preset]  Cargan configuraciones de rehabilitacion')
    print('  [Campos X/Y/phi]     Posicion objetivo para cinematica inversa')
    print('  [Boton CALCULAR IK]  Calcula configuracion articular')
    print('  [Radio soluciones]   Alterna entre codo+ y codo-')
    print('  [Boton RESET]        Regresa a posicion neutra')
    print()

    ui = RobotInterface()
    ui.show()
