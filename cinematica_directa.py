import numpy as np
import matplotlib.pyplot as plt


def forward_kinematics(theta1, theta2, theta3, l_proximal, l_media, l_distal):
    """
    Cinemática directa para robot planar 3R usando transformaciones homogéneas.

    Parámetros:
        theta1, theta2, theta3  : ángulos articulares en radianes
        l_proximal, l_media, l_distal : longitudes de falanges en metros

    Retorna:
        x, y        : posición del efector final (m)
        orientation : orientación total del efector (rad)
    """
    def rotation_matrix(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0],
                         [s,  c, 0],
                         [0,  0, 1]])

    def translation_matrix(l):
        return np.array([[1, 0, l],
                         [0, 1, 0],
                         [0, 0, 1]])

    T1 = rotation_matrix(theta1) @ translation_matrix(l_proximal)
    T2 = rotation_matrix(theta2) @ translation_matrix(l_media)
    T3 = rotation_matrix(theta3) @ translation_matrix(l_distal)
    T_total = T1 @ T2 @ T3

    x = T_total[0, 2]
    y = T_total[1, 2]
    orientation = theta1 + theta2 + theta3
    return x, y, orientation


def get_joint_positions(theta1, theta2, theta3, l1, l2, l3):
    """Calcula las posiciones cartesianas de cada articulación."""
    p0 = np.array([0.0, 0.0])
    p1 = p0 + l1 * np.array([np.cos(theta1),           np.sin(theta1)])
    p2 = p1 + l2 * np.array([np.cos(theta1 + theta2),  np.sin(theta1 + theta2)])
    p3 = p2 + l3 * np.array([np.cos(theta1 + theta2 + theta3),
                              np.sin(theta1 + theta2 + theta3)])
    return p0, p1, p2, p3


def plot_robot(theta1, theta2, theta3, l_proximal, l_media, l_distal, title='CINEMATICA DIRECTA'):
    """
    Visualiza el robot en la pose dada.

    Retorna fig, ax para integración opcional con otras interfaces.
    """
    p0, p1, p2, p3 = get_joint_positions(theta1, theta2, theta3,
                                          l_proximal, l_media, l_distal)

    fig, ax = plt.subplots(figsize=(8, 7))
    fig.patch.set_facecolor('#f8f9fa')
    ax.set_facecolor('#ffffff')

    # Falanges
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color='#1f77b4', lw=4,
            solid_capstyle='round', label='Falange proximal')
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#2ca02c', lw=4,
            solid_capstyle='round', label='Falange media')
    ax.plot([p2[0], p3[0]], [p2[1], p3[1]], color='#9467bd', lw=4,
            solid_capstyle='round', label='Falange distal')

    # Articulaciones
    for p, label in zip([p0, p1, p2], ['Base', 'Nudo 1', 'Nudo 2']):
        ax.scatter(p[0], p[1], c='#ff7f0e', s=120, edgecolors='#333',
                   zorder=5, linewidths=1.5)

    # Efector final
    ax.scatter(p3[0], p3[1], c='red', marker='s', s=140,
               edgecolors='#333', zorder=6, label='Efector final')

    # Eje de orientación del efector
    phi = theta1 + theta2 + theta3
    arr_len = (l_proximal + l_media + l_distal) * 0.12
    ax.annotate('', xy=(p3[0] + arr_len * np.cos(phi), p3[1] + arr_len * np.sin(phi)),
                xytext=(p3[0], p3[1]),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))

    # Ejes dinámicos centrados — cubre todos los cuadrantes
    total = l_proximal + l_media + l_distal
    margin = total * 0.4
    ax.set_xlim(-(total + margin), total + margin)
    ax.set_ylim(-(total + margin), total + margin)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.35)
    ax.axhline(0, color='#aaa', lw=0.8, ls='--')
    ax.axvline(0, color='#aaa', lw=0.8, ls='--')
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)

    # Info box
    x, y, ori = forward_kinematics(theta1, theta2, theta3,
                                   l_proximal, l_media, l_distal)
    info = (f'Falanges : P={l_proximal*1000:.0f} mm  '
            f'M={l_media*1000:.0f} mm  D={l_distal*1000:.0f} mm\n'
            f'Angulos  : th1={np.degrees(theta1):.1f} deg  '
            f'th2={np.degrees(theta2):.1f} deg  th3={np.degrees(theta3):.1f} deg\n'
            f'Efector  : X={x*1000:.2f} mm   Y={y*1000:.2f} mm\n'
            f'Orient.  : phi={np.degrees(ori):.1f} deg')
    ax.text(0.02, 0.97, info, transform=ax.transAxes,
            va='top', fontsize=9, family='monospace',
            bbox=dict(boxstyle='round', facecolor='#e8f4fd',
                      edgecolor='#1f77b4', alpha=0.9))

    return fig, ax


def get_exercise_angles(exercise_number):
    """
    Ángulos articulares para cada ejercicio de rehabilitación.

    Ejercicios:
        1 - Posicion neutra       : extension completa (0, 0, 0)
        2 - Flexion leve          : inicio de movimiento (30, 30, 15) deg
        3 - Flexion moderada      : rango medio (45, 45, 45) deg
        4 - Flexion profunda      : maximo alcanzable (30, 60, 15) deg
    """
    exercises = {
        1: (0,          0,          0),
        2: (np.pi/6,    np.pi/6,    np.pi/12),
        3: (np.pi/4,    np.pi/4,    np.pi/4),
        4: (np.pi/6,    np.pi/3,    np.pi/12),
    }
    if exercise_number not in exercises:
        raise ValueError(f'Ejercicio invalido: {exercise_number}. Debe ser 1-4.')
    return exercises[exercise_number]


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    print('\n' + '=' * 55)
    print('  CINEMATICA DIRECTA — Robot Planar 3R')
    print('  Rehabilitacion de Dedo')
    print('=' * 55)

    try:
        l_proximal_mm = float(input('\nLongitud falange proximal (mm): '))
        l_media_mm    = float(input('Longitud falange media    (mm): '))
        l_distal_mm   = float(input('Longitud falange distal   (mm): '))

        l_proximal = l_proximal_mm / 1000.0
        l_media    = l_media_mm    / 1000.0
        l_distal   = l_distal_mm   / 1000.0

        print('\nEjercicios disponibles:')
        print('  1. Posicion neutra       (0 deg, 0 deg, 0 deg)')
        print('  2. Flexion leve          (30 deg, 30 deg, 15 deg)')
        print('  3. Flexion moderada      (45 deg, 45 deg, 45 deg)')
        print('  4. Flexion profunda      (30 deg, 60 deg, 15 deg)')

        exercise = int(input('\nSelecciona ejercicio (1-4): '))
        theta1, theta2, theta3 = get_exercise_angles(exercise)

        x, y, ori = forward_kinematics(theta1, theta2, theta3,
                                       l_proximal, l_media, l_distal)

        print('\n--- RESULTADO ---')
        print(f'Angulos : th1={np.degrees(theta1):.1f} deg  '
              f'th2={np.degrees(theta2):.1f} deg  th3={np.degrees(theta3):.1f} deg')
        print(f'Efector : X={x*1000:.2f} mm   Y={y*1000:.2f} mm')
        print(f'Orient. : phi={np.degrees(ori):.1f} deg')

        titles = {
            1: 'CINEMATICA DIRECTA — Ejercicio 1: Posicion Neutra',
            2: 'CINEMATICA DIRECTA — Ejercicio 2: Flexion Leve',
            3: 'CINEMATICA DIRECTA — Ejercicio 3: Flexion Moderada',
            4: 'CINEMATICA DIRECTA — Ejercicio 4: Flexion Profunda',
        }
        fig, ax = plot_robot(theta1, theta2, theta3,
                             l_proximal, l_media, l_distal,
                             title=titles[exercise])
        plt.tight_layout()
        plt.show()

    except ValueError as e:
        print(f'Error: {e}')
    except KeyboardInterrupt:
        print('\nInterrumpido.')
