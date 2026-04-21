import numpy as np
import matplotlib.pyplot as plt


def normalize_angle(angle):
    """Normaliza ángulo al rango [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def inverse_kinematics(x, y, phi, l_proximal, l_media, l_distal):
    """
    Cinemática inversa analítica para robot planar 3R.

    Estrategia:
        1. Calcula posición de la muñeca restando la falange distal.
        2. Resuelve theta2 con la ley de cosenos (dos soluciones: codo+ y codo-).
        3. Calcula theta1 con arctan2.
        4. Calcula theta3 = phi - theta1 - theta2.

    Parámetros:
        x, y                        : posición objetivo del efector (m)
        phi                         : orientación objetivo del efector (rad)
        l_proximal, l_media, l_distal : longitudes de falanges (m)

    Retorna:
        Lista de tuplas (theta1, theta2, theta3) — dos soluciones.
        None si el objetivo está fuera del workspace.
    """
    # Posición de la muñeca (descontando falange distal)
    wrist_x = x - l_distal * np.cos(phi)
    wrist_y = y - l_distal * np.sin(phi)

    r = np.hypot(wrist_x, wrist_y)

    # Validación del workspace
    r_max = l_proximal + l_media
    r_min = abs(l_proximal - l_media)
    if r > r_max or r < r_min:
        return None

    # Ley de cosenos para theta2
    cos_theta2 = (r**2 - l_proximal**2 - l_media**2) / (2 * l_proximal * l_media)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    solutions = []
    alpha = np.arctan2(wrist_y, wrist_x)

    for theta2 in [np.arccos(cos_theta2), -np.arccos(cos_theta2)]:
        beta   = np.arctan2(l_media * np.sin(theta2),
                            l_proximal + l_media * np.cos(theta2))
        theta1 = normalize_angle(alpha - beta)
        theta3 = normalize_angle(phi - theta1 - theta2)
        solutions.append((theta1, normalize_angle(theta2), theta3))

    return solutions


def get_joint_positions(theta1, theta2, theta3, l1, l2, l3):
    """Calcula las posiciones cartesianas de cada articulación."""
    p0 = np.array([0.0, 0.0])
    p1 = p0 + l1 * np.array([np.cos(theta1),           np.sin(theta1)])
    p2 = p1 + l2 * np.array([np.cos(theta1 + theta2),  np.sin(theta1 + theta2)])
    p3 = p2 + l3 * np.array([np.cos(theta1 + theta2 + theta3),
                              np.sin(theta1 + theta2 + theta3)])
    return p0, p1, p2, p3


def plot_robot_ik(theta1, theta2, theta3, l_proximal, l_media, l_distal,
                  target_x=None, target_y=None, sol_label=''):
    """
    Visualiza el robot en la pose calculada por IK.
    Muestra el punto objetivo si se provee.
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
    ax.scatter([p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]],
               c='#ff7f0e', s=120, edgecolors='#333', zorder=5,
               label='Articulaciones')

    # Efector final
    ax.scatter(p3[0], p3[1], c='red', marker='s', s=140,
               edgecolors='#333', zorder=6, label='Efector final')

    # Punto objetivo
    if target_x is not None and target_y is not None:
        ax.plot(target_x, target_y, 'k+', ms=18, mew=2.5,
                zorder=7, label='Objetivo')
        ax.plot([target_x], [target_y], 'ko', ms=6, zorder=7, alpha=0.5)

    # Ejes dinámicos — cubre todos los cuadrantes
    total  = l_proximal + l_media + l_distal
    margin = total * 0.4
    ax.set_xlim(-(total + margin), total + margin)
    ax.set_ylim(-(total + margin), total + margin)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.35)
    ax.axhline(0, color='#aaa', lw=0.8, ls='--')
    ax.axvline(0, color='#aaa', lw=0.8, ls='--')
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_title(f'CINEMATICA INVERSA — {sol_label}', fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)

    # Info box
    info = (f'th1={np.degrees(theta1):.1f} deg  '
            f'th2={np.degrees(theta2):.1f} deg  '
            f'th3={np.degrees(theta3):.1f} deg\n'
            f'Efector: X={p3[0]*1000:.2f} mm   Y={p3[1]*1000:.2f} mm')
    if target_x is not None:
        err_x = abs(p3[0] - target_x) * 1000
        err_y = abs(p3[1] - target_y) * 1000
        info += f'\nError: dx={err_x:.3f} mm   dy={err_y:.3f} mm'
    ax.text(0.02, 0.97, info, transform=ax.transAxes,
            va='top', fontsize=9, family='monospace',
            bbox=dict(boxstyle='round', facecolor='#fff3cd',
                      edgecolor='#d62728', alpha=0.9))

    return fig, ax


def forward_kinematics(theta1, theta2, theta3, l1, l2, l3):
    """FK auxiliar para verificación."""
    def R(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    def T(l):
        return np.array([[1, 0, l], [0, 1, 0], [0, 0, 1]])
    Tt = R(theta1) @ T(l1) @ R(theta2) @ T(l2) @ R(theta3) @ T(l3)
    return Tt[0, 2], Tt[1, 2], theta1 + theta2 + theta3


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    print('\n' + '=' * 55)
    print('  CINEMATICA INVERSA — Robot Planar 3R')
    print('  Rehabilitacion de Dedo')
    print('=' * 55)

    try:
        l_proximal_mm = float(input('\nLongitud falange proximal (mm): '))
        l_media_mm    = float(input('Longitud falange media    (mm): '))
        l_distal_mm   = float(input('Longitud falange distal   (mm): '))

        l_proximal = l_proximal_mm / 1000.0
        l_media    = l_media_mm    / 1000.0
        l_distal   = l_distal_mm   / 1000.0

        r_max = l_proximal + l_media + l_distal
        print(f'\nWorkspace: alcance maximo = {r_max*1000:.1f} mm')

        print('\nIngresa la posicion objetivo del efector:')
        x_mm    = float(input('  X objetivo (mm): '))
        y_mm    = float(input('  Y objetivo (mm): '))
        phi_deg = float(input('  Orientacion phi (deg): '))

        x   = x_mm   / 1000.0
        y   = y_mm   / 1000.0
        phi = np.radians(phi_deg)

        solutions = inverse_kinematics(x, y, phi, l_proximal, l_media, l_distal)

        if solutions is None:
            dist = np.hypot(x, y)
            print(f'\n[FUERA DEL WORKSPACE]')
            print(f'  Distancia al origen : {dist*1000:.1f} mm')
            print(f'  Alcance maximo (2R) : {(l_proximal+l_media)*1000:.1f} mm')
        else:
            print('\n--- SOLUCIONES ENCONTRADAS ---')
            labels = ['Codo arriba (theta2 > 0)', 'Codo abajo  (theta2 < 0)']
            for idx, (t1, t2, t3) in enumerate(solutions):
                print(f'\n  [{labels[idx]}]')
                print(f'  th1={np.degrees(t1):+7.2f} deg   '
                      f'th2={np.degrees(t2):+7.2f} deg   '
                      f'th3={np.degrees(t3):+7.2f} deg')
                # Verificación FK
                xv, yv, _ = forward_kinematics(t1, t2, t3, l_proximal, l_media, l_distal)
                print(f'  Verificacion FK: X={xv*1000:.2f} mm  Y={yv*1000:.2f} mm  '
                      f'(error: {np.hypot(xv-x, yv-y)*1000:.4f} mm)')

            print('\nSelecciona solución a visualizar:')
            for i, lbl in enumerate(labels, 1):
                print(f'  {i}. {lbl}')
            sel = int(input('Opcion (1 o 2): ')) - 1
            sel = max(0, min(1, sel))

            t1, t2, t3 = solutions[sel]
            fig, ax = plot_robot_ik(t1, t2, t3,
                                    l_proximal, l_media, l_distal,
                                    target_x=x, target_y=y,
                                    sol_label=labels[sel])
            plt.tight_layout()
            plt.show()

    except ValueError as e:
        print(f'Error de entrada: {e}')
    except KeyboardInterrupt:
        print('\nInterrumpido.')
