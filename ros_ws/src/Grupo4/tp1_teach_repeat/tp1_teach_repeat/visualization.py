"""Marcadores RViz en odom; nunca cambia poses para mejorar su apariencia."""
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

HISTORY = (.45, .60, .72, .85)
ACTIVE = (.15, .95, .35, 1.)
TARGET = (1., .15, .75, 1.)


def marker(namespace, index, kind, stamp, pose, color):
    m = Marker()
    m.header.frame_id, m.header.stamp = 'odom', stamp
    m.ns, m.id, m.type, m.action = namespace, index, kind, Marker.ADD
    m.pose.position.x, m.pose.position.y = float(pose[0]), float(pose[1])
    m.pose.orientation.z, m.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def keyframes(frames, stage, index, done, stamp, label_stride=3):
    result = MarkerArray()
    # Limpia etiquetas de rutas anteriores cuando cambia el publicador.
    clear = Marker()
    clear.header.frame_id, clear.header.stamp = 'odom', stamp
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    active = (len(frames)-1) if stage == 'teach' else index
    following = min(index+1, len(frames)-1) if stage == 'repeat' and not done else None
    for i, frame in enumerate(frames):
        final = done and i == len(frames)-1
        color = ACTIVE if i == active else TARGET if i == following or final else HISTORY
        selected = i in (active, following) or final
        m = marker('keyframes', i, Marker.ARROW if selected else Marker.SPHERE, stamp, frame['pose'], color)
        m.pose.position.z = .04
        m.scale.x, m.scale.y, m.scale.z = (.20, .025, .025) if selected else (.045, .045, .045)
        result.markers.append(m)
        if not selected and i not in (0, len(frames)-1) and i % max(1, label_stride):
            continue
        if not selected and i not in (0, len(frames)-1):
            important = [j for j in (active, following, len(frames)-1 if done else None) if j is not None]
            if any(math.hypot(frame['pose'][0]-frames[j]['pose'][0],
                              frame['pose'][1]-frames[j]['pose'][1]) < .32 for j in important):
                continue
        text = marker('keyframe_labels', i, Marker.TEXT_VIEW_FACING, stamp, frame['pose'], color)
        text.pose.orientation.z, text.pose.orientation.w = 0., 1.
        # Distintas alturas permiten leer anclas de un giro en el lugar.
        text.pose.position.y += .15
        text.pose.position.x += -.25 if i == active and not final else .25 if selected else 0.
        text.pose.position.z = .14
        text.scale.z = .10
        suffix = '\nFINAL' if final else '\nACTIVO' if i == active else '\nSIGUIENTE' if i == following else ''
        text.text = f'KF{i}{suffix}'
        result.markers.append(text)
    return result


def navigation(pose, goal, goal_index, done, stamp, quality=None,
               mode=None, stage='repeat', mode_position=None):
    result = MarkerArray()
    clear = Marker()
    clear.header.frame_id, clear.header.stamp = 'odom', stamp
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    if mode is not None:
        label, color = {
            'encoders': ('ENCODERS', (.35, .70, 1., 1.)),
            'lidar': ('LIDAR', (.80, .55, 1., 1.)),
            'mixed': ('MIXTO', (.25, 1., .65, 1.)),
        }[mode]
        # Posición fija junto al recorrido: no se mezcla con las metas móviles.
        text = marker('localization_mode', 0, Marker.TEXT_VIEW_FACING, stamp,
                      mode_position if mode_position is not None else pose, color)
        text.pose.position.z, text.scale.z = .45, .18
        text.text = label
        result.markers.append(text)
    if goal is not None:
        arrow = marker('setpoint', 0, Marker.ARROW, stamp, goal, TARGET)
        arrow.pose.position.z = .08
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .32, .055, .055
        result.markers.append(arrow)
        circle = marker('setpoint', 1, Marker.CYLINDER, stamp, goal, (1., .15, .75, .35))
        circle.pose.position.z = .025
        circle.scale.x, circle.scale.y, circle.scale.z = .20, .20, .025
        result.markers.append(circle)
        line = marker('setpoint', 2, Marker.LINE_STRIP, stamp, (0., 0., 0.), TARGET)
        line.scale.x = .025
        line.points = [Point(x=float(pose[0]), y=float(pose[1]), z=.08),
                       Point(x=float(goal[0]), y=float(goal[1]), z=.08)]
        result.markers.append(line)
        text = marker('setpoint', 3, Marker.TEXT_VIEW_FACING, stamp, goal, TARGET)
        text.pose.position.y += .38
        text.pose.position.z, text.scale.z = .20, .11
        text.text = f'META:KF{goal_index}' + ('\nOK' if done else '')
        result.markers.append(text)
    if quality is not None:
        good = not quality.get('stale') and quality['rmse_m'] is not None and quality['rmse_m'] <= .06 and quality['overlap'] >= .55
        color = ACTIVE if good else (1., .40, .12, 1.)
        text = marker('alignment', 0, Marker.TEXT_VIEW_FACING, stamp, pose, color)
        text.pose.position.x += .65
        text.pose.position.y -= .15
        text.pose.position.z, text.scale.z = .38, .105
        residual = '--' if quality['rmse_m'] is None else f"{100*quality['rmse_m']:.1f}cm"
        text.text = f"RMSE mapa:{residual}\nOverlap:{100*quality['overlap']:.0f}%"
        if quality.get('stale'):
            text.text = 'RMSE mapa:--\nSIN_SCAN'
        result.markers.append(text)
    return result


def walls(grid, origin, resolution, stamp):
    import numpy as np
    result = MarkerArray()
    clear = Marker()
    clear.header.frame_id, clear.header.stamp = 'odom', stamp
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    cells = marker('reference_walls', 0, Marker.CUBE_LIST, stamp, (0., 0., 0.), (.62, .73, .80, 1.))
    cells.scale.x = cells.scale.y = float(resolution)
    cells.scale.z = .025
    y, x = np.where(grid == 100)
    cells.points = [Point(x=float(origin[0]+(ix+.5)*resolution),
                         y=float(origin[1]+(iy+.5)*resolution), z=-.02) for ix, iy in zip(x, y)]
    result.markers.append(cells)
    return result
