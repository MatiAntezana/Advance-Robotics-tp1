"""Evita simulaciones duplicadas del TP en la misma partición Gazebo."""
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile
import signal
import threading

_simulation_locks = []


def spin_until_stopped(node):
    """No inyectar KeyboardInterrupt dentro de take_message/pybind11 de RoboStack."""
    import rclpy
    stopped = threading.Event()
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: stopped.set())
    try:
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def acquire_simulation_lock(path=None):
    if path is None:
        partition = os.environ.get('GZ_PARTITION', 'default')
        key = hashlib.sha256(partition.encode()).hexdigest()[:16]
        path = Path(tempfile.gettempdir()) / f'tp1-gazebo-{os.getuid()}-{key}.lock'
    handle = open(path, 'a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError(
            'Ya hay una simulación TP1 activa en esta partición Gazebo. '
            'Usar la ventana existente o detenerla con Ctrl+C antes de iniciar otra.'
        ) from None
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid()))
    handle.flush()
    # Este módulo pertenece a sys.modules: mantiene el bloqueo hasta salir de launch.
    _simulation_locks.append(handle)
    return handle
