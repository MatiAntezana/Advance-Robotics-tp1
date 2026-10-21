"""Evita simulaciones duplicadas del TP en la misma partición Gazebo."""
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile

_simulation_locks = []


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
