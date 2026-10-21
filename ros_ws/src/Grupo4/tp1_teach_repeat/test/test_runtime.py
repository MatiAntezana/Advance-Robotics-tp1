"""Un segundo proceso no debe crear otro servidor ni otro bridge."""
import fcntl
import subprocess
import sys

from tp1_teach_repeat.runtime import acquire_simulation_lock


def test_simulation_lock_rejects_second_process_and_releases(tmp_path):
    path = tmp_path/'simulation.lock'
    command = [sys.executable, '-c',
               'import sys; from tp1_teach_repeat.runtime import acquire_simulation_lock; '
               'acquire_simulation_lock(sys.argv[1])', str(path)]
    handle = acquire_simulation_lock(path)
    try:
        blocked = subprocess.run(command, capture_output=True, text=True)
        assert blocked.returncode != 0
        assert 'Ya hay una simulación TP1 activa' in blocked.stderr
    finally:
        fcntl.flock(handle, fcntl.LOCK_UN)
        handle.close()
    assert subprocess.run(command, capture_output=True).returncode == 0
