"""Un segundo proceso no debe crear otro servidor ni otro bridge."""
import fcntl
import subprocess
import sys
import signal

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


def test_sigint_stops_executor_before_ros_context_is_closed():
    program = '''
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from tp1_teach_repeat.runtime import spin_until_stopped
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = Node('tp1_shutdown_test')
print('ready', flush=True)
spin_until_stopped(node)
assert rclpy.ok()
print('context valid', flush=True)
node.destroy_node()
rclpy.shutdown()
'''
    process = subprocess.Popen([sys.executable, '-c', program], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True)
    try:
        assert process.stdout.readline().strip() == 'ready'
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 0, stderr
        assert 'context valid' in stdout
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
