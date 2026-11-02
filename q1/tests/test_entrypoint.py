"""Entrypoint dispatch tests; ROS setup is stubbed for host-only execution."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EntrypointTests(unittest.TestCase):
    def run_entrypoint(self, *args):
        with tempfile.TemporaryDirectory() as directory:
            setup = Path(directory) / 'bash-env'
            setup.write_text('source() { :; }\n')
            env = dict(os.environ, BASH_ENV=str(setup), PYTHONDONTWRITEBYTECODE='1')
            return subprocess.run(
                ['bash', str(ROOT / 'scripts/entrypoint.sh'), *args],
                cwd=ROOT, env=env, capture_output=True, text=True, timeout=10,
            )

    def test_shell_command_preserves_output_and_exit_code(self):
        result = self.run_entrypoint('bash', '-c', 'printf shell-ready; exit 17')
        self.assertEqual(result.stdout, 'shell-ready', result.stderr)
        self.assertEqual(result.returncode, 17, result.stderr)

    def test_existing_application_flags_still_work(self):
        result = self.run_entrypoint('--backend', 'mock', '--diagnose')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['backend'], 'mock')
