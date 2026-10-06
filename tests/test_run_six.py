"""Unit tests of path/config orchestration. No robot dynamics are simulated here."""
import contextlib
import importlib.util
import io
import json
import pathlib
import sys
import tempfile
import unittest
from unittest.mock import patch

PROJECT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("runner", PROJECT / "tools/run_six.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class RunnerPaths(unittest.TestCase):
    def exercise(self, options):
        with tempfile.TemporaryDirectory() as temporary:
            base = pathlib.Path(temporary)
            project, source, build = base / "planner", base / "dynobench", base / "build"
            models = source / "models"
            paper_models = project / "benchmark/models/paper"
            configs = project / "benchmark/config"
            models.mkdir(parents=True)
            paper_models.mkdir(parents=True)
            configs.mkdir(parents=True)
            (configs / "full_df").mkdir()
            for name, (env, model) in runner.CASES.items():
                env_file = source / "envs" / env
                env_file.parent.mkdir(parents=True, exist_ok=True)
                env_file.write_text("fixture\n")
                (models / (model + ".yaml")).write_text("fixture\n")
                if model in runner.PAPER_MODELS:
                    (paper_models / (model + ".yaml")).write_text("fixture\n")
                (configs / (name + ".yaml")).write_text(
                    (PROJECT / "benchmark/config" / (name + ".yaml")).read_text())
                (configs / "full_df" / (name + ".yaml")).write_text(
                    (PROJECT / "benchmark/config/full_df" / (name + ".yaml")).read_text())
            for name in ("benchmark.exe", "test_real_dynobench", "external/dynobench/check_trajectory"):
                binary = build / name
                binary.parent.mkdir(parents=True, exist_ok=True)
                binary.write_text("unit test stub, never executed")
            output = base / "results"
            checker_calls = []

            def execute(command, log, timeout):
                log.write_text("Unit-test stub; not an experimental result.\n")
                if pathlib.Path(command[0]).stem == "benchmark":
                    case = pathlib.Path(command[2]).stem
                    config = pathlib.Path(command[2]).read_text()
                    expected_mode = "full_df" if "--df-mode" in options else "paper_r2"
                    self.assertIn("  mode: " + expected_mode, config)
                    self.assertIn("  guidance_probability: 0", config)
                    self.assertNotIn("tilt_hints:", config)
                    self.assertNotIn("  regions:", config)
                    if case.startswith("quad2d"):
                        self.assertNotIn("models_directory:", config)
                    else:
                        self.assertIn(
                            '  model_file: ' + json.dumps(
                                paper_models.joinpath(runner.CASES[case][1] + ".yaml")
                                .resolve().as_posix()),
                            config)
                    self.assertIn('  stop_at_first_solution: ' + str('--first-solution' in options).lower(), config)
                    case_dir = output / case
                    case_dir.mkdir()
                    (case_dir / "results.csv").write_text(
                        "run,success,validation_backend\n0,1,simd_discrete_float8\n")
                    (case_dir / "validation.csv").write_text("run,dynobench_success\n0,1\n")
                elif pathlib.Path(command[0]).name == "check_trajectory":
                    self.fail("The paper SIMD runner must not invoke scalar check_trajectory")
                return 0

            argv = ["run_six", "--dynobench-source", str(source), "--build-dir", str(build),
                    "--output", str(output), "--runs", "1", *options]
            with patch.object(runner, "__file__", str(project / "tools/run_six.py")), \
                    patch.object(runner, "execute", execute), patch.object(sys, "argv", argv), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(runner.main(), 0)
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(len(manifest["inputs"]), 6)
            self.assertTrue((source / "models").exists())

    def test_default_profile_paths(self):
        self.exercise([])

    def test_independent_ab_switches(self):
        self.exercise(["--first-solution"])

    def test_full_df_paths(self):
        self.exercise(["--df-mode", "full_df"])


if __name__ == "__main__":
    unittest.main()
