"""Read-only verification of this delivery's real smoke outputs; not performance data."""
import csv
import json
from pathlib import Path

root = Path(__file__).resolve().parents[2]
report = {}
for mode, directory in [('paper_r2', 'r2_smoke_20261006'), ('full_df', 'full_df_smoke_20261006')]:
    folder = root / 'results' / directory
    manifest = json.loads((folder / 'manifest.json').read_text())
    assert manifest['df_mode'] == mode and manifest['profile'] == 'paper_fair'
    entries = []
    for case in manifest['cases']:
        result_dir = folder / case
        with (result_dir / 'results.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 1
        row = rows[0]
        order = 2 if mode == 'paper_r2' or case.startswith('uni_') else 4
        assert row['df_mode'] == mode and int(row['pseudo_control_order']) == order
        assert row['solver_precision'] == 'float64' and row['validation_precision'] == 'float32'
        assert row['reduced_order_result'] == ('1' if mode == 'paper_r2' else '0')
        with (result_dir / 'boundary_diagnostics_0.csv').open(newline='') as stream:
            diagnostics = list(csv.DictReader(stream))
        trajectory = result_dir / 'trajectory_0.yaml'
        if row['success'] == '1':
            assert diagnostics[0]['kind'] == 'start' and diagnostics[-1]['kind'] == 'goal'
            assert all(float(d['flat_position_error']) < 1e-6 and float(d['flat_velocity_error']) < 1e-6 for d in diagnostics)
            assert trajectory.is_file()
            content = trajectory.read_text()
            assert 'df_mode: ' + mode in content
            assert 'physical_continuity_certified: false' in content
            assert 'dynobench_verified: false' in content
            if mode == 'full_df':
                assert row['physical_boundary_match'] == '1'
        else:
            assert not trajectory.exists()
        with (result_dir / 'validation.csv').open(newline='') as stream:
            validation = next(csv.DictReader(stream))
        entries.append(dict(case=case, success=int(row['success']), reason=validation['reason'],
                            pseudo_control_order=order, physical_boundary_match=int(row['physical_boundary_match']),
                            diagnostic_records=len(diagnostics),
                            maximum_attitude_error_rad=max((float(d['attitude_error_rad']) for d in diagnostics), default=None),
                            maximum_omega_error=max((float(d['omega_error']) for d in diagnostics), default=None)))
    report[mode] = entries
destination = Path(__file__).with_name('smoke_report.json')
destination.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
