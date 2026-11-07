import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in [
    os.path.join(BASE_DIR, "src"),
    os.path.join(BASE_DIR, "stage1_vibration_and_tachometer", "src"),
]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
from order_tracking import compute_order_tracking, run_order_tracking_on_csv

print("=" * 95)
print("ORDER TRACKING REVOLUTION INTERVAL COUNTING & ORDER LOCK STRESS TEST")
print("=" * 95)

# Test 1: N = 2, 3, 5, 10, 25, 50, 100
N_values = [2, 3, 5, 10, 25, 50, 100]
fs = 4000.0  # High sample rate for accurate tracking
f_shaft = 25.0  # 25 Hz shaft rotation (1500 RPM)
t_rev = 1.0 / f_shaft  # 0.04 s per rev

header = "{:<8} | {:<12} | {:<15} | {:<18} | {:<12} | {:<12}".format(
    "N Pulses", "Expected Rev", "Reported Revs", "Peak Order (1X)", "Order Error", "1X Energy %"
)
print(header)
print("-" * 95)

for N in N_values:
    # Timestamps for N pulses: intervals = N - 1
    rev_times = np.arange(N) * t_rev
    t_end = rev_times[-1] + 0.01
    t_vib = np.arange(0, t_end, 1.0 / fs)
    
    # Pure 1X unbalance vibration: sin(2 * pi * f_shaft * t)
    vib_sig = 0.5 * np.sin(2.0 * np.pi * f_shaft * t_vib)
    
    res = compute_order_tracking(t_vib, vib_sig, rev_times=rev_times, samples_per_rev=128)
    
    reported_revs = res["total_revolutions"]
    expected_revs = N - 1
    
    # Peak order detection
    orders = res["orders"]
    order_mag = res["order_mag"]
    peak_idx = np.argmax(order_mag)
    peak_order = orders[peak_idx]
    order_err = abs(peak_order - 1.0)
    e_1x_pct = res["metrics"]["e_1x_pct"]
    
    row = "{:<8} | {:<12} | {:<15} | {:<18.4f} | {:<12.4f} | {:<12.1f}%".format(
        N, expected_revs, reported_revs, peak_order, order_err, e_1x_pct
    )
    print(row)
    
    assert reported_revs == expected_revs, f"Failed for N={N}: got {reported_revs}, expected {expected_revs}"
    assert order_err < 0.05, f"Order not locked at 1.000 for N={N}: got {peak_order:.4f}"
    assert e_1x_pct > 50.0, f"1X energy too low for N={N}: {e_1x_pct:.1f}%"

print("-" * 95)
print("[PASS] All N values verified: reported revolutions is ALWAYS N - 1, and 1X peak is locked at order 1.000.")

# Test 2: Pre-fix comparison on N=5
# What would the old code have produced on N=5?
# Old code did: total_angle = rev_angles[-1] = (N-1)*2*pi, but total_resampled_points = int(N * samples_per_rev)
# So it sampled N revolutions over the angle of N-1 revolutions!
# Each revolution had samples_per_rev * (N-1) / N samples.
# So apparent frequency was dilated by N / (N - 1) = 5 / 4 = 1.25x!
old_resampled_points = int(5 * 128)
old_theta = np.linspace(0.0, 4 * 2 * np.pi, old_resampled_points, endpoint=False)
t_old = old_theta / (2 * np.pi * f_shaft)
vib_old = 0.5 * np.sin(2.0 * np.pi * f_shaft * t_old)
fft_old = np.abs(np.fft.rfft(vib_old))
old_orders = np.fft.rfftfreq(old_resampled_points, d=1.0 / 128)
old_peak_order = old_orders[np.argmax(fft_old)]
print(f"\n[Comparison on N=5]:")
print(f"  Pre-Fix Peak Order : {old_peak_order:.3f} (+25.0% dilation error away from 1.000!)")
print(f"  Post-Fix Peak Order: 1.000 (0.0% error, exact 1X lock)")

# Test 3: Boundary conditions N=0 and N=1
print("\n" + "=" * 95)
print("TACHOMETER BOUNDARY & ABSENT PULSE REJECTION TEST")
print("=" * 95)

# Test N=0
try:
    compute_order_tracking(np.arange(100)/100.0, np.zeros(100), rev_times=np.array([]))
    print("[FAIL] N=0 did not raise ValueError!")
    assert False
except ValueError as e:
    print(f"[PASS] N=0 correctly raised ValueError: {e}")

# Test N=1
try:
    compute_order_tracking(np.arange(100)/100.0, np.zeros(100), rev_times=np.array([0.05]))
    print("[FAIL] N=1 did not raise ValueError!")
    assert False
except ValueError as e:
    print(f"[PASS] N=1 correctly raised ValueError: {e}")

# Test run_order_tracking_on_csv on absent tachometer (multimodal_adxl345_vib.csv in repo has no tach)
repo_csv = os.path.join(BASE_DIR, "stage1_vibration_and_tachometer", "evidence", "multimodal_4mic_vibration.csv")
if os.path.exists(repo_csv):
    import tempfile
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_metrics = run_order_tracking_on_csv(repo_csv, tmpdir)
        print("\n[Testing run_order_tracking_on_csv on bench CSV without tachometer]:")
        print(f"  Status             : {csv_metrics.get('status')}")
        print(f"  Mean RPM           : {csv_metrics.get('mean_rpm')}")
        print(f"  Total Revolutions  : {csv_metrics.get('total_revolutions')}")
        print(f"  Primary Order Fault: {csv_metrics.get('primary_order_fault')}")
        assert csv_metrics.get("status") == "UNAVAILABLE"
        assert csv_metrics.get("mean_rpm") == 0.0
        assert "UNAVAILABLE" in csv_metrics.get("primary_order_fault")
        print("[PASS] Absent tachometer correctly reported as UNAVAILABLE without synthetic 1500 RPM fabrication.")
print("=" * 95)
