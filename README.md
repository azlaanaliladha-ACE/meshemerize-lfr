# Meshemerize LFR – maze line follower

ESP32 + Makeblock 4-ch I2C line sensor + L298N. Left-hand-rule maze solver.

- `sim.py` – Python physics simulator (note: Python, not MATLAB) with firmware mirror
- `batch.py`, `tune*.py`, `calib.py`, `compare.py`, `plot.py`, `render_video.py` – sweeps, tuning, plots, video
- `lfr_original_exact/` – original PD code (BASE_SPEED 130), currently flashed
- `meshemerize_lfr/` – sim-tuned firmware (not hardware-validated)
- `lfr_v3/`, `lfr_pd_original/` – experimental variants
- `lfr_run.mp4` – 10 s simulation video

Sim results did not transfer to the real robot; treat the simulator as approximate.
