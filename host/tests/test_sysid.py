# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Unit tests for sysid's first-order step fit.

The headline guard is `test_model_rmse_zero_on_exact_model`: the PR that added this fitter had a
sign bug (`r_inf + span*exp` instead of `r_inf - span*exp`). Data generated FROM the model must
reconstruct with ~0 RMSE; a flipped sign makes it large. This is that regression, in the repo.
"""
import math

import sysid


def _synth(tau, L, r0, r_inf, dt=0.02, n_tau=6):
    """Ideal first-order step: r(t) = r_inf - (r_inf - r0)*exp(-(t-L)/tau), held at r0 for t < L."""
    span = r_inf - r0
    ts, ys, t = [], [], 0.0
    tmax = L + n_tau * tau
    while t <= tmax + 1e-9:
        y = r0 if t < L else r_inf - span * math.exp(-(t - L) / tau)
        ts.append(t)
        ys.append(y)
        t += dt
    return ts, ys


def test_model_rmse_zero_on_exact_model():
    tau, L, r0, r_inf = 0.18, 0.05, 100.0, 1500.0
    ts, ys = _synth(tau, L, r0, r_inf)
    rmse = sysid._model_rmse(ts, ys, r0, r_inf, tau, L)
    assert rmse < 1e-6, f"exact-model RMSE must be ~0 (sign bug guard), got {rmse}"


def test_model_rmse_penalises_wrong_tau():
    tau, L, r0, r_inf = 0.18, 0.05, 100.0, 1500.0
    ts, ys = _synth(tau, L, r0, r_inf)
    good = sysid._model_rmse(ts, ys, r0, r_inf, tau, L)
    bad = sysid._model_rmse(ts, ys, r0, r_inf, tau * 3.0, L)
    assert bad > good + 1.0


def test_fit_first_order_recovers_tau_and_L():
    tau, L, r0, r_inf = 0.18, 0.05, 100.0, 1500.0
    ts, ys = _synth(tau, L, r0, r_inf)
    tau_hat, L_hat, _sse, rmse = sysid.fit_first_order(ts, ys, r0, r_inf)
    assert abs(tau_hat - tau) < 0.045, f"tau {tau_hat} vs {tau}"
    assert L_hat <= L + 0.06, f"L {L_hat} vs {L}"
    assert rmse < 0.05 * (r_inf - r0), f"clean-synthetic fit RMSE too high: {rmse}"  # < 5% of span


def test_fit_first_order_reverse_step():
    # decaying step (r_inf < r0) — the span sign flips; the fit must still recover tau.
    tau, L, r0, r_inf = 0.10, 0.0, 2000.0, 300.0
    ts, ys = _synth(tau, L, r0, r_inf)
    tau_hat, _L, _sse, _rmse = sysid.fit_first_order(ts, ys, r0, r_inf)
    assert abs(tau_hat - tau) < 0.03, f"tau {tau_hat} vs {tau}"


def test_fit_degenerate_flat_returns_zero():
    ts = [i * 0.02 for i in range(50)]
    ys = [100.0] * 50
    tau, L, sse, rmse = sysid.fit_first_order(ts, ys, 100.0, 100.0)  # r_inf == r0
    assert tau == 0.0
