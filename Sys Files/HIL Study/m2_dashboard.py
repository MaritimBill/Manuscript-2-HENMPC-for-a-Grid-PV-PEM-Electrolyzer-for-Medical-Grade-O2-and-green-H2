"""Manuscript 2 — SIL evaluation and Arduino/Simulink HIL supervisor.

Architecture retained:
  * Six-state PEM plant mirror.
  * PID and single-MPC local baselines.
  * Full-model EMPC and hybrid exact-first-step/neural-surrogate EMPC.
  * Arduino GIGA lower current tracker; MATLAB/Simulink plant exchanges each
    request/response over USB Serial, while MQTT carries supervisory commands
    and telemetry between the GIGA and the Python/web dashboard.

The surrogate is trained once by `train`; HIL `collect` only loads and validates the frozen
m2_data/surrogate.npz. No online retraining occurs.

Important evidence boundary:
  The plant is a mathematical model. Modelled purity is not a gas-analyser
  measurement, and Arduino/Simulink HIL is not a physical-stack test.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import queue
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "m2_data"
DATA.mkdir(parents=True, exist_ok=True)
CFG_PATH = DATA / "m2_config.json"
WEATHER_CSV = ROOT / "nairobi_nasa_weather.csv"
SURROGATE = DATA / "surrogate.npz"
RESULTS = DATA / "results"
RESULTS.mkdir(parents=True, exist_ok=True)
PAPER = DATA / "paper_outputs"
PAPER.mkdir(parents=True, exist_ok=True)

R = 8.314462618
F = 96485.33212
COMMAND_TIMEOUT_S = 600.0

FEATURES = [
    "I_stack_A", "I_command_A", "T_stack_C", "P_O2_bar", "P_H2_bar",
    "PV_available_W", "O2_demand_Lpm", "Tamb_C", "hour", "stress_idx",
]
# Faraday gas production, gas delivery, pressure, BoP power and efficiency are
# calculated algebraically from current and inventory. The neural model only
# approximates the nonlinear electrochemical/thermal states used by the hybrid
# predictor; it does not learn mass-flow identities.
TARGETS = ["V_cell_V", "T_next_C", "purity_next_pct", "stress_idx_next"]
# Primary paper comparison required by the research brief. BUFFER_RULE remains
# implemented as an optional experimental controller class, but is not part of
# the primary matched comparison. Five scenarios x four controllers x three
# repeats = 60 runs; larger campaigns require an explicit CLI opt-in.
CONTROLLER_NAMES = ("PID", "SINGLE_MPC", "FULL_MODEL_EMPC", "PROPOSED")
SCENARIO_NAMES = ("clear_sky", "cloudy_rainy", "oxygen_surge", "communication_loss", "sensor_fault_leak")
HIL_SCENARIOS = SCENARIO_NAMES + ("sensor_out_of_range", "purity_alarm")
DEFAULT_SIL_REPEATS = 3
MAX_DEFAULT_SIL_REPEATS = 3


def sha256_file(path: Path) -> str:
    """Return a stable SHA-256 fingerprint for a generated model artifact."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

DEFAULT_CFG = {
    "N_cells": 80,
    "A_cell_cm2": 641.0,
    "O2_tank_volume_m3": 2.0,
    "H2_tank_volume_m3": 2.0,
    "O2_reserve_Nm3": 15.0,
    "initial_O2_inventory_Nm3": 30.0,
    "alpha": 0.448938,
    "j0_A_cm2": 0.05428292,
    "r_ohm_cm2": 0.169475,
    "k_conc": 4.0e-6,
    "faradayEff": 0.95,
    "I_min_A": 100.0,
    "I_max_A": 1400.0,
    "pressure_max_bar": 30.0,
    "max_voltage_V": 2.5,
    "max_temperature_C": 80.0,
    "purity_min_pct": 99.5,
    "P_grid_max_W": 200000.0,
    "mqtt_host": "broker.hivemq.com",
    "mqtt_port": 1883,
    "serialPort": "COM5",
    "serialBaud": 115200,
    "pvCapacity": 250000.0,
    "initial_temperature_C": 65.0,
    "tau_I_s": 5.0,
    "tau_PV_s": 60.0,
    "UA_cool_W_per_K": 60.0,
    "C_th_J_per_K": 500000.0,
    "k_cross_0": 50.0,
    "k_cross_1": 5.0,
    "tariff_off_KES": 12.0,
    "tariff_mid_KES": 18.0,
    "tariff_peak_KES": 24.0,
    "hospital_demand_Lpm": 48.0,
    "hospital_surge_Lpm": 340.0,  # retained as a site-level reference; scenario uses +50% of base
    "BOP_sep_dp_bar": 0.3,
    "BOP_dryer_power_frac": 0.03,
    "BOP_O2_comp_kWh_kg": 0.15,
    "BOP_H2_comp_kWh_kg": 1.5,
    "sensor_ADC_bits": 16,
    "sensor_ACS712_noise_A": 0.4,
    "sensor_ADS1115_noise_V": 0.005,
    "sensor_purity_noise_pct": 0.10,
    "H2_price_KES_kg": 800.0,
    # Zero means no declared external hydrogen offtake. Set this only from a
    # documented demand/offtake assumption; exported mass is inventory-limited.
    "H2_export_demand_kg_s": 0.0,
    "H2_revenue_weight": 0.5,
    "H2_in_O2_limit_ppm": 20000.0,
    "w_terminal_O2_KES_per_Nm3_sq": 100.0,
    "w_terminal_H2_KES_per_Nm3_sq": 100.0,
    "forecast_error_pct": 0.0,
    "sil_dt_s": 300.0,
    "hil_dt_s": 1.0,
    "MPC_horizon_steps": 6,
    "upper_candidate_count": 17,
    "lower_mpc_horizon": 5,
    "lower_move_A": 50.0,
    "lower_grid_A": 5.0,
    "O2_surge_factor": 1.5,
    "O2_surge_start_s": 3 * 86400 + 8 * 3600,
    "O2_surge_duration_s": 86400,
    "fault_start_s": 3 * 86400 + 12 * 3600,
    "fault_duration_s": 3600,
    "train_trajectories": 15,
    "train_steps_per_trajectory": 2500,
    "train_trajectory_fraction": 0.6666666667,
    "surrogate_train_T_min_C": 20.0,
    "surrogate_train_T_max_C": 80.0,
    "surrogate_train_Tamb_min_C": 10.0,
    "surrogate_train_Tamb_max_C": 35.0,
    "min_operating_temperature_C": 50.0,
    # One simulated day is run as warm-up and excluded from reported SIL metrics.
    "warmup_hours": 24.0,
    "train_epochs": 1500,
    "train_batch": 128,
    "train_learning_rate": 0.001,
    "surrogate_envelope_margin": 0.05,
    "w_grid": 1.0,
    "w_o2_unmet_KES_per_L": 100000.0,
    "w_reserve_KES_per_Nm3_sq": 1000.0,
    "w_stress": 200.0,
}


def load_cfg() -> dict:
    """Load the shared JSON config and map legacy field names without overwriting it."""
    if not CFG_PATH.exists():
        CFG_PATH.write_text(json.dumps(DEFAULT_CFG, indent=2), encoding="utf-8")
        print(f"[cfg] created {CFG_PATH}")
    cfg = json.loads(CFG_PATH.read_text(encoding="utf-8"))
    for key, value in DEFAULT_CFG.items():
        cfg.setdefault(key, value)

    # Support the field names used in the originally supplied configuration.
    cfg.setdefault("BOP_O2_comp_kWh_kg", cfg.get("BOP_O2_comp_kWh_per_kg", DEFAULT_CFG["BOP_O2_comp_kWh_kg"]))
    cfg.setdefault("BOP_H2_comp_kWh_kg", cfg.get("BOP_H2_comp_kWh_per_kg", DEFAULT_CFG["BOP_H2_comp_kWh_kg"]))
    cfg.setdefault("hospital_demand_Lpm", cfg.get("hospitalDemand", DEFAULT_CFG["hospital_demand_Lpm"]))
    cfg.setdefault("H2_export_demand_kg_s", cfg.get("H2_draw_kg_s", DEFAULT_CFG["H2_export_demand_kg_s"]))
    cfg.setdefault("sil_dt_s", cfg.get("dt_batch_s", DEFAULT_CFG["sil_dt_s"]))
    cfg.setdefault("hil_dt_s", cfg.get("lower_dt_s", DEFAULT_CFG["hil_dt_s"]))

    numeric_positive = (
        "N_cells", "A_cell_cm2", "O2_tank_volume_m3", "H2_tank_volume_m3", "initial_O2_inventory_Nm3",
        "pressure_max_bar", "faradayEff", "I_min_A", "I_max_A", "sil_dt_s", "hil_dt_s",
    )
    for key in numeric_positive:
        if not np.isfinite(float(cfg[key])) or float(cfg[key]) <= 0:
            raise ValueError(f"Configuration field {key} must be finite and > 0; got {cfg[key]!r}")
    if cfg["I_max_A"] <= cfg["I_min_A"]:
        raise ValueError("I_max_A must exceed I_min_A")
    if not (20.0 <= float(cfg["surrogate_train_T_min_C"]) < float(cfg["surrogate_train_T_max_C"]) <= 80.0):
        raise ValueError("Surrogate stack-temperature training bounds must lie within 20..80 °C")
    if not (10.0 <= float(cfg["surrogate_train_Tamb_min_C"]) < float(cfg["surrogate_train_Tamb_max_C"]) <= 35.0):
        raise ValueError("Surrogate ambient-temperature training bounds must lie within 10..35 °C")
    if not (20.0 <= float(cfg["min_operating_temperature_C"]) <= float(cfg["max_temperature_C"])):
        raise ValueError("min_operating_temperature_C must be within the plant thermal limits")
    if not 0 < cfg["faradayEff"] <= 1:
        raise ValueError("faradayEff must be in (0, 1]")
    if float(cfg["H2_export_demand_kg_s"]) < 0:
        raise ValueError("H2_export_demand_kg_s must be >= 0")
    if float(cfg["H2_in_O2_limit_ppm"]) <= 0:
        raise ValueError("H2_in_O2_limit_ppm must be > 0")
    if not 0.0 <= float(cfg["forecast_error_pct"]) <= 100.0:
        raise ValueError("forecast_error_pct must be between 0 and 100")
    return cfg


_SRNG = np.random.default_rng(1234)


def _noise(sigma: float) -> float:
    return float(_SRNG.normal(0.0, sigma))


def _progress(i: int, n: int, prefix: str = "", width: int = 40) -> None:
    frac = i / max(n, 1)
    filled = int(frac * width)
    bar = "#" * filled + "." * (width - filled)
    sys.stdout.write(f"\r{prefix:30s} [{bar}] {100 * frac:5.1f}%  ({i}/{n})")
    sys.stdout.flush()
    if i >= n:
        sys.stdout.write("\n")


# ==================== SIX-STATE PLANT MIRROR ====================
def vcell(T_C: float, nO2: float, nH2: float, I_A: float, cfg: dict) -> float:
    TK = max(float(T_C) + 273.15, 250.0)
    pmax = float(cfg["pressure_max_bar"])
    pO2 = min(pmax, max(0.05, float(nO2) * R * TK / max(cfg["O2_tank_volume_m3"], 1e-6) / 1e5))
    pH2 = min(pmax, max(0.05, float(nH2) * R * TK / max(cfg["H2_tank_volume_m3"], 1e-6) / 1e5))
    Jcm = max(float(I_A), 0.0) / max(cfg["A_cell_cm2"], 1e-9)
    Erev = 1.42 - 0.0009 * (float(T_C) - 75.0)
    etaA = (R * TK / (cfg["alpha"] * 2.0 * F)) * np.arcsinh(Jcm / (2.0 * max(cfg["j0_A_cm2"], 1e-9)))
    etaP = (R * TK / (2.0 * F)) * np.log(max(pH2 * np.sqrt(pO2), 1e-8))
    return float(max(0.5, Erev + etaA + cfg["r_ohm_cm2"] * Jcm + cfg["k_conc"] * Jcm * Jcm + etaP))


def plant_step(x: Sequence[float], I_cmd: float, fc: dict, cfg: dict, dt: Optional[float] = None):
    """Advance the six-state plant with closed oxygen/hydrogen mass balances.

    Oxygen delivery includes gas produced during the interval. Hydrogen export
    is limited by available gas; unexported gas is stored up to the pressure
    cap and any excess is explicitly counted as vented. A zero current command
    is an immediate shutdown and bypasses the normal minimum-load clamp.
    """
    if dt is None:
        dt = float(cfg["sil_dt_s"])
    if dt <= 0 or not np.isfinite(dt):
        raise ValueError(f"dt must be finite and > 0; got {dt!r}")
    x = np.asarray(x, dtype=float).reshape(-1)
    if x.size != 6 or not np.all(np.isfinite(x)):
        raise ValueError("plant state must contain six finite values")
    T_C, nO2, nH2, I_act, PVf, stress = map(float, x)
    shutdown = (not np.isfinite(float(I_cmd))) or float(I_cmd) <= 0.0
    if shutdown:
        I_n = 0.0
    else:
        cmd = float(np.clip(float(I_cmd), cfg["I_min_A"], cfg["I_max_A"]))
        aI = math.exp(-dt / max(cfg["tau_I_s"], 1e-3))
        I_n = float(np.clip(aI * I_act + (1.0 - aI) * cmd, 0.0, cfg["I_max_A"]))

    N = float(cfg["N_cells"])
    A_cm2 = float(cfg["A_cell_cm2"])
    V_O2 = float(cfg["O2_tank_volume_m3"])
    V_H2 = float(cfg["H2_tank_volume_m3"])
    fe = float(cfg["faradayEff"])
    V = vcell(T_C, nO2, nH2, I_n, cfg)

    # Faraday production is algebraic and is never learned by the surrogate.
    nd_O2 = N * I_n / (4.0 * F) * fe             # mol/s O2
    nd_H2 = N * I_n / (2.0 * F) * fe             # mol/s H2
    m_O2 = nd_O2 * 0.032                         # kg/s O2
    m_H2 = nd_H2 * 0.002016                      # kg/s H2
    P_stack = N * V * I_n
    P_dry = float(cfg["BOP_dryer_power_frac"]) * P_stack
    P_O2_comp = float(cfg["BOP_O2_comp_kWh_kg"]) * m_O2 * 3.6e6
    P_H2_comp = float(cfg["BOP_H2_comp_kWh_kg"]) * m_H2 * 3.6e6
    P_BoP = P_dry + P_O2_comp + P_H2_comp
    P_system = P_stack + P_BoP

    O2_demand = max(0.0, float(fc.get("O2_demand_Lpm", cfg["hospital_demand_Lpm"])))
    O2_demand_mol = O2_demand / 60.0 / 22.414       # mol/s
    O2_available_mol = max(0.0, nO2) + nd_O2 * dt
    O2_requested_mol = O2_demand_mol * dt
    O2_served_mol = min(O2_available_mol, O2_requested_mol)
    O2_supply = O2_served_mol / dt * 60.0 * 22.414
    O2_unmet_L = max(0.0, (O2_demand - O2_supply) * dt / 60.0)
    nO2_before_cap = max(0.0, O2_available_mol - O2_served_mol)

    # Hydrogen withdrawal is an explicit exogenous offtake request, not an
    # assumed sale of all produced hydrogen.
    H2_export_demand = max(0.0, float(fc.get(
        "H2_export_demand_kg_s", fc.get("H2_draw_kg_s", cfg.get("H2_export_demand_kg_s", 0.0)))))
    H2_available_kg = max(0.0, nH2) * 0.002016 + m_H2 * dt
    H2_export_kg_step = min(H2_available_kg, H2_export_demand * dt)
    H2_after_export_kg = max(0.0, H2_available_kg - H2_export_kg_step)
    nH2_before_cap = H2_after_export_kg / 0.002016

    P_heat = N * I_n * max(V - 1.48, 0.0)
    Tamb = float(fc.get("Tamb_C", 25.0))
    P_cool = float(cfg["UA_cool_W_per_K"]) * (T_C - Tamb)
    T_next = float(np.clip(T_C + (P_heat - P_cool) * dt / max(cfg["C_th_J_per_K"], 1e-3), 20.0, 120.0))

    PV_available = float(np.clip(fc.get("PV_available_W", 0.0), 0.0, cfg["pvCapacity"]))
    aPV = math.exp(-dt / max(cfg["tau_PV_s"], 1e-3))
    PV_filtered_next = aPV * PVf + (1.0 - aPV) * PV_available
    P_PV = min(P_system, max(0.0, PV_filtered_next))
    P_grid = max(0.0, P_system - P_PV)
    grid_overlimit = max(0.0, P_grid - float(cfg["P_grid_max_W"]))

    TK_next = max(T_next + 273.15, 250.0)
    nO2_cap = cfg["pressure_max_bar"] * 1e5 * V_O2 / (R * TK_next)
    nH2_cap = cfg["pressure_max_bar"] * 1e5 * V_H2 / (R * TK_next)
    O2_vented_mol = max(0.0, nO2_before_cap - nO2_cap)
    H2_vented_kg_step = max(0.0, (nH2_before_cap - nH2_cap) * 0.002016)
    nO2_next = min(nO2_cap, nO2_before_cap)
    nH2_next = min(nH2_cap, nH2_before_cap)

    pO2_tank = min(float(cfg["pressure_max_bar"]),
                    max(0.05, nO2_next * R * TK_next / max(V_O2, 1e-6) / 1e5))
    pH2_tank = min(float(cfg["pressure_max_bar"]),
                    max(0.05, nH2_next * R * TK_next / max(V_H2, 1e-6) / 1e5))
    pO2_product = max(0.05, pO2_tank - float(cfg["BOP_sep_dp_bar"]))
    Jcm = I_n / max(A_cm2, 1e-9)
    crossover_ppm = (float(cfg["k_cross_0"]) + float(cfg["k_cross_1"]) * pH2_tank) / (Jcm + 0.1)
    purity = float(np.clip(100.0 - crossover_ppm / 1e4, 90.0, 100.0))
    ds = (
        max(0.0, T_next - 70.0) / 10.0
        + max(0.0, Jcm - 1.5)
        + 5.0 * max(0.0, V - 2.2)
    ) * (dt / 3600.0) * 1e-5
    stress_next = stress + ds
    H2_LHV_eff = 100.0 * (nd_H2 * 241.8e3) / max(P_stack, 1e-3) if I_n > 0 else 0.0
    system_eff = 100.0 * (nd_H2 * 120e6) / max(P_system, 1e-3) if I_n > 0 else 0.0

    x_next = np.array([T_next, nO2_next, nH2_next, I_n, PV_filtered_next, stress_next], dtype=float)
    y = {
        "O2_flow_Lpm": O2_supply,
        "O2_prod_Lpm": nd_O2 * 60.0 * 22.414,
        "O2_unmet_L_step": O2_unmet_L,
        "O2_vented_L_step": O2_vented_mol * 22.414,
        "H2_kg_s": m_H2,
        "H2_produced_kg_step": m_H2 * dt,
        "H2_export_kg_s": H2_export_kg_step / dt,
        "H2_export_kg_step": H2_export_kg_step,
        "H2_vent_kg_s": H2_vented_kg_step / dt,
        "H2_vent_kg_step": H2_vented_kg_step,
        "V_cell_V": V,
        "P_stack_W": P_stack,
        "P_BoP_W": P_BoP,
        "P_system_W": P_system,
        "P_grid_W": P_grid,
        "P_PV_W": P_PV,
        "PV_available_W": PV_available,
        "PV_filtered_W": PV_filtered_next,
        "grid_overlimit_W": grid_overlimit,
        "T_next_C": T_next,
        "P_O2_next_bar": pO2_product,
        "P_H2_next_bar": pH2_tank,
        "P_O2_tank_bar": pO2_tank,
        "purity_next_pct": purity,
        "H2_in_O2_ppm": crossover_ppm,
        "crossover_ppm_model": crossover_ppm,
        "stress_idx_next": stress_next,
        "efficiency_pct": H2_LHV_eff,
        "system_efficiency_pct": system_eff,
        "P_O2_bar": pO2_product,
        "P_H2_bar": pH2_tank,
        "I_stack_A": I_n,
        "T_stack_C": T_next,
        "O2_buffer_Nm3": nO2_next * R * 273.15 / 1.01325e5,
        "H2_buffer_Nm3": nH2_next * R * 273.15 / 1.01325e5,
        "I_sensor_A": I_n,
        "V_cell_sensor_V": V,
        "T_sensor_C": T_next,
        "purity_sensor_pct": purity,
        "H2_leak_ppm": float(fc.get("H2_leak_ppm_inject", 0.0)),
        "shutdown": bool(shutdown),
    }
    return x_next, y


# ==================== WEATHER, DEMAND AND TARIFF ====================
def load_weather(cfg: dict, weather_path: Optional[str] = None) -> pd.DataFrame:
    """Load an hourly PV/weather profile; an alternate file can be selected for HIL."""
    source = Path(weather_path) if weather_path else WEATHER_CSV
    if not source.is_absolute():
        source = ROOT / source
    if not source.exists():
        raise FileNotFoundError(
            f"Missing {source}. Supply a verified hourly PV/weather CSV; "
            "the HIL collector will not invent a weather trace."
        )
    df = pd.read_csv(source)
    required = {"time_s", "PV_available_W", "Tamb_C"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Weather file is missing columns: {sorted(missing)}")
    keep = ["time_s", "PV_available_W", "Tamb_C"]
    if "hour_local" in df.columns:
        keep.append("hour_local")
    if "time_nairobi_local" in df.columns:
        keep.append("time_nairobi_local")
    df = df[keep].copy().sort_values("time_s").dropna()
    if len(df) < 2 or not np.all(np.isfinite(df.to_numpy(dtype=float))):
        raise ValueError("Weather file must contain at least two finite hourly records")
    df["time_s"] -= float(df["time_s"].iloc[0])
    df["PV_available_W"] = np.clip(df["PV_available_W"].astype(float), 0.0, float(cfg["pvCapacity"]))
    # Preserve the NASA-to-PV mapping made by download_nasa_scenarios.py.
    # Do not re-normalise each downloaded year to its own maximum: that would
    # erase the irradiance-to-rated-capacity assumption and exaggerate weak years.
    return df.reset_index(drop=True)


def select_rainy_season_week_offsets(df: pd.DataFrame, count: int) -> List[float]:
    """Select distinct 7-day windows from Nairobi long/short-rains months.

    Uses timestamps from NASA POWER when present. Without timestamps, returns
    separated weekly offsets from the available record and marks that limitation
    in the SIL run metadata.
    """
    count = max(1, int(count))
    if "time_nairobi_local" not in df.columns:
        return [float(i * 7 * 86400.0) for i in range(count)]
    stamps = pd.to_datetime(df["time_nairobi_local"], errors="coerce")
    good = stamps.notna()
    if good.sum() < 24 * 14:
        return [float(i * 7 * 86400.0) for i in range(count)]
    start_stamp = stamps[good].iloc[0]
    end_stamp = stamps[good].iloc[-1]
    options = []
    for month in (3, 4, 5, 10, 11, 12):
        month_start = pd.Timestamp(year=int(start_stamp.year), month=month, day=1)
        if month_start < start_stamp:
            continue
        for week in (0, 7, 14, 21):
            candidate = month_start + pd.Timedelta(days=week)
            if candidate + pd.Timedelta(days=7) <= end_stamp:
                options.append(candidate)
    # Deduplicate and spread selections across long/short-rains weeks.
    options = sorted(set(options))
    if not options:
        return [float(i * 7 * 86400.0) for i in range(count)]
    selected = []
    if count <= len(options):
        ix = np.linspace(0, len(options) - 1, count).round().astype(int)
        selected = [options[i] for i in ix]
    else:
        selected = [options[i % len(options)] for i in range(count)]
    offsets = []
    for stamp in selected:
        idx = int(np.argmin(np.abs((stamps - stamp).dt.total_seconds().fillna(np.inf).to_numpy())))
        offsets.append(float(df["time_s"].iloc[idx]))
    return offsets


def weather_at(df: pd.DataFrame, t_s: float) -> dict:
    if len(df) < 2:
        raise ValueError("At least two weather samples are required")
    interval = float(df["time_s"].iloc[1] - df["time_s"].iloc[0])
    period = float(df["time_s"].iloc[-1] - df["time_s"].iloc[0] + interval)
    t = float(t_s) % max(period, interval)
    return {
        "PV_available_W": float(np.interp(t, df["time_s"], df["PV_available_W"])),
        "Tamb_C": float(np.interp(t, df["time_s"], df["Tamb_C"])),
        "hour": float(((float(df["hour_local"].iloc[0]) if "hour_local" in df.columns else 0.0)
                        + t / 3600.0) % 24.0),
    }


def o2_demand(hour: float, cfg: dict, t_s: Optional[float] = None, noise: float = 0.0) -> float:
    base = float(cfg["hospital_demand_Lpm"]) * (1.0 + 0.15 * math.sin((hour - 6.0) / 24.0 * 2.0 * math.pi))
    if t_s is not None:
        start = float(cfg["O2_surge_start_s"])
        end = start + float(cfg["O2_surge_duration_s"])
        if start <= t_s < end:
            base *= float(cfg["O2_surge_factor"])
    return float(max(0.0, base * (1.0 + noise)))


def load_demand_profile(path: str) -> pd.DataFrame:
    """Load a measured hospital demand trace with columns time_s,O2_demand_Lpm.

    time_s is elapsed seconds from the beginning of the profile. A 24-h trace
    may be repeated cyclically; the original values are never rescaled.
    """
    source = Path(path)
    if not source.is_absolute():
        source = ROOT / source
    if not source.exists():
        raise FileNotFoundError(f"Demand profile not found: {source}")
    frame = pd.read_csv(source)
    required = {"time_s", "O2_demand_Lpm"}
    missing = required - set(frame.columns)
    if missing:
        raise ValueError(
            f"Demand profile {source} must contain columns {sorted(required)}; missing {sorted(missing)}"
        )
    frame = frame[["time_s", "O2_demand_Lpm"]].copy()
    frame["time_s"] = pd.to_numeric(frame["time_s"], errors="coerce")
    frame["O2_demand_Lpm"] = pd.to_numeric(frame["O2_demand_Lpm"], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna().sort_values("time_s")
    frame = frame.drop_duplicates(subset=["time_s"], keep="last").reset_index(drop=True)
    if len(frame) < 2 or float(frame["time_s"].iloc[-1]) <= float(frame["time_s"].iloc[0]):
        raise ValueError("Demand profile needs at least two distinct, increasing time_s samples")
    if (frame["O2_demand_Lpm"] < 0).any():
        raise ValueError("O2_demand_Lpm must be non-negative")
    frame["time_s"] = frame["time_s"] - float(frame["time_s"].iloc[0])
    return frame


def demand_at_profile(profile: pd.DataFrame, time_s: float) -> float:
    """Interpolate a demand profile; repeat its full period when a run is longer."""
    times = profile["time_s"].to_numpy(dtype=float)
    values = profile["O2_demand_Lpm"].to_numpy(dtype=float)
    period = float(times[-1])
    if period <= 0:
        raise ValueError("Demand profile period must be positive")
    t = float(time_s) % period
    return float(max(0.0, np.interp(t, times, values)))


def tariff_for(cfg: dict, hour: float) -> float:
    if hour < 6.0 or hour > 22.0:
        return float(cfg["tariff_off_KES"])
    if 8.0 <= hour <= 20.0:
        return float(cfg["tariff_peak_KES"])
    return float(cfg["tariff_mid_KES"])


def demand_current(cfg: dict, demand_Lpm: float) -> float:
    current = demand_Lpm / 60.0 / 22.414 * 4.0 * F / (cfg["N_cells"] * cfg["faradayEff"])
    return float(np.clip(current, cfg["I_min_A"], cfg["I_max_A"]))


def build_scenario_profile(cfg: dict, weather_df: pd.DataFrame, scenario: str, days: int, seed: int,
                           weather_offset_s: float = 0.0, warmup_hours: float = 0.0) -> List[dict]:
    """Create one immutable disturbance trace, shared by every controller in a repeat."""
    if scenario not in SCENARIO_NAMES:
        raise ValueError(f"Unknown scenario {scenario!r}; choose from {SCENARIO_NAMES}")
    rng = np.random.default_rng(seed)
    dt = float(cfg["sil_dt_s"])
    warmup_s = max(0.0, float(warmup_hours) * 3600.0)
    total_days = float(days) + warmup_s / 86400.0
    steps = int(round(total_days * 86400.0 / dt))
    out: List[dict] = []
    leak_start = float(cfg["fault_start_s"])
    leak_end = leak_start + float(cfg["fault_duration_s"])
    both_down_start = 3 * 86400 + 13 * 3600
    both_down_end = both_down_start + 3600
    gsm_start = 3 * 86400 + 12 * 3600
    gsm_end = gsm_start + 3600
    for i in range(steps):
        t = i * dt
        w = weather_at(weather_df, t + float(weather_offset_s))
        if scenario == "cloudy_rainy":
            w["PV_available_W"] *= 0.4
        scenario_t = t - warmup_s
        surge_time = scenario_t if scenario == "oxygen_surge" else None
        demand = o2_demand(w["hour"], cfg, t_s=surge_time, noise=float(rng.normal(0.0, 0.05)))
        fault = "nominal"
        if scenario == "communication_loss":
            if gsm_start <= scenario_t < gsm_end:
                fault = "gsm_fallback_simulated"
            elif both_down_start <= scenario_t < both_down_end:
                fault = "both_down"
        elif scenario == "sensor_fault_leak" and leak_start <= scenario_t < leak_end:
            fault = "h2_leak"
        out.append({
            "t_s": t,
            "measurement_time_s": scenario_t,
            "is_warmup": bool(t < warmup_s),
            "PV_available_W": float(w["PV_available_W"]),
            "Tamb_C": float(w["Tamb_C"]),
            "hour": float(w["hour"]),
            "O2_demand_Lpm": float(demand),
            "tariff_KES_kWh": tariff_for(cfg, w["hour"]),
            "fault_state": fault,
            "H2_leak_ppm_inject": 800.0 if fault == "h2_leak" else 0.0,
            "H2_export_demand_kg_s": float(cfg.get("H2_export_demand_kg_s", 0.0)),
        })
    return out


# ==================== SURROGATE ====================
class MLP:
    """Small standardised 10-64-64-4 MLP with a persisted input envelope."""
    def __init__(self, sizes=None, seed: int = 42):
        if sizes is None:
            sizes = (len(FEATURES), 64, 64, len(TARGETS))
        self.sizes = tuple(int(v) for v in sizes)
        rng = np.random.default_rng(seed)
        self.W = [rng.normal(0.0, np.sqrt(2.0 / a), (a, b)) for a, b in zip(self.sizes[:-1], self.sizes[1:])]
        self.b = [np.zeros((1, b)) for b in self.sizes[1:]]
        self.xm = self.xs = self.ym = self.ys = None
        self.xmin = self.xmax = None

    def _fw(self, X):
        A = [X]
        Z = []
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            z = A[-1] @ w + b
            Z.append(z)
            A.append(z if i == len(self.W) - 1 else np.maximum(z, 0.0))
        return A, Z

    def fit(self, X, Y, epochs=1500, lr=0.001, batch=128):
        if len(X) < 10:
            raise ValueError("At least 10 training samples are required")
        self.xm = X.mean(0)
        self.xs = np.where(X.std(0) < 1e-9, 1.0, X.std(0))
        self.ym = Y.mean(0)
        self.ys = np.where(Y.std(0) < 1e-9, 1.0, Y.std(0))
        self.xmin = X.min(0)
        self.xmax = X.max(0)
        Xn = (X - self.xm) / self.xs
        Yn = (Y - self.ym) / self.ys
        rng = np.random.default_rng(42)
        mW = [np.zeros_like(w) for w in self.W]
        vW = [np.zeros_like(w) for w in self.W]
        mb = [np.zeros_like(b) for b in self.b]
        vb = [np.zeros_like(b) for b in self.b]
        b1, b2, eps, step = 0.9, 0.999, 1e-8, 0
        report_every = max(1, int(epochs) // 20)
        for ep_i in range(int(epochs)):
            idx = rng.permutation(len(Xn))
            for start in range(0, len(idx), int(batch)):
                ids = idx[start:start + int(batch)]
                A, Z = self._fw(Xn[ids])
                d = (A[-1] - Yn[ids]) * (2.0 / len(ids))
                gW = [None] * len(self.W)
                gb = [None] * len(self.b)
                for layer in reversed(range(len(self.W))):
                    gW[layer] = A[layer].T @ d
                    gb[layer] = d.sum(0, keepdims=True)
                    if layer > 0:
                        d = (d @ self.W[layer].T) * (Z[layer - 1] > 0)
                step += 1
                for layer in range(len(self.W)):
                    mW[layer] = b1 * mW[layer] + (1 - b1) * gW[layer]
                    vW[layer] = b2 * vW[layer] + (1 - b2) * (gW[layer] ** 2)
                    mb[layer] = b1 * mb[layer] + (1 - b1) * gb[layer]
                    vb[layer] = b2 * vb[layer] + (1 - b2) * (gb[layer] ** 2)
                    self.W[layer] -= lr * (mW[layer] / (1 - b1 ** step)) / (np.sqrt(vW[layer] / (1 - b2 ** step)) + eps)
                    self.b[layer] -= lr * (mb[layer] / (1 - b1 ** step)) / (np.sqrt(vb[layer] / (1 - b2 ** step)) + eps)
            if (ep_i + 1) % report_every == 0 or ep_i == epochs - 1:
                probe = Xn[:min(1000, len(Xn))]
                probe_y = Yn[:min(1000, len(Yn))]
                A_probe, _ = self._fw(probe)
                mse = float(np.mean((A_probe[-1] - probe_y) ** 2))
                _progress(ep_i + 1, epochs, f"train mse={mse:.5f}")
        return self

    def predict(self, X):
        if self.xm is None:
            raise RuntimeError("Surrogate has not been fitted or loaded")
        X = np.asarray(X, dtype=float)
        one = X.ndim == 1
        if one:
            X = X.reshape(1, -1)
        if X.shape[1] != len(FEATURES):
            raise ValueError(f"Surrogate expects {len(FEATURES)} features, received {X.shape[1]}")
        A, _ = self._fw((X - self.xm) / self.xs)
        result = A[-1] * self.ys + self.ym
        return result[0] if one else result

    def in_envelope(self, x, margin=0.05) -> bool:
        x = np.asarray(x, dtype=float).reshape(-1)
        if self.xmin is not None and self.xmax is not None:
            span = np.maximum(self.xmax - self.xmin, 1e-9)
            return bool(np.all(x >= self.xmin - margin * span) and np.all(x <= self.xmax + margin * span))
        # Backwards-compatible fallback for older saved surrogate files.
        return bool(np.all(x >= self.xm - 8.0 * self.xs) and np.all(x <= self.xm + 8.0 * self.xs))

    def save(self, path: Path) -> None:
        payload = {"sizes": np.asarray(self.sizes), "xm": self.xm, "xs": self.xs,
                   "ym": self.ym, "ys": self.ys}
        if self.xmin is not None:
            payload["xmin"] = self.xmin
            payload["xmax"] = self.xmax
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            payload[f"W{i}"] = w
            payload[f"b{i}"] = b
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = path.with_name(path.stem + ".tmp.npz")
        np.savez_compressed(tmp_path, **payload)
        tmp_path.replace(path)

    @classmethod
    def load(cls, path: Path):
        if not Path(path).exists():
            raise FileNotFoundError(f"Frozen surrogate not found: {path}. Run `python m2_dashboard.py train` once before HIL.")
        with np.load(path, allow_pickle=False) as z:
            net = cls(tuple(int(v) for v in z["sizes"]))
            net.xm, net.xs = z["xm"], z["xs"]
            net.ym, net.ys = z["ym"], z["ys"]
            net.xmin = z["xmin"] if "xmin" in z.files else None
            net.xmax = z["xmax"] if "xmax" in z.files else None
            for i in range(len(net.W)):
                net.W[i] = z[f"W{i}"]
                net.b[i] = z[f"b{i}"]
        return net


def _feature_row(state: Sequence[float], command_A: float, fc: dict, cfg: dict) -> np.ndarray:
    T, nO2, nH2, I_act, _, stress = map(float, state)
    TK = max(T + 273.15, 250.0)
    pO2 = max(0.05, nO2 * R * TK / cfg["O2_tank_volume_m3"] / 1e5)
    pH2 = max(0.05, nH2 * R * TK / cfg["H2_tank_volume_m3"] / 1e5)
    return np.array([
        I_act, float(command_A), T, pO2, pH2,
        float(fc.get("PV_available_W", 0.0)), float(fc.get("O2_demand_Lpm", cfg["hospital_demand_Lpm"])),
        float(fc.get("Tamb_C", 25.0)), float(fc.get("hour", 12.0)), stress,
    ], dtype=float)


_TRAIN_WEATHER_CACHE = {"df": None}


def validate_surrogate_contract(net: MLP, context: str) -> None:
    """Reject stale surrogate files before they are used for SIL or HIL."""
    expected = (len(FEATURES), 64, 64, len(TARGETS))
    if tuple(net.sizes) != expected:
        raise ValueError(
            f"{context}: frozen surrogate architecture {net.sizes} is stale/incompatible; "
            f"expected {expected}. Run `python m2_dashboard.py train` once."
        )
    arrays = [net.xm, net.xs, net.ym, net.ys, net.xmin, net.xmax, *net.W, *net.b]
    if any(a is None or not np.all(np.isfinite(a)) for a in arrays):
        raise ValueError(f"{context}: surrogate contains missing or non-finite parameters; retrain it")
    if net.ym.shape[-1] != len(TARGETS) or net.ys.shape[-1] != len(TARGETS):
        raise ValueError(f"{context}: surrogate target scaling does not match the {len(TARGETS)}-output model")


def validate_surrogate_artifact(net: MLP, context: str) -> str:
    """Validate model structure and, when present, match it to training metadata."""
    validate_surrogate_contract(net, context)
    actual_hash = sha256_file(SURROGATE)
    metrics_path = DATA / "surrogate_metrics.json"
    if metrics_path.exists():
        try:
            metadata = json.loads(metrics_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"{context}: surrogate_metrics.json is unreadable: {exc}") from exc
        recorded_hash = metadata.get("surrogate_sha256")
        if recorded_hash and str(recorded_hash).lower() != actual_hash.lower():
            raise ValueError(
                f"{context}: surrogate.npz does not match the model fingerprint recorded by training. "
                "Restore the trained artifact/metrics pair or retrain once."
            )
        recorded_arch = metadata.get("architecture")
        if recorded_arch and tuple(int(v) for v in recorded_arch) != tuple(net.sizes):
            raise ValueError(f"{context}: surrogate architecture does not match its training metrics metadata")
    return actual_hash


def _training_weather_inputs(cfg: dict, rng: np.random.Generator, t_s: float, offset_s: float):
    """Correlated PV/ambient disturbances based on NASA weather when available."""
    weather = _TRAIN_WEATHER_CACHE.get("df")
    if weather is not None:
        w = weather_at(weather, t_s + offset_s)
        tamb = float(np.clip(w["Tamb_C"] + rng.normal(0.0, 0.5),
                             cfg["surrogate_train_Tamb_min_C"], cfg["surrogate_train_Tamb_max_C"]))
        pv = float(np.clip(w["PV_available_W"] * rng.uniform(0.92, 1.08), 0.0, cfg["pvCapacity"]))
        return pv, tamb, float(w["hour"])
    hour = (t_s + offset_s) / 3600.0 % 24.0
    solar = max(0.0, math.sin(math.pi * ((hour - 6.0) % 24.0) / 12.0))
    cloud = float(np.clip(0.75 + 0.18 * math.sin(2.0 * math.pi * (t_s + offset_s) / 18000.0)
                          + rng.normal(0.0, 0.06), 0.25, 1.0))
    tamb = float(np.clip(22.5 + 8.5 * math.sin(2.0 * math.pi * (hour - 7.0) / 24.0)
                         + 1.5 * math.sin(2.0 * math.pi * (t_s + offset_s) / 86400.0),
                         cfg["surrogate_train_Tamb_min_C"], cfg["surrogate_train_Tamb_max_C"]))
    pv = float(cfg["pvCapacity"] * solar * cloud * cfg.get("pv_performance_ratio", 0.85))
    return pv, tamb, hour


def _make_training_row(state, command_A, fc, cfg, group_id, step_index=0):
    feature = _feature_row(state, command_A, fc, cfg)
    _, y = plant_step(state, command_A, fc, cfg, dt=cfg["sil_dt_s"])
    row = {name: float(value) for name, value in zip(FEATURES, feature)}
    row["run_id"] = group_id
    row["step_index"] = int(step_index)
    for name, value in zip(
        ("state_T_C", "state_nO2_mol", "state_nH2_mol", "state_I_A", "state_PV_filtered_W", "state_stress_idx"),
        np.asarray(state, dtype=float).reshape(6),
    ):
        row[name] = float(value)
    row.update({name: float(y[name]) for name in TARGETS})
    return row


def train(cfg: dict, n_traj: Optional[int] = None, steps: Optional[int] = None, epochs: Optional[int] = None) -> None:
    """Train the 10-64-64-4 MLP on propagated trajectories plus envelope samples.

    Entire source groups are held out. Trajectory samples are state-propagated
    through the shared plant; envelope samples provide targeted one-step coverage.
    """
    group_count = int(n_traj or cfg["train_trajectories"])
    steps = int(steps or cfg["train_steps_per_trajectory"])
    epochs = int(epochs or cfg["train_epochs"])
    if group_count < 6:
        raise ValueError("Use at least six groups for trajectory and envelope holdouts")
    frac = float(cfg["train_trajectory_fraction"])
    n_dynamic = min(max(int(round(group_count * frac)), 2), group_count - 2)
    n_envelope = group_count - n_dynamic
    print(f"[train] {n_dynamic} dynamic groups + {n_envelope} envelope groups; "
          f"{steps} samples/group at dt={cfg['sil_dt_s']:.1f} s")

    rng = np.random.default_rng(42)
    nO2_min = cfg["O2_reserve_Nm3"] * 1.01325e5 / (R * 273.15)
    nO2_max = cfg["pressure_max_bar"] * 1e5 * cfg["O2_tank_volume_m3"] / (R * 320.0)
    nH2_max = cfg["pressure_max_bar"] * 1e5 * cfg["H2_tank_volume_m3"] / (R * 320.0)
    t_lo, t_hi = float(cfg["surrogate_train_T_min_C"]), float(cfg["surrogate_train_T_max_C"])
    tamb_lo, tamb_hi = float(cfg["surrogate_train_Tamb_min_C"]), float(cfg["surrogate_train_Tamb_max_C"])
    dt = float(cfg["sil_dt_s"])
    _TRAIN_WEATHER_CACHE["df"] = load_weather(cfg) if WEATHER_CSV.exists() else None
    rows = []

    # A) State-propagated trajectories. Initial conditions stratify the full
    # thermal range. Piecewise current commands create heating/cooling segments.
    for group in range(n_dynamic):
        group_id = f"trajectory_{group:03d}"
        T0 = float(np.linspace(t_lo, t_hi, n_dynamic)[group])
        state = np.array([
            T0,
            float(rng.uniform(nO2_min, nO2_max)),
            float(rng.uniform(0.0, nH2_max)),
            float(rng.uniform(cfg["I_min_A"], cfg["I_max_A"])),
            float(rng.uniform(0.0, cfg["pvCapacity"])),
            float(rng.uniform(0.0, 0.15)),
        ], dtype=float)
        offset_s = float(group * 37 * 3600)
        command = float(rng.uniform(cfg["I_min_A"], cfg["I_max_A"]))
        command_until = 0
        for k in range(steps):
            t_s = k * dt
            pv, tamb, hour = _training_weather_inputs(cfg, rng, t_s, offset_s)
            if k >= command_until:
                command = float(rng.uniform(cfg["I_min_A"], cfg["I_max_A"]))
                command_until = k + int(rng.integers(6, 49))
            fc = {
                "PV_available_W": pv,
                "Tamb_C": tamb,
                "O2_demand_Lpm": o2_demand(hour, cfg, noise=float(rng.normal(0.0, 0.04))),
                "hour": hour,
            }
            _, trial_y = plant_step(state, command, fc, cfg, dt=dt)
            if trial_y["T_next_C"] > t_hi:
                # Keep generated training inputs within the declared 20..80 °C
                # coverage by reducing the candidate current, not by changing plant physics.
                safe_command = float(cfg["I_min_A"])
                for candidate in np.linspace(cfg["I_min_A"], command, 16):
                    _, candidate_y = plant_step(state, float(candidate), fc, cfg, dt=dt)
                    if candidate_y["T_next_C"] <= t_hi:
                        safe_command = float(candidate)
                command = safe_command
            row = _make_training_row(state, command, fc, cfg, group_id, step_index=k)
            rows.append(row)
            state, _ = plant_step(state, command, fc, cfg, dt=dt)
            if not (t_lo - 1e-8 <= state[0] <= t_hi + 1e-8):
                raise RuntimeError(f"Trajectory {group_id} escaped training temperature range: {state[0]:.3f} °C")

    # B) Targeted one-step envelope samples. Explicitly hit temperature and
    # ambient boundaries while varying electrical, pressure, storage and demand inputs.
    t_grid = np.linspace(t_lo, t_hi, 12)
    tamb_grid = np.linspace(tamb_lo, tamb_hi, 8)
    for group in range(n_envelope):
        group_id = f"envelope_{group:03d}"
        for k in range(steps):
            T = float(t_grid[(k + 3 * group) % len(t_grid)] if k % 4 == 0 else rng.uniform(t_lo, t_hi))
            Tamb = float(tamb_grid[(k + group) % len(tamb_grid)] if k % 5 == 0 else rng.uniform(tamb_lo, tamb_hi))
            TK = max(T + 273.15, 250.0)
            pO2 = float(rng.uniform(0.5, cfg["pressure_max_bar"]))
            pH2 = float(rng.uniform(0.05, cfg["pressure_max_bar"]))
            nO2 = pO2 * 1e5 * cfg["O2_tank_volume_m3"] / (R * TK)
            nH2 = pH2 * 1e5 * cfg["H2_tank_volume_m3"] / (R * TK)
            state = np.array([
                T, nO2, nH2,
                float(rng.uniform(cfg["I_min_A"], cfg["I_max_A"])),
                float(rng.uniform(0.0, cfg["pvCapacity"])),
                float(rng.uniform(0.0, 0.30)),
            ], dtype=float)
            command = float(rng.uniform(cfg["I_min_A"], cfg["I_max_A"]))
            fc = {
                "PV_available_W": float(rng.uniform(0.0, cfg["pvCapacity"])),
                "Tamb_C": Tamb,
                "O2_demand_Lpm": float(rng.uniform(20.0, 400.0)),
                "hour": float(rng.uniform(0.0, 24.0)),
            }
            rows.append(_make_training_row(state, command, fc, cfg, group_id, step_index=k))

    df = pd.DataFrame(rows)
    expected = group_count * steps
    if len(df) != expected:
        raise RuntimeError(f"Expected {expected} samples, generated {len(df)}")
    training_csv = DATA / "surrogate_training.csv"
    df.to_csv(training_csv, index=False)

    # Hold out complete groups separately for both data sources.
    split_rng = np.random.default_rng(2026)
    train_groups, test_groups = [], []
    for prefix, count in (("trajectory", n_dynamic), ("envelope", n_envelope)):
        ids = np.array([f"{prefix}_{i:03d}" for i in range(count)], dtype=object)
        split_rng.shuffle(ids)
        n_test = max(1, int(round(0.20 * count)))
        test_groups.extend(ids[:n_test].tolist())
        train_groups.extend(ids[n_test:].tolist())
    train_df = df[df["run_id"].isin(train_groups)]
    test_df = df[df["run_id"].isin(test_groups)]
    Xtr, Ytr = train_df[FEATURES].to_numpy(float), train_df[TARGETS].to_numpy(float)
    Xte, Yte = test_df[FEATURES].to_numpy(float), test_df[TARGETS].to_numpy(float)
    if not all(np.all(np.isfinite(a)) for a in (Xtr, Ytr, Xte, Yte)):
        raise RuntimeError("Surrogate training/held-out data contain NaN or infinite values")

    net = MLP(sizes=(len(FEATURES), 64, 64, len(TARGETS))).fit(
        Xtr, Ytr, epochs=epochs,
        lr=float(cfg["train_learning_rate"]), batch=int(cfg["train_batch"]),
    )
    Yp = net.predict(Xte)
    if not np.all(np.isfinite(Yp)):
        raise RuntimeError("Newly trained surrogate produced NaN/Inf on held-out one-step inputs")
    rmse = np.sqrt(np.mean((Yp - Yte) ** 2, axis=0))
    output_scale = np.maximum(np.ptp(Yte, axis=0), 1e-12)
    nrmse_pct = 100.0 * rmse / output_scale
    denom = np.sum((Yte - Yte.mean(axis=0)) ** 2, axis=0)
    r2 = np.where(denom > 1e-12,
                  1.0 - np.sum((Yp - Yte) ** 2, axis=0) / np.maximum(denom, 1e-12), 0.0)
    metrics = {
        "architecture": list(net.sizes), "sample_time_s": dt,
        "features": FEATURES, "targets": TARGETS,
        "training_groups": sorted(train_groups), "held_out_groups": sorted(test_groups),
        "training_group_types": {
            "trajectory": sum(g.startswith("trajectory_") for g in train_groups),
            "envelope": sum(g.startswith("envelope_") for g in train_groups),
        },
        "held_out_group_types": {
            "trajectory": sum(g.startswith("trajectory_") for g in test_groups),
            "envelope": sum(g.startswith("envelope_") for g in test_groups),
        },
        "train_samples": int(len(train_df)), "test_samples": int(len(test_df)),
        "data_generation": "mixed_state_propagated_trajectories_and_targeted_envelope_samples",
        "training_input_ranges": {name: [float(train_df[name].min()), float(train_df[name].max())] for name in FEATURES},
        "stack_temperature_training_range_C": [t_lo, t_hi],
        "ambient_temperature_training_range_C": [tamb_lo, tamb_hi],
        "normal_operation_minimum_temperature_C": float(cfg["min_operating_temperature_C"]),
        "rmse": {k: float(v) for k, v in zip(TARGETS, rmse)},
        "nrmse_percent_of_heldout_range": {k: float(v) for k, v in zip(TARGETS, nrmse_pct)},
        "r2": {k: float(v) for k, v in zip(TARGETS, r2)},
        "high_current_error_I_ge_1000_A": {},
    }
    high_mask = Xte[:, FEATURES.index("I_stack_A")] >= 1000.0
    if np.any(high_mask):
        high_rmse = np.sqrt(np.mean((Yp[high_mask] - Yte[high_mask]) ** 2, axis=0))
        metrics["high_current_error_I_ge_1000_A"] = {k: float(v) for k, v in zip(TARGETS, high_rmse)}
    else:
        metrics["high_current_error_I_ge_1000_A"] = None
    # Recursive held-out trajectory error is evaluated after the controller
    # classes have been defined (train() is invoked only after module loading).
    rollout_metrics = _evaluate_surrogate_rollouts(net, test_df, cfg)
    if rollout_metrics.get("status") != "ok":
        raise RuntimeError(f"Recursive surrogate evaluation did not complete: {rollout_metrics}")
    for lead, values in rollout_metrics.get("by_lead_step", {}).items():
        required = ("T_C_RMSE", "O2_buffer_Nm3_RMSE", "H2_buffer_Nm3_RMSE")
        if any(k not in values or not np.isfinite(values[k]) for k in required):
            raise RuntimeError(f"Recursive rollout lead {lead} has missing/non-finite state errors: {values}")
    metrics["recursive_rollout_errors"] = rollout_metrics
    metrics["step_size_convergence"] = _step_size_convergence(test_df, cfg)
    print(f"[train] groups train={len(train_groups)} test={len(test_groups)}; samples train={len(train_df)} test={len(test_df)}")
    for lead, values in rollout_metrics.get("by_lead_step", {}).items():
        print(f"[train] recursive rollout lead {lead} steps: T RMSE={values['T_C_RMSE']:.4g} °C; "
              f"O2 buffer RMSE={values['O2_buffer_Nm3_RMSE']:.4g} Nm³; "
              f"H2 buffer RMSE={values['H2_buffer_Nm3_RMSE']:.4g} Nm³")
    print(f"[train] exact-plant fallback steps during recursive evaluation={rollout_metrics['surrogate_fallback_count_during_rollout']}")
    print("[train] coarse-step vs sub-stepped plant convergence saved in surrogate_metrics.json")
    print(f"[train] stack training coverage={train_df['T_stack_C'].min():.2f}..{train_df['T_stack_C'].max():.2f} °C; "
          f"ambient={train_df['Tamb_C'].min():.2f}..{train_df['Tamb_C'].max():.2f} °C")
    for i, target in enumerate(TARGETS):
        print(f"  {target:22s} RMSE={rmse[i]:.5g}  NRMSE={nrmse_pct[i]:.3f}%  R²={r2[i]:.5f}")

    # This is a local Python timing diagnostic only, not HIL or Simulink timing.
    nO2_probe = float(cfg["initial_O2_inventory_Nm3"]) * 1.01325e5 / (R * 273.15)
    probe_state = np.array([35.0, nO2_probe, 0.1*nH2_max, 500.0, 0.0, 0.01])
    probe_fc = {"PV_available_W": 80000.0, "Tamb_C": 12.0,
                "O2_demand_Lpm": cfg["hospital_demand_Lpm"], "hour": 6.0}
    probe = _feature_row(probe_state, 500.0, probe_fc, cfg)
    n_bench = 1000
    t0 = time.perf_counter()
    for _ in range(n_bench): net.predict(probe)
    inference_ms = (time.perf_counter() - t0) / n_bench * 1000.0
    t0 = time.perf_counter()
    for _ in range(n_bench): plant_step(probe_state, 500.0, probe_fc, cfg, dt=dt)
    plant_ms = (time.perf_counter() - t0) / n_bench * 1000.0
    hybrid = ProposedEMPC(cfg, net)
    candidate_count = int(cfg["upper_candidate_count"])
    horizon = int(cfg["MPC_horizon_steps"])
    probe_states = np.repeat(probe_state.reshape(1, 6), candidate_count, axis=0)
    probe_commands = np.linspace(cfg["I_min_A"], cfg["I_max_A"], candidate_count)
    batch_repeats = max(100, n_bench // 5)
    t0 = time.perf_counter()
    for _ in range(batch_repeats):
        hybrid._surrogate_step_batch(probe_states, probe_commands, probe_fc, cfg)
    hybrid_batch_ms = (time.perf_counter() - t0) / batch_repeats * 1000.0
    # Both estimates use the same candidate count and horizon. The hybrid
    # estimate charges exact plant calls for k=0 and one batched surrogate
    # stage for each later k; offline network training is excluded.
    full_rollout = candidate_count * horizon * plant_ms
    hybrid_rollout = candidate_count * plant_ms + max(0, horizon - 1) * hybrid_batch_ms
    print(f"[train] plant step={plant_ms:.4f} ms; single MLP inference={inference_ms:.4f} ms; "
          f"batched hybrid stage ({candidate_count} candidates)={hybrid_batch_ms:.4f} ms")
    print(f"[train] estimated Python rollout kernels: full={full_rollout:.3f} ms; "
          f"hybrid={hybrid_rollout:.3f} ms; ratio={full_rollout/max(hybrid_rollout,1e-12):.2f}x")
    print("[train] timing excludes offline network training and is not a Simulink/HIL timing claim.")
    validate_surrogate_contract(net, "newly trained model")
    net.save(SURROGATE)
    surrogate_hash = sha256_file(SURROGATE)
    metrics["surrogate_artifact"] = SURROGATE.name
    metrics["surrogate_sha256"] = surrogate_hash
    metrics["training_dataset_artifact"] = training_csv.name
    metrics["training_dataset_sha256"] = sha256_file(training_csv)
    metrics["architecture"] = list(net.sizes)
    (DATA / "surrogate_metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False), encoding="utf-8")
    print(f"[train] frozen surrogate saved: {SURROGATE} | architecture={net.sizes} | SHA-256={surrogate_hash}")
    print(f"[train] dataset saved: {training_csv}")


def evaluate_surrogate(cfg: dict) -> None:
    """Recompute model metrics from the saved frozen network and its training CSV.

    This command does not fit, update, or overwrite surrogate.npz. It refreshes
    recursive rollout metrics after an evaluator/reporting correction.
    """
    if not SURROGATE.exists():
        raise FileNotFoundError(f"Missing {SURROGATE}; train the surrogate before evaluation")
    training_csv = DATA / "surrogate_training.csv"
    if not training_csv.exists():
        raise FileNotFoundError(
            f"Missing {training_csv}; evaluation-only mode needs the training CSV created by the original train run"
        )
    net = MLP.load(SURROGATE)
    model_hash = validate_surrogate_artifact(net, "surrogate evaluation")
    df = pd.read_csv(training_csv)
    required_columns = set(FEATURES + TARGETS + [
        "run_id", "step_index", "state_T_C", "state_nO2_mol", "state_nH2_mol",
        "state_I_A", "state_PV_filtered_W", "state_stress_idx"
    ])
    missing = sorted(required_columns - set(df.columns))
    if missing:
        raise RuntimeError(f"Training CSV is missing columns required for evaluation: {missing}")
    numeric = df[FEATURES + TARGETS].to_numpy(dtype=float)
    if not np.all(np.isfinite(numeric)):
        raise RuntimeError("Training CSV contains non-finite feature/target values; evaluation stopped")

    metrics_path = DATA / "surrogate_metrics.json"
    metrics = {}
    if metrics_path.exists():
        try:
            metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Cannot read existing surrogate metrics metadata: {exc}") from exc
    stored_dataset_hash = metrics.get("training_dataset_sha256")
    if stored_dataset_hash and str(stored_dataset_hash).lower() != sha256_file(training_csv).lower():
        raise RuntimeError(
            "surrogate_training.csv does not match the dataset fingerprint recorded by training; "
            "restore the matching model/dataset pair rather than evaluating mixed artifacts"
        )
    stored_groups = metrics.get("held_out_groups")
    if stored_groups:
        test_groups = [str(g) for g in stored_groups]
        if not set(test_groups).issubset(set(df["run_id"].astype(str).unique())):
            raise RuntimeError("Held-out group identifiers in metrics do not match surrogate_training.csv")
    else:
        # Recreate the deterministic split used in train() for older metadata.
        trajectory_ids = sorted(g for g in df["run_id"].astype(str).unique() if g.startswith("trajectory_"))
        envelope_ids = sorted(g for g in df["run_id"].astype(str).unique() if g.startswith("envelope_"))
        split_rng = np.random.default_rng(2026)
        test_groups = []
        for ids in (trajectory_ids, envelope_ids):
            shuffled = np.asarray(ids, dtype=object)
            split_rng.shuffle(shuffled)
            n_test = max(1, int(round(0.20 * len(ids))))
            test_groups.extend(str(x) for x in shuffled[:n_test])
    all_groups = set(df["run_id"].astype(str).unique())
    test_groups = sorted(set(test_groups))
    if not test_groups or not set(test_groups).issubset(all_groups):
        raise RuntimeError("Invalid held-out groups in surrogate training dataset")
    test_df = df.loc[df["run_id"].astype(str).isin(test_groups)].copy()
    train_df = df.loc[~df["run_id"].astype(str).isin(test_groups)].copy()
    if test_df.empty or train_df.empty:
        raise RuntimeError("The saved training CSV does not contain both training and held-out groups")

    Xte = test_df[FEATURES].to_numpy(dtype=float)
    Yte = test_df[TARGETS].to_numpy(dtype=float)
    Yp = net.predict(Xte)
    if not np.all(np.isfinite(Yp)):
        raise RuntimeError("Frozen surrogate produced non-finite held-out predictions")
    rmse = np.sqrt(np.mean((Yp - Yte) ** 2, axis=0))
    scale = np.maximum(np.ptp(Yte, axis=0), 1e-12)
    nrmse = 100.0 * rmse / scale
    denom = np.sum((Yte - Yte.mean(axis=0)) ** 2, axis=0)
    r2 = np.where(denom > 1e-12,
                  1.0 - np.sum((Yp - Yte) ** 2, axis=0) / np.maximum(denom, 1e-12), 0.0)
    high_mask = test_df["I_stack_A"].to_numpy(dtype=float) >= 1000.0
    high_error = None
    if np.any(high_mask):
        high_rmse = np.sqrt(np.mean((Yp[high_mask] - Yte[high_mask]) ** 2, axis=0))
        high_error = {k: float(v) for k, v in zip(TARGETS, high_rmse)}

    rollout = _evaluate_surrogate_rollouts(net, test_df, cfg)
    if rollout.get("status") != "ok":
        raise RuntimeError(f"Recursive rollout evaluation did not complete: {rollout}")
    required_rollout = ("T_C_RMSE", "O2_buffer_Nm3_RMSE", "H2_buffer_Nm3_RMSE")
    for lead, values in rollout.get("by_lead_step", {}).items():
        if any(k not in values or not np.isfinite(values[k]) for k in required_rollout):
            raise RuntimeError(f"Recursive rollout lead {lead} contains missing/non-finite metrics: {values}")

    metrics.update({
        "architecture": list(net.sizes), "features": FEATURES, "targets": TARGETS,
        "training_groups": sorted(all_groups - set(test_groups)),
        "held_out_groups": test_groups,
        "train_samples": int(len(train_df)), "test_samples": int(len(test_df)),
        "rmse": {k: float(v) for k, v in zip(TARGETS, rmse)},
        "nrmse_percent_of_heldout_range": {k: float(v) for k, v in zip(TARGETS, nrmse)},
        "r2": {k: float(v) for k, v in zip(TARGETS, r2)},
        "high_current_error_I_ge_1000_A": high_error,
        "recursive_rollout_errors": rollout,
        "step_size_convergence": _step_size_convergence(test_df, cfg),
        "surrogate_artifact": SURROGATE.name, "surrogate_sha256": model_hash,
        "training_dataset_artifact": training_csv.name,
        "training_dataset_sha256": sha256_file(training_csv),
        "evaluation_mode": "frozen_model_no_retraining",
    })
    metrics_path.write_text(json.dumps(metrics, indent=2, allow_nan=False), encoding="utf-8")
    print(f"[evaluate-surrogate] frozen model unchanged: {SURROGATE} | SHA-256={model_hash}")
    print(f"[evaluate-surrogate] held-out samples={len(test_df)}; train samples={len(train_df)}; groups={len(test_groups)}")
    for lead, values in rollout["by_lead_step"].items():
        print(f"[evaluate-surrogate] recursive lead {lead}: T RMSE={values['T_C_RMSE']:.5g} °C; "
              f"O2 buffer RMSE={values['O2_buffer_Nm3_RMSE']:.5g} Nm³; "
              f"H2 buffer RMSE={values['H2_buffer_Nm3_RMSE']:.5g} Nm³")
    print(f"[evaluate-surrogate] exact-plant fallback steps during rollout={rollout['surrogate_fallback_count_during_rollout']}; "
          "inspect this count before describing the test as surrogate-only recursion.")
    print("[evaluate-surrogate] no training, optimizer update, or surrogate.npz overwrite was performed.")
    print(f"[evaluate-surrogate] metrics saved: {metrics_path}")


# ==================== CONTROL LAYERS ====================
class CurrentMPCTracker:
    """Five-step current tracker matching the Arduino's applied-command ramp limit."""
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.applied_command_A = float(cfg["I_min_A"])

    def reset(self, current_A: Optional[float] = None) -> None:
        self.applied_command_A = float(self.cfg["I_min_A"] if current_A is None else current_A)

    def command(self, I_plant: float, I_ref: float, dt: float) -> float:
        if I_ref <= 0.0:
            self.applied_command_A = 0.0
            return 0.0
        a = math.exp(-dt / max(self.cfg["tau_I_s"], 1e-3))
        lo = max(self.cfg["I_min_A"], self.applied_command_A - float(self.cfg["lower_move_A"]))
        hi = min(self.cfg["I_max_A"], self.applied_command_A + float(self.cfg["lower_move_A"]))
        grid = max(1.0, float(self.cfg["lower_grid_A"]))
        candidates = np.arange(lo, hi + 0.5 * grid, grid)
        if len(candidates) == 0:
            candidates = np.array([float(np.clip(I_ref, self.cfg["I_min_A"], self.cfg["I_max_A"]))])
        best_u = float(candidates[0])
        best_cost = float("inf")
        horizon = int(self.cfg["lower_mpc_horizon"])
        for u in candidates:
            x = float(I_plant)
            cost = 0.0
            for _ in range(horizon):
                x = a * x + (1.0 - a) * float(u)
                cost += 10.0 * (x - I_ref) ** 2 + 0.08 * (u - self.applied_command_A) ** 2
            cost += 40.0 * (x - I_ref) ** 2
            if cost < best_cost:
                best_cost, best_u = cost, float(u)
        self.applied_command_A = float(np.clip(best_u, self.cfg["I_min_A"], self.cfg["I_max_A"]))
        return self.applied_command_A


class PID:
    name = "PID"
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.integral = 0.0
        self.applied_command_A = float(cfg["I_min_A"])

    def reset(self) -> None:
        self.integral = 0.0
        self.applied_command_A = float(self.cfg["I_min_A"])

    def upper(self, state, w, cfg):
        return demand_current(cfg, w["O2_demand_Lpm"])

    def lower(self, I_plant, I_ref, dt):
        if I_ref <= 0.0:
            self.integral = 0.0
            self.applied_command_A = 0.0
            return 0.0
        error = I_ref - I_plant
        self.integral = float(np.clip(self.integral + error * dt, -5000.0, 5000.0))
        raw = I_plant + 1.5 * error + 0.05 * self.integral
        # All four SIL variants share the same physical actuator slew limit.
        delta = float(self.cfg["lower_move_A"])
        limited = float(np.clip(raw, self.applied_command_A - delta, self.applied_command_A + delta))
        self.applied_command_A = float(np.clip(limited, self.cfg["I_min_A"], self.cfg["I_max_A"]))
        return self.applied_command_A


class SingleMPC:
    name = "SINGLE_MPC"
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.tracker = CurrentMPCTracker(cfg)

    def upper(self, state, w, cfg):
        return demand_current(cfg, w["O2_demand_Lpm"])

    def lower(self, I_plant, I_ref, dt):
        return self.tracker.command(I_plant, I_ref, dt)


class BufferAwareRule:
    """Demand/buffer hysteresis baseline with a bounded PV-following bias."""
    name = "BUFFER_RULE"

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.tracker = CurrentMPCTracker(cfg)
        self.charge_mode = False

    def upper(self, state, w, cfg):
        o2_buffer = float(state[1]) * R * 273.15 / 1.01325e5
        reserve = float(cfg["O2_reserve_Nm3"])
        low = 1.10 * reserve
        high = 1.45 * reserve
        if o2_buffer < low:
            self.charge_mode = True
        elif o2_buffer > high:
            self.charge_mode = False
        demand_ref = demand_current(cfg, float(w["O2_demand_Lpm"]))
        if self.charge_mode:
            demand_ref = min(float(cfg["I_max_A"]), 1.25 * demand_ref)
        # PV following is allowed only while oxygen inventory remains below its
        # upper hysteresis threshold, avoiding deliberate tank overfill.
        pv_fraction = float(np.clip(w.get("PV_available_W", 0.0) / max(cfg["pvCapacity"], 1.0), 0.0, 1.0))
        if o2_buffer < high and pv_fraction > 0:
            pv_bias = 0.10 * pv_fraction * (float(cfg["I_max_A"]) - demand_ref)
            demand_ref += max(0.0, pv_bias)
        return float(np.clip(demand_ref, cfg["I_min_A"], cfg["I_max_A"]))

    def lower(self, I_plant, I_ref, dt):
        return self.tracker.command(I_plant, I_ref, dt)


def _constraints_ok(y: dict, x_next: np.ndarray, cfg: dict, x_previous: Optional[np.ndarray] = None) -> bool:
    # Shared 50 °C normal-operation floor for both economic controllers.
    # Below the floor, only non-cooling recovery steps are admissible; after
    # reaching it, the prediction must remain at or above the normal minimum.
    t_min = float(cfg["min_operating_temperature_C"])
    if x_previous is None:
        thermal_ok = y["T_next_C"] >= t_min - 1e-6
    else:
        t_previous = float(np.asarray(x_previous, dtype=float)[0])
        thermal_ok = (y["T_next_C"] >= t_min - 1e-6) if t_previous >= t_min - 1e-6 \
                     else (y["T_next_C"] >= t_previous - 1e-6)
    return bool(
        thermal_ok
        and y["V_cell_V"] <= cfg["max_voltage_V"]
        and y["T_next_C"] <= cfg["max_temperature_C"]
        and y["P_O2_tank_bar"] <= cfg["pressure_max_bar"] + 1e-6
        and y["P_H2_next_bar"] <= cfg["pressure_max_bar"] + 1e-6
        and y["purity_next_pct"] >= cfg["purity_min_pct"]
        and y["H2_in_O2_ppm"] <= cfg["H2_in_O2_limit_ppm"] + 1e-6
        and y["P_grid_W"] <= cfg["P_grid_max_W"] + 1e-6
        and y["O2_buffer_Nm3"] >= cfg["O2_reserve_Nm3"]
        and y["O2_unmet_L_step"] <= 1e-6
        and np.all(np.isfinite(x_next))
    )


def _economic_cost(y: dict, cfg: dict, tariff: float, dt: float, stress_previous: float) -> float:
    grid_cost = max(0.0, y["P_grid_W"]) * (dt / 3600.0) * tariff / 1000.0
    unmet_cost = float(cfg["w_o2_unmet_KES_per_L"]) * max(0.0, y["O2_unmet_L_step"])
    reserve_shortfall = max(0.0, 1.2 * cfg["O2_reserve_Nm3"] - y["O2_buffer_Nm3"])
    reserve_cost = float(cfg["w_reserve_KES_per_Nm3_sq"]) * reserve_shortfall ** 2
    # Credit only hydrogen that is explicitly exported against the configured
    # exogenous offtake request. Production and stored inventory are not sales.
    hydrogen_credit = (float(cfg["H2_revenue_weight"]) * float(cfg["H2_price_KES_kg"])
                       * float(y.get("H2_export_kg_s", 0.0)) * dt)
    stress_increment = max(0.0, y["stress_idx_next"] - float(stress_previous))
    return (
        float(cfg["w_grid"]) * grid_cost
        + unmet_cost
        + reserve_cost
        + float(cfg["w_stress"]) * stress_increment
        - hydrogen_credit
    )


class EconomicMPCBase:
    name = "EMPC_BASE"

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.I_ref = float(cfg["I_min_A"])
        self.tracker = CurrentMPCTracker(cfg)
        self.last_feasible = True

    def _step(self, x, u, fc, cfg, stage):
        raise NotImplementedError

    def upper(self, state, w, cfg):
        horizon = int(cfg["MPC_horizon_steps"])
        candidates = np.linspace(cfg["I_min_A"], cfg["I_max_A"], int(cfg["upper_candidate_count"]))
        forecast = w.get("forecast", [w] * horizon)
        if len(forecast) < horizon:
            forecast = list(forecast) + [forecast[-1]] * (horizon - len(forecast))
        best_u = None
        best_cost = float("inf")
        feasible_count = 0
        for u in candidates:
            x_sim = np.asarray(state, dtype=float).copy()
            total_cost = 0.0
            feasible = True
            for k in range(horizon):
                fc = forecast[k]
                x_next, y = self._step(x_sim, float(u), fc, cfg, k)
                if not _constraints_ok(y, x_next, cfg, x_previous=x_sim):
                    feasible = False
                    break
                total_cost += _economic_cost(
                    y, cfg, float(fc["tariff_KES_kWh"]), float(cfg["sil_dt_s"]), stress_previous=float(x_sim[5])
                )
                x_sim = x_next
            if feasible:
                feasible_count += 1
                o2_start = float(state[1]) * R * 273.15 / 1.01325e5
                h2_start = float(state[2]) * R * 273.15 / 1.01325e5
                o2_end = float(x_sim[1]) * R * 273.15 / 1.01325e5
                h2_end = float(x_sim[2]) * R * 273.15 / 1.01325e5
                total_cost += float(cfg["w_terminal_O2_KES_per_Nm3_sq"]) * (o2_end - o2_start) ** 2
                total_cost += float(cfg["w_terminal_H2_KES_per_Nm3_sq"]) * (h2_end - h2_start) ** 2
                if total_cost < best_cost:
                    best_cost, best_u = total_cost, float(u)
        self.last_feasible = best_u is not None
        if best_u is None:
            # No candidate is silently labelled feasible. Conservative demand
            # tracking is used as a fallback and the infeasibility is logged.
            best_u = demand_current(cfg, float(w["O2_demand_Lpm"]))
        self.I_ref = float(np.clip(best_u, cfg["I_min_A"], cfg["I_max_A"]))
        self.last_feasible_candidates = feasible_count
        return self.I_ref

    def lower(self, I_plant, I_ref, dt):
        return self.tracker.command(I_plant, I_ref, dt)


class FullEMPC(EconomicMPCBase):
    name = "FULL_MODEL_EMPC"

    def _step(self, x, u, fc, cfg, stage):
        return plant_step(x, u, fc, cfg, dt=cfg["sil_dt_s"])


class ProposedEMPC(EconomicMPCBase):
    """Physics-consistent hybrid rollout: exact first step, surrogate thereafter."""
    name = "PROPOSED"

    def __init__(self, cfg: dict, surrogate: MLP):
        super().__init__(cfg)
        self.surr = surrogate
        self.surrogate_fallback_count = 0

    def _surrogate_step(self, state, command_A, fc, cfg):
        """Single-row wrapper around the batched hybrid predictor."""
        xn, rows = self._surrogate_step_batch(
            np.asarray(state, dtype=float).reshape(1, 6),
            np.asarray([command_A], dtype=float), fc, cfg
        )
        return xn[0], rows[0]

    def _surrogate_step_batch(self, states, commands_A, fc, cfg):
        """Batched nonlinear-state prediction with exact gas/power accounting."""
        X = np.asarray(states, dtype=float)
        U = np.asarray(commands_A, dtype=float).reshape(-1)
        if X.ndim != 2 or X.shape[1] != 6 or len(X) != len(U):
            raise ValueError("states must be (n,6) and commands_A must have n entries")
        n = len(U)
        if n == 0:
            return np.empty((0, 6), dtype=float), []

        T, nO2, nH2, I_act, PVf, stress = X.T
        TK = np.maximum(T + 273.15, 250.0)
        pmax = float(cfg["pressure_max_bar"])
        pO2 = np.minimum(pmax, np.maximum(0.05, nO2 * R * TK / max(cfg["O2_tank_volume_m3"], 1e-6) / 1e5))
        pH2 = np.minimum(pmax, np.maximum(0.05, nH2 * R * TK / max(cfg["H2_tank_volume_m3"], 1e-6) / 1e5))
        features = np.column_stack((
            I_act, U, T, pO2, pH2,
            np.full(n, float(fc.get("PV_available_W", 0.0))),
            np.full(n, float(fc.get("O2_demand_Lpm", cfg["hospital_demand_Lpm"]))),
            np.full(n, float(fc.get("Tamb_C", 25.0))),
            np.full(n, float(fc.get("hour", 12.0))), stress,
        ))
        margin = float(cfg["surrogate_envelope_margin"])
        if self.surr.xmin is not None and self.surr.xmax is not None:
            span = np.maximum(self.surr.xmax - self.surr.xmin, 1e-9)
            valid = np.all((features >= self.surr.xmin - margin * span)
                           & (features <= self.surr.xmax + margin * span), axis=1)
        else:
            valid = np.all((features >= self.surr.xm - 8.0 * self.surr.xs)
                           & (features <= self.surr.xm + 8.0 * self.surr.xs), axis=1)

        x_next = np.empty((n, 6), dtype=float)
        y_rows = [None] * n
        valid_idx = np.flatnonzero(valid)
        if valid_idx.size:
            pred = self.surr.predict(features[valid_idx])
            pV = pred[:, TARGETS.index("V_cell_V")]
            pT = pred[:, TARGETS.index("T_next_C")]
            pPurity = pred[:, TARGETS.index("purity_next_pct")]
            pStress = pred[:, TARGETS.index("stress_idx_next")]

            dt = float(cfg["sil_dt_s"])
            N = float(cfg["N_cells"])
            A_cm2 = float(cfg["A_cell_cm2"])
            V_O2 = float(cfg["O2_tank_volume_m3"])
            V_H2 = float(cfg["H2_tank_volume_m3"])
            In_prev = I_act[valid_idx]
            Uv = np.clip(U[valid_idx], cfg["I_min_A"], cfg["I_max_A"])
            aI = math.exp(-dt / max(cfg["tau_I_s"], 1e-3))
            In = aI * In_prev + (1.0 - aI) * Uv
            Jcm = In / max(A_cm2, 1e-9)
            V = np.maximum(0.5, pV)
            Tn = np.clip(pT, 20.0, 120.0)
            TKn = np.maximum(Tn + 273.15, 250.0)

            # Exact Faraday production, compressor/BoP loads and gas inventory.
            ndO2 = N * In / (4.0 * F) * cfg["faradayEff"]
            ndH2 = N * In / (2.0 * F) * cfg["faradayEff"]
            mO2 = ndO2 * 0.032
            mH2 = ndH2 * 0.002016
            Pstack = N * V * In
            Pbop = (cfg["BOP_dryer_power_frac"] * Pstack
                    + cfg["BOP_O2_comp_kWh_kg"] * mO2 * 3.6e6
                    + cfg["BOP_H2_comp_kWh_kg"] * mH2 * 3.6e6)
            Psystem = Pstack + Pbop

            demand = max(0.0, float(fc.get("O2_demand_Lpm", cfg["hospital_demand_Lpm"])))
            O2_available = np.maximum(0.0, nO2[valid_idx]) + ndO2 * dt
            O2_requested = demand / 60.0 / 22.414 * dt
            O2_served = np.minimum(O2_available, O2_requested)
            O2_supply = O2_served / dt * 60.0 * 22.414
            O2_unmet = np.maximum(0.0, (demand - O2_supply) * dt / 60.0)
            nO2_pre_cap = np.maximum(0.0, O2_available - O2_served)

            h2_export_demand = max(0.0, float(fc.get(
                "H2_export_demand_kg_s", fc.get("H2_draw_kg_s", cfg.get("H2_export_demand_kg_s", 0.0)))))
            H2_available_kg = np.maximum(0.0, nH2[valid_idx]) * 0.002016 + mH2 * dt
            H2_export_step = np.minimum(H2_available_kg, h2_export_demand * dt)
            H2_after_export = np.maximum(0.0, H2_available_kg - H2_export_step)
            nH2_pre_cap = H2_after_export / 0.002016

            nO2_cap = cfg["pressure_max_bar"] * 1e5 * V_O2 / (R * TKn)
            nH2_cap = cfg["pressure_max_bar"] * 1e5 * V_H2 / (R * TKn)
            O2_vent_mol = np.maximum(0.0, nO2_pre_cap - nO2_cap)
            H2_vent_kg = np.maximum(0.0, (nH2_pre_cap - nH2_cap) * 0.002016)
            nO2n = np.minimum(nO2_pre_cap, nO2_cap)
            nH2n = np.minimum(nH2_pre_cap, nH2_cap)
            pO2tank = np.minimum(float(cfg["pressure_max_bar"]),
                                 np.maximum(0.05, nO2n * R * TKn / V_O2 / 1e5))
            pH2tank = np.minimum(float(cfg["pressure_max_bar"]),
                                 np.maximum(0.05, nH2n * R * TKn / V_H2 / 1e5))
            pO2product = np.maximum(0.05, pO2tank - cfg["BOP_sep_dp_bar"])

            aPV = math.exp(-dt / max(cfg["tau_PV_s"], 1e-3))
            PVav = float(np.clip(fc.get("PV_available_W", 0.0), 0.0, cfg["pvCapacity"]))
            PVn = aPV * PVf[valid_idx] + (1.0 - aPV) * PVav
            PPV = np.minimum(Psystem, np.maximum(0.0, PVn))
            Pgrid = np.maximum(0.0, Psystem - PPV)
            crossover = (cfg["k_cross_0"] + cfg["k_cross_1"] * pH2tank) / (Jcm + 0.1)
            purity = np.clip(pPurity, 90.0, 100.0)
            stressn = np.maximum(stress[valid_idx], pStress)
            eff = np.where(In > 0.0, 100.0 * (ndH2 * 241.8e3) / np.maximum(Pstack, 1e-3), 0.0)
            system_eff = np.where(In > 0.0, 100.0 * ndH2 * 120e6 / np.maximum(Psystem, 1e-3), 0.0)

            x_next[valid_idx] = np.column_stack((Tn, nO2n, nH2n, In, PVn, stressn))
            arrays = {
                "O2_flow_Lpm": O2_supply, "O2_prod_Lpm": ndO2 * 60.0 * 22.414,
                "O2_unmet_L_step": O2_unmet, "O2_vented_L_step": O2_vent_mol * 22.414,
                "H2_kg_s": mH2, "H2_produced_kg_step": mH2 * dt,
                "H2_export_kg_s": H2_export_step / dt, "H2_export_kg_step": H2_export_step,
                "H2_vent_kg_s": H2_vent_kg / dt, "H2_vent_kg_step": H2_vent_kg,
                "V_cell_V": V, "P_stack_W": Pstack, "P_BoP_W": Pbop,
                "P_system_W": Psystem, "P_grid_W": Pgrid, "P_PV_W": PPV,
                "PV_available_W": np.full(len(valid_idx), PVav), "PV_filtered_W": PVn,
                "grid_overlimit_W": np.maximum(0.0, Pgrid - cfg["P_grid_max_W"]),
                "T_next_C": Tn, "P_O2_next_bar": pO2product, "P_H2_next_bar": pH2tank,
                "P_O2_tank_bar": pO2tank, "purity_next_pct": purity,
                "H2_in_O2_ppm": crossover, "crossover_ppm_model": crossover,
                "stress_idx_next": stressn, "efficiency_pct": eff,
                "system_efficiency_pct": system_eff,
                "O2_buffer_Nm3": nO2n * R * 273.15 / 1.01325e5,
                "H2_buffer_Nm3": nH2n * R * 273.15 / 1.01325e5,
                "I_stack_A": In, "T_stack_C": Tn, "P_O2_bar": pO2product, "P_H2_bar": pH2tank,
            }
            for local_i, original_i in enumerate(valid_idx):
                y_rows[original_i] = {key: float(values[local_i]) for key, values in arrays.items()}
                y_rows[original_i]["shutdown"] = False

        for i in np.flatnonzero(~valid):
            self.surrogate_fallback_count += 1
            xn, yi = plant_step(X[i], float(U[i]), fc, cfg, dt=cfg["sil_dt_s"])
            x_next[i] = xn
            y_rows[i] = yi
        return x_next, y_rows

    def upper(self, state, w, cfg):
        """Evaluate the same 17 x 6 candidate/horizon problem with batched MLP calls."""
        horizon = int(cfg["MPC_horizon_steps"])
        candidates = np.linspace(cfg["I_min_A"], cfg["I_max_A"], int(cfg["upper_candidate_count"]))
        forecast = w.get("forecast", [w] * horizon)
        if len(forecast) < horizon:
            forecast = list(forecast) + [forecast[-1]] * (horizon - len(forecast))

        candidate_count = len(candidates)
        states = np.repeat(np.asarray(state, dtype=float).reshape(1, 6), candidate_count, axis=0)
        costs = np.zeros(candidate_count, dtype=float)
        feasible = np.ones(candidate_count, dtype=bool)

        for k in range(horizon):
            active = np.flatnonzero(feasible)
            if active.size == 0:
                break
            fc = forecast[k]
            if k == 0:
                # Preserve exact first-step evaluation for every candidate.
                next_states = np.empty((len(active), 6), dtype=float)
                y_rows = []
                for j, idx in enumerate(active):
                    xn, yi = plant_step(states[idx], float(candidates[idx]), fc, cfg, dt=cfg["sil_dt_s"])
                    next_states[j] = xn
                    y_rows.append(yi)
            else:
                next_states, y_rows = self._surrogate_step_batch(
                    states[active], candidates[active], fc, cfg
                )

            tariff = float(fc["tariff_KES_kWh"])
            dt = float(cfg["sil_dt_s"])
            for j, idx in enumerate(active):
                yi = y_rows[j]
                if not _constraints_ok(yi, next_states[j], cfg, x_previous=states[idx]):
                    feasible[idx] = False
                    continue
                costs[idx] += _economic_cost(
                    yi, cfg, tariff, dt, stress_previous=float(states[idx, 5])
                )
                states[idx] = next_states[j]

        if np.any(feasible):
            o2_start = float(state[1]) * R * 273.15 / 1.01325e5
            h2_start = float(state[2]) * R * 273.15 / 1.01325e5
            for idx in np.flatnonzero(feasible):
                o2_end = float(states[idx, 1]) * R * 273.15 / 1.01325e5
                h2_end = float(states[idx, 2]) * R * 273.15 / 1.01325e5
                costs[idx] += float(cfg["w_terminal_O2_KES_per_Nm3_sq"]) * (o2_end - o2_start) ** 2
                costs[idx] += float(cfg["w_terminal_H2_KES_per_Nm3_sq"]) * (h2_end - h2_start) ** 2
        feasible_indices = np.flatnonzero(feasible)
        self.last_feasible = bool(feasible_indices.size)
        self.last_feasible_candidates = int(feasible_indices.size)
        if feasible_indices.size:
            best_idx = feasible_indices[int(np.argmin(costs[feasible_indices]))]
            best_u = float(candidates[best_idx])
        else:
            best_u = demand_current(cfg, float(w["O2_demand_Lpm"]))
        self.I_ref = float(np.clip(best_u, cfg["I_min_A"], cfg["I_max_A"]))
        return self.I_ref

    def _step(self, x, u, fc, cfg, stage):
        if stage == 0:
            return plant_step(x, u, fc, cfg, dt=cfg["sil_dt_s"])
        return self._surrogate_step(x, u, fc, cfg)


def _evaluate_surrogate_rollouts(net: MLP, test_df: pd.DataFrame, cfg: dict) -> dict:
    """Recursive surrogate-versus-plant rollout errors on held-out trajectories.

    Commands and disturbances are taken from held-out trajectories. Both models
    start from the same recorded state and are advanced recursively; the plant
    mirror is the reference. The returned horizons are in supervisory steps.
    """
    leads = (1, 6, 12, 24)
    groups = sorted(g for g in test_df["run_id"].astype(str).unique()
                    if g.startswith("trajectory_"))
    if not groups:
        raise RuntimeError("No held-out trajectory groups are available for recursive surrogate evaluation")
    error_bins = {lead: {} for lead in leads}
    fallback_total = 0
    comparison_count = 0
    predictor = ProposedEMPC(cfg, net)
    state_error_names = ("T_C", "O2_buffer_Nm3", "H2_buffer_Nm3", "I_A", "PV_filtered_W", "stress_idx")
    output_error_names = ("V_cell_V", "purity_pct", "H2_kg_s", "O2_prod_Lpm")

    for group_id in groups:
        g = test_df.loc[test_df["run_id"].astype(str) == group_id].sort_values("step_index")
        if len(g) < max(leads):
            continue
        first = g.iloc[0]
        x_exact = np.array([
            first["state_T_C"], first["state_nO2_mol"], first["state_nH2_mol"],
            first["state_I_A"], first["state_PV_filtered_W"], first["state_stress_idx"],
        ], dtype=float)
        x_surr = x_exact.copy()
        for step_no, (_, row) in enumerate(g.iloc[:max(leads)].iterrows(), start=1):
            fc = {
                "PV_available_W": float(row["PV_available_W"]),
                "O2_demand_Lpm": float(row["O2_demand_Lpm"]),
                "Tamb_C": float(row["Tamb_C"]),
                "hour": float(row["hour"]),
                "H2_export_demand_kg_s": float(cfg.get("H2_export_demand_kg_s", 0.0)),
            }
            command = float(row["I_command_A"])
            x_exact, y_exact = plant_step(x_exact, command, fc, cfg, dt=cfg["sil_dt_s"])
            x_surr, y_surr = predictor._surrogate_step(x_surr, command, fc, cfg)
            if step_no in leads:
                e = error_bins[step_no]
                exact_states = {
                    "T_C": float(x_exact[0]),
                    "O2_buffer_Nm3": float(x_exact[1] * R * 273.15 / 1.01325e5),
                    "H2_buffer_Nm3": float(x_exact[2] * R * 273.15 / 1.01325e5),
                    "I_A": float(x_exact[3]), "PV_filtered_W": float(x_exact[4]),
                    "stress_idx": float(x_exact[5]),
                }
                surr_states = {
                    "T_C": float(x_surr[0]),
                    "O2_buffer_Nm3": float(x_surr[1] * R * 273.15 / 1.01325e5),
                    "H2_buffer_Nm3": float(x_surr[2] * R * 273.15 / 1.01325e5),
                    "I_A": float(x_surr[3]), "PV_filtered_W": float(x_surr[4]),
                    "stress_idx": float(x_surr[5]),
                }
                exact_outputs = {
                    "V_cell_V": float(y_exact["V_cell_V"]),
                    "purity_pct": float(y_exact["purity_next_pct"]),
                    "H2_kg_s": float(y_exact["H2_kg_s"]),
                    "O2_prod_Lpm": float(y_exact["O2_prod_Lpm"]),
                }
                surr_outputs = {
                    "V_cell_V": float(y_surr["V_cell_V"]),
                    "purity_pct": float(y_surr["purity_next_pct"]),
                    "H2_kg_s": float(y_surr["H2_kg_s"]),
                    "O2_prod_Lpm": float(y_surr["O2_prod_Lpm"]),
                }
                for name in state_error_names:
                    e.setdefault(f"{name}_error", []).append(surr_states[name] - exact_states[name])
                for name in output_error_names:
                    e.setdefault(f"{name}_error", []).append(surr_outputs[name] - exact_outputs[name])
                comparison_count += 1
    for values in error_bins.values():
        # Values remain signed errors; summary below reports bias and RMSE.
        pass
    result = {"held_out_trajectory_groups": groups, "compared_rollout_endpoints": comparison_count,
              "surrogate_fallback_count_during_rollout": int(predictor.surrogate_fallback_count),
              "by_lead_step": {}}
    for lead in leads:
        metrics = {}
        for name, errors in error_bins[lead].items():
            a = np.asarray(errors, dtype=float)
            if a.size == 0 or not np.all(np.isfinite(a)):
                raise RuntimeError(f"Non-finite or empty recursive error series for {name} at lead {lead}")
            metric_name = name[:-6] if name.endswith("_error") else name
            metrics[f"{metric_name}_bias"] = float(np.mean(a))
            metrics[f"{metric_name}_RMSE"] = float(np.sqrt(np.mean(a ** 2)))
            metrics[f"{metric_name}_n"] = int(a.size)
        if not metrics:
            raise RuntimeError(f"No recursive rollout endpoints were compared at lead {lead}")
        result["by_lead_step"][str(lead)] = metrics
    fallback_total += predictor.surrogate_fallback_count
    result["surrogate_fallback_count_during_rollout"] = int(fallback_total)
    result["status"] = "ok"
    return result


def _step_size_convergence(test_df: pd.DataFrame, cfg: dict, sample_count: int = 12) -> dict:
    """Compare one macro step with progressively refined sub-stepping.

    Each held-out state/command/disturbance is propagated over one supervisory
    interval. The 1-s solution is the reference; 300/150/60/10-s results are
    reported as errors, not described as mathematically equivalent.
    """
    macro_dt = float(cfg["sil_dt_s"])
    fine_dt = float(cfg["hil_dt_s"])
    if macro_dt < fine_dt or abs(round(macro_dt / fine_dt) * fine_dt - macro_dt) > 1e-8:
        return {"status": "not_run", "reason": "sil_dt_s must be an integer multiple of hil_dt_s"}
    candidates = [float(v) for v in (300, 150, 60, 10, 1)
                  if v <= macro_dt and abs(round(macro_dt / v) * v - macro_dt) < 1e-8]
    samples = test_df.loc[test_df["run_id"].astype(str).str.startswith("trajectory_")].head(sample_count)
    if samples.empty:
        return {"status": "not_run", "reason": "no held-out trajectory states"}
    state_names = ("T_C", "O2_buffer_Nm3", "H2_buffer_Nm3", "I_A", "PV_filtered_W", "stress_idx")
    ref = {n: [] for n in state_names}
    propagated = {int(v): {n: [] for n in state_names} for v in candidates}
    for _, row in samples.iterrows():
        x0 = np.array([row["state_T_C"], row["state_nO2_mol"], row["state_nH2_mol"],
                       row["state_I_A"], row["state_PV_filtered_W"], row["state_stress_idx"]], dtype=float)
        fc = {"PV_available_W": float(row["PV_available_W"]),
              "O2_demand_Lpm": float(row["O2_demand_Lpm"]), "Tamb_C": float(row["Tamb_C"]),
              "hour": float(row["hour"]), "H2_export_demand_kg_s": float(cfg.get("H2_export_demand_kg_s", 0.0))}
        command = float(row["I_command_A"])
        xref = x0.copy()
        for _ in range(int(round(macro_dt / fine_dt))):
            xref, _ = plant_step(xref, command, fc, cfg, dt=fine_dt)
        ref_values = [xref[0], xref[1] * R * 273.15 / 1.01325e5,
                      xref[2] * R * 273.15 / 1.01325e5, xref[3], xref[4], xref[5]]
        for name, value in zip(state_names, ref_values): ref[name].append(float(value))
        for step_dt in candidates:
            x = x0.copy()
            for _ in range(int(round(macro_dt / step_dt))):
                x, _ = plant_step(x, command, fc, cfg, dt=step_dt)
            values = [x[0], x[1] * R * 273.15 / 1.01325e5,
                      x[2] * R * 273.15 / 1.01325e5, x[3], x[4], x[5]]
            for name, value in zip(state_names, values): propagated[int(step_dt)][name].append(float(value))
    result = {"reference_step_s": fine_dt, "macro_interval_s": macro_dt,
              "reference_substeps": int(round(macro_dt / fine_dt)), "sample_count": int(len(samples)),
              "state_error_rmse_vs_1s": {}}
    for step_dt in candidates:
        by_state = {}
        for name in state_names:
            errors = np.asarray(propagated[int(step_dt)][name]) - np.asarray(ref[name])
            by_state[name] = float(np.sqrt(np.mean(errors ** 2)))
        result["state_error_rmse_vs_1s"][str(int(step_dt))] = by_state
    result["status"] = "completed"
    return result


# ==================== SCENARIOS AND SIL ====================
def _make_forecast(profile: List[dict], idx: int, horizon: int,
                   error_pct: float = 0.0, rng: Optional[np.random.Generator] = None) -> List[dict]:
    forecast = [dict(profile[min(idx + k, len(profile) - 1)]) for k in range(horizon)]
    if error_pct > 0.0:
        if rng is None:
            raise ValueError("A seeded RNG is required for forecast-error experiments")
        sigma = float(error_pct) / 100.0
        for fc in forecast:
            pv_scale = max(0.0, 1.0 + float(rng.normal(0.0, sigma)))
            demand_scale = max(0.0, 1.0 + float(rng.normal(0.0, sigma)))
            fc["PV_available_W"] = float(fc["PV_available_W"]) * pv_scale
            fc["O2_demand_Lpm"] = max(0.0, float(fc["O2_demand_Lpm"]) * demand_scale)
    return forecast


def run_one(cfg: dict, profile: List[dict], scenario: str, controller, days: int,
            show_progress=True, forecast_error_pct: float = 0.0, forecast_seed: int = 0) -> pd.DataFrame:
    """Run the plant/lower MPC at hil_dt_s and the upper layer at sil_dt_s.

    The returned file is downsampled to one row per upper interval, with exact
    interval-integrated energy, gas, deficit and violation totals retained.
    """
    plant_dt = float(cfg["hil_dt_s"])
    upper_dt = float(cfg["sil_dt_s"])
    inner_steps = int(round(upper_dt / plant_dt))
    if inner_steps < 1 or abs(inner_steps * plant_dt - upper_dt) > 1e-9:
        raise ValueError("sil_dt_s must be an integer multiple of hil_dt_s")
    nO2_initial = float(cfg["initial_O2_inventory_Nm3"]) * 1.01325e5 / (R * 273.15)
    x = np.array([cfg["initial_temperature_C"], nO2_initial, 0.0, cfg["I_min_A"], 0.0, 0.0], dtype=float)
    rows = []
    last_I_ref = float(cfg["I_min_A"])
    latched_trip = False
    trip_reason = ""
    previous_fault = "nominal"
    last_upper_command_time_s = 0.0
    total_blocks = len(profile)
    forecast_rng = np.random.default_rng(int(forecast_seed))

    # Reset the local tracker to the firmware's initial applied command.
    if hasattr(controller, "tracker"):
        controller.tracker.reset(cfg["I_min_A"])

    for block_idx, disturbance in enumerate(profile):
        fault = disturbance["fault_state"]
        reset_authorized = False
        if fault == "h2_leak" and not latched_trip:
            latched_trip = True
            trip_reason = "modelled_external_h2_leak_alarm"
        elif previous_fault == "h2_leak" and fault == "nominal":
            # Explicit operator-authorised reset represented in SIL only.
            latched_trip = False
            trip_reason = ""
            reset_authorized = True
            if hasattr(controller, "tracker"):
                controller.tracker.reset(cfg["I_min_A"])
            if isinstance(controller, PID):
                controller.reset()
        previous_fault = fault

        forecast = _make_forecast(profile, block_idx, int(cfg["MPC_horizon_steps"]),
                                  error_pct=forecast_error_pct, rng=forecast_rng)
        w = dict(disturbance)
        w["forecast"] = forecast
        upper_solve_ms = 0.0
        held = False
        safe_state = False
        if latched_trip:
            I_ref = 0.0
        elif fault == "both_down" and controller.name in ("FULL_MODEL_EMPC", "PROPOSED"):
            command_age_s = float(disturbance["t_s"]) - last_upper_command_time_s
            if command_age_s <= COMMAND_TIMEOUT_S:
                I_ref = last_I_ref
                held = True
            else:
                # Bounded command hold expires after 600 s, then the lower
                # layer drives current to the safe zero-current state.
                I_ref = 0.0
                safe_state = True
        else:
            t0 = time.perf_counter()
            I_ref = controller.upper(x, w, cfg)
            upper_solve_ms = (time.perf_counter() - t0) * 1000.0
            last_I_ref = float(I_ref)
            last_upper_command_time_s = float(disturbance["t_s"])

        # Integrate the plant and embedded controller at the 1 s HIL rate.
        accum = {
            "energy_kWh_block": 0.0, "grid_cost_KES_block": 0.0,
            "h2_kg_block": 0.0, "h2_export_kg_block": 0.0, "h2_vent_kg_block": 0.0,
            "o2_vent_L_block": 0.0, "pv_energy_Wh_block": 0.0,
            "pv_available_energy_Wh_block": 0.0, "pv_curtailment_Wh_block": 0.0,
            "grid_energy_Wh_block": 0.0, "o2_deficit_L_block": 0.0,
            "objective_KES_block": 0.0,
            "o2_deficit_min_block": 0.0, "purity_violations_block": 0,
            "h2_o2_violations_block": 0, "h2_in_o2_max_ppm_block": 0.0,
            "temperature_violations_block": 0, "voltage_violations_block": 0,
            "grid_limit_violations_block": 0, "grid_overlimit_peak_W_block": 0.0,
            "o2_deficit_L_fault_block": 0.0, "h2_kg_fault_block": 0.0,
            "purity_min_pct_block": 100.0, "held_seconds_block": 0,
            "safe_state_seconds_block": 0, "low_temperature_seconds_block": 0, "min_stack_temperature_C_block": float("inf"),
        }
        last_y = None
        for _ in range(inner_steps):
            if latched_trip or safe_state:
                I_cmd = 0.0
            elif held or controller.name in ("PID", "SINGLE_MPC", "BUFFER_RULE", "FULL_MODEL_EMPC", "PROPOSED"):
                I_cmd = controller.lower(x[3], I_ref, plant_dt)
            else:
                I_cmd = 0.0

            inner_fc = {
                "PV_available_W": disturbance["PV_available_W"],
                "Tamb_C": disturbance["Tamb_C"],
                "O2_demand_Lpm": disturbance["O2_demand_Lpm"],
                "H2_export_demand_kg_s": disturbance.get("H2_export_demand_kg_s", cfg.get("H2_export_demand_kg_s", 0.0)),
                "H2_leak_ppm_inject": disturbance.get("H2_leak_ppm_inject", 0.0),
                "hour": disturbance["hour"],
            }
            x_previous = x.copy()
            x, y = plant_step(x, I_cmd, inner_fc, cfg, dt=plant_dt)
            last_y = y
            # Local safety overrides are independent of the supervisory optimiser
            # and are applied consistently to every SIL controller.
            if not latched_trip:
                if y["H2_leak_ppm"] > 500.0:
                    latched_trip, trip_reason = True, "modelled_external_h2_leak_alarm"
                elif y["H2_in_O2_ppm"] > cfg["H2_in_O2_limit_ppm"]:
                    latched_trip, trip_reason = True, "modelled_h2_in_o2_safety_limit"
                elif y["purity_next_pct"] < cfg["purity_min_pct"]:
                    latched_trip, trip_reason = True, "modelled_oxygen_product_quality_limit"
                elif y["T_next_C"] >= cfg["max_temperature_C"]:
                    latched_trip, trip_reason = True, "stack_temperature_limit"
                elif y["V_cell_V"] >= cfg["max_voltage_V"]:
                    latched_trip, trip_reason = True, "cell_voltage_limit"
                elif max(y["P_O2_tank_bar"], y["P_H2_next_bar"]) > cfg["pressure_max_bar"] + 1e-6:
                    latched_trip, trip_reason = True, "pressure_limit"
            tariff = float(disturbance["tariff_KES_kWh"])
            accum["energy_kWh_block"] += y["P_system_W"] * plant_dt / 3.6e6
            accum["grid_cost_KES_block"] += y["P_grid_W"] * plant_dt / 3.6e6 * tariff
            accum["h2_kg_block"] += y["H2_kg_s"] * plant_dt
            accum["h2_export_kg_block"] += y["H2_export_kg_step"]
            accum["h2_vent_kg_block"] += y["H2_vent_kg_step"]
            accum["o2_vent_L_block"] += y["O2_vented_L_step"]
            accum["objective_KES_block"] += _economic_cost(
                y, cfg, tariff, plant_dt, stress_previous=float(x_previous[5]))
            accum["pv_energy_Wh_block"] += y["P_PV_W"] * plant_dt / 3600.0
            accum["pv_available_energy_Wh_block"] += y["PV_available_W"] * plant_dt / 3600.0
            accum["pv_curtailment_Wh_block"] += max(0.0, y["PV_available_W"] - y["P_PV_W"]) * plant_dt / 3600.0
            accum["grid_energy_Wh_block"] += y["P_grid_W"] * plant_dt / 3600.0
            accum["o2_deficit_L_block"] += y["O2_unmet_L_step"]
            if y["O2_flow_Lpm"] + 1e-9 < disturbance["O2_demand_Lpm"]:
                accum["o2_deficit_min_block"] += plant_dt / 60.0
            accum["purity_violations_block"] += int(y["purity_next_pct"] < cfg["purity_min_pct"])
            accum["h2_o2_violations_block"] += int(y["H2_in_O2_ppm"] > cfg["H2_in_O2_limit_ppm"])
            accum["h2_in_o2_max_ppm_block"] = max(accum["h2_in_o2_max_ppm_block"], y["H2_in_O2_ppm"])
            accum["temperature_violations_block"] += int(y["T_next_C"] > cfg["max_temperature_C"])
            accum["voltage_violations_block"] += int(y["V_cell_V"] > cfg["max_voltage_V"])
            accum["grid_limit_violations_block"] += int(y["grid_overlimit_W"] > 1e-6)
            accum["grid_overlimit_peak_W_block"] = max(accum["grid_overlimit_peak_W_block"], y["grid_overlimit_W"])
            accum["purity_min_pct_block"] = min(accum["purity_min_pct_block"], y["purity_next_pct"])
            accum["low_temperature_seconds_block"] += int(y["T_next_C"] < cfg["min_operating_temperature_C"]) * plant_dt
            accum["min_stack_temperature_C_block"] = min(accum["min_stack_temperature_C_block"], y["T_next_C"])
            if fault == "h2_leak":
                accum["o2_deficit_L_fault_block"] += y["O2_unmet_L_step"]
                accum["h2_kg_fault_block"] += y["H2_kg_s"] * plant_dt
            if held:
                accum["held_seconds_block"] += 1
            if safe_state:
                accum["safe_state_seconds_block"] += 1

        assert last_y is not None
        row = {
            "t_s": disturbance["t_s"], "t_end_s": disturbance["t_s"] + upper_dt,
            "controller": controller.name, "scenario": scenario,
            "I_ref": I_ref, "I_cmd": I_cmd, "I_actual": x[3], "T_C": x[0],
            "tariff": disturbance["tariff_KES_kWh"], "PV_avail": disturbance["PV_available_W"],
            "O2_demand_Lpm": disturbance["O2_demand_Lpm"], "upper_solve_ms": upper_solve_ms,
            "fault_state": fault, "held": held, "safe_state": safe_state,
            "interlock_latched": latched_trip, "trip_reason": trip_reason,
            "operator_reset_authorized": reset_authorized,
            "empc_feasible_candidates": getattr(controller, "last_feasible_candidates", np.nan),
            "surrogate_fallback_count": getattr(controller, "surrogate_fallback_count", 0),
            "forecast_error_pct": float(forecast_error_pct),
            **last_y, **accum,
        }
        # Means are retained for plotting while block-integrated quantities above
        # are used for exact whole-run energy/economics/deficit metrics.
        rows.append(row)
        if show_progress and (block_idx + 1) % 24 == 0:
            _progress(block_idx + 1, total_blocks, f"{scenario}/{controller.name}")

    if show_progress:
        _progress(total_blocks, total_blocks, f"{scenario}/{controller.name}")
    return pd.DataFrame(rows)


def compute_metrics(df: pd.DataFrame, cfg: dict, days: int, scenario: str, controller: str, repeat: int) -> dict:
    warmup_s = max(0.0, float(cfg.get("warmup_hours", 0.0)) * 3600.0)
    measured = df.loc[pd.to_numeric(df["t_s"], errors="coerce") >= warmup_s].copy()
    if measured.empty:
        raise ValueError("No post-warm-up SIL samples remain; increase the simulation duration")
    before = df.loc[pd.to_numeric(df["t_s"], errors="coerce") < warmup_s]
    initial_row = before.iloc[-1] if not before.empty else measured.iloc[0]
    energy = float(measured["energy_kWh_block"].sum())
    grid_cost = float(measured["grid_cost_KES_block"].sum())
    h2_kg = float(measured["h2_kg_block"].sum())
    h2_export = float(measured["h2_export_kg_block"].sum())
    h2_vent = float(measured["h2_vent_kg_block"].sum())
    o2_vent = float(measured["o2_vent_L_block"].sum())
    pv_energy_Wh = float(measured["pv_energy_Wh_block"].sum())
    pv_available_energy_Wh = float(measured["pv_available_energy_Wh_block"].sum())
    pv_curtailment_Wh = float(measured["pv_curtailment_Wh_block"].sum())
    grid_energy_Wh = float(measured["grid_energy_Wh_block"].sum())
    solar_share = 100.0 * pv_energy_Wh / max(pv_energy_Wh + grid_energy_Wh, 1e-9)
    upper = measured.loc[measured["upper_solve_ms"] > 0, "upper_solve_ms"]
    iref = pd.to_numeric(measured["I_ref"], errors="coerce").dropna().to_numpy(dtype=float)
    iref_total_variation_A = float(np.abs(np.diff(iref)).sum()) if len(iref) > 1 else 0.0
    iref_switches = int(np.count_nonzero(np.abs(np.diff(iref)) > 1e-9)) if len(iref) > 1 else 0
    start_o2 = float(initial_row["O2_buffer_Nm3"])
    end_o2 = float(measured["O2_buffer_Nm3"].iloc[-1])
    start_h2 = float(initial_row["H2_buffer_Nm3"])
    end_h2 = float(measured["H2_buffer_Nm3"].iloc[-1])
    terminal_penalty = (
        float(cfg["w_terminal_O2_KES_per_Nm3_sq"]) * (end_o2 - start_o2) ** 2
        + float(cfg["w_terminal_H2_KES_per_Nm3_sq"]) * (end_h2 - start_h2) ** 2
    )
    objective = float(measured["objective_KES_block"].sum()) + terminal_penalty
    return {
        "scenario": scenario, "controller": controller, "repeat": repeat, "days": days,
        "forecast_error_pct": float(measured["forecast_error_pct"].iloc[0]),
        "H2_price_KES_kg": float(cfg["H2_price_KES_kg"]),
        "H2_export_demand_kg_s": float(cfg.get("H2_export_demand_kg_s", 0.0)),
        "H2_revenue_weight": float(cfg["H2_revenue_weight"]),
        "warmup_excluded_hours": float(cfg.get("warmup_hours", 0.0)),
        "grid_cost_KES_total": grid_cost,
        "grid_cost_per_day_KES": grid_cost / max(days, 1),
        "grid_cost_per_kg_H2_produced_KES": grid_cost / max(h2_kg, 1e-9),
        "system_energy_kWh": energy,
        "specific_energy_kWh_kg": energy / max(h2_kg, 1e-9),
        "solar_share_pct": solar_share,
        "pv_available_energy_kWh": pv_available_energy_Wh / 1000.0,
        "pv_used_energy_kWh": pv_energy_Wh / 1000.0,
        "pv_curtailment_kWh": pv_curtailment_Wh / 1000.0,
        "pv_curtailment_pct": 100.0 * pv_curtailment_Wh / max(pv_available_energy_Wh, 1e-9),
        "h2_kg": h2_kg,
        "h2_exported_kg": h2_export,
        "h2_vented_kg": h2_vent,
        "h2_export_credit_KES": h2_export * float(cfg["H2_price_KES_kg"]) * float(cfg["H2_revenue_weight"]),
        "o2_vented_L": o2_vent,
        "o2_deficit_L": float(measured["o2_deficit_L_block"].sum()),
        "o2_deficit_min": float(measured["o2_deficit_min_block"].sum()),
        "purity_violations": int(measured["purity_violations_block"].sum()),
        "purity_min_pct": float(measured["purity_min_pct_block"].min()),
        "h2_o2_violations": int(measured["h2_o2_violations_block"].sum()),
        "h2_in_o2_max_ppm": float(measured["h2_in_o2_max_ppm_block"].max()),
        "temperature_violations": int(measured["temperature_violations_block"].sum()),
        "low_temperature_seconds": int(measured["low_temperature_seconds_block"].sum()),
        "min_stack_temperature_C": float(measured["min_stack_temperature_C_block"].min()),
        "voltage_violations": int(measured["voltage_violations_block"].sum()),
        "grid_limit_violations": int(measured["grid_limit_violations_block"].sum()),
        "grid_overlimit_peak_W": float(measured["grid_overlimit_peak_W_block"].max()),
        "stress_end": float(measured["stress_idx_next"].iloc[-1]),
        "o2_buffer_start_Nm3": start_o2,
        "o2_buffer_end_Nm3": end_o2,
        "h2_buffer_start_Nm3": start_h2,
        "h2_buffer_end_Nm3": end_h2,
        "terminal_inventory_penalty_KES": terminal_penalty,
        "objective_realised_KES": objective,
        "objective_realised_KES_per_day": objective / max(days, 1),
        "upper_solve_ms_median": float(upper.median()) if len(upper) else 0.0,
        "upper_solve_ms_p95": float(upper.quantile(0.95)) if len(upper) else 0.0,
        "upper_solve_ms_max": float(upper.max()) if len(upper) else 0.0,
        "I_ref_total_variation_A": iref_total_variation_A,
        "I_ref_switch_count": iref_switches,
        "held_seconds": int(measured["held_seconds_block"].sum()),
        "safe_state_seconds": int(measured["safe_state_seconds_block"].sum()),
        "h2_kg_fault": float(measured["h2_kg_fault_block"].sum()),
        "o2_deficit_L_fault": float(measured["o2_deficit_L_fault_block"].sum()),
        "operator_reset_count": int(measured["operator_reset_authorized"].sum()),
        "interlock_steps": int(measured["interlock_latched"].sum()),
        "trip_reasons": ";".join(sorted(set(str(v) for v in measured["trip_reason"].dropna() if str(v)))),
        "surrogate_fallback_count": int(measured["surrogate_fallback_count"].max()),
    }



def _flatten_cols(df: pd.DataFrame) -> pd.DataFrame:
    out = []
    for col in df.columns:
        if isinstance(col, tuple):
            out.append("_".join(str(x) for x in col if x not in ("", None)))
        else:
            out.append(str(col))
    df = df.copy()
    df.columns = out
    return df


def _assert_fresh_sil_output_folder() -> None:
    """Prevent a new campaign from silently mixing with old run traces/summaries."""
    existing = sorted(RESULTS.glob("sil_r*_*.csv"))
    summaries = [RESULTS / name for name in ("sil_summary_raw.csv", "sil_summary.csv")
                 if (RESULTS / name).exists()]
    if existing or summaries:
        examples = [p.name for p in (existing[:5] + summaries[:2])]
        raise RuntimeError(
            "Existing SIL campaign files were found in m2_data/results. "
            "This command starts a new campaign; it does not resume one. "
            "Move the previous SIL run CSVs and sil_summary*.csv into an archive "
            "folder outside m2_data/results, then rerun. Found: " + ", ".join(examples)
        )


def _build_sil_aggregate(raw: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Build numeric SIL aggregates and a separate categorical trip-reason table.

    `trip_reasons` stays in the raw experiment record, but it must not be sent
    to mean/std aggregation. This is compatible with Pandas versions that
    reject aggregation of object/string columns.
    """
    required = {"scenario", "controller", "repeat", "days"}
    missing = sorted(required - set(raw.columns))
    if missing:
        raise RuntimeError(f"SIL raw summary is missing identifiers: {missing}")

    out = raw.copy()
    identifiers = required
    numeric_cols: list[str] = []
    for col in out.columns:
        if col in identifiers:
            continue
        converted = pd.to_numeric(out[col], errors="coerce")
        if converted.notna().any() or out[col].isna().all():
            out[col] = converted
            numeric_cols.append(col)
    if not numeric_cols:
        raise RuntimeError("SIL raw summary contains no numeric result metrics")

    grouped = out.groupby(["scenario", "controller"], sort=True, dropna=False)
    aggregate = _flatten_cols(grouped[numeric_cols].agg(["mean", "std"]).reset_index())
    tcrit_by_n = {2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776, 6: 2.571,
                  7: 2.447, 8: 2.365, 9: 2.306, 10: 2.262, 11: 2.228,
                  12: 2.201, 13: 2.179, 14: 2.160, 15: 2.145, 16: 2.131,
                  17: 2.120, 18: 2.110, 19: 2.101, 20: 2.093, 21: 2.086,
                  22: 2.080, 23: 2.074, 24: 2.069, 25: 2.064, 26: 2.060,
                  27: 2.056, 28: 2.052, 29: 2.048, 30: 2.045}
    for metric in numeric_cols:
        means, stds = f"{metric}_mean", f"{metric}_std"
        if means in aggregate.columns and stds in aggregate.columns:
            nvals = grouped[metric].count().to_numpy(dtype=float)
            tvals = np.array([tcrit_by_n.get(int(n), 1.96) for n in nvals])
            half = tvals * aggregate[stds].fillna(0.0).to_numpy(dtype=float) / np.sqrt(np.maximum(nvals, 1.0))
            aggregate[f"{metric}_ci95_low"] = aggregate[means].to_numpy(dtype=float) - half
            aggregate[f"{metric}_ci95_high"] = aggregate[means].to_numpy(dtype=float) + half

    reason_rows = []
    if "trip_reasons" in raw.columns:
        for row in raw[["scenario", "controller", "repeat", "trip_reasons"]].itertuples(index=False):
            value = str(row.trip_reasons or "").strip()
            if value and value.lower() != "nan":
                for reason in sorted({x.strip() for x in value.split(";") if x.strip()}):
                    reason_rows.append({"scenario": row.scenario, "controller": row.controller,
                                        "repeat": int(row.repeat), "trip_reason": reason})
    if reason_rows:
        detail = pd.DataFrame(reason_rows)
        reason_counts = (detail.groupby(["scenario", "controller", "trip_reason"], as_index=False)
                         .agg(runs_with_reason=("repeat", "nunique")))
    else:
        reason_counts = pd.DataFrame(columns=["scenario", "controller", "trip_reason", "runs_with_reason"])
    return aggregate, reason_counts


def rebuild_sil_summary(cfg: dict, days: int, repeats: int) -> dict:
    """Rebuild summary files from completed SIL outputs without rerunning controllers."""
    raw_path = RESULTS / "sil_summary_raw.csv"
    if not raw_path.exists():
        raise FileNotFoundError(f"Cannot rebuild SIL summary; missing {raw_path}")
    raw = pd.read_csv(raw_path)
    expected = len(SCENARIO_NAMES) * len(CONTROLLER_NAMES) * int(repeats)
    if len(raw) != expected:
        raise RuntimeError(f"Raw SIL summary has {len(raw)} rows; expected {expected}. No simulations were rerun.")
    if not (pd.to_numeric(raw["days"], errors="coerce") == float(days)).all():
        raise RuntimeError(f"Raw SIL summary does not contain exactly {days}-day rows; no simulations were rerun.")
    if raw.duplicated(["scenario", "controller", "repeat"]).any():
        raise RuntimeError("Raw SIL summary contains duplicate scenario/controller/repeat rows")
    aggregate, reason_counts = _build_sil_aggregate(raw)
    aggregate.to_csv(RESULTS / "sil_summary.csv", index=False)
    reason_counts.to_csv(RESULTS / "sil_trip_reason_counts.csv", index=False)
    print(f"[rebuild-summary] aggregated {len(raw)} existing raw SIL rows; no controller simulations were rerun")
    print(f"[rebuild-summary] aggregate: {RESULTS / 'sil_summary.csv'}")
    print(f"[rebuild-summary] trip reasons: {RESULTS / 'sil_trip_reason_counts.csv'}")
    report = audit_sil(cfg, days=days, repeats=repeats, require_complete=False)
    print(f"[rebuild-summary] audit status={report['status']}; {report['run_files_found']}/{expected} run files")
    if not report["data_quality_pass"]:
        raise RuntimeError("Summary files were rebuilt, but SIL audit found issues. Inspect m2_data/results/sil_validation_report.json")
    return report


def sil(cfg: dict, days: int, repeats: int = DEFAULT_SIL_REPEATS, forecast_error_pct: float = 0.0) -> None:
    if days < 1 or repeats < 1:
        raise ValueError("days and repeats must be >= 1")
    _assert_fresh_sil_output_folder()
    if not 0.0 <= forecast_error_pct <= 100.0:
        raise ValueError("forecast_error_pct must be between 0 and 100")
    weather = load_weather(cfg)
    if not SURROGATE.exists():
        raise FileNotFoundError(f"Missing frozen surrogate {SURROGATE}. Run `python m2_dashboard.py train` first.")
    surrogate = MLP.load(SURROGATE)
    surrogate_hash = validate_surrogate_artifact(surrogate, "SIL")
    print(f"[sil] frozen surrogate: {surrogate.sizes}; SHA-256={surrogate_hash}")
    all_metrics = []
    total = len(SCENARIO_NAMES) * len(CONTROLLER_NAMES) * repeats
    done = 0
    warmup_hours = float(cfg.get("warmup_hours", 24.0))
    simulated_hours_per_run = warmup_hours + float(days) * 24.0
    simulated_hours_total = total * simulated_hours_per_run
    nominal_steps = int(round(simulated_hours_total * 3600.0 / float(cfg["sil_dt_s"])))
    print(f"[sil] campaign plan: {len(SCENARIO_NAMES)} scenarios x {len(CONTROLLER_NAMES)} primary controllers x "
          f"{repeats} repeats = {total} runs")
    print(f"[sil] simulated workload: {simulated_hours_total:.1f} h total, approximately {nominal_steps:,} "
          f"{float(cfg['sil_dt_s']):g}-s supervisory steps")
    print("[sil] primary controllers: " + ", ".join(CONTROLLER_NAMES))
    weather_offsets = select_rainy_season_week_offsets(weather, repeats)
    warmup_s = warmup_hours * 3600.0
    weather_offsets = [offset - warmup_s for offset in weather_offsets]
    print(f"[sil] Each repeat includes {warmup_hours:.1f} h warm-up (excluded from metrics), then {days} measured days.")
    print("[sil] Every controller within a repeat receives the same weather/demand/fault trace.")
    if "time_nairobi_local" in weather.columns:
        print("[sil] Weather-week selection uses Nairobi timestamps in March-May and October-December where available.")
    else:
        print("[sil] Weather timestamps unavailable; repeat offsets are sequential weekly windows.")
    for rep in range(repeats):
        for scenario in SCENARIO_NAMES:
            profile = build_scenario_profile(
                cfg, weather, scenario, days,
                seed=10000 + rep * 100 + SCENARIO_NAMES.index(scenario),
                weather_offset_s=weather_offsets[rep], warmup_hours=warmup_hours,
            )
            controllers = [PID(cfg), SingleMPC(cfg), FullEMPC(cfg), ProposedEMPC(cfg, surrogate)]
            for controller in controllers:
                done += 1
                print(f"\n[sil] repeat {rep + 1}/{repeats}: {scenario}/{controller.name}; "
                      f"measurement-week offset={(weather_offsets[rep] + warmup_s) / 86400.0:.1f} d; forecast error={forecast_error_pct:.1f}%")
                result = run_one(
                    cfg, profile, scenario, controller, days, show_progress=False,
                    forecast_error_pct=forecast_error_pct,
                    forecast_seed=500000 + rep * 100 + SCENARIO_NAMES.index(scenario),
                )
                out = RESULTS / f"sil_r{rep + 1}_{scenario}_{controller.name}.csv"
                result.to_csv(out, index=False)
                all_metrics.append(compute_metrics(result, cfg, days, scenario, controller.name, rep + 1))
                _progress(done, total, "SIL runs")
    raw = pd.DataFrame(all_metrics)
    raw.to_csv(RESULTS / "sil_summary_raw.csv", index=False)
    agg, reason_counts = _build_sil_aggregate(raw)
    agg.to_csv(RESULTS / "sil_summary.csv", index=False)
    reason_counts.to_csv(RESULTS / "sil_trip_reason_counts.csv", index=False)
    print("\n=== SIL summary (mean, standard deviation and 95% CI across repeats) ===")
    print(agg.to_string(index=False, max_cols=24))
    print(f"\n[sil] raw summary: {RESULTS / 'sil_summary_raw.csv'}")
    print(f"[sil] aggregate:   {RESULTS / 'sil_summary.csv'}")
    audit = audit_sil(cfg, days=days, repeats=repeats, require_complete=False)
    print(f"[sil] completion audit: {audit['run_files_found']}/{audit['expected_run_files']} run files; "
          f"data-quality={'PASS' if audit['data_quality_pass'] else 'CHECK'}")



# ==================== ARDUINO + MATLAB/SLX HIL SUPERVISOR ====================
def collect(args) -> None:
    try:
        import paho.mqtt.client as mqtt
    except ImportError as exc:
        raise RuntimeError("HIL collection requires `pip install paho-mqtt`") from exc

    cfg = load_cfg()
    net = MLP.load(SURROGATE)  # frozen network only; no HIL retraining
    surrogate_hash = validate_surrogate_artifact(net, "HIL collector")
    weather = load_weather(cfg, getattr(args, "weather_profile_csv", None))
    demand_profile = (load_demand_profile(args.demand_profile_csv)
                      if getattr(args, "demand_profile_csv", None) else None)
    if getattr(args, "require_real_profiles", False):
        if demand_profile is None:
            raise ValueError("--require-real-profiles requires --demand-profile-csv with measured/sourced hospital demand")
        if weather["PV_available_W"].nunique() < 6 or float(weather["PV_available_W"].std()) < 500.0:
            raise ValueError("Selected weather/PV CSV is effectively flat; provide a varying, source-documented Nairobi PV profile")
        if demand_profile["O2_demand_Lpm"].nunique() < 5:
            raise ValueError("Selected demand profile is effectively flat; provide a varying hospital demand trace")
    scenario = args.scenario
    if scenario not in HIL_SCENARIOS:
        raise ValueError(f"Unsupported HIL scenario {scenario!r}; choose from {HIL_SCENARIOS}")
    run_id = args.run_id or str(uuid.uuid4())
    out_path = RESULTS / f"hil_{scenario}_{run_id[:8]}.csv"
    print(f"[hil] {scenario} -> {out_path}")

    broker_host = args.broker or str(cfg.get("mqtt_host", "broker.hivemq.com"))
    broker_port = int(args.port if args.port is not None else cfg.get("mqtt_port", 1883))
    cmd_topic = "m2/knh_pem_hil_m2_2026/upper/setpoint"
    tele_topic = "m2/knh_pem_hil_m2_2026/arduino/telemetry"
    design_ack_topic = "arduino/design_ack"
    run_ack_topic = "arduino/run_ack"
    board_log_topic = "m2/knh_pem_hil_m2_2026/arduino/log"
    telemetry_queue: queue.Queue = queue.Queue(maxsize=8192)
    board_log_queue: queue.Queue = queue.Queue(maxsize=512)
    mqtt_connected_event = threading.Event()
    subscriptions_ready_event = threading.Event()
    board_ready_event = threading.Event()
    design_ack_event = threading.Event()
    reset_accepted_event = threading.Event()
    matlab_reset_ack_event = threading.Event()
    handshake_state: Dict[str, object] = {
        "design_ack": {}, "reset_ack": {}, "mqtt_error": "",
        "last_heartbeat": {}, "last_startup_heartbeat": {},
        "last_board_telemetry": {}, "last_board_log": {},
    }

    def _on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            client.subscribe([(tele_topic, 0), (design_ack_topic, 1), (run_ack_topic, 1), (board_log_topic, 0)])
            mqtt_connected_event.set()
        else:
            handshake_state["mqtt_error"] = f"MQTT CONNACK rejected: {reason_code}"

    def _on_subscribe(client, userdata, mid, reason_code_list, properties):
        subscriptions_ready_event.set()

    def _on_telemetry(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8", errors="strict"))
            if msg.topic == design_ack_topic:
                if payload.get("type") == "he_nmpc_schedule_ack":
                    handshake_state["design_ack"] = payload
                    if payload.get("status") == "accepted" and payload.get("design_locked") is True:
                        design_ack_event.set()
                return
            if msg.topic == board_log_topic:
                if payload.get("type") == "log":
                    handshake_state["last_board_log"] = payload
                    try:
                        board_log_queue.put_nowait(payload)
                    except queue.Full:
                        try: board_log_queue.get_nowait()
                        except queue.Empty: pass
                        try: board_log_queue.put_nowait(payload)
                        except queue.Full: pass
                return
            if msg.topic == run_ack_topic:
                if payload.get("type") == "reset_run_ack" and payload.get("run_id") == run_id:
                    handshake_state["reset_ack"] = payload
                    status = str(payload.get("status", ""))
                    if status == "accepted_plant_reset_pending":
                        reset_accepted_event.set()
                    elif status == "plant_reset_accepted":
                        reset_accepted_event.set()
                        matlab_reset_ack_event.set()
                    elif status.startswith("rejected"):
                        reset_accepted_event.set()
                return
            if msg.topic != tele_topic or payload.get("type") != "telemetry":
                return
            if payload.get("board_status") == "mqtt_ready":
                board_ready_event.set()
                if int(payload.get("seq", 0) or 0) <= 0:
                    if payload.get("run_id") == run_id:
                        handshake_state["last_heartbeat"] = payload
                    else:
                        handshake_state["last_startup_heartbeat"] = payload
            # seq=0 is a readiness heartbeat, not a plant measurement row.
            if int(payload.get("seq", 0) or 0) <= 0:
                return
            if payload.get("run_id") != run_id:
                return  # Ignore delayed measurements from any preceding HIL run.
            handshake_state["last_board_telemetry"] = payload
            try:
                telemetry_queue.put_nowait(payload)
            except queue.Full:
                try: telemetry_queue.get_nowait()
                except queue.Empty: pass
                try: telemetry_queue.put_nowait(payload)
                except queue.Full: pass
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
            return

    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"m2-{run_id[:8]}")
    cli.on_connect = _on_connect
    cli.on_subscribe = _on_subscribe
    cli.on_message = _on_telemetry
    cli.connect(broker_host, broker_port, keepalive=60)
    cli.loop_start()
    if not mqtt_connected_event.wait(15.0):
        cli.loop_stop(); cli.disconnect()
        detail = handshake_state.get("mqtt_error") or "No MQTT CONNACK received."
        raise RuntimeError(f"HIL MQTT client could not connect to {broker_host}:{broker_port}. {detail}")
    if not subscriptions_ready_event.wait(10.0):
        cli.loop_stop(); cli.disconnect()
        raise RuntimeError("HIL collector connected to MQTT but did not receive SUBACK for telemetry/design/run acknowledgements.")

    log_fields = [
        "seq", "time_s", "scenario", "I_ref_A", "I_command_A", "I_stack_A", "V_cell_V",
        "P_stack_W", "P_BoP_W", "P_system_W", "T_stack_C", "P_O2_bar", "P_H2_bar",
        "O2_flow_Lpm", "O2_sensor_Lpm", "H2_kg_s", "O2_demand_Lpm", "O2_buffer_Nm3",
        "H2_buffer_Nm3", "P_PV_W", "PV_available_W", "P_grid_W", "tariff_KES_kWh",
        "purity_model_pct", "H2_in_O2_ppm", "stress_idx", "O2_prod_Lpm", "H2_export_kg_s",
        "H2_vent_kg_s", "O2_vented_L_step", "lower_loop_ms", "lower_solve_ms",
        "lower_tick_interval_ms", "plant_step_ms",
        "cycle_latency_ms", "missed_deadline", "missed_deadline_count", "interlock", "trip_reason", "H2_leak_ppm",
        "hil_time_mode", "plant_engine", "hil_dt_s", "controller_sample_period_sim_s",
        "telemetry_stride_sim_s", "controller_batch_steps", "batch_steps_completed",
        "lower_batch_wall_ms", "plant_batch_wall_ms", "plant_batch_avg_step_ms",
        "controller_estimator_error_A", "plant_request_pending", "plant_reset_pending",
        "plant_bridge_error", "reset_status", "design_locked", "controller",
        "communication_path", "plant_link", "supervisory_link", "wall_elapsed_s",
        "fault_state", "sensor_fault_inject", "purity_alarm_inject",
        "upper_solve_ms", "upper_command_sent_A",
    ]
    # Do not send one-shot startup commands until the physical GIGA has proved
    # that it is online and subscribed. This closes the old boot-time race where
    # Python's schedule/reset messages were published before the GIGA's first
    # five-second Wi-Fi service slot.
    print(f"[hil] MQTT connected to {broker_host}:{broker_port}; waiting for GIGA readiness heartbeat.")
    ready_deadline = time.time() + 90.0
    last_ready_notice = 0.0
    while not board_ready_event.is_set() and time.time() < ready_deadline:
        time.sleep(0.25)
        elapsed = 90.0 - max(0.0, ready_deadline - time.time())
        if elapsed - last_ready_notice >= 15.0:
            print(f"[hil] waiting for GIGA MQTT-ready heartbeat ({elapsed:.0f}s elapsed)")
            last_ready_notice = elapsed
    if not board_ready_event.is_set():
        cli.loop_stop(); cli.disconnect()
        raise RuntimeError(
            "No GIGA MQTT-ready heartbeat received within 90 s. Check that the uploaded INO is the startup-heartbeat version, "
            "the fallback Wi-Fi SSID/password are correct, the phone hotspot allows 2.4-GHz clients/internet, and the GIGA is connected to HiveMQ."
        )

    # Publish the locked design; require an explicit firmware acknowledgement.
    design_msg = {
        "type": "he_nmpc_schedule",
        "design": {
            "areaCm2": float(cfg["A_cell_cm2"]),
            "nCells": int(cfg["N_cells"]),
            "J": float(cfg["I_max_A"] * 10000.0 / cfg["A_cell_cm2"]),
            "temperatureC": float(cfg["initial_temperature_C"]),
            "pressurePa": 15.0e5,
            "I_min_A": float(cfg["I_min_A"]),
            "I_max_A": float(cfg["I_max_A"]),
        },
        "hospitalDemand": float(cfg["hospital_demand_Lpm"]),
        "design_locked": True,
    }
    design_payload = json.dumps(design_msg)
    design_deadline = time.time() + 30.0
    while not design_ack_event.is_set() and time.time() < design_deadline:
        info = cli.publish("dashboard/command", design_payload, qos=1)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            time.sleep(1.0)
            continue
        design_ack_event.wait(3.0)
    if not design_ack_event.is_set():
        cli.loop_stop(); cli.disconnect()
        raise RuntimeError(
            "GIGA MQTT heartbeat was received, but no accepted he_nmpc_schedule_ack arrived. "
            "Check that the GIGA subscribes to dashboard/command and that design limits match m2_config.json. "
            f"Last acknowledgement: {handshake_state.get('design_ack')}"
        )

    # Only now select the controller and request a plant reset. MATLAB owns USB COM.
    cli.publish(cmd_topic, json.dumps({"type": "controller_mode", "controller": "PROPOSED"}), qos=1)
    cli.publish(cmd_topic, json.dumps({"type": "reset_run", "run_id": run_id}), qos=1)
    if not reset_accepted_event.wait(15.0):
        cli.loop_stop(); cli.disconnect()
        raise RuntimeError(
            "GIGA did not acknowledge reset_run on arduino/run_ack. "
            f"Last reset acknowledgement: {handshake_state.get('reset_ack')}"
        )
    reset_ack_status = str(handshake_state.get("reset_ack", {}).get("status", ""))
    if reset_ack_status.startswith("rejected"):
        cli.loop_stop(); cli.disconnect()
        raise RuntimeError(f"GIGA rejected reset_run: {handshake_state.get('reset_ack')}")
    if reset_ack_status == "accepted_plant_reset_pending":
        print("[hil] GIGA accepted reset_run and sent reset to MATLAB over USB Serial.")
        print("[hil] Waiting for MATLAB reset_ack (six-state initial condition) to return through COM7...")
        reset_deadline = time.time() + 20.0
        last_reset_notice = 0.0
        while not matlab_reset_ack_event.is_set() and time.time() < reset_deadline:
            time.sleep(0.25)
            elapsed = 20.0 - max(0.0, reset_deadline - time.time())
            if elapsed - last_reset_notice >= 5.0:
                hb = handshake_state.get("last_heartbeat", {})
                log = handshake_state.get("last_board_log", {})
                print(
                    "[hil][diag] waiting MATLAB ACK | "
                    f"GIGA reset_status={hb.get('reset_status', 'unknown')} "
                    f"reset_pending={hb.get('plant_reset_pending', 'unknown')} "
                    f"serial_request_pending={hb.get('plant_request_pending', 'unknown')} "
                    f"last_GIGA_log={log.get('message', 'none')}"
                )
                last_reset_notice = elapsed
        if not matlab_reset_ack_event.is_set():
            hb = handshake_state.get("last_heartbeat", {})
            log = handshake_state.get("last_board_log", {})
            cli.loop_stop(); cli.disconnect()
            raise RuntimeError(
                "GIGA received reset_run but MATLAB's reset_ack never returned on COM7. "
                "The failure is on the USB Serial request/response boundary, before the first 300-step batch. "
                f"Last GIGA heartbeat: {hb}. Last GIGA log: {log}. "
                "In MATLAB, check for '[HE-NMPC] Serial RX reset' and 'Serial TX reset_ack'."
            )
    elif reset_ack_status == "plant_reset_accepted":
        matlab_reset_ack_event.set()
    if matlab_reset_ack_event.is_set():
        print("[hil] MATLAB reset_ack received by GIGA; six-state plant initial condition accepted.")

    csv_file = out_path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(csv_file, fieldnames=log_fields, extrasaction="ignore")
    writer.writeheader()
    csv_file.flush()
    diag_path = out_path.with_name(out_path.stem + "_diagnostics.log")
    diag_file = diag_path.open("w", encoding="utf-8", buffering=1)
    diag_lock = threading.Lock()

    def _diag(message: str) -> None:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{stamp}] {message}"
        print(line, flush=True)
        with diag_lock:
            diag_file.write(line + "\n")
            diag_file.flush()

    def _drain_board_logs() -> None:
        while True:
            try:
                board_log = board_log_queue.get_nowait()
            except queue.Empty:
                break
            _diag(f"[GIGA:{board_log.get('level', 'INFO')}] {board_log.get('message', '')}")

    def _clock(seconds: float) -> str:
        seconds = max(0, int(seconds))
        days, rem = divmod(seconds, 86400)
        hours, rem = divmod(rem, 3600)
        minutes, secs = divmod(rem, 60)
        return (f"{days}d {hours:02d}:{minutes:02d}:{secs:02d}" if days
                else f"{hours:02d}:{minutes:02d}:{secs:02d}")

    def _progress(sim_time_s: Optional[float], wall_elapsed_s: float, note: str = "") -> None:
        sim_target_s = getattr(args, "sim_duration_s", None)
        if sim_target_s is not None and float(sim_target_s) > 0:
            done_s = max(0.0, float(sim_time_s or 0.0))
            fraction = min(1.0, done_s / float(sim_target_s))
            progress_text = f"sim {_clock(done_s)} / {_clock(float(sim_target_s))}"
            percent = 100.0 * fraction
        else:
            wall_target_s = max(1.0, float(args.duration_s))
            fraction = min(1.0, max(0.0, wall_elapsed_s / wall_target_s))
            progress_text = f"wall {_clock(wall_elapsed_s)} / {_clock(wall_target_s)}"
            percent = 100.0 * fraction
        width = 30
        filled = min(width, int(round(width * fraction)))
        bar = "#" * filled + "-" * (width - filled)
        suffix = f" | {note}" if note else ""
        _diag(f"[progress] [{bar}] {percent:5.1f}% | {progress_text} | wall {_clock(wall_elapsed_s)}{suffix}")

    _diag(f"[hil] MQTT telemetry/commands at {broker_host}:{broker_port}; MATLAB owns Arduino USB serial; upper period {args.upper_period_s:.0f}s")
    _diag("[hil] Plant path: MATLAB/Simulink <-> Arduino GIGA over USB Serial; Arduino <-> Python/web via MQTT.")
    _diag(f"[hil] Python does not open the Arduino COM port. Frozen surrogate={net.sizes}; SHA-256={surrogate_hash}.")
    _diag("[hil] No training or retraining occurs during HIL collection.")
    _diag(f"[hil] Diagnostic log -> {diag_path}")
    _diag(f"[handshake] design_ack={handshake_state.get('design_ack', {})}")
    _diag(f"[handshake] reset_ack={handshake_state.get('reset_ack', {})}")
    _drain_board_logs()
    sim_target_for_progress = getattr(args, "sim_duration_s", None)
    if sim_target_for_progress is not None:
        _diag(f"[hil] Progress is based on completed plant simulated time; target={float(sim_target_for_progress):.0f}s.")
    else:
        _diag(f"[hil] Progress is based on wall-clock duration; target={float(args.duration_s):.0f}s.")
    t0 = time.time()
    _progress(0.0, 0.0, "waiting for first completed plant batch")
    last_upper_sim_s = -float("inf")
    rows = 0
    last_sim_t = None
    last_warn = 0.0
    pending_fault_reset_ack = False
    fault_ack_sent = False
    prev_fault = "nominal"
    fault = "nominal"
    upper_ms = 0.0
    controller = ProposedEMPC(cfg, net)
    # Do not pre-label a run as completed: only the loop's explicit target
    # check may set sim_duration_reached. Exceptions and early exits retain
    # truthful statuses in the CSV summary and diagnostics log.
    termination_reason = "running"
    try:
        while True:
            elapsed_wall = time.time() - t0
            sim_target = getattr(args, "sim_duration_s", None)
            if sim_target is None:
                if elapsed_wall >= float(args.duration_s):
                    termination_reason = "wall_duration_reached"
                    break
            else:
                if last_sim_t is not None and last_sim_t >= float(sim_target):
                    termination_reason = "sim_duration_reached"
                    break
                wall_timeout = getattr(args, "wall_timeout_s", None)
                if wall_timeout is None or float(wall_timeout) <= 0:
                    # Preserve the familiar --duration-s 1800 invocation as a
                    # 30-min wall-clock cap even when --sim-duration-s is set.
                    wall_timeout = getattr(args, "duration_s", 0.0)
                if wall_timeout is not None and float(wall_timeout) > 0 and elapsed_wall >= float(wall_timeout):
                    termination_reason = "wall_timeout_before_sim_target"
                    break
            try:
                d = telemetry_queue.get(timeout=0.5)
            except queue.Empty:
                now = time.time()
                # Always initialize the run-specific heartbeat snapshot before
                # testing for a trip. The old code initialized `hb` only after
                # 15 s, then referenced it unconditionally on the first 0.5-s
                # queue timeout, raising UnboundLocalError during normal waits.
                hb = (handshake_state.get("last_heartbeat", {})
                      or handshake_state.get("last_startup_heartbeat", {})
                      or {})
                board = handshake_state.get("last_board_telemetry", {}) or {}
                log = handshake_state.get("last_board_log", {}) or {}
                if now - t0 > 15 and now - last_warn > 15:
                    _progress(last_sim_t, now - t0, "WAITING_FOR_PLANT_TELEMETRY")
                    _diag(
                        f"[diag] WAITING_FOR_PLANT_TELEMETRY | elapsed_wall={now - t0:.1f}s "
                        f"active_run={run_id} | heartbeat_run={hb.get('run_id', 'none')} "
                        f"heartbeat_seq={hb.get('seq', 'n/a')} "
                        f"plant_seq={hb.get('plant_seq', board.get('seq', 'n/a'))} "
                        f"plant_time_s={hb.get('plant_time_s', board.get('time_s', 'n/a'))} "
                        f"request_age_ms={hb.get('plant_request_age_ms', 'n/a')} "
                        f"plant_bridge_error={hb.get('plant_bridge_error', board.get('plant_bridge_error', ''))} "
                        f"reset_status={hb.get('reset_status', board.get('reset_status', 'n/a'))} "
                        f"reset_pending={hb.get('plant_reset_pending', board.get('plant_reset_pending', 'n/a'))} "
                        f"request_pending={hb.get('plant_request_pending', board.get('plant_request_pending', 'n/a'))} "
                        f"trip={hb.get('trip_latched', board.get('interlock', 'n/a'))} "
                        f"trip_reason={hb.get('trip_reason', board.get('trip_reason', 'n/a'))} "
                        f"controller={hb.get('controller', board.get('controller', 'n/a'))} | "
                        f"last_GIGA_log={log.get('level', 'none')}: {log.get('message', 'none')}"
                    )
                    last_warn = now
                _drain_board_logs()
                if hb.get("run_id") == run_id and bool(hb.get("trip_latched", False)):
                    termination_reason = "giga_trip_before_plant_telemetry"
                    _diag(
                        f"[diag][ABORT] GIGA reported a latched trip before any new plant sample: "
                        f"reason={hb.get('trip_reason', 'unknown')} "
                        f"plant_bridge_error={hb.get('plant_bridge_error', '')} "
                        f"plant_seq={hb.get('plant_seq', 'n/a')} "
                        f"request_age_ms={hb.get('plant_request_age_ms', 'n/a')}"
                    )
                    break
                continue

            # Ignore reset/startup snapshots. seq=1 is the first completed
            # Simulink plant step and is the first valid data row.
            if int(d.get("seq", 0) or 0) <= 0:
                continue
            _drain_board_logs()
            sim_t = float(d.get("time_s", 0.0))
            if last_sim_t is not None and sim_t <= last_sim_t:
                continue  # MQTT QoS/reconnect duplicates must not inflate trajectory duration.
            upper_ms_sample = 0.0
            upper_command_sent = ""
            if last_sim_t is None:
                _diag(f"[hil] first completed plant telemetry received: seq={d.get('seq')} t={sim_t:.1f}s")
            if sim_t - last_upper_sim_s >= args.upper_period_s:
                weather_time = sim_t + float(args.weather_offset_hours) * 3600.0
                w0 = weather_at(weather, weather_time)
                noise = 0.0  # deterministic HIL disturbance profile; no hidden random resampling
                if args.event_start_s is not None:
                    cfg["O2_surge_start_s"] = float(args.event_start_s)
                    cfg["fault_start_s"] = float(args.event_start_s)
                if args.event_duration_s is not None:
                    cfg["O2_surge_duration_s"] = float(args.event_duration_s)
                    cfg["fault_duration_s"] = float(args.event_duration_s)
                surge_time = sim_t if scenario == "oxygen_surge" else None
                dmd = (demand_at_profile(demand_profile, sim_t)
                       if demand_profile is not None
                       else o2_demand(w0["hour"], cfg, t_s=surge_time, noise=noise))
                if scenario == "oxygen_surge" and demand_profile is not None:
                    surge_start = float(args.event_start_s if args.event_start_s is not None else cfg["O2_surge_start_s"])
                    surge_duration = float(args.event_duration_s if args.event_duration_s is not None else cfg["O2_surge_duration_s"])
                    if surge_start <= sim_t < surge_start + surge_duration:
                        dmd *= float(cfg["O2_surge_factor"])
                gsm_start = float(args.event_start_s) if args.event_start_s is not None else 3 * 86400 + 12 * 3600
                gsm_end = gsm_start + float(args.event_duration_s or 3600.0)
                both_start = gsm_end
                both_end = both_start + float(args.event_duration_s or 3600.0)
                leak_start = float(cfg["fault_start_s"])
                leak_end = leak_start + float(cfg["fault_duration_s"])
                fault = "nominal"
                if scenario == "cloudy_rainy":
                    w0["PV_available_W"] *= 0.4
                elif scenario == "communication_loss":
                    if gsm_start <= sim_t < gsm_end:
                        fault = "gsm_fallback_simulated"
                    elif both_start <= sim_t < both_end:
                        fault = "both_down"
                elif scenario == "sensor_fault_leak" and leak_start <= sim_t < leak_end:
                    fault = "h2_leak"
                elif scenario == "sensor_out_of_range" and leak_start <= sim_t < leak_end:
                    fault = "sensor_out_of_range"
                elif scenario == "purity_alarm" and leak_start <= sim_t < leak_end:
                    fault = "purity_alarm"
                sensor_fault_inject = fault == "sensor_out_of_range"
                purity_alarm_inject = fault == "purity_alarm"
                fc = {
                    "PV_available_W": w0["PV_available_W"],
                    "Tamb_C": w0["Tamb_C"],
                    "hour": w0["hour"],
                    "O2_demand_Lpm": dmd,
                    "tariff_KES_kWh": tariff_for(cfg, w0["hour"]),
                    "H2_leak_ppm_inject": 800.0 if fault == "h2_leak" else 0.0,
                    "H2_export_demand_kg_s": float(cfg.get("H2_export_demand_kg_s", 0.0)),
                }
                forecast = []
                for k in range(int(cfg["MPC_horizon_steps"])):
                    tk = sim_t + k * float(cfg["sil_dt_s"])
                    wk = weather_at(weather, tk + float(args.weather_offset_hours) * 3600.0)
                    if scenario == "cloudy_rainy":
                        wk["PV_available_W"] *= 0.4
                    surge_time_k = tk if scenario == "oxygen_surge" else None
                    wk_demand = (demand_at_profile(demand_profile, tk)
                                 if demand_profile is not None
                                 else o2_demand(wk["hour"], cfg, t_s=surge_time_k, noise=0.0))
                    if scenario == "oxygen_surge" and demand_profile is not None:
                        surge_start = float(args.event_start_s if args.event_start_s is not None else cfg["O2_surge_start_s"])
                        surge_duration = float(args.event_duration_s if args.event_duration_s is not None else cfg["O2_surge_duration_s"])
                        if surge_start <= tk < surge_start + surge_duration:
                            wk_demand *= float(cfg["O2_surge_factor"])
                    forecast.append({
                        "PV_available_W": wk["PV_available_W"], "Tamb_C": wk["Tamb_C"],
                        "hour": wk["hour"], "O2_demand_Lpm": wk_demand,
                        "tariff_KES_kWh": tariff_for(cfg, wk["hour"]),
                    })
                state = np.array([
                    float(d.get("T_stack_C", cfg["initial_temperature_C"])),
                    float(d.get("O2_buffer_Nm3", cfg["O2_reserve_Nm3"])) * 1.01325e5 / (R * 273.15),
                    float(d.get("H2_buffer_Nm3", 0.0)) * 1.01325e5 / (R * 273.15),
                    float(d.get("I_stack_A", cfg["I_min_A"])),
                    float(d.get("PV_filtered_W", 0.0)),
                    float(d.get("stress_idx", 0.0)),
                ], dtype=float)
                w = dict(fc, forecast=forecast)
                if fault == "both_down":
                    # Do not publish a new setpoint during the simulated dual-link outage.
                    pass
                else:
                    upper_start = time.perf_counter()
                    i_ref = controller.upper(state, w, cfg)
                    upper_ms = (time.perf_counter() - upper_start) * 1000.0
                    upper_ms_sample = upper_ms
                    upper_command_sent = float(i_ref)
                    payload = {
                        "type": "upper_hierarchical", "controller_variant": "PROPOSED",
                        "I_ref_A": float(i_ref), "seq": int(d.get("seq", 0)),
                        "valid_for_s": COMMAND_TIMEOUT_S,
                        # These are forwarded unchanged by the GIGA to MATLAB/Simulink.
                        "PV_available_W": float(fc["PV_available_W"]),
                        "O2_demand_Lpm": float(fc["O2_demand_Lpm"]),
                        "Tamb_C": float(fc["Tamb_C"]),
                        "tariff_KES_kWh": float(fc["tariff_KES_kWh"]),
                        "H2_draw_kg_s": float(fc["H2_export_demand_kg_s"]),
                        "H2_leak_ppm_inject": float(fc["H2_leak_ppm_inject"]),
                        "sensor_fault_inject": bool(sensor_fault_inject),
                        "purity_alarm_inject": bool(purity_alarm_inject),
                        "hour": float(fc["hour"]),
                        "scenario": scenario,
                        "fault_state": fault,
                    }
                    cli.publish(cmd_topic, json.dumps(payload), qos=1)
                    last_upper_sim_s = sim_t
                    _diag(f"[hil] upper t={sim_t:.0f}s Iref={i_ref:.1f}A solve={upper_ms:.2f}ms fault={fault}")
                if prev_fault != "nominal" and fault == "nominal":
                    pending_fault_reset_ack = True
                    fault_ack_sent = False
                prev_fault = fault

            if pending_fault_reset_ack and not fault_ack_sent:
                clean_leak = float(d.get("H2_leak_ppm", 0.0) or 0.0) <= 500.0
                clean_ppm = float(d.get("H2_in_O2_ppm", 0.0) or 0.0) <= cfg["H2_in_O2_limit_ppm"]
                if bool(d.get("interlock", False)) and clean_leak and clean_ppm:
                    cli.publish(cmd_topic, json.dumps({"type": "acknowledge_fault_reset", "run_id": run_id}), qos=1)
                    fault_ack_sent = True
                    pending_fault_reset_ack = False
                    _diag(f"[hil] operator fault acknowledgement requested at t={sim_t:.0f}s; trajectory state is preserved")

            record = {key: d.get(key, "") for key in log_fields}
            record["scenario"] = scenario
            record["fault_state"] = fault
            record["sensor_fault_inject"] = bool(fault == "sensor_out_of_range")
            record["purity_alarm_inject"] = bool(fault == "purity_alarm")
            record["upper_solve_ms"] = upper_ms_sample
            record["upper_command_sent_A"] = upper_command_sent
            # The telemetry communication_path is the supervisory MQTT path.
            # The plant bridge is independently owned by MATLAB over USB CDC Serial.
            record["plant_link"] = "USB_SERIAL"
            record["supervisory_link"] = d.get("communication_path", "unknown")
            record["wall_elapsed_s"] = float(time.time() - t0)
            writer.writerow(record)
            csv_file.flush()
            rows += 1
            last_sim_t = sim_t
            wall_now = time.time() - t0
            _progress(sim_t, wall_now, f"completed plant batches={rows}")
            _diag(
                f"[plant] row={rows} seq={d.get('seq', 'n/a')} t={sim_t:.1f}s "
                f"I={float(d.get('I_stack_A', 0) or 0):.2f}A "
                f"Iref={float(d.get('I_ref_A', 0) or 0):.2f}A "
                f"Vcell={float(d.get('V_cell_V', 0) or 0):.4f}V "
                f"Pstack={float(d.get('P_stack_W', 0) or 0):.1f}W "
                f"Psystem={float(d.get('P_system_W', 0) or 0):.1f}W "
                f"T={float(d.get('T_stack_C', 0) or 0):.2f}C "
                f"O2={float(d.get('O2_flow_Lpm', 0) or 0):.3f}/"
                f"{float(d.get('O2_demand_Lpm', 0) or 0):.3f}L/min "
                f"purity={float(d.get('purity_model_pct', 0) or 0):.5f}% "
                f"H2-in-O2={float(d.get('H2_in_O2_ppm', 0) or 0):.2f}ppm "
                f"PV={float(d.get('P_PV_W', 0) or 0):.1f}W "
                f"grid={float(d.get('P_grid_W', 0) or 0):.1f}W "
                f"O2buf={float(d.get('O2_buffer_Nm3', 0) or 0):.3f}Nm3 "
                f"H2buf={float(d.get('H2_buffer_Nm3', 0) or 0):.3f}Nm3 "
                f"P_H2={float(d.get('P_H2_bar', 0) or 0):.6f}bar "
                f"H2vent={float(d.get('H2_vent_kg_s', 0) or 0):.6g}kg/s "
                f"H2export={float(d.get('H2_export_kg_s', 0) or 0):.6g}kg/s "
                f"batch={d.get('batch_steps_completed', 'n/a')}/"
                f"{d.get('controller_batch_steps', 'n/a')} "
                f"MATLAB_batch_ms={float(d.get('plant_batch_wall_ms', 0) or 0):.2f} "
                f"lower_batch_ms={float(d.get('lower_batch_wall_ms', 0) or 0):.2f} "
                f"interlock={bool(d.get('interlock', False))} "
                f"trip_reason={d.get('trip_reason', '')}"
            )
    except KeyboardInterrupt:
        termination_reason = "interrupted_by_user"
        _diag("[hil] interrupted by user")
    except Exception as exc:
        termination_reason = "collector_error"
        _diag(f"[hil][FATAL] collector exception: {type(exc).__name__}: {exc}")
        raise
    finally:
        try:
            cli.publish(cmd_topic, json.dumps({"type": "stop_run"}), qos=1)
            time.sleep(0.2)
        except Exception:
            pass
        cli.loop_stop()
        cli.disconnect()
        csv_file.close()
        _diag(f"[hil] wrote {rows} telemetry rows -> {out_path}")
        _drain_board_logs()
        wall_final = float(time.time() - t0)
        _progress(last_sim_t, wall_final, f"final status={termination_reason}; rows={rows}")
        hb_final = (handshake_state.get("last_heartbeat", {})
                    or handshake_state.get("last_startup_heartbeat", {}))
        _diag(f"[final] reason={termination_reason} rows={rows} last_sim_time_s={last_sim_t} "
              f"run_heartbeat={hb_final}")
        try:
            logged = pd.read_csv(out_path)
            wall_elapsed = float(time.time() - t0)
            first_sim = float(logged["time_s"].iloc[0]) if len(logged) else None
            last_sim = float(logged["time_s"].iloc[-1]) if len(logged) else None
            sim_span = (last_sim - first_sim) if first_sim is not None and last_sim is not None else 0.0
            summary = {"csv_file": out_path.name, "diagnostics_file": diag_path.name,
                       "rows": int(len(logged)),
                       "status": termination_reason,
                       "demand_source": (str(args.demand_profile_csv) if demand_profile is not None else "synthetic_diurnal_model"),
                       "weather_source": str(getattr(args, "weather_profile_csv", None) or WEATHER_CSV.name),
                       "plant_link": "USB_SERIAL",
                       "supervisory_link": "MQTT",
                       "wall_duration_s": wall_elapsed,
                       "simulated_span_s": sim_span,
                       "simulated_hours": sim_span / 3600.0,
                       "simulated_elapsed_from_reset_s": last_sim,
                       "simulated_hours_from_reset": (last_sim / 3600.0) if last_sim is not None else None,
                       "simulated_seconds_per_wall_second": (sim_span / wall_elapsed) if wall_elapsed > 0 else None,
                       "first_sim_time_s": first_sim,
                       "last_sim_time_s": last_sim,
                       "sim_duration_target_s": getattr(args, "sim_duration_s", None),
                       "sim_duration_target_reached": bool(
                           getattr(args, "sim_duration_s", None) is not None
                           and last_sim is not None
                           and last_sim >= float(args.sim_duration_s)
                       ),
                       "sim_duration_completion_pct": (
                           min(100.0, 100.0 * float(last_sim) / float(args.sim_duration_s))
                           if getattr(args, "sim_duration_s", None) is not None
                           and float(args.sim_duration_s) > 0 and last_sim is not None else None
                       ),
                       "first_seq": int(logged["seq"].iloc[0]) if len(logged) else None,
                       "last_seq": int(logged["seq"].iloc[-1]) if len(logged) else None,
                       "scenario": scenario,
                       "event_start_s": (float(args.event_start_s) if args.event_start_s is not None else None),
                       "event_duration_s": (float(args.event_duration_s) if args.event_duration_s is not None else None),
                       "weather_offset_hours": float(args.weather_offset_hours)}
            numeric = ["I_stack_A", "T_stack_C", "V_cell_V", "purity_model_pct", "H2_in_O2_ppm",
                       "O2_flow_Lpm", "O2_demand_Lpm", "P_PV_W", "PV_available_W", "P_grid_W",
                       "lower_loop_ms", "lower_solve_ms", "plant_step_ms", "cycle_latency_ms", "upper_solve_ms",
                       "lower_batch_wall_ms", "plant_batch_wall_ms", "plant_batch_avg_step_ms"]
            for col in numeric:
                if col in logged.columns:
                    values = pd.to_numeric(logged[col], errors="coerce").dropna()
                    if len(values):
                        summary[col] = {"min": float(values.min()), "median": float(values.median()),
                                        "p95": float(values.quantile(0.95)), "max": float(values.max()),
                                        "mean": float(values.mean())}
            if len(logged):
                missed_col = (logged["missed_deadline"].astype(str).str.lower().isin(("true", "1", "yes"))
                              if "missed_deadline" in logged.columns else pd.Series(False, index=logged.index))
                deadline_counts = (pd.to_numeric(logged["missed_deadline_count"], errors="coerce").fillna(0)
                                   if "missed_deadline_count" in logged.columns else pd.Series(0, index=logged.index))
                summary["missed_deadline_rows"] = int(missed_col.sum())
                summary["missed_deadline_fraction"] = float(missed_col.mean())
                summary["deadline_count_final"] = int(deadline_counts.max())
                summary["deadline_count_increments_observed"] = int(max(0, deadline_counts.max() - deadline_counts.iloc[0]))
                summary["interlock_rows"] = int(logged["interlock"].astype(str).str.lower().isin(("true", "1", "yes")).sum())
                summary["O2_tracking_MAE_Lpm"] = float((pd.to_numeric(logged["O2_flow_Lpm"], errors="coerce")
                                                          - pd.to_numeric(logged["O2_demand_Lpm"], errors="coerce")).abs().mean())
                iref = pd.to_numeric(logged.get("I_ref_A", pd.Series(dtype=float)), errors="coerce").dropna()
                if len(iref):
                    summary["I_ref_min_A"] = float(iref.min())
                    summary["I_ref_median_A"] = float(iref.median())
                    summary["I_ref_max_A"] = float(iref.max())
                    summary["I_ref_unique_values_A"] = sorted(float(v) for v in iref.unique())
                    summary["I_ref_unique_count"] = int(iref.nunique())
                    summary["I_ref_change_count"] = int((iref.diff().fillna(0).abs() > 1e-6).sum())
                    summary["variable_setpoint_demonstrated"] = bool(iref.nunique() > 1)
                upper_cmd = pd.to_numeric(logged.get("upper_command_sent_A", pd.Series(dtype=float)), errors="coerce")
                summary["upper_command_update_count"] = int(upper_cmd.notna().sum())
                if "PV_available_W" in logged.columns:
                    pv = pd.to_numeric(logged["PV_available_W"], errors="coerce").dropna()
                    if len(pv):
                        summary["PV_available_min_W"] = float(pv.min())
                        summary["PV_available_max_W"] = float(pv.max())
                        summary["PV_available_unique_count"] = int(pv.nunique())
                if "O2_demand_Lpm" in logged.columns:
                    demand_values = pd.to_numeric(logged["O2_demand_Lpm"], errors="coerce").dropna()
                    if len(demand_values):
                        summary["O2_demand_min_Lpm"] = float(demand_values.min())
                        summary["O2_demand_max_Lpm"] = float(demand_values.max())
                        summary["O2_demand_unique_count"] = int(demand_values.nunique())
            summary["last_run_heartbeat"] = handshake_state.get("last_heartbeat", {})
            summary["last_startup_heartbeat"] = handshake_state.get("last_startup_heartbeat", {})
            summary["last_board_telemetry"] = handshake_state.get("last_board_telemetry", {})
            summary["last_giga_log"] = handshake_state.get("last_board_log", {})
            summary["plant_dt_s"] = 1.0
            summary["batch_steps_expected"] = 300
            summary["simulated_seconds_per_completed_batch"] = 300.0
            summary_path = out_path.with_name(out_path.stem + "_summary.json")
            summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
            _diag(f"[hil] compact summary -> {summary_path}")
        except Exception as summary_error:
            _diag(f"[hil] warning: could not make compact summary: {summary_error}")
        finally:
            try:
                diag_file.flush()
                diag_file.close()
            except Exception:
                pass


def compare_pil_to_sil(pil_file: str, sil_file: str) -> dict:
    """Interpolate a PIL trace onto SIL block-end timestamps and report errors.

    The two CSVs must represent the same scenario, initial state, weather offset,
    demand profile and controller settings. This utility does not make unmatched
    experiments comparable; it records the assumptions and common time span.
    """
    pil_path, sil_path = Path(pil_file), Path(sil_file)
    if not pil_path.is_absolute(): pil_path = ROOT / pil_path
    if not sil_path.is_absolute(): sil_path = ROOT / sil_path
    if not pil_path.exists() or not sil_path.exists():
        raise FileNotFoundError(f"PIL/SIL file not found: {pil_path} | {sil_path}")
    pil = pd.read_csv(pil_path)
    sil_df = pd.read_csv(sil_path)
    required_pil = {"time_s"}
    if missing := required_pil - set(pil.columns):
        raise ValueError(f"PIL CSV missing columns: {sorted(missing)}")
    required_sil = {"t_s"}
    if missing := required_sil - set(sil_df.columns):
        raise ValueError(f"SIL CSV missing columns: {sorted(missing)}")
    t_pil = pd.to_numeric(pil["time_s"], errors="coerce").to_numpy(float)
    t_sil_start = pd.to_numeric(sil_df["t_s"], errors="coerce").to_numpy(float)
    if "t_end_s" in sil_df.columns:
        t_sil = pd.to_numeric(sil_df["t_end_s"], errors="coerce").to_numpy(float)
    else:
        dt = float(load_cfg()["sil_dt_s"])
        t_sil = t_sil_start + dt
    valid_pil = np.isfinite(t_pil)
    valid_sil = np.isfinite(t_sil)
    pil, t_pil = pil.loc[valid_pil].copy(), t_pil[valid_pil]
    sil_df, t_sil = sil_df.loc[valid_sil].copy(), t_sil[valid_sil]
    if len(pil) < 2 or len(sil_df) < 2:
        raise ValueError("At least two valid timestamped rows are required in each file")
    order = np.argsort(t_pil); pil = pil.iloc[order].reset_index(drop=True); t_pil = t_pil[order]
    order = np.argsort(t_sil); sil_df = sil_df.iloc[order].reset_index(drop=True); t_sil = t_sil[order]
    # Drop duplicate PIL times before interpolation.
    keep = np.r_[True, np.diff(t_pil) > 0]
    pil = pil.loc[keep].reset_index(drop=True); t_pil = t_pil[keep]
    common = (t_sil >= t_pil[0]) & (t_sil <= t_pil[-1])
    if common.sum() < 2:
        raise ValueError("PIL and SIL files have fewer than two overlapping block-end timestamps")
    t_common = t_sil[common]
    pairs = {
        "I_stack_A": ("I_stack_A", "I_actual"),
        "V_cell_V": ("V_cell_V", "V_cell_V"),
        "T_stack_C": ("T_stack_C", "T_C"),
        "O2_flow_Lpm": ("O2_flow_Lpm", "O2_flow_Lpm"),
        "P_system_W": ("P_system_W", "P_system_W"),
        "P_grid_W": ("P_grid_W", "P_grid_W"),
        "P_PV_W": ("P_PV_W", "P_PV_W"),
        "O2_buffer_Nm3": ("O2_buffer_Nm3", "O2_buffer_Nm3"),
        "H2_buffer_Nm3": ("H2_buffer_Nm3", "H2_buffer_Nm3"),
        "purity_model_pct": ("purity_model_pct", "purity_next_pct"),
    }
    report = {"pil_file": pil_path.name, "sil_file": sil_path.name,
              "pil_rows": int(len(pil)), "sil_rows": int(len(sil_df)),
              "common_points": int(len(t_common)),
              "common_time_start_s": float(t_common[0]), "common_time_end_s": float(t_common[-1]),
              "signals": {},
              "warning": "Interpret errors only after confirming identical initial state, controller commands, weather, demand, tariff and fault schedule."}
    for name, (pil_col, sil_col) in pairs.items():
        if pil_col not in pil.columns or sil_col not in sil_df.columns:
            continue
        yp = pd.to_numeric(pil[pil_col], errors="coerce").to_numpy(float)
        ys = pd.to_numeric(sil_df.loc[common, sil_col], errors="coerce").to_numpy(float)
        good = np.isfinite(ys) & np.isfinite(np.interp(t_common, t_pil, yp))
        if not np.any(good):
            continue
        yp_i = np.interp(t_common[good], t_pil, yp)
        err = yp_i - ys[good]
        report["signals"][name] = {
            "n": int(len(err)), "bias": float(np.mean(err)),
            "MAE": float(np.mean(np.abs(err))), "RMSE": float(np.sqrt(np.mean(err ** 2))),
            "max_abs_error": float(np.max(np.abs(err))),
        }
    out = pil_path.with_name(pil_path.stem + "_vs_" + sil_path.stem + "_comparison.json")
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"[compare-pil] saved {out}")
    return report


def audit_sil(cfg: dict, days: int = 7, repeats: int = DEFAULT_SIL_REPEATS, require_complete: bool = True) -> dict:
    """Audit paired SIL traces and summaries before paper tables/figures are generated."""
    expected_runs = len(SCENARIO_NAMES) * len(CONTROLLER_NAMES) * int(repeats)
    expected_pairs = {(scenario, controller) for scenario in SCENARIO_NAMES for controller in CONTROLLER_NAMES}
    report = {
        "expected_days_per_repeat": int(days), "expected_repeats": int(repeats),
        "expected_scenarios": list(SCENARIO_NAMES), "expected_controllers": list(CONTROLLER_NAMES),
        "expected_run_files": int(expected_runs), "run_files_found": 0,
        "missing_run_files": [], "unexpected_run_files": [], "bad_run_files": [], "duplicate_summary_keys": [],
        "summary_rows": 0, "aggregate_summary_rows": 0,
        "expected_aggregate_rows": int(len(expected_pairs)),
        "data_quality_pass": False, "complete": False,
        "operational_constraint_totals": {},
        "note": "Constraint violations are reported as experiment outcomes; they are not silently treated as data-file errors."
    }
    raw_path = RESULTS / "sil_summary_raw.csv"
    agg_path = RESULTS / "sil_summary.csv"
    if not raw_path.exists() or not agg_path.exists():
        report["missing_summary_files"] = [x.name for x in (raw_path, agg_path) if not x.exists()]
        report["status"] = "missing_summaries"
        (RESULTS / "sil_validation_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        if require_complete:
            raise RuntimeError("SIL summary files are missing. Finish the requested SIL campaign before paper generation.")
        return report

    raw = pd.read_csv(raw_path)
    aggregate = pd.read_csv(agg_path)
    expected_filenames = {
        f"sil_r{repeat}_{scenario}_{controller}.csv"
        for repeat in range(1, int(repeats) + 1)
        for scenario in SCENARIO_NAMES
        for controller in CONTROLLER_NAMES
    }
    actual_filenames = {p.name for p in RESULTS.glob("sil_r*_*.csv")}
    report["unexpected_run_files"] = sorted(actual_filenames - expected_filenames)
    report["summary_rows"] = int(len(raw))
    report["aggregate_summary_rows"] = int(len(aggregate))
    key_cols = {"scenario", "controller", "repeat", "days"}
    missing_keys = sorted(key_cols - set(raw.columns))
    report["summary_key_columns_missing"] = missing_keys
    summary_key_counts = {}
    if not missing_keys:
        for _, r in raw.iterrows():
            try:
                key = (str(r["scenario"]), str(r["controller"]), int(r["repeat"]))
                summary_key_counts[key] = summary_key_counts.get(key, 0) + 1
            except (TypeError, ValueError, OverflowError):
                report.setdefault("bad_summary_rows", []).append("invalid_repeat_identifier")
        report["duplicate_summary_keys"] = [list(k) for k, v in summary_key_counts.items() if v > 1]
        expected_keys = {
            (scenario, controller, repeat)
            for repeat in range(1, int(repeats) + 1)
            for scenario, controller in expected_pairs
        }
        found_keys = set(summary_key_counts)
        report["summary_key_mismatches"] = {
            "missing": [list(k) for k in sorted(expected_keys - found_keys)],
            "unexpected": [list(k) for k in sorted(found_keys - expected_keys)]
        }
        day_values = pd.to_numeric(raw["days"], errors="coerce")
        report["summary_days_mismatch_rows"] = int((~np.isfinite(day_values.to_numpy(float)) | (day_values.to_numpy(float) != float(days))).sum())
    else:
        report["summary_key_mismatches"] = {"missing": ["key columns unavailable"], "unexpected": []}
        report["summary_days_mismatch_rows"] = int(len(raw))

    summary_numeric_cols = [
        "grid_cost_per_day_KES", "h2_kg", "o2_deficit_L", "purity_min_pct",
        "solar_share_pct", "objective_realised_KES", "upper_solve_ms_median"
    ]
    missing_metric_cols = sorted(set(summary_numeric_cols) - set(raw.columns))
    summary_nonfinite = {}
    for col in summary_numeric_cols:
        if col in raw.columns:
            vals = pd.to_numeric(raw[col], errors="coerce").to_numpy(float)
            count_bad = int((~np.isfinite(vals)).sum())
            if count_bad:
                summary_nonfinite[col] = count_bad
    report["summary_metric_columns_missing"] = missing_metric_cols
    report["summary_nonfinite_metrics"] = summary_nonfinite

    aggregate_pairs = set()
    aggregate_missing = sorted({"scenario", "controller"} - set(aggregate.columns))
    if not aggregate_missing:
        aggregate_pairs = set(zip(aggregate["scenario"].astype(str), aggregate["controller"].astype(str)))
    report["aggregate_key_columns_missing"] = aggregate_missing
    report["aggregate_key_mismatches"] = {
        "missing": [list(k) for k in sorted(expected_pairs - aggregate_pairs)],
        "unexpected": [list(k) for k in sorted(aggregate_pairs - expected_pairs)]
    } if not aggregate_missing else {"missing": ["key columns unavailable"], "unexpected": []}

    critical_cols = [
        "t_s", "t_end_s", "scenario", "controller", "I_ref", "I_cmd", "I_actual", "T_C", "V_cell_V",
        "O2_flow_Lpm", "O2_demand_Lpm", "O2_buffer_Nm3", "H2_buffer_Nm3",
        "PV_avail", "P_PV_W", "P_grid_W", "purity_next_pct",
        "energy_kWh_block", "grid_cost_KES_block", "o2_deficit_L_block",
        "purity_violations_block", "h2_o2_violations_block",
        "temperature_violations_block", "voltage_violations_block", "grid_limit_violations_block"
    ]
    warmup_s = max(0.0, float(cfg.get("warmup_hours", 0.0)) * 3600.0)
    requested_end_s = warmup_s + float(days) * 86400.0
    expected_dt = float(cfg.get("sil_dt_s", 300.0))
    for repeat in range(1, int(repeats) + 1):
        for scenario in SCENARIO_NAMES:
            for controller in CONTROLLER_NAMES:
                path = RESULTS / f"sil_r{repeat}_{scenario}_{controller}.csv"
                if not path.exists():
                    report["missing_run_files"].append(path.name)
                    continue
                report["run_files_found"] += 1
                try:
                    d = pd.read_csv(path)
                    miss = sorted(set(critical_cols) - set(d.columns))
                    if miss:
                        report["bad_run_files"].append({"file": path.name, "reason": "missing_columns", "columns": miss})
                        continue
                    bad = []
                    numeric_critical = [c for c in critical_cols if c not in ("scenario", "controller")]
                    for col in numeric_critical:
                        values = pd.to_numeric(d[col], errors="coerce").to_numpy(float)
                        if not np.all(np.isfinite(values)):
                            bad.append(f"nonfinite:{col}")
                    if len(d) == 0:
                        bad.append("empty_file")
                    else:
                        t_end = pd.to_numeric(d["t_end_s"], errors="coerce").to_numpy(float)
                        if not np.isfinite(t_end).all() or float(np.max(t_end)) < requested_end_s - expected_dt - 1e-6:
                            end_value = float(np.nanmax(t_end)) if np.isfinite(t_end).any() else float("nan")
                            bad.append(f"short_or_invalid_run:end={end_value:.3f}s expected>={requested_end_s-expected_dt:.3f}s")
                        if set(d["scenario"].astype(str).unique()) != {scenario} or set(d["controller"].astype(str).unique()) != {controller}:
                            bad.append("scenario_or_controller_mismatch")
                        if not np.all(np.diff(pd.to_numeric(d["t_s"], errors="coerce").to_numpy(float)) >= 0):
                            bad.append("nonmonotonic_time")
                    if bad:
                        report["bad_run_files"].append({"file": path.name, "reason": bad})
                except Exception as exc:
                    report["bad_run_files"].append({"file": path.name, "reason": f"read_error:{type(exc).__name__}:{exc}"})

    constraint_cols = ["o2_deficit_L", "purity_violations", "h2_o2_violations",
                       "temperature_violations", "voltage_violations", "grid_limit_violations",
                       "interlock_steps"]
    for col in constraint_cols:
        if col in raw.columns:
            vals = pd.to_numeric(raw[col], errors="coerce")
            report["operational_constraint_totals"][col] = float(vals.sum(min_count=1)) if vals.notna().any() else None
    report["operational_constraints_by_scenario_controller"] = {}
    if {"scenario", "controller"}.issubset(raw.columns):
        for (scenario, controller), group in raw.groupby(["scenario", "controller"], dropna=False):
            key = f"{scenario}/{controller}"
            report["operational_constraints_by_scenario_controller"][key] = {}
            for col in constraint_cols:
                if col in group.columns:
                    values = pd.to_numeric(group[col], errors="coerce")
                    report["operational_constraints_by_scenario_controller"][key][col] = (
                        float(values.sum(min_count=1)) if values.notna().any() else None
                    )

    # Keep completeness conditions explicit so missing metadata never passes by default.
    report["complete"] = bool(
        report["run_files_found"] == expected_runs
        and not report["missing_run_files"]
        and not report["unexpected_run_files"]
        and not report["bad_run_files"]
        and not missing_keys
        and not report.get("bad_summary_rows", [])
        and not report.get("summary_key_mismatches", {}).get("missing", [])
        and not report.get("summary_key_mismatches", {}).get("unexpected", [])
        and report["summary_days_mismatch_rows"] == 0
        and not report["summary_metric_columns_missing"]
        and not report["summary_nonfinite_metrics"]
        and not report["aggregate_key_columns_missing"]
        and not report["aggregate_key_mismatches"]["missing"]
        and not report["aggregate_key_mismatches"]["unexpected"]
        and report["summary_rows"] == expected_runs
        and report["aggregate_summary_rows"] == report["expected_aggregate_rows"]
    )
    report["data_quality_pass"] = report["complete"] and not report["duplicate_summary_keys"]
    report["status"] = "PASS" if report["data_quality_pass"] else "INCOMPLETE_OR_INVALID"
    out = RESULTS / "sil_validation_report.json"
    out.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(f"[audit] {report['status']} — {report['run_files_found']}/{expected_runs} matched run files; "
          f"{len(report['missing_run_files'])} missing; {len(report['unexpected_run_files'])} unexpected; "
          f"{len(report['bad_run_files'])} invalid; "
          f"constraint totals: {report['operational_constraint_totals']}")
    print(f"[audit] report saved: {out}")
    if require_complete and not report["data_quality_pass"]:
        raise RuntimeError(
            f"SIL audit failed ({report['run_files_found']}/{expected_runs} matched). "
            "Do not generate manuscript tables yet; inspect m2_data/results/sil_validation_report.json."
        )
    return report


# ==================== PAPER FIGURES / SUMMARY ====================
def paper(cfg: dict, days: int = 7, repeats: int = DEFAULT_SIL_REPEATS) -> None:
    """Generate paper figures only from a complete, audited matched SIL campaign."""
    audit_sil(cfg, days=days, repeats=repeats, require_complete=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    raw_path = RESULTS / "sil_summary_raw.csv"
    agg_path = RESULTS / "sil_summary.csv"
    if not raw_path.exists() or not agg_path.exists():
        raise FileNotFoundError("Run `python m2_dashboard.py sil` before `paper`.")
    raw = pd.read_csv(raw_path)
    agg = pd.read_csv(agg_path)
    PAPER.mkdir(parents=True, exist_ok=True)

    # Fig. 9: controller metrics are grouped by scenario; error bars are 95% CIs.
    metrics = [
        ("grid_cost_per_day_KES", "Grid cost (KES/day)"),
        ("h2_kg", "H$_2$ produced (kg/7 d)"),
        ("o2_deficit_L", "O$_2$ deficit (L/7 d)"),
        ("upper_solve_ms_median", "Upper-layer median solve (ms)"),
    ]
    scenarios = list(SCENARIO_NAMES)
    controllers = list(CONTROLLER_NAMES)
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), dpi=220)
    x = np.arange(len(scenarios))
    width = 0.8 / max(1, len(controllers))
    for ax, (metric, ylabel) in zip(axes.flat, metrics):
        for ci, controller in enumerate(controllers):
            means, errors = [], []
            for scenario in scenarios:
                subset = raw[(raw["scenario"] == scenario) & (raw["controller"] == controller)][metric].dropna()
                mean = float(subset.mean()) if len(subset) else np.nan
                sd = float(subset.std(ddof=1)) if len(subset) > 1 else 0.0
                # Student-t 95% CI lookup for the requested three matched repeats; normal
                # approximation is retained for larger samples.
                tcrit = {2:12.706,3:4.303,4:3.182,5:2.776,6:2.571,7:2.447,8:2.365,9:2.306,10:2.262}.get(len(subset),1.96)
                means.append(mean)
                errors.append(tcrit * sd / math.sqrt(len(subset)) if len(subset) > 1 else 0.0)
            xpos = x - 0.4 + (ci + 0.5) * width
            ax.bar(xpos, means, width=width, yerr=errors, capsize=2, label=controller)
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels([v.replace("_", "\n") for v in scenarios], fontsize=8)
        ax.grid(axis="y", alpha=0.25)
    axes[0, 0].legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(PAPER / "fig09_comparison.png", bbox_inches="tight")
    plt.close(fig)

    # Figs. 5--8: Proposed time series. No embedded title; the LaTeX caption
    # supplies the figure title. PV is displayed in kW.
    scenario_figures = [(5, "clear_sky"), (6, "cloudy_rainy"),
                        (7, "oxygen_surge"), (8, "communication_loss")]
    fault_styles = [("gsm_fallback_simulated", "tab:orange"),
                    ("both_down", "tab:red"), ("h2_leak", "tab:purple")]
    for number, scenario in scenario_figures:
        csv_path = RESULTS / f"sil_r1_{scenario}_PROPOSED.csv"
        if not csv_path.exists():
            print(f"[paper] warning: missing {csv_path.name}; Fig. {number} skipped")
            continue
        d = pd.read_csv(csv_path).copy()
        warmup_s = float(cfg.get("warmup_hours", 0.0)) * 3600.0
        d = d.loc[pd.to_numeric(d["t_s"], errors="coerce") >= warmup_s].copy()
        d["h"] = (pd.to_numeric(d["t_s"], errors="coerce") - warmup_s) / 3600.0
        d["PV_avail_kW"] = pd.to_numeric(d["PV_avail"], errors="coerce") / 1000.0
        series = [("PV_avail_kW", "PV available (kW)"),
                  ("P_grid_W", "Grid power (W)"),
                  ("I_stack_A", "Stack current (A)"),
                  ("O2_buffer_Nm3", "O$_2$ buffer (Nm$^3$)")]
        fig, axes = plt.subplots(4, 1, figsize=(11, 8), dpi=220, sharex=True)
        for ax, (column, ylabel) in zip(axes, series):
            ax.plot(d["h"], d[column], linewidth=0.85)
            ax.set_ylabel(ylabel, fontsize=9)
            ax.tick_params(labelsize=8)
            ax.grid(alpha=0.25)
            if "fault_state" in d.columns:
                for fault_name, colour in fault_styles:
                    mask = d["fault_state"].astype(str) == fault_name
                    if mask.any():
                        ax.axvspan(d.loc[mask, "h"].min(), d.loc[mask, "h"].max(),
                                   color=colour, alpha=0.12)
        axes[-1].set_xlabel("Simulated time (h)", fontsize=9)
        fig.tight_layout()
        fig.savefig(PAPER / f"fig{number:02d}_{scenario}.png", bbox_inches="tight")
        plt.close(fig)

    # Fig. 10--11: fault response of Proposed.
    for number, scenario in [(10, "sensor_fault_leak"), (11, "communication_loss")]:
        csv_path = RESULTS / f"sil_r1_{scenario}_PROPOSED.csv"
        if not csv_path.exists():
            continue
        d = pd.read_csv(csv_path).copy()
        warmup_s = float(cfg.get("warmup_hours", 0.0)) * 3600.0
        d = d.loc[pd.to_numeric(d["t_s"], errors="coerce") >= warmup_s].copy()
        d["h"] = (pd.to_numeric(d["t_s"], errors="coerce") - warmup_s) / 3600.0
        series = [("I_stack_A", "Stack current (A)"),
                  ("H2_kg_s", "H$_2$ production (kg/s)"),
                  ("purity_next_pct", "Modelled oxygen purity (%)")]
        fig, axes = plt.subplots(3, 1, figsize=(11, 6.5), dpi=220, sharex=True)
        for ax, (column, ylabel) in zip(axes, series):
            ax.plot(d["h"], d[column], linewidth=0.9)
            ax.set_ylabel(ylabel, fontsize=9)
            ax.tick_params(labelsize=8)
            ax.grid(alpha=0.25)
            if "fault_state" in d.columns:
                for fault_name, colour in fault_styles:
                    mask = d["fault_state"].astype(str) == fault_name
                    if mask.any():
                        ax.axvspan(d.loc[mask, "h"].min(), d.loc[mask, "h"].max(),
                                   color=colour, alpha=0.12)
        axes[-1].set_xlabel("Simulated time (h)", fontsize=9)
        fig.tight_layout()
        fig.savefig(PAPER / f"fig{number:02d}_{scenario}.png", bbox_inches="tight")
        plt.close(fig)

    # Fig. 12: one shared-axis, four-controller oxygen-surge comparison.
    fig_path = RESULTS / "sil_r1_oxygen_surge_"
    surge_files = {c: RESULTS / f"sil_r1_oxygen_surge_{c}.csv" for c in controllers}
    if all(path.exists() for path in surge_files.values()):
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), dpi=220, sharex=True)
        for controller in controllers:
            d = pd.read_csv(surge_files[controller])
            h = d["t_s"].to_numpy(dtype=float) / 3600.0
            axes[0, 0].plot(h, d["O2_demand_Lpm"], linestyle="--", linewidth=0.8, alpha=0.5)
            axes[0, 0].plot(h, d["O2_flow_Lpm"], linewidth=0.85, label=controller)
            axes[0, 1].plot(h, d["O2_buffer_Nm3"], linewidth=0.85, label=controller)
            axes[1, 0].plot(h, d["purity_next_pct"], linewidth=0.85, label=controller)
            if "PV_avail" in d.columns and "P_PV_W" in d.columns:
                curtail = np.maximum(0.0, d["PV_avail"].to_numpy() - d["P_PV_W"].to_numpy()) / 1000.0
                axes[1, 1].plot(h, curtail, linewidth=0.85, label=controller)
        axes[0, 0].set_ylabel("O$_2$ flow / demand (L/min)")
        axes[0, 1].set_ylabel("O$_2$ buffer (Nm$^3$)")
        axes[1, 0].set_ylabel("Modelled purity (%)")
        axes[1, 1].set_ylabel("PV curtailment (kW)")
        for ax in axes.flat:
            ax.grid(alpha=0.25)
            ax.tick_params(labelsize=8)
        axes[1, 0].set_xlabel("Simulated time (h)")
        axes[1, 1].set_xlabel("Simulated time (h)")
        axes[0, 0].set_xlabel("Simulated time (h)")
        axes[0, 1].set_xlabel("Simulated time (h)")
        axes[0, 1].legend(fontsize=7, ncol=2)
        fig.tight_layout()
        fig.savefig(PAPER / "fig12_oxygen_surge_comparison.png", bbox_inches="tight")
        plt.close(fig)

    agg.to_csv(PAPER / "table6_summary.csv", index=False)
    constraint_cols = [c for c in (
        "o2_deficit_L", "purity_violations", "h2_o2_violations", "temperature_violations",
        "voltage_violations", "grid_limit_violations", "interlock_steps"
    ) if c in raw.columns]
    if constraint_cols:
        constraint_audit = raw.groupby(["scenario", "controller"])[constraint_cols].agg(["sum", "mean", "max"])
        constraint_audit = _flatten_cols(constraint_audit.reset_index())
        constraint_audit.to_csv(PAPER / "table_sil_constraint_audit.csv", index=False)
    validation_report = RESULTS / "sil_validation_report.json"
    if validation_report.exists():
        (PAPER / "sil_validation_report.json").write_text(validation_report.read_text(encoding="utf-8"), encoding="utf-8")
    print(f"[paper] generated the audited {len(SCENARIO_NAMES)}-scenario/{len(CONTROLLER_NAMES)}-controller comparison, "
          f"constraint audit table and figures in {PAPER}")



# ==================== SELF-TESTS / CLI ====================
def self_test(cfg: dict) -> None:
    x = np.array([
        cfg["initial_temperature_C"],
        float(cfg["initial_O2_inventory_Nm3"]) * 1.01325e5 / (R * 273.15),
        0.0, cfg["I_min_A"], 0.0, 0.0,
    ], dtype=float)
    fc = {"PV_available_W": 150000.0, "Tamb_C": 25.0,
          "O2_demand_Lpm": cfg["hospital_demand_Lpm"], "hour": 12.0,
          "tariff_KES_kWh": cfg["tariff_mid_KES"]}
    x_next, y = plant_step(x, 500.0, fc, cfg, dt=cfg["sil_dt_s"])
    assert x_next.shape == (6,)
    assert y["P_grid_W"] >= 0.0
    assert y["O2_prod_Lpm"] > 0.0
    assert y["O2_flow_Lpm"] <= fc["O2_demand_Lpm"] + 1e-9
    assert y["H2_export_kg_step"] == 0.0  # no declared offtake means no assumed sale
    assert y["H2_vent_kg_step"] >= 0.0
    assert y["H2_in_O2_ppm"] <= cfg["H2_in_O2_limit_ppm"]
    # Safety regression: a zero-current trip is not clipped to I_min_A.
    _, trip_y = plant_step(x, 0.0, fc, cfg, dt=cfg["hil_dt_s"])
    assert trip_y["I_stack_A"] == 0.0
    assert trip_y["H2_kg_s"] == 0.0
    assert trip_y["O2_prod_Lpm"] == 0.0
    # Hydrogen offtake is capped by actual inventory plus interval production.
    export_fc = dict(fc, H2_export_demand_kg_s=1.0)
    _, export_y = plant_step(x, 500.0, export_fc, cfg, dt=cfg["hil_dt_s"])
    assert export_y["H2_export_kg_step"] <= export_y["H2_produced_kg_step"] + 1e-12
    assert export_y["H2_buffer_Nm3"] >= 0.0
    assert cfg["surrogate_train_T_min_C"] == 20.0
    assert cfg["surrogate_train_T_max_C"] == 80.0
    assert cfg["surrogate_train_Tamb_min_C"] == 10.0
    assert cfg["surrogate_train_Tamb_max_C"] == 35.0
    thermal_y = {"V_cell_V": 1.8, "T_next_C": 30.1, "P_O2_tank_bar": 10.0,
                 "P_H2_next_bar": 5.0, "purity_next_pct": 99.9, "H2_in_O2_ppm": 100.0, "P_grid_W": 1000.0,
                 "O2_buffer_Nm3": cfg["O2_reserve_Nm3"] + 1.0, "O2_unmet_L_step": 0.0}
    thermal_x = np.array([30.0, 1.0, 1.0, 100.0, 0.0, 0.0])
    assert _constraints_ok(thermal_y, thermal_x, cfg, x_previous=thermal_x)
    thermal_y["T_next_C"] = 29.9
    assert not _constraints_ok(thermal_y, thermal_x, cfg, x_previous=thermal_x)
    thermal_x[0] = 50.0
    thermal_y["T_next_C"] = 49.9
    assert not _constraints_ok(thermal_y, thermal_x, cfg, x_previous=thermal_x)
    print(f"[self-test] plant V={y['V_cell_V']:.3f} V; O2 production at 500 A={y['O2_prod_Lpm']:.2f} L/min; "
          f"hospital demand={fc['O2_demand_Lpm']:.2f} L/min; "
          f"grid={y['P_grid_W'] / 1000:.2f} kW; purity(model)={y['purity_next_pct']:.4f}%")
    print("[self-test] O2 production is Faraday stack output; 48 L/min is the hospital demand, not a stack-production cap.")
    print("[self-test] zero-current safety override: PASS")
    print("[self-test] Faraday gas production, interval oxygen delivery and capped H2 export: PASS")
    print("[self-test] 20–80 °C stack / 10–35 °C ambient surrogate coverage and 50 °C EMPC floor: PASS")
    if WEATHER_CSV.exists():
        weather = load_weather(cfg)
        print(f"[self-test] weather rows={len(weather)}; PV peak={weather.PV_available_W.max():.0f} W")
    else:
        print(f"[self-test] weather file not present (SIL unavailable until {WEATHER_CSV.name} is generated)")
    if SURROGATE.exists():
        try:
            net = MLP.load(SURROGATE)
            surrogate_hash = validate_surrogate_artifact(net, "self-test")
            print(f"[self-test] compatible frozen surrogate: {net.sizes}; SHA-256={surrogate_hash}")
        except Exception as exc:
            print(f"[self-test] frozen surrogate is stale/incompatible: {exc}")
            print("[self-test] run `python m2_dashboard.py train` before SIL/HIL; no old 14-output model will be accepted")
    else:
        print("[self-test] frozen surrogate absent; train before SIL/HIL")


def main() -> None:
    parser = argparse.ArgumentParser(description="Manuscript 2 PEM EMPC SIL/HIL toolkit")
    parser.add_argument("cmd", choices=["train", "evaluate-surrogate", "sil", "sil-quick", "rebuild-summary", "collect", "compare-pil", "compare-hil", "audit", "paper", "self-test"])
    parser.add_argument("--days", type=int, default=7)
    parser.add_argument("--repeats", type=int, default=DEFAULT_SIL_REPEATS,
                        help="Matched SIL repeats (default 3; five scenarios x four controllers x repeats).")
    parser.add_argument("--allow-expanded-campaign", action="store_true",
                        help="Explicitly permit more than 3 repeats and the longer SIL runtime.")
    parser.add_argument("--forecast-error-pct", type=float, default=None)
    parser.add_argument("--h2-price-kes-kg", type=float, default=None,
                        help="Override the declared H2 offtake price for a sensitivity run")
    parser.add_argument("--h2-export-demand-kg-s", type=float, default=None,
                        help="Explicit exogenous H2 offtake; zero means no sale/revenue credit")
    parser.add_argument("--weather-offset-hours", type=float, default=7.0)
    parser.add_argument("--scenario", choices=HIL_SCENARIOS, default="clear_sky")
    parser.add_argument("--broker", default=None, help="Override MQTT broker host (otherwise m2_config.json mqtt_host)")
    parser.add_argument("--port", type=int, default=None, help="Override MQTT port (otherwise m2_config.json mqtt_port)")
    parser.add_argument("--duration-s", type=float, default=3600.0,
                        help="Wall-clock run limit in seconds; also acts as the default wall timeout when --sim-duration-s is supplied.")
    parser.add_argument("--sim-duration-s", type=float, default=None,
                        help="Simulated plant-time endpoint. In compressed HIL, virtual 1-s steps are batched in 300-step exchanges; actual throughput is measured during the run.")
    parser.add_argument("--wall-timeout-s", type=float, default=0.0,
                        help="Explicit wall-clock safety timeout; when 0, --duration-s supplies the wall-clock limit.")
    parser.add_argument("--weather-profile-csv", default=None,
                        help="Optional verified hourly weather/PV CSV; defaults to nairobi_nasa_weather.csv beside this script.")
    parser.add_argument("--demand-profile-csv", default=None,
                        help="Measured/sourced O2 demand CSV with columns time_s,O2_demand_Lpm; if omitted, the synthetic diurnal demand model is used.")
    parser.add_argument("--require-real-profiles", action="store_true",
                        help="Fail unless both PV/weather and hospital demand profiles vary and are supplied from source data.")
    parser.add_argument("--upper-period-s", type=float, default=300.0)
    parser.add_argument("--event-start-s", type=float, default=None,
                        help="Override HIL surge/fault start in simulated seconds")
    parser.add_argument("--event-duration-s", type=float, default=None,
                        help="Override HIL event duration in simulated seconds")
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--pil-file", default=None, help="PIL CSV path for compare-pil")
    parser.add_argument("--sil-file", default=None, help="Matching SIL trajectory CSV path for compare-pil")
    parser.add_argument("--train-trajectories", type=int, default=None)
    parser.add_argument("--train-steps", type=int, default=None)
    parser.add_argument("--train-epochs", type=int, default=None, help="Override training epochs (useful for smoke tests)")
    args = parser.parse_args()
    cfg = load_cfg()
    if args.sim_duration_s is not None and args.sim_duration_s <= 0:
        parser.error("--sim-duration-s must be > 0")
    if args.wall_timeout_s < 0:
        parser.error("--wall-timeout-s must be >= 0")
    if args.upper_period_s <= 0:
        parser.error("--upper-period-s must be > 0")
    if args.h2_price_kes_kg is not None:
        if args.h2_price_kes_kg < 0:
            parser.error("--h2-price-kes-kg must be >= 0")
        cfg["H2_price_KES_kg"] = float(args.h2_price_kes_kg)
    if args.h2_export_demand_kg_s is not None:
        if args.h2_export_demand_kg_s < 0:
            parser.error("--h2-export-demand-kg-s must be >= 0")
        cfg["H2_export_demand_kg_s"] = float(args.h2_export_demand_kg_s)
    if args.cmd == "self-test":
        self_test(cfg)
    elif args.cmd == "train":
        train(cfg, args.train_trajectories, args.train_steps, args.train_epochs)
    elif args.cmd == "evaluate-surrogate":
        evaluate_surrogate(cfg)
    elif args.cmd == "sil":
        if args.repeats > MAX_DEFAULT_SIL_REPEATS and not args.allow_expanded_campaign:
            parser.error(
                f"--repeats {args.repeats} requests {len(SCENARIO_NAMES) * len(CONTROLLER_NAMES) * args.repeats} runs. "
                f"The standard campaign is {len(SCENARIO_NAMES)} scenarios x {len(CONTROLLER_NAMES)} controllers "
                f"x {DEFAULT_SIL_REPEATS} repeats = {len(SCENARIO_NAMES) * len(CONTROLLER_NAMES) * DEFAULT_SIL_REPEATS} runs. "
                "Use --allow-expanded-campaign only when the larger run count is intentional."
            )
        error_pct = float(cfg.get("forecast_error_pct", 0.0) if args.forecast_error_pct is None
                          else args.forecast_error_pct)
        sil(cfg, args.days, args.repeats, forecast_error_pct=error_pct)
    elif args.cmd == "sil-quick":
        sil(cfg, 1, 1, forecast_error_pct=0.0)
    elif args.cmd == "rebuild-summary":
        rebuild_sil_summary(cfg, args.days, args.repeats)
    elif args.cmd == "collect":
        collect(args)
    elif args.cmd in ("compare-pil", "compare-hil"):
        if not args.pil_file or not args.sil_file:
            parser.error(f"{args.cmd} requires --pil-file and --sil-file")
        compare_pil_to_sil(args.pil_file, args.sil_file)
    elif args.cmd == "audit":
        audit_sil(cfg, days=args.days, repeats=args.repeats, require_complete=True)
    elif args.cmd == "paper":
        paper(cfg, days=args.days, repeats=args.repeats)


if __name__ == "__main__":
    main()
