# Hierarchical EMPC for Grid–PV PEM Electrolysis with Hospital Oxygen and Green Hydrogen Co-Production

This repository contains the simulation, neural-surrogate training, MATLAB/Simulink plant-bridge, and embedded Arduino GIGA R1 WiFi code accompanying the manuscript:

> **Hierarchical Economic Model Predictive Control for Grid–PV PEM Electrolysis with Hospital Oxygen and Green Hydrogen Co-Production: Neural Surrogate and Embedded Processor-in-the-Loop Study**  
> Bill Kipkorir Maritim, Davies Rene Segera, and Thomas Ochuku Mbuya  
> *University of Nairobi*

---

## 1. Repository Structure

```text
PEM-Hospital-O2-H2-EMPC/
├── README.md                                   # Setup, architecture & reproducibility guide
├── Data/                                       # Input data, frozen surrogate, results & figures
│   ├── nairobi_nasa_weather.csv                # Required: 8,760-row hourly Nairobi solar & ambient trace
│   ├── sys_config.json                         # Required: locked plant, BoP, tariff & controller parameters
│   ├── surrogate.npz                           # Required: frozen 10-64-64-4 MLP weights & input scaler
│   ├── surrogate_metrics.json                  # Required: held-out 1-step & rollout metrics + SHA-256 hash
│   ├── surrogate_training.csv                  # Optional: 37,500-row dataset (needed only for evaluate-surrogate)
│   ├── results/                                # All archived SIL & PIL/HIL run outputs
│   │   ├── sil_summary.csv                     # 250-run SIL aggregate summary (mean, SD, 95% CI)
│   │   ├── sil_summary_raw.csv                 # Per-repeat SIL metrics across all 250 runs
│   │   ├── sil_validation_report.json          # Automated SIL completeness & conservation audit
│   │   ├── hil_clear_sky_1b56ddb2.csv          # 7-day clear-sky PIL/HIL telemetry (2,016 batches)
│   │   ├── hil_clear_sky_1b56ddb2_summary.json # Summary statistics for the 7-day PIL/HIL run
│   │   ├── hil_sensor_out_of_range_b5578d38.csv
│   │   ├── hil_purity_alarm_86a97ea1.csv
│   │   ├── hil_sensor_fault_leak_25755226.csv
│   │   └── hil_communication_loss_2771cc39.csv
│   └── paper_outputs/                          # Publication figures & summary tables
│       ├── fig05_clear_sky.png
│       ├── fig06_cloudy_rainy.png
│       ├── fig07_oxygen_surge.png
│       ├── fig08_communication_loss.png
│       ├── fig09_comparison.png
│       ├── fig10_sensor_fault_leak.png
│       ├── fig11_communication_loss.png
│       ├── fig12_oxygen_surge_comparison.png
│       └── table_sil_constraint_audit.csv
└── Sys Files/                                  # Executable scripts, embedded firmware & Simulink model
    ├── pem_empc_runner.py                      # Unified CLI runner (surrogate, SIL, PIL & paper plots)
    ├── Arduino_GIGA_LowerMPC.ino               # Arduino GIGA R1 WiFi (Cortex-M7) embedded MPC & interlocks
    ├── plant_fn.m                              # Six-state dynamic PEM plant function (1.0 s virtual step)
    └── PEM_Step.slx                            # Fixed-step Simulink reference verification model
```

### Architecture Overview

1. **Upper Supervisory Layer (`300 s` interval, `6`-step / `30 min` horizon):**  
   Evaluates `17` candidate stack-current references using a frozen **$10\text{--}64\text{--}64\text{--}4$ hybrid neural-network surrogate** (predicting cell voltage, next stack temperature, next modelled dry-basis $\text{O}_2$ purity, and next stress index) coupled with algebraic Faraday gas flows, buffer balances, and balance-of-plant (BoP) power equations.
2. **Lower Embedded Layer (`1.0 s` virtual interval, `5`-step horizon):**  
   Runs natively on an **Arduino GIGA R1 WiFi (STM32H747XI Cortex-M7 @ 480 MHz)** to track the supervisory current reference subject to $\pm 50\text{ A/s}$ ramp limits, $5\text{ A}$ current granularity, command freshness checks, and local protective interlocks.

---

## 2. Requirements & Setup

### Python Environment
Python `3.10+` is required with standard scientific and communication packages:
```bash
pip install numpy pandas scipy matplotlib torch paho-mqtt pyserial
```

### Hardware & MATLAB/Simulink (For PIL/HIL Runs Only)
* **Microcontroller:** Arduino GIGA R1 WiFi (`STM32H747XI`, compiled for the Cortex-M7 core via Arduino IDE `2.x` with `Arduino Mbed OS Giga Boards`).
* **Plant Bridge:** MATLAB/Simulink (`R2023b` or newer) connected to the Arduino GIGA R1 over USB Serial (`115200` baud).
* **Broker:** MQTT broker (configured in `Data/sys_config.json`; laboratory prototype tested via Wi-Fi MQTT).

---

## 3. Reproducing the Results

All CLI commands below are executed from inside the **`Sys Files/`** directory:
```bash
cd "Sys Files"
```

### A. Self-Test & Configuration Audit
Verify environment integrity and configuration parameters:
```bash
python pem_empc_runner.py self-test
```

### B. Train & Evaluate the $10\text{--}64\text{--}64\text{--}4$ Hybrid Neural Surrogate
Trains on $30{,}000$ grouped transitions ($12$ groups) and evaluates held-out 1-step and recursive multi-step accuracy on $7{,}500$ samples ($3$ held-out groups) plus step-size convergence ($300\text{ s}$ vs. $1\text{ s}$):
```bash
python pem_empc_runner.py train
python pem_empc_runner.py evaluate-surrogate
```

### C. Run the 250-Run Software-in-the-Loop (SIL) Campaign
Executes all **5 controllers** (`PID`, `SINGLE_MPC`, `BUFFER_RULE`, `FULL_MODEL_EMPC`, `PROPOSED`) across **5 scenarios** (`clear_sky`, `cloudy_rainy`, `oxygen_surge`, `communication_loss`, `sensor_fault_leak`) and **10 weekly repeats** ($24\text{ h}$ warm-up excluded + $7$ measured days per run):
```bash
python pem_empc_runner.py sil --days 7 --repeats 10 --allow-expanded-campaign
python pem_empc_runner.py paper --days 7 --repeats 10
```
Outputs are written directly to `Data/results/` and `Data/paper_outputs/`.

### D. Active Hydrogen-Offtake Sensitivity Check
Evaluates downstream hydrogen export ($\dot{m}_{\text{H}_2,\text{export,req}} = 1.5 \times 10^{-4}\text{ kg/s} = 12.96\text{ kg/day}$ at $800\text{ KES/kg}$, weight $0.5$):
```bash
python pem_empc_runner.py sil --days 1 --repeats 1 --h2-export-demand-kg-s 0.00015 --h2-price-kes-kg 800
```

### E. Embedded Processor-in-the-Loop (PIL / Time-Compressed HIL) Runs
With `Arduino_GIGA_LowerMPC.ino` flashed to the Arduino GIGA R1 WiFi and the MATLAB USB-serial bridge active inside `Sys Files/`:
```bash
# 7-day clear-sky batch-accelerated trajectory (604,800 simulated seconds; 1-h wall cap)
python pem_empc_runner.py collect --scenario clear_sky --sim-duration-s 604800 --duration-s 3600

# Planned hardware fault-injection verification runs (triggered at t = 900 s)
python pem_empc_runner.py collect --scenario sensor_out_of_range --sim-duration-s 1800 --duration-s 1800 --event-start-s 900
python pem_empc_runner.py collect --scenario purity_alarm --sim-duration-s 1800 --duration-s 1800 --event-start-s 900
python pem_empc_runner.py collect --scenario sensor_fault_leak --sim-duration-s 1800 --duration-s 1800 --event-start-s 900
python pem_empc_runner.py collect --scenario communication_loss --sim-duration-s 2400 --duration-s 1800 --event-start-s 900
```

---

## 4. Scope & Safety Disclaimer

* **Simulation & Embedded Co-Simulation Boundary:** All electrochemical, thermal, gas-buffer, and crossover/purity states in the SIL and PIL/HIL runs are generated by the six-state dynamic plant model; physical product-gas analysers and physical electrolyser hardware are not part of the test bench.
* **Clinical Use:** The $99.5\%$ dry-basis oxygen purity threshold is a model-enforced control constraint and does not constitute medical-gas certification or clinical validation (cf. ISO 7396-1 / ISO 22734-1).

---

## Citation

If you use this code or dataset in your research, please cite:
```bibtex
@article{maritim2026hierarchical,
  title   = {Hierarchical Economic Model Predictive Control for Grid--PV PEM Electrolysis with Hospital Oxygen and Green Hydrogen Co-Production: Neural Surrogate and Embedded Processor-in-the-Loop Study},
  author  = {Maritim, Bill Kipkorir and Segera, Davies Rene and Mbuya, Thomas Ochuku},
  year    = {2026},
  note    = {University of Nairobi}
}
```
