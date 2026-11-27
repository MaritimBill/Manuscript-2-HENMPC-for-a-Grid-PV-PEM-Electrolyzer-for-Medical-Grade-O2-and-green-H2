# Hierarchical EMPC for Grid–PV PEM Electrolysis with Hospital Oxygen and Green Hydrogen Co-Production

This repository contains the simulation, neural-surrogate training, MATLAB/Simulink plant-bridge, and embedded Arduino GIGA R1 WiFi code accompanying the manuscript:

> **Hierarchical Economic Model Predictive Control for Grid–PV PEM Electrolysis with Hospital Oxygen and Green Hydrogen Co-Production: Neural Surrogate and Embedded Processor-in-the-Loop Study**  
> Bill Kipkorir Maritim, Davies Rene Segera, and Thomas Ochuku Mbuya  
> *University of Nairobi*

---

## 1. Repository Overview

The control architecture separates economic scheduling from fast current tracking and local safety interlocks across two layers:

1. **Upper Supervisory Layer (`300 s` interval, `6`-step / `30 min` horizon):**  
   Evaluates `17` candidate stack-current references using a frozen **$10\text{--}64\text{--}64\text{--}4$ hybrid neural-network surrogate** (predicting cell voltage, next stack temperature, next modelled dry-basis $\text{O}_2$ purity, and next stress index) coupled with algebraic Faraday gas flows, buffer balances, and balance-of-plant (BoP) power equations.
2. **Lower Embedded Layer (`1.0 s` virtual interval, `5`-step horizon):**  
   Runs natively on an **Arduino GIGA R1 WiFi (STM32H747XI Cortex-M7 @ 480 MHz)** to track the supervisory current reference subject to $\pm 50\text{ A/s}$ ramp limits, $5\text{ A}$ current granularity, command freshness checks, and local protective interlocks.

### Key Files

| File / Directory | Description |
| :--- | :--- |
| `m2_dashboard_3.py` | Main CLI runner for surrogate training, evaluation, 250-run SIL campaigns, PIL/HIL collection, auditing, and figure/table generation. |
| `M2_Arduino_GIGA_LowerMPC.ino` | Embedded Arduino GIGA R1 WiFi firmware implementing the lower-layer MPC, supervisory watchdog, USB-serial batch bridge, and Wi-Fi MQTT telemetry. |
| `m2_plant_fn.m` / `M2_PEM_Step.slx` | Six-state dynamic PEM electrolyzer plant function and programmatically generated fixed-step Simulink reference model. |
| `m2_data/m2_config.json` | Locked plant, electrochemical, thermal, gas-buffer, BoP, tariff, and controller parameter manifest. |
| `m2_data/surrogate_metrics.json` | Held-out 1-step ($7{,}500$ samples) and recursive multi-step rollout ($1, 6, 12, 24$ steps) accuracy metrics for the frozen $10\text{--}64\text{--}64\text{--}4$ MLP. |


---

## 2. Requirements & Setup

### Python Environment
Python `3.10+` is required with standard scientific and MQTT packages:
```bash
pip install numpy pandas scipy matplotlib torch paho-mqtt pyserial
```

### Hardware & MATLAB/Simulink (For PIL/HIL Runs Only)
* **Microcontroller:** Arduino GIGA R1 WiFi (`STM32H747XI`, compiled for the Cortex-M7 core via Arduino IDE `2.x` with `Arduino Mbed OS Giga Boards`).
* **Plant Bridge:** MATLAB/Simulink (`R2023b` or newer) connected to the Arduino GIGA R1 over USB Serial (`115200` baud).
* **Broker:** MQTT broker (configured in `m2_config.json`; laboratory prototype tested via Wi-Fi MQTT).

---

## 3. Reproducing the Results

### A. Self-Test & Configuration Audit
Verify environment integrity and configuration parameters:
```bash
python m2_dashboard_3.py self-test
```

### B. Train & Evaluate the $10\text{--}64\text{--}64\text{--}4$ Hybrid Neural Surrogate
Trains on $30{,}000$ grouped transitions ($12$ groups) and evaluates held-out 1-step and recursive multi-step accuracy on $7{,}500$ samples ($3$ held-out groups) plus step-size convergence ($300\text{ s}$ vs. $1\text{ s}$):
```bash
python m2_dashboard_3.py train
python m2_dashboard_3.py evaluate-surrogate
```

### C. Run the 250-Run Software-in-the-Loop (SIL) Campaign
Executes all **5 controllers** (`PID`, `SINGLE_MPC`, `BUFFER_RULE`, `FULL_MODEL_EMPC`, `PROPOSED`) across **5 scenarios** (`clear_sky`, `cloudy_rainy`, `oxygen_surge`, `communication_loss`, `sensor_fault_leak`) and **10 weekly repeats** ($24\text{ h}$ warm-up excluded + $7$ measured days per run):
```bash
python m2_dashboard_3.py sil --days 7 --repeats 10 --allow-expanded-campaign
python m2_dashboard_3.py paper --days 7 --repeats 10
```
Outputs are written to `m2_data/results/sil_summary.csv` and `m2_data/paper_outputs/`.

### D. Active Hydrogen-Offtake Sensitivity Check
Evaluates downstream hydrogen export ($\dot{m}_{\text{H}_2,\text{export,req}} = 1.5 \times 10^{-4}\text{ kg/s} = 12.96\text{ kg/day}$ at $800\text{ KES/kg}$, weight $0.5$):
```bash
python m2_dashboard_3.py sil --days 1 --repeats 1 --h2-export-demand-kg-s 0.00015 --h2-price-kes-kg 800
```

### E. Embedded Processor-in-the-Loop (PIL / Time-Compressed HIL) Runs
With `M2_Arduino_GIGA_LowerMPC.ino` flashed to the Arduino GIGA R1 WiFi and the MATLAB USB-serial bridge active:
```bash
# 7-day clear-sky batch-accelerated trajectory (604,800 simulated seconds)
python m2_dashboard_3.py collect --scenario clear_sky --duration-s 604800

# Planned hardware fault-injection verification runs (triggered at t = 900 s)
python m2_dashboard_3.py collect --scenario sensor_out_of_range --duration-s 1800 --event-start-s 900
python m2_dashboard_3.py collect --scenario purity_alarm --duration-s 1800 --event-start-s 900
python m2_dashboard_3.py collect --scenario sensor_fault_leak --duration-s 1800 --event-start-s 900
python m2_dashboard_3.py collect --scenario communication_loss --duration-s 2400 --event-start-s 900
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
