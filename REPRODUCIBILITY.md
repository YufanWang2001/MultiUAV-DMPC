# Reproducibility Guide

This document explains how to reproduce the thesis PDF and the main simulation figures.

## 1. Build the Thesis (LaTeX)
- Install TeX Live 2024/2025 (or newer) with `latexmk`, `xelatex`, and `biber`.
- From the repository root:
```bash
latexmk -pdfxe -interaction=nonstopmode -shell-escape -file-line-error "main.tex"
biber "main" || true
latexmk -pdfxe -interaction=nonstopmode -shell-escape -file-line-error "main.tex"
```

## 2. Reproduce MATLAB Experiments
**Environment**
- MATLAB R2025a (recommended)
- CasADi (with IPOPT) added to MATLAB path

**Entry scripts**
- `Codes for Thesis/MultiUAV_ET_DMPC_CasADi_run.m`
- `Codes for Thesis/run_dmpc_simulation3_0.m`

**Typical usage**
```matlab
addpath(genpath(pwd));           % add repository to path
is_event_triggered = true;       % toggle ET-DMPC
Hp = 15; Hc = 5;                 % controller horizons
% run a scenario (edit in the script or function call)
% e.g., run_dmpc_simulation_sci('scenario','S1');
```

**Outputs**
- Figures saved under `Figures/` (or as configured)
- Raw data logs (if enabled by the scripts)

> If a solver error occurs (e.g., IPOPT time limit), reduce horizon (Hp), relax tolerances, or enable warm-start if supported by the script.

## 3. Data and Parameters
If external datasets are used, place them under `data/` and update paths in scripts accordingly. Document any custom parameters in the script header or here.

## 4. Determinism
- Some planners (e.g., RRT) are randomized. Set `rng(seed)` in MATLAB to ensure repeatability.
- Record the MATLAB and CasADi versions in the appendix if exact replication is required.
