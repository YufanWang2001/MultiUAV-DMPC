# Thesis: Repository

> Last updated: 2025-08-14

This repository contains the master's thesis (LaTeX source, if provided) and simulation code (MATLAB/CasADi). Start here to understand the structure and how to reproduce the experiments.

## Thesis Information
- **Title**: Cooperative Path Planning and Collision Avoidance for Multi-UAV Systems in Dynamic Environments
- **Author**: Yufan Wang
- **Program**: Communication Technology and System Design
- **Institution**: (Technical University of Denmark)
- **Supervisors**: Sarah Renée Ruepp; Radheshyam Singh

## Directory Structure (excerpt)
```
└── Codes for Thesis/
    ├── MultiUAV_ET_DMPC_CasADi_3_0.m
    ├── MultiUAV_ET_DMPC_CasADi_run.m
    ├── Quadrotor_NMPC_Integrated5_0.m
    ├── SingleUAV_Algorithms_Comparison4_0.m
    ├── optimized_drone_simulation (4.1).html
    ├── quadrotor_nmpc_demo (full).html
    └── run_dmpc_simulation3_0.m
```

> Note: Only a subset of files/folders is shown here.

## Quick Start (LaTeX Build)
Use **TeX Live 2024/2025** or newer. Recommended toolchain: `latexmk + xelatex + biber`.

### One-command build
```bash
latexmk -pdfxe -interaction=nonstopmode -shell-escape -file-line-error "(main file not detected)"
```
If you use `biblatex/biber`, after the first compile run:
```bash
biber "main"
latexmk -pdfxe -interaction=nonstopmode -shell-escape -file-line-error "(main file not detected)"
```

### Common dependencies
- `latexmk`, `xelatex`, `biber`/`bibtex`
- CJK fonts (e.g., `Fandol` or `Noto` families) if the thesis includes Chinese
- LaTeX packages such as `geometry`, `hyperref`, `graphicx`, `float`, `subcaption`, `amsmath`, `amssymb`, `booktabs`, `siunitx` (match your `.tex` usage)

## Reproducing the Simulations (MATLAB + CasADi)
> Suggested environment: **MATLAB R2025a**, with **CasADi** (IPOPT backend) on the MATLAB path.

**Entry scripts (start with these):**
- `Codes for Thesis/MultiUAV_ET_DMPC_CasADi_run.m`
- `Codes for Thesis/run_dmpc_simulation3_0.m`

**Typical workflow:**
1. In MATLAB, add the repository (including subfolders) to the path:
   ```matlab
   addpath(genpath(pwd));
   ```
2. Ensure CasADi is installed and configured (if used).
3. Run one of the entry scripts. To reproduce figures from the thesis, look for parameters like `scenario`, `is_event_triggered`, `Hp`, etc., inside the scripts.
4. After execution, check `/Figures` or the script's configured output folder for generated plots and data.

## References
Bibliography files:
- (no `.bib` files detected in this archive)

## How to Cite
Use the `CITATION.cff` included in this repository or cite as follows:
```bibtex
@thesis{wang2025thesis,
  author    = {Yufan Wang},
  title     = {Cooperative Path Planning and Collision Avoidance for Multi-UAV Systems in Dynamic Environments},
  school    = {Technical University of Denmark},
  year      = {2025},
}
```

## Licensing
- Thesis text: **CC BY-NC-ND 4.0** is recommended (see `LICENSE-THESIS.md`).  
  > If you prefer not to allow reuse, keep it as “All Rights Reserved.”
- Code: **MIT License** (see `LICENSE-CODE`).

## Contact
- **Author**: Yufan Wang  
- **Email**: (w.yufan@outlook.com)

---

### Notes
- If you plan to include LaTeX sources, add your main `.tex` file to the repository and update the build section accordingly. I can update this README once you provide the path to your main file and `.bib`.
