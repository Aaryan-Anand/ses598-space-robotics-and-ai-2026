# SES 598 Final Project: Robust MEMS Preprocessing and Post-Flight Sensor Fusion

This project preserves the original proposal skeleton:

`intrinsic calibration -> Hampel-style robust frontend -> biquadratic low-pass filtering -> sensor fusion -> state estimation`

The repo is structured so the project can be presented even if some data files are private or too large for GitHub. Put flight logs in `data/raw/`, run the pipeline scripts, and generate figures for the report and presentation.

## Core claim

Low-cost MEMS flight data from high-dynamic rockets is corrupted by deterministic sensor errors, impulsive outliers, vibration, transonic pressure effects, and recovery deployment shocks. A state estimator should therefore be built as a full pipeline, not as a naked Kalman filter:

1. calibrate sensor axes and bias,
2. suppress non-Gaussian outliers,
3. shape high-frequency noise,
4. fuse gated measurements into an attitude/state estimate,
5. evaluate consistency and event behavior on real flight logs.

## Expected data files

Place exported CSVs here:

```text
data/raw/dustdevil_primary_ascent.csv
data/raw/carbon_copy_flight_data.csv
data/raw/carbon_copy_other_flight_data.csv
data/raw/cosmog_ascent_r2_flight_data.csv
```

The corresponding Google Drive source files are:

```text
DustDevil Primary Ascent Data Analysis.xlsx
Carbon Copy fight data?
Carbon Copy Other fight data
cosmog_ascent_r2_flight_data
```

## Fast run

```bash
cd final_project
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python scripts/00_inspect_columns.py --csv data/raw/dustdevil_primary_ascent.csv
python scripts/01_run_pipeline.py --csv data/raw/dustdevil_primary_ascent.csv --name dustdevil
python scripts/02_make_figures.py --run outputs/dustdevil_processed.csv
```

## Output artifacts

```text
outputs/*_processed.csv        processed pipeline outputs
figures/*.png                 report/presentation plots
report/main.tex               IEEE-style report draft
report/references.bib         BibTeX references
notes/literature_matrix.md    what each paper supports
```

## Presentation fallback demo

If live code is risky, show:

1. this README + repo structure,
2. `src/estimation/fusion.py`,
3. plots from `figures/`,
4. Blender gyro-integrated attitude video,
5. Cosmog panic deploy event as recovery logic case study.
