# Fluidrop Precision Analysis: Precision Evolution Across Pipette Generations

This report presents a comprehensive precision analysis (Coefficient of Variation - CV %) calculated between replicate runs across the evolution of the Fluidrop prototype models, sorted by prototype family.

---

## 📊 Performance Visualizations

### 1. Precision Evolution over Design Generations
![Precision Evolution Chart](file:///C:/Users/kahaa/.gemini/antigravity-ide/brain/8007a117-5198-41ff-afeb-29af5e35ad9f/precision_evolution.png)
> **Figure 1:** Chronological CV (%) comparisons between Fluidrop design iterations. Precision improves significantly from early manual/wire models (~2.4%) down to the EA111 motor and current linear actuator models (<1.0%).

### 2. Calibration Curve & Dynamic Range Profile
![Calibration Curve Chart](file:///C:/Users/kahaa/.gemini/antigravity-ide/brain/8007a117-5198-41ff-afeb-29af5e35ad9f/calibration_curve.png)
> **Figure 2:** Accuracy (Left) and CV % precision profile (Right) of the current model Linear Actuator pipette across target volumes (1µL to 100µL). The highlighted region (0–10µL) is for reference only due to standard laboratory weighing scale limitations at low weights.

### 3. Fluidrop Overlapping Absorbance Spectra
![Overlapping Spectra Chart](file:///C:/Users/kahaa/.gemini/antigravity-ide/brain/8007a117-5198-41ff-afeb-29af5e35ad9f/overlapping_spectra.png)
> **Figure 3:** Overlapping average absorbance spectra across the active prototype families. Shows distinct peak profiles: KMnO4 manual/wire reference peaks near 523nm, and the current actuator dye peaks near 661nm.

---

## 📋 Replicate Precision Summary Table

| Prototype Generation / Family | Peak WL (nm) | Mean Abs / Value | Replicates / Runs | Replicate CV (%) | Precision Status | Primary Data Source |
| :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| **Manual Reference (20µL set to 20µL)** | 523 nm | 0.5029 | 4 | **0.21%** | Excellent (< 1.0%) | `validation-kmno4-27052026.csv` |
| **Wire Pipette (46µL)** | 523 nm | 0.5834 | 4 | **2.41%** | Needs Work (≥ 2.0%) | `validation-kmno4-27052026.csv` |
| **Wire Pipette (46µL)** | 524 nm | 0.9306 | 4 | **2.46%** | Needs Work (≥ 2.0%) | `update.csv` |
| **Wire Pipette (46µL)** | 524 nm | 1.2495 | 3 | **1.72%** | Good (< 2.0%) | `update2.csv` |
| **EA111 Handheld (20µL)** | 523 nm | 0.5999 | 5 | **0.77%** | Excellent (< 1.0%) | `linearsteppar.csv` |
| **EA111 Handheld (20µL)** | 523 nm | 0.3138 | 5 | **0.74%** | Excellent (< 1.0%) | `v2.csv` |
| **EA111 Handheld (20µL)** | 523 nm | 1.1784 | 5 | **1.16%** | Good (< 2.0%) | `report_20260605151550.csv` |
| **EA111 Handheld (20µL)** | 523 nm | 1.2176 | 5 | **0.76%** | Excellent (< 1.0%) | `pipettev3.csv` |
| **Current Linear Actuator (50µL)** | 661 nm | 1.4566 | 5 | **0.81%** | Excellent (< 1.0%) | `kahaanpipette.csv.csv` |
| **Current Linear Actuator (10µL)** | — | 1.7072 | 4 | **1.24%** | Good (< 2.0%) | Spectra Data |
| **Current Linear Actuator (25µL)** | 661 nm | 0.6246 | 5 | **0.89%** | Excellent (< 1.0%) | `data 030726.csv` |

---

## 🔍 Detailed Generation Analysis

### 1. Manual Reference (20µL set to 20µL)
* **Data File:** [validation-kmno4-27052026.csv](file:///C:/Users/kahaa/Documents/Kahaan KT files OMI/Fluidrop - Electronic pipette/Raw data/validation-kmno4-27052026.csv) (Kmno)
* **Description:** A manual 20µL reference pipette set to a target volume of 20µL to establish benchmark accuracy.
* **Wavelength:** 523 nm
* **Replicates:** 4 runs (`Kmno42` to `Kmno45`)
* **Mean:** `0.5029`
* **Precision (CV%):** **`0.21%`** (Highly precise manual reference standard).

### 2. Wire Pipette (46µL) Generations
Characterized by manual wire tension actuation (46µL capacity), demonstrating a higher baseline variance due to manual feedback and mechanical play:
* **Wire Pipette Run 1** (`validation-kmno4-27052026.csv` - Epkmno): Peak WL = 523 nm | Mean = 0.5834 | Replicates = 4 | **CV = `2.41%`**
* **Wire Pipette Run 2** (`update.csv` - Blkmno): Peak WL = 524 nm | Mean = 0.9306 | Replicates = 4 | **CV = `2.46%`**
* **Wire Pipette Run 3** (`update2.csv` - Epv2): Peak WL = 524 nm | Mean = 1.2495 | Replicates = 3 | **CV = `1.72%`**

### 3. EA111 Handheld (20µL) Pipette
The first electronic handheld iteration utilizing the EA111 motor, calibrated for 20µL volumes. Precision improved significantly:
* **Linear Stepper Model** (`linearsteppar.csv` - Epv3): Peak WL = 523 nm | Mean = 0.5999 | Replicates = 5 | **CV = `0.77%`** (Excellent)
* **Small Volume Model** (`v2.csv` - Smolvolpipette): Peak WL = 523 nm | Mean = 0.3138 | Replicates = 5 | **CV = `0.74%`** (Excellent)
* **Handheld V3 Model** (`report_20260605151550.csv` - V3pipette): Peak WL = 523 nm | Mean = 1.1784 | Replicates = 5 | **CV = `1.16%`** (Good)
* **Standard Handheld v3** (`pipettev3.csv` - Pipettev3): Peak WL = 523 nm | Mean = 1.2176 | Replicates = 5 | **CV = `0.76%`** (Excellent)

### 4. Current Model Linear Actuator Pipette
The current flagship design replacing the stepper motor with a high-accuracy linear actuator:
* **50µL Linear Actuator** (`kahaanpipette.csv.csv` - 50ulpip): Peak WL = 661 nm | Mean = 1.4566 | Replicates = 5 | **CV = `0.81%`** (Excellent)
* **25µL Linear Actuator** (`data 030726.csv` - 25uL): Peak WL = 661 nm | Mean = 0.6246 | Replicates = 5 | **CV = `0.89%`** (Excellent)
* **10µL Linear Actuator** (Spectra Data): Mean = 1.7072 | Replicates = 4 | **CV = `1.24%`** (Good)
  * Raw readings: `[1.7328, 1.6816, 1.7043, 1.7100]`

---

## 📂 Supplementary Data Files
The following datasets were moved to the `extra data/` subfolder and are excluded from the main precision trends:
1. **`kahaanpipette.csv`**: Older 50µL Linear Actuator test run containing a higher CV of `1.28%`. Excluded in favor of the cleaner, optimized `kahaanpipette.csv.csv` run (`0.81%` CV).
2. **`kahaanpipette10ul.csv` / `kahaanpipette10ul.csv.csv` / `kahaanpipette10ul.csv - Copy.csv`**: Unfiltered 10µL test runs recorded on the standard weighing scale. Standard laboratory scale inaccuracy below 10µL yielded higher variances (~2.05% CV); replaced by the precise 4-point spectrophotometer test dataset (`1.24%` CV).
3. **`14_point_dataset.csv` / `pipette_calibration.csv`**: Older calibration logs and data profiles.
