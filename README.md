# AER1516 Assignment 2 — RRT and RRT* Path Planning with Dubins Paths

Motion planning assignment implementing **RRT** and **RRT\*** for a Dubins car model.

## Overview

This assignment implements two sampling-based motion planning algorithms for a vehicle constrained to Dubins-style paths (constant-speed, minimum turning radius):

- **RRT** — Rapidly-Exploring Random Trees
- **RRT\*** — Asymptotically optimal variant of RRT

## File Structure

```
aer1516_winter_2026_assignment_02_code/
├── dubins_path_planning.py   # Dubins path geometry library
├── dubins_path_problem.py    # Problem class (RRT_dubins_problem) with helper methods
├── rrt_planner.py            # RRT implementation (submitted)
├── rrt_star_planner.py       # RRT* implementation (submitted)
├── generate_comparison.py    # Q3: compares RRT vs RRT* performance, outputs comparison.pdf
├── validate_submission.py    # Pre-submission validator (PASS/FAIL checks)
└── handin/
    ├── rrt_planner.py        # Final submission copy
    └── rrt_star_planner.py   # Final submission copy
```

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install numpy matplotlib
```

## Usage

**Run the planner (visualise a single plan):**
```bash
python aer1516_winter_2026_assignment_02_code/dubins_path_problem.py
```

**Validate your implementation before submitting:**
```bash
python aer1516_winter_2026_assignment_02_code/validate_submission.py
```

**Generate the RRT vs RRT\* comparison report (Question 3):**
```bash
python aer1516_winter_2026_assignment_02_code/generate_comparison.py
# Outputs: comparison.pdf
```

## Algorithm Notes

| Property | RRT | RRT\* |
|---|---|---|
| Complete | Probabilistically | Probabilistically |
| Optimal | No | Asymptotically |
| Key extra step | — | Rewiring of nearby nodes |

Both planners use Dubins paths as the local steering function and must respect:
- Maximum iterations (`max_iter`)
- Map boundaries (`map_area`)
- Collision-free paths through the obstacle environment
