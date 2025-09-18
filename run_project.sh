#!/bin/bash

# This script runs the entire project pipeline.

# Exit on error
set -e

# 1. Generate the flowchart
echo "Generating flowchart..."
python3 robotic_arm_simulation/generate_flowchart.py

# 2. Run the simulation to generate screenshots
echo "Running simulation and generating screenshots..."
# The simulation script creates the screenshots directory at the root
xvfb-run python3 robotic_arm_simulation/simulation.py

# 3. Generate the PDF report
echo "Generating PDF report..."
cd robotic_arm_simulation
pandoc report.md -o report.pdf
cd ..

echo "Project pipeline finished successfully."
echo "The final report is in robotic_arm_simulation/report.pdf"
echo "The screenshots are in the screenshots/ directory"
echo "The flowchart is in robotic_arm_simulation/flowchart.png"
