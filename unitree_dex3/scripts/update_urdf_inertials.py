#!/usr/bin/env python3
"""
URDF Inertial Parameter Updater
=================================
Reads calibration_result.yaml and updates the URDF file with calibrated
mass and CoM values for the right arm links.

Usage:
  python3 scripts/update_urdf_inertials.py [--dry-run]

Input:  data/calibration_result.yaml
Output: Updated URDF (backup created as .urdf.bak)
"""

import os
import re
import sys
import shutil
import yaml
from pathlib import Path


def update_link_inertial(urdf_content, link_name, new_mass, new_com_xyz):
    """Update mass and origin xyz in the <inertial> block of a specific link."""

    # Find the link block
    # Pattern: <link name="link_name"> ... <inertial> ... </inertial> ... </link>
    link_pattern = re.compile(
        r'(<link\s+name="' + re.escape(link_name) + r'">\s*'
        r'<inertial>\s*)'
        r'(<origin\s+xyz="[^"]*")'
        r'([^<]*<mass\s+value=")[^"]*(")',
        re.DOTALL
    )

    def replacer(match):
        prefix = match.group(1)
        origin_tag = f'<origin xyz="{new_com_xyz[0]:.9g} {new_com_xyz[1]:.9g} {new_com_xyz[2]:.9g}"'
        between = match.group(3)
        mass_suffix = match.group(4)
        return f'{prefix}{origin_tag}{between}{new_mass:.9g}{mass_suffix}'

    new_content, count = link_pattern.subn(replacer, urdf_content)
    return new_content, count


def main():
    dry_run = "--dry-run" in sys.argv

    script_dir = Path(__file__).parent
    project_dir = script_dir.parent
    data_dir = project_dir / "data"
    urdf_path = project_dir / "src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf"
    result_path = data_dir / "calibration_result.yaml"

    if not result_path.exists():
        print(f"ERROR: {result_path} not found. Run gravity_calibration_solve.py first.")
        sys.exit(1)

    # Load calibration results
    with open(result_path, "r") as f:
        results = yaml.safe_load(f)

    cal = results["calibration_result"]
    links = cal["links"]

    print("=" * 60)
    print("  URDF Inertial Parameter Updater")
    print("=" * 60)
    print(f"\n  Calibration improvement: {cal['improvement_pct']:.1f}%")
    print(f"  RMSE: {cal['rmse_before']:.4f} -> {cal['rmse_after']:.4f} Nm")

    if dry_run:
        print("\n  [DRY RUN] No files will be modified.\n")

    # Read URDF
    with open(urdf_path, "r") as f:
        urdf_content = f.read()

    # Update each link
    print("\n  Updating links:")
    total_updates = 0
    for link_name, params in links.items():
        new_mass = params["mass"]
        new_com = params["com_xyz"]
        print(f"    {link_name}: mass={new_mass:.4f}, CoM={new_com}")

        urdf_content, count = update_link_inertial(urdf_content, link_name, new_mass, new_com)
        if count == 0:
            print(f"      WARNING: link not found or pattern mismatch!")
        else:
            total_updates += count

    print(f"\n  Total links updated: {total_updates}")

    if dry_run:
        print("\n  [DRY RUN] Would write to:")
        print(f"    {urdf_path}")
        print("\n  Run without --dry-run to apply changes.")
    else:
        # Create backup
        backup_path = str(urdf_path) + ".bak"
        if not os.path.exists(backup_path):
            shutil.copy2(urdf_path, backup_path)
            print(f"\n  Backup created: {backup_path}")
        else:
            print(f"\n  Backup already exists: {backup_path}")

        # Write updated URDF
        with open(urdf_path, "w") as f:
            f.write(urdf_content)
        print(f"  Updated URDF written: {urdf_path}")

        print(f"\n{'=' * 60}")
        print(f"  Done! Rebuild the package to use calibrated parameters:")
        print(f"  colcon build --packages-select unitree_g1_dex3_stack")
        print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
