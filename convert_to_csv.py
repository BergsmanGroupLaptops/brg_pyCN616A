# -*- coding: utf-8 -*-
"""
Created on Mon Feb 16 14:35:43 2026

@author: bergsman_lab_admin
"""

import json
import csv
from pathlib import Path
from typing import Union, Dict, Any


def flatten_dict(d: Dict[str, Any], parent_key: str = "", sep: str = ".") -> Dict[str, Any]:
    """
    Recursively flatten a nested dictionary using dot notation.
    Lists are preserved as JSON strings.
    """
    items = {}
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k

        if isinstance(v, dict):
            items.update(flatten_dict(v, new_key, sep=sep))
        elif isinstance(v, list):
            items[new_key] = json.dumps(v)
        else:
            items[new_key] = v

    return items


def jsonl_to_csv(jsonl_path: Union[str, Path],
                 csv_path: Union[str, Path],
                 flatten: bool = True) -> None:
    """
    Convert a JSONL file to CSV.

    Parameters
    ----------
    jsonl_path : str or Path
        Path to input JSONL file.
    csv_path : str or Path
        Path to output CSV file.
    flatten : bool
        If True, nested dicts are flattened with dot notation.
    """

    jsonl_path = Path(jsonl_path)
    csv_path = Path(csv_path)

    rows = []
    fieldnames = set()

    # First pass: read + collect fields
    with jsonl_path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue

            obj = json.loads(line)

            if flatten:
                obj = flatten_dict(obj)

            rows.append(obj)
            fieldnames.update(obj.keys())

    fieldnames = sorted(fieldnames)

    # Write CSV
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in rows:
            writer.writerow(row)

    print(f"Converted {len(rows)} rows → {csv_path}")