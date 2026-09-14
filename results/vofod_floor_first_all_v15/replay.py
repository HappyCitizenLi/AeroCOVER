#!/usr/bin/env python3
"""Retained-result compatibility entry: verify/report only, never rerun."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'retained_experiments'))
from report import main
if '--help' in sys.argv:
    print('Checks retained results and rebuilds their report. --validate (or validate): checks only. No experiment is launched.')
else:
    main(group='V15', validate_only='--validate' in sys.argv or 'validate' in sys.argv)
