#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb  7 15:28:30 2024

@author: xf17id1
"""

from mxtools.governor import _make_governors
print(f"Loading {__file__}")

gov = _make_governors("XF:17IDB-ES:AMX", name="gov")
gov_rbt = gov.gov.Robot
