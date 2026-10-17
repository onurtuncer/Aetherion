#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
"""Load and step the ConsumerAtmosphere FMU built against an installed SDK.

usage: check_fmu.py <ConsumerAtmosphere.fmu>     (needs fmpy)

Fails unless the FMU loads under FMPy, steps, and returns the US1976 density
at 3000 m (0.909253 kg/m^3, the value tests/InstallConsumer/main.cpp checks).
"""
import shutil
import sys
import tempfile

from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave


def main(path):
    md = read_model_description(path)
    vr = {v.name: v.valueReference for v in md.modelVariables}
    unzipdir = tempfile.mkdtemp(prefix="consumer_fmu_")
    try:
        extract(path, unzipdir)
        fmu = FMU2Slave(guid=md.guid, unzipDirectory=unzipdir,
                        modelIdentifier=md.coSimulation.modelIdentifier, instanceName="consumer")
        fmu.instantiate()
        fmu.setupExperiment(startTime=0.0)
        fmu.enterInitializationMode()
        fmu.exitInitializationMode()
        fmu.setReal([vr["in.alt_m"]], [3000.0])
        fmu.doStep(0.0, 0.1)
        rho = fmu.getReal([vr["out.rho_kg_m3"]])[0]
        fmu.terminate()
        fmu.freeInstance()
    finally:
        shutil.rmtree(unzipdir, ignore_errors=True)
    print(f"{md.modelName} ({md.coSimulation.modelIdentifier}): rho(3000 m) = {rho:.6f} kg/m^3")
    ok = abs(rho - 0.909253) < 1e-5
    print("installed-SDK FMU: OK" if ok else "installed-SDK FMU: WRONG VALUE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
