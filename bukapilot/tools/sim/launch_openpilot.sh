#!/bin/bash

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export FINGERPRINT="HONDA CIVIC 2016"

export BLOCK="camerad,loggerd"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# The simulator has no device keys, and bukapilot's register() otherwise fails and
# aborts manager. Mark the device as unregistered so registration is skipped offline.
python3 -c "from common.params import Params; p = Params(); p.get('DongleId') or p.put('DongleId', 'UnregisteredDevice')"

cd ../../selfdrive/manager && ./manager.py
