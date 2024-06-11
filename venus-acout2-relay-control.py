#!/usr/bin/env python

SCRIPT_VERSION = '0.1'

"""
Created by Richard Smith - 2023 - https://github.com/rsmithsa/venus-acout2-relay-control

State machine to control a Venus relay linked to AC-Out2 based on SOC, time of day & generation.
"""

import collections
import logging
import os
import signal
import sys
from typing import NamedTuple

from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

sys.path.insert(1, os.path.join(os.path.dirname(__file__), './ext/velib_python'))
from dbusmonitor import DbusMonitor

from statemachine import StateMachine, State
import statsd

logger = logging.getLogger("venus-acout2-relay-control")

def mean(data: collections.deque):
    n = len(data)
    if n == 0:
        return 0.0
    return sum(data) / n

class ACOut2OverrideSettings(NamedTuple):
    MaxDCPower: float
    MaxDCPowerSOC: float
    DCPowerPeriodCount: int
    MinSOC: float
    SOCHisteresis: float
    DCPowerHisteresis: float

class ACOut2OverrideParameters(NamedTuple):
    SOC: float
    DCPower: float
    ACSource: int

class ACOut2Override(StateMachine):

    def __init__(self, settings: ACOut2OverrideSettings, relayCallback = lambda closed: None):
        self._settings = settings
        self.relayCallback = relayCallback

        self._currentParameters = ACOut2OverrideParameters(0, 0, 1)
        self._dcPowerBuffer = collections.deque([-99999.9] * self._settings.DCPowerPeriodCount, maxlen=self._settings.DCPowerPeriodCount)

        # Ensure we are off to start
        self.toggle_relay(False)
        super().__init__()

    @property
    def currentParameters(self):
        return self._currentParameters

    # off - Relay open - default behaviour ACOut2 disconnected when grid is lost
    offState = State("off", initial = True)
    # on - Relay closed - ACOut2 powered when grid is lost
    onState = State("on")

    # transitions of the state
    switchOn = onState.from_(onState, offState, cond="ACOut_allowed")
    switchOff = offState.from_(offState, onState, cond="ACOut_notallowed")

    poll = switchOn | switchOff

    def on_poll(self, event_data):
        if event_data.source.id != event_data.target.id:
            self.toggle_relay(event_data.target.id == self.onState.id)

    def toggle_relay(self, closed: bool):
        if closed:
            logger.info("Toggle Venus relay to closed - %s - Average DC Power: %.2f", self._currentParameters, mean(self._dcPowerBuffer))
        else:
            logger.info("Toggle Venus relay to open - %s - Average DC Power: %.2f", self._currentParameters, mean(self._dcPowerBuffer))
        self.relayCallback(closed)

    def ACOut_allowed(self, event_data):
        if event_data.source.id == event_data.target.id:
            if (mean(self._dcPowerBuffer) > self._settings.MaxDCPower
                and self._currentParameters.SOC > self._settings.MaxDCPowerSOC):
                
                return True
            else:
                return self._currentParameters.SOC > self._settings.MinSOC
        else:
            # No AC-In, don't turn on if we weren't on already
            if self._currentParameters.ACSource == 240:
                return False
            
            # apply histeresis
            if (mean(self._dcPowerBuffer) > (self._settings.MaxDCPower + self._settings.DCPowerHisteresis)
                and self._currentParameters.SOC > (self._settings.MaxDCPowerSOC + self._settings.SOCHisteresis)):
                
                return True
            else:
                return self._currentParameters.SOC > (self._settings.MinSOC + self._settings.SOCHisteresis)
    
    def ACOut_notallowed(self, event_data):
        if event_data.source.id == event_data.target.id:
            # apply histeresis
            if (mean(self._dcPowerBuffer) <= (self._settings.MaxDCPower + self._settings.DCPowerHisteresis)
                or self._currentParameters.SOC <= (self._settings.MaxDCPowerSOC + self._settings.SOCHisteresis)):
                
                return True
            else:
                return self._currentParameters.SOC <= (self._settings.MinSOC + self._settings.SOCHisteresis)
        else:
            if (mean(self._dcPowerBuffer) <= self._settings.MaxDCPower
                or self._currentParameters.SOC <= self._settings.MaxDCPowerSOC):
                
                return True
            else:
                return self._currentParameters.SOC <= self._settings.MinSOC
    
    def update_parameters(self, parameters: ACOut2OverrideParameters):
        self._currentParameters = parameters
        self._dcPowerBuffer.append(self._currentParameters.DCPower)
        self.poll()


def dbus_value_change(override: ACOut2Override, dbusServiceName, dbusPath, options, changes, deviceInstance):
    if dbusPath == "/Dc/Battery/Soc":
        override.update_parameters(ACOut2OverrideParameters(changes["Value"], override.currentParameters.DCPower, override.currentParameters.ACSource))
    elif dbusPath == "/Dc/Battery/Power":
        override.update_parameters(ACOut2OverrideParameters(override.currentParameters.SOC, changes["Value"], override.currentParameters.ACSource))
    elif dbusPath == "/Ac/ActiveIn/Source":
        statsd.gauge("loadshedding", 1 if changes["Value"] == 240 else 0)
        override.update_parameters(ACOut2OverrideParameters(override.currentParameters.SOC, override.currentParameters.DCPower, changes["Value"]))

def set_venus_relay(monitor: DbusMonitor, closed: bool):
    statsd.gauge("relay_state", 1 if closed else 0)
    monitor.set_value("com.victronenergy.system", "/Relay/0/State", 1 if closed else 0)

def main():
    logging.basicConfig(level=logging.INFO)
    logger.info("venus-acout2-relay-control - Starting")

    statsd.init_statsd({"STATSD_HOST": "192.168.11.11", "STATSD_BUCKET_PREFIX": "venus-acout2-relay-control"})
    
    DBusGMainLoop(set_as_default=True)

    monitor = DbusMonitor({"com.victronenergy.system": { "/Relay/0/State": None, "/Dc/Battery/Soc": None, "/Dc/Battery/Power": None, "/Ac/ActiveIn/Source": None }})
    override = ACOut2Override(ACOut2OverrideSettings(-250, 50, 180, 60, 3, 1000), lambda closed: set_venus_relay(monitor, closed))

    monitor.valueChangedCallback = lambda dbusServiceName, dbusPath, options, changes, deviceInstance: dbus_value_change(override, dbusServiceName, dbusPath, options, changes, deviceInstance)
    initialSOC = monitor.get_value("com.victronenergy.system", "/Dc/Battery/Soc", 0.0)
    initialDCPower = monitor.get_value("com.victronenergy.system", "/Dc/Battery/Power", 0.0)
    initialACSource = monitor.get_value("com.victronenergy.system", "/Ac/ActiveIn/Source", 0)

    override.update_parameters(ACOut2OverrideParameters(initialSOC, initialDCPower, initialACSource))

    mainloop = GLib.MainLoop()
    
    def kill_handler(sig, frame):
        if sig == signal.SIGINT or sig == signal.SIGTERM:
            logger.info("venus-acout2-relay-control - Exiting")
            try:
                set_venus_relay(monitor, False)
            finally:
                mainloop.quit()

    signal.signal(signal.SIGINT, kill_handler)
    signal.signal(signal.SIGTERM, kill_handler)
    
    mainloop.run()

if __name__ == "__main__":
    main()
