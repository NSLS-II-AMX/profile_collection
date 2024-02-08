#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  6 10:34:00 2024

@author: rschaffer
"""

import time as ttime

from ophyd import Component as Cpt
from ophyd import Device, EpicsSignal, EpicsSignalNoValidation, EpicsSignalRO
from ophyd.status import SubscriptionStatus

import logging
logger = logging.getLogger(__name__)
print(f"Loading {__file__}")


class RobotTaskSignal(EpicsSignal):
    def _set_and_wait(self, value, timeout=100, **kwargs):
        start_value = int(self.get()[0])
        expiration_time = ttime.time() + timeout if timeout is not None else None
        self.put(value, timeout=timeout)
        while True:
            current_value = int(self.get()[0])
            if current_value == start_value + 1:
                break
            if current_value != start_value:
                raise RuntimeError(
                    f"SW:startRobotTask indexed by {current_value - start_value} on task {value} (should be 1).")
            if expiration_time is not None and ttime.time() > expiration_time:
                raise TimeoutError(
                    "Attempted to set %r to value %r and timed "
                    "out after %r seconds."
                    % (self, value, timeout, current_value)
                )
            ttime.sleep(0.05)


class RobotVariableSignal(EpicsSignal):
    def set(self, value, timeout=100, **kwargs):
        self.put(value, timeout=timeout)


robot_tasks = {
    'turnOnAutoFill': ["TurnOnAutoFill"],
    'turnOffAutoFill': ["TurnOffAutoFill"],
    'dewarHeaterOn':  ["DewarHeaterOn"],
    'dewarHeaterOff': ["DewarHeaterOff"],
    'enableTScreen':  ["EnableTScreen"],
    'openPortLid':    ["OpenPortLid"],
    'openLoadLid':    ["OpenLoadLid"],
    'openParkLid':    ["OpenParkLid"],
    'closePorts':     ["ClosePorts"],
    'park':           ['Initialize', 'Home', 'Park'],
    'goHome':         ["Home"],
    'initialize':     ["Initialize"],
    'finish':         ['UnlatchRobGov', 'Home', 'Finish'],
    'warmupGripper':  ['Initialize', 'UnlatchRobGov', 'Home', 'Finish'],
    'cooldownGripper': ['Initialize', 'CoolDown'],
    'openGripper':    ['OpenGripper'],
    'closeGripper':   ['CloseGripper'],
    'testRobot':      ['Initialize', 'TestRobot']
}


command_list = ["TurnOnAutoFill", "TurnOffAutoFill", "DewarHeaterOn",
                "DewarHeaterOff", "EnableTScreen", "OpenPortLid",
                "OpenLoadLid", "OpenParkLid", "ClosePorts", "Home",
                "Initialize", "Park", "UnlatchRobGov", "Finish",
                "CoolDown", "OpenGripper", "CloseGripper", "TestRobot",
                "Recover", "TraceSample", "LatchRobGov", "CoolDown",
                "Load", "Mount", "WarmUp", "Park", "Unmount", "Unload",
                "LoadSpecial", "MountSpecial", "UnmountSpecial",
                "UnloadSpecial"]


class RobotError(RuntimeError):
    def __init__(self, message=None):
        if not message:
            self.message = "Error encountered running robot, exiting."
        else:
            self.message = message
        super().__init__(message)


class Robot(Device):
    task = Cpt(RobotTaskSignal, "startRobotTask")
    abort = Cpt(EpicsSignal, "abort")
    restart = Cpt(EpicsSignal, "restart")
    robot_pause = Cpt(EpicsSignal, "pauseRobot")
    robot_resume = Cpt(EpicsSignal, "resumeRobot")
    start_test = Cpt(EpicsSignal, "startTestTask")

    set_variable = Cpt(RobotVariableSignal, "setRobotVariable")
    get_variable = Cpt(RobotVariableSignal, "getRobotVariable")
    speed = Cpt(EpicsSignal, "RobotSpeed")
    gui_lockout = Cpt(EpicsSignal, "LocalGuiLockOut")

    status = Cpt(EpicsSignalRO, "Status")
    robot_state = Cpt(EpicsSignalRO, "RobotState")
    state = Cpt(EpicsSignalRO, "State")
    info = Cpt(EpicsSignalRO, "getTaskInfo")
    running = Cpt(EpicsSignalRO, "isTaskRunning")
    distance = Cpt(EpicsSignalRO, "getDistance")
    mounted = Cpt(EpicsSignalRO, "isSampleMounted")
    check_result = Cpt(EpicsSignalRO, "checkTaskResult")
    task_exception = Cpt(EpicsSignalRO, "LastTaskException")
    task_output = Cpt(EpicsSignalRO, "LastTaskOutput")
    task_info = Cpt(EpicsSignalRO, "LastTaskInfo")
    cartesian_positions = Cpt(EpicsSignalRO, "CartesianPositions")
    joint_positions = Cpt(EpicsSignalRO, "JointPositions")
    version = Cpt(EpicsSignalRO, "Version")
    uptime = Cpt(EpicsSignalRO, "Uptime")
    alarm_list = Cpt(EpicsSignalRO, "AlarmList")

    def set_nsample(self, value, value_max=48):
        if value < 1 or value > value_max:
            raise ValueError(
                f"Sample number {value} is out of range (1-{value_max}).")
        self.set_variable.set(['nSample', value, 100000])
        self.get_variable.set('nDummy')
        # sleep(0.1)
        self.get_variable.set('nSample')
        # sleep(0.1)
        # print(self.get_variable.get())
        if int(float(self.get_variable.get())) != value:
            raise RobotError(f"Failed to set robot variable nSample")

    def run_command(self, command, timeout=-1):

        def check_done(*, old_value, value, **kwargs):
            not_done = ["Running", "Moving", "Busy", "Initialize"]
            return(old_value in not_done and value not in not_done)
        status = SubscriptionStatus(self.state, check_done)
        self.task.set([command, timeout])
        self.running.get()
        logger.info(f"Running Task {command.upper()}.")
        return status

    def get_return(self):
        smpStat = []
        # print(self.task_info.get())
        stat, exception = self.task_info.get()[4:6]
        if stat.startswith("ABORT"):
            stat, exception = stat.split(": ")
        if stat.startswith("Done") and stat.find("/") >= 0:
            stat, tmp = stat.split(" ")
            smpStat = [s == "True" for s in tmp.split("/")]
        if stat not in ("Done", "ABORT") and exception not in ("", "null"):
            stat = "ABORT"
        return stat, smpStat, exception

    def run_and_wait(self, command, timeout=-1):
        old_info = self.task_info.get()[3]
        status = self.run_command(command, timeout=timeout)
        current_time = ttime.time()
        while True:
            current_info = self.task_info.get()[3]
            if current_info == old_info:
                ttime.sleep(0.1)
            elif current_info == 'null':
                ttime.sleep(0.1)
            else:
                break
            if (ttime.time() - current_time) > timeout/1000:
                raise TimeoutError(
                    f"Task {command} failed to execute within {timeout/1000} seconds.")
        status.wait(timeout/1000)  # timeout in seconds, ms above
        logger.info(
            f"Task {command.upper()} is running = {self.running.get()}")
        return self.get_return()

    def run_task(self, task, timeout=-1):
        if task not in robot_tasks:
            raise KeyError(f"Task {task} is not a recognized task.")
        for cmd in robot_tasks[task]:
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)
            if (tskStat.lower() != "done"):
                raise RobotError(cmd+" "+tskStat+" with exception "+exception)

    def reset_heartbeat(self):
        self.set_variable.set(["nDummy", 882])

    def reboot_EMBL(self):
        self.abort.set("__EMPTY__")
        time.sleep(2)
        self.restart.set("__EMPTY__")

    def recover(self, timeout=-1):
        cmdList = ['Recover', 'TraceSample']
        for cmd in cmdList:
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)
            if (tskStat.lower() != "done"):
                raise RobotError(cmd+" "+tskStat+" with exception "+exception)
        return sampleStat

    def initialize(self, timeout=-1):
        self.run_task("initialize", timeout=timeout)

    def open_gripper(self, timeout=-1):
        self.run_task("openGripper", timeout=timeout)

    def close_gripper(self, timeout=-1):
        self.run_task("closeGripper", timeout=timeout)

    def test_robot(self, timeout=-1):
        self.run_task("testRobot", timeout=timeout)

    def openPort(self, nPort=1, timeout=-1):
        if nPort == 1:
            self.run_task("openPortLid", timeout=timeout)
        elif nPort == 2:
            self.run_task("openLoadLid", timeout=timeout)
        else:
            self.run_task("openParkLid", timeout=timeout)

    def pre_mount(self, nSample, timeout=-1):
        cmdList = ['Initialize', 'LatchRobGov', 'TraceSample', 'CoolDown']

        for cmd in cmdList:
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)
            if (tskStat.lower() != "done"):
                raise RobotError(cmd+" "+tskStat+" with exception "+exception)

            if len(sampleStat) == 3:
                [bMounted, bLoaded, bTilted] = sampleStat

            if cmd == "TraceSample":
                if bMounted or bLoaded:
                    raise RobotError(
                        "Aborting mount: Found pin on gonio or in gripper")

    def mount(self, nSample=0, warmup=False, timeout=-1):
        cmdList = ['Load',
                   'Mount',
                   'WarmUp',
                   'Park',
                   'UnlatchRobGov']

        self.set_nsample(nSample)

        for cmd in cmdList:
            if cmd == 'WarmUp' and warmup == False:
                continue
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)

            if (tskStat.lower() != "done"):
                # solution for SE timeout during Mount scenario
                if cmd == "Mount" and exception.find("SE") >= 0:
                    self.run_and_wait("Home", timeout=timeout)
                    tskStat2, sampleStat, exception2 = self.run_and_wait(
                        "Unload", timeout=timeout)
                    if (tskStat2.lower() == "done"):
                        [bMounted, bLoaded, bTilted] = sampleStat
                        if bLoaded:
                            self.run_and_wait("TraceSample", timeout=timeout)
                            self.run_and_wait("ClosePorts", timeout=timeout)
                            self.run_and_wait("UnlatchRobGov", timeout=timeout)
                            exception2 = "Failed to save the sample"
                        else:
                            self.run_and_wait("Home", timeout=timeout)
                            self.run_and_wait("Park", timeout=timeout)
                            self.run_and_wait("UnlatchRobGov", timeout=timeout)
                            exception2 = "Sample is saved back to the dewar"
                    exception = exception + "/" + exception2
                raise RobotError(
                    cmd+" "+tskStat+" with exception "+exception+";")

            if len(sampleStat) == 3:
                [bMounted, bLoaded, bTilted] = sampleStat

            if cmd == "Load":
                if not bLoaded:
                    logger.error("Failed to load sample, parking robot.")
                    self.run_and_wait("Home", timeout=timeout)
                    self.run_and_wait("Park", timeout=timeout)
                    raise RobotError("Failed to load sample, aborting.")
                elif bLoaded and bTilted:
                    logger.error(
                        "Pin is tilted after loading. Attempting to unload the pin.")
                    self.run_and_wait("Home", timeout=timeout)
                    tskStat, sampleStat, exception = self.run_and_wait(
                        "Unload", timeout=timeout)
                    if (tskStat.lower() != "done"):
                        raise RobotError("Failed to unload the pin.")
                    [bMounted, bLoaded, bTilted] = sampleStat

                    if bLoaded:
                        self.run_and_wait("TraceSample", timeout=timeout)
                        self.run_and_wait("ClosePorts", timeout=timeout)
                    else:
                        self.run_and_wait("Home", timeout=timeout)
                        self.run_and_wait("Park", timeout=timeout)
                    raise RobotError(
                        "Mount aborted due to tilted sample during load.")

            if cmd == "Mount":
                if not bMounted and bLoaded:
                    logger.error("Mount failed. Returning puck to the dewar.")
                    tskStat, sampleStat, exception = self.run_and_wait(
                        "Unload", timeout=timeout)
                    if tskStat.lower() != "done":
                        raise RobotError("Failed to unload sample.")
                    [bMounted, bLoaded, bTilted] = sampleStat

                    if bLoaded:
                        self.run_and_wait("TraceSample", timeout=timeout)
                        self.run_and_wait("ClosePorts", timeout=timeout)
                        raise RobotError(
                            "Mount Failed: Failed to unload back to dewar @"+exception)
                    else:
                        self.run_task("finish", timeout=timeout)
                        raise RobotError(
                            "Mount failed. Sample returned to dewar.")

                elif not bMounted and not bLoaded:
                    self.run_and_wait("TraceSample", timeout=timeout)
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Pin lost during mount transaction.")
                elif bMounted and bLoaded:
                    self.run_and_wait("TraceSample", timeout=timeout)
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Found pin on both Gonio and gripper")

    def pre_unmount(self, init=True, cooldown=True, timeout=-1):
        cmdList = ['Initialize', 'CoolDown']

        for cmd in cmdList:
            if cmd == "Initialize" and init == False:
                continue
            if cmd == "CoolDown" and cooldown == False:
                continue
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)

            if (tskStat.lower() != "done"):
                raise RobotError(cmd+" "+tskStat+" with exception "+exception)

    def unmount(self, nSample=0, timeout=-1):
        cmdList = ['Unmount', 'Unload']
        self.set_nsample(nSample)

        for cmd in cmdList:
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)

            if (tskStat.lower() != "done"):
                raise RobotError(cmd+" "+tskStat+" with exception "+exception)

            if len(sampleStat) == 3:
                [bMounted, bLoaded, bTilted] = sampleStat

            if cmd == "Unmount":
                if bMounted and not bLoaded:
                    logger.warn(
                        "Unmount failed. Attempting warmup and unmount again.")
                    unmountCmd = ['Home', 'WarmUp', 'CoolDown', 'Unmount']
                    for subCmd in unmountCmd:
                        tskStat, sampleStat, exception = self.run_and_wait(
                            subCmd, timeout=timeout)
                        if tskStat.lower() != "done":
                            raise RobotError(
                                "Failed to unmount again @"+exception)

                    [bMounted, bLoaded, bTilted] = sampleStat

                if bMounted and bLoaded:
                    # For current solution, just ignore the smart Magnet
                    logger.warn(
                        "Found pin on Gonio and gripper. Attempting unload.")
                elif not bMounted and not bLoaded:
                    self.run_and_wait("TraceSample", timeout=timeout)
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Pin lost during unmount transaction")
                elif bMounted and not bLoaded:
                    self.run_and_wait("TraceSample", timeout=timeout)
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Failed to unmount second time. Sticky pin on Gonio")
                else:
                    if bTilted:
                        logger.warn(
                            "Pin is tilted after unmounting. Attempting unload anyway")

            if cmd == "Unload":
                if bLoaded and not bTilted:
                    logger.warn("Unload failed. Attempting unload again.")
                    tskStat, sampleStat, exception = self.run_and_wait(
                        "Unload", timeout=timeout)
                    if tskStat.lower() != "done":
                        raise RobotError("Unload failure")

                    [bMounted, bLoaded, bTilted] = sampleStat

                if bLoaded:
                    self.run_and_wait("TraceSample", timeout=timeout)
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Failed unloading. Sticky or tilted pin.")

    def mount_special(self, nSample=0, timeout=-1):
        cmdList = ['Initialize', 'TraceSample',
                   'LoadSpecial', 'MountSpecial', 'UnlatchRobGov']
        self.set_nsample(nSample, value_max=16)

        for cmd in cmdList:
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)
            if (tskStat.lower() != "done"):
                raise RobotError(cmd+" "+tskStat+" with exception "+exception)

            if len(sampleStat) == 3:
                [bMounted, bLoaded, bTilted] = sampleStat

            if cmd == "TraceSample":
                if bMounted or bLoaded:
                    raise RobotError(
                        "Mount abort. Found pin on gonio or in gripper")

            if cmd == "LoadSpecial":
                if not bLoaded:
                    logger.error("Load Alignment Pin failed.")
                    raise RobotError("Load Alignment Pin Failed. Abort!")
                elif bLoaded and bTilted:
                    logger.warn(
                        "Alignment Pin is tilted. Attempting to unload the pin")
                    tskStat, sampleStat, exception = self.run_and_wait(
                        "UnloadSpecial", timeout=timeout)
                    raise RobotError(
                        "Alignment Pin tilted after loading. Unloadin the pin.")

            if cmd == "MountSpecial":
                if not bMounted and bLoaded:
                    logger.warn(
                        "Mount failed. Unloading Alignment Pin back to the Puck")
                    tskStat, sampleStat, exception = self.run_and_wait(
                        "UnloadSpecial", timeout=timeout)
                    raise RobotError(
                        "Fatal: Mount Alignment Pin Failed. Attempting to unload it.")
                elif not bMounted and not bLoaded:
                    raise RobotError(
                        "Fatal: Alignment Pin lost during mount transaction")
                elif bMounted and bLoaded:
                    raise RobotError("Fatal: Found A pin on Gonio and gripper")

    def unmount_special(self, nSample=0, timeout=-1):
        cmdList = ['Initialize', 'UnmountSpecial',
                   'UnloadSpecial', 'UnlatchRobgov', 'Home']
        self.set_nsample(nSample, value_max=16)

        for cmd in cmdList:
            tskStat, sampleStat, exception = self.run_and_wait(
                cmd, timeout=timeout)

            if (tskStat.lower() != "done"):
                raise RobotError(cmd+" "+tskStat+" with exception "+exception)

            if len(sampleStat) == 3:
                [bMounted, bLoaded, bTilted] = sampleStat

            if cmd == "UnmountSpecial":
                if bMounted and bLoaded:
                    # For current solution, just ignore the smart Magnet
                    logger.warn(
                        "Found pin on Gonio and gripper. Ignore smartMagnet Signal")
                elif not bMounted and not bLoaded:
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Alignment Pin lost during unmount transaction")
                elif bMounted and not bLoaded:
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Failed to unmount. Sticky pin on Gonio")
                else:
                    logger.info("Normal unmount procedure")

            if cmd == "UnloadSpecial":
                if bLoaded:
                    self.run_and_wait("ClosePorts", timeout=timeout)
                    raise RobotError(
                        "Fatal: Failed to unload Alignment Pin. Pin stuck in Gripper")


robrob = Robot('XF:17IDB-ES:AMX{EMBL}:', name='robrob')
