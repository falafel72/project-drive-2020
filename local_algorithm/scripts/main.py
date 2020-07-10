#! /usr/bin/env python
import control.decider_sim
import time

if __name__ == "__main__":
    time.sleep(1)
    # control.decider_sim.pid_handle()
    control.decider_sim.cost_handle(False, False, 10)
