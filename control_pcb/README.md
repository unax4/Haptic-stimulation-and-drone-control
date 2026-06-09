# Control PCB

This folder contains the standalone sketch used to validate the electro-tactile PCB independently of the drone controllers.

## Purpose

Use this sketch when the goal is to test the hardware itself rather than the flight stack:

- potentiometer control
- HV switch routing
- pulse generation
- direct selection of stimulation positions
- basic IMU/analog streaming used during bench testing

## Main file

- [control_pcb.ino](./control_pcb.ino)

## When to use it

Use `control_pcb.ino` before flight integration when you want to confirm:

- the PCB is powered correctly
- the selected route is really the active route
- pulses are being generated with the requested timing
- a stimulation problem is electrical rather than protocol-related
