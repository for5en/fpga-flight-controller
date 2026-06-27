# AGENTS.md

## Cursor Cloud specific instructions

This repository is a collection of independent Verilog (HDL) IP cores for FPGAs.
There are no servers, databases, or web services. "Running the application" means
compiling a module with its testbench and running the simulation with Icarus Verilog.

### Toolchain
- Simulation toolchain: **Icarus Verilog** (`iverilog` compiler + `vvp` runtime). Installed via the update script (`apt-get install iverilog`).
- `gtkwave` is installed for viewing the `.vcd` waveforms that the testbenches emit, but it is optional (the testbenches print pass/fail via `$display`).
- The base VM has Icarus Verilog 12.0 (stable); the original author used 14.0. The testbenches compile and run cleanly on 12.0.

### Modules and how to simulate
Each module has a design file (`<name>.v`) and a testbench (`tb_<name>.v`):
- `pid_controller/` — fixed-point signed PID controller.
- `fpga-drivers/uart/` — UART TX/RX (self-checking; prints `OK!` per byte).
- `fpga-drivers/i2c_master/` — I²C master.
- `fpga-drivers/pwm_generator/` — servo/ESC-style PWM generator.

To compile and run a module (example: PID controller):
```bash
iverilog -o /tmp/sim/test_pid_controller pid_controller/tb_pid_controller.v pid_controller/pid_controller.v
cd /tmp/sim && vvp test_pid_controller   # prints results, writes test_pid_controller.vcd
```

### Non-obvious notes
- Compile build outputs to a scratch dir like `/tmp/sim` (or run `vvp` from there). The
  testbenches call `$dumpfile("<name>.vcd")` with a relative path, so the `.vcd` is written
  to the current working directory. `*.vcd` is git-ignored, but keeping build artifacts out
  of the repo tree avoids accidentally overwriting the committed `test_*` binaries.
- The committed `test_*` files are prebuilt `vvp` binaries with a hardcoded shebang pointing
  at the original author's home dir (`/home/for5en/oss-cad-suite/...`), which does not exist
  here. **Always recompile with `iverilog` rather than executing the committed `test_*` files
  directly.** They still run if invoked as `vvp test_<name>` (the shebang is ignored).
- There is no `Makefile` or package manifest; each module is compiled independently.
