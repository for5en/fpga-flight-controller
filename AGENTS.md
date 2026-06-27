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

#### Preferred workflow: `fpga_sim` (the author's convention)
The author drives simulation with a shell helper called `fpga_sim`. Run it **from
inside the module's folder** (where `<name>.v` and `tb_<name>.v` live). It compiles
`tb_<name>.v` + `<name>.v` into the binary `test_<name>`, then runs `vvp`, which dumps
`test_<name>.vcd` (this matches the `$dumpfile("test_<name>.vcd")` calls in every testbench):

```bash
cd pid_controller
fpga_sim pid_controller        # compile + simulate
fpga_sim pid_controller -k     # also kill any running gtkwave first, then compile + simulate
fpga_sim pid_controller -g     # kill gtkwave, compile + simulate, then open gtkwave on the .vcd
```

The `-k` flag exists because closing gtkwave from the terminal with Ctrl+Z hangs it instead
of quitting; `-k` cleans up the stale windows. `-g` is `-k` plus auto-launching a fresh gtkwave.

`fpga_sim` is defined in the author's `~/.bashrc` on their own machine; on the Cursor Cloud
VM it has been recreated in the agent's `~/.bashrc` so the same command works there. The
manual fallback (no helper needed) is:
```bash
iverilog -o /tmp/sim/test_pid_controller pid_controller/tb_pid_controller.v pid_controller/pid_controller.v
cd /tmp/sim && vvp test_pid_controller
```

Note: running `fpga_sim` inside the repo folder regenerates the committed `test_<name>`
binary in place, so it may show up as modified in `git status`; that is expected.

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
