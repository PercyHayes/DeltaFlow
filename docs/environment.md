# Environment

This project runs on Linux x86-64 with GCC/G++, Make and Verilator. RTL simulation runs on the CPU.

Recommended RAM is about **220 GiB** for the complete RTL generation, synthesis and full-system STA flow. For RTL simulation, plan for **32–64 GiB** when compiling the simulator and **8–16 GiB** when running the compiled examples. Set `BUILD_JOBS` to control parallel compilation and its memory use.

## Tool versions

The following versions were used to build and run DeltaFlow:

| Tool | Version | Purpose |
| --- | --- | --- |
| Python | 3.13.14 | Compiler, example data and flow scripts |
| NumPy | 2.2.5 | Numerical reference and tensor packing |
| GCC/G++ | 13.3 | Simulator and FP32 reference compilation |
| GNU Make | 4.3 | Simulator compilation |
| Verilator | 5.022 | RTL simulation |
| Java | 20.0.2 | Scala build runtime |
| sbt | 1.8.2 | Scala build |
| Scala | 2.13.16 | RTL sources |
| Chisel / compiler plugin | 6.7.0 | RTL generation |
| CIRCT firtool | 1.75.0 | FIRRTL to SystemVerilog |
| Yosys with Slang plugin | 0.69+136 | SystemVerilog synthesis |
| OpenSTA | 3.1.0 | Static timing analysis |

## Python setup

Use Python 3.11 or later for the NumPy version in `requirements.txt`:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
source scripts/env.sh
```

Alternatively, create the environment with `uv venv .venv --python 3.13` and install packages with `uv pip install -r requirements.txt`. A Conda environment can use the same requirements file.

The examples calculate expected results with `examples/arithmetic.py` and `examples/arithmetic_native.c`. On first use, Python compiles the C reference with GCC in `.local/reference/`. The compiler flags `-fno-fast-math` and `-ffp-contract=off` preserve the reference's floating-point operation order.

## RTL simulation tools

Install Verilator 5.022, GCC/G++ and Make using your Linux package manager or a Conda environment. The simulator builds directly from `generated/DeltaFlowHarness.sv`.

```bash
verilator --version
g++ --version
make --version
```

For a Conda Verilator installation, `scripts/build_sim.sh` prepares the executable and include files under `.local/verilator_root`. Set `BUILD_JOBS` to control parallel compilation and `SIM_THREADS` to control simulator threads. Start with `BUILD_JOBS=32` and `SIM_THREADS=8`, then adjust them to the CPU and memory available on your machine. Simulator threads change host runtime; hardware cycle counts come from the simulated circuit.

## Chisel and Chipyard

Follow [Chipyard's setup guide](https://chipyard.readthedocs.io/en/stable/Chipyard-Basics/Initial-Repo-Setup.html) to install Conda. The commands below install the Chipyard build environment used with the tool versions above. The setup script initializes its submodules and Scala dependencies:

```bash
mkdir -p .deps
git clone --branch msaga-main https://github.com/VCA-EPFL/chipyard-fsa.git .deps/chipyard-fsa
cd .deps/chipyard-fsa
./build-setup.sh --skip-ctags --skip-firesim --skip-marshal
cd ../..
source scripts/env.sh
```

`scripts/env.sh` sets `CHIPYARD_DIR` to `.deps/chipyard-fsa` and loads that checkout's `env.sh`. The Scala build reads its floating-point sources from `$CHIPYARD_DIR/generators/easyfloat/src/main/scala`. Project links and credits are in the README's [Acknowledgements](../README.md#acknowledgements).

To select an existing installation, set `CHIPYARD_DIR=/path/to/chipyard-fsa` before sourcing `scripts/env.sh`. If your shell already has the required tools configured, set `DELTAFLOW_SKIP_CHIPYARD_ENV=1`. Configure Maven/Coursier mirrors and JVM proxy options before running sbt when your network requires them. The environment script uses `.local/tmp` for JVM temporary files so sbt can run from long workspace paths.

## Synthesis and STA tools

Download a Linux [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build/releases) build with the Yosys Slang plugin and extract it under `.deps/`. Pass `.deps/oss-cad-suite/bin/yosys` to synthesis with `--yosys`, so the scripts use that installation. The synthesis flow uses `read_slang` and `synth -hieropt`; display their options with:

```bash
.deps/oss-cad-suite/bin/yosys -Q -p 'help read_slang; help synth'
```

Build [OpenSTA](https://github.com/The-OpenROAD-Project/OpenSTA) with CMake, a C++ compiler, Tcl, SWIG, zlib and Eigen. Follow its README for OS dependency packages, then run:

```bash
git clone https://github.com/The-OpenROAD-Project/OpenSTA.git .deps/OpenSTA
cmake -S .deps/OpenSTA -B .local/opensta-build -DCMAKE_BUILD_TYPE=Release
cmake --build .local/opensta-build -j 32
```

Pass `.local/opensta-build/sta` to timing analysis with `--sta`. The full-system STA run peaked at about 185 GiB RAM; allocate about 210–220 GiB and several GiB of disk space for the same design. Use the [PE example](synthesis.md#pe-synthesis-and-sta) to try the flow on a smaller module.
