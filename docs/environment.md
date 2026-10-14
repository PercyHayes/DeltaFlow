# Environment

Use Linux x86-64 with a working GCC/G++ toolchain. RTL simulation does not require a GPU.

## Tested tools

| Tool | Tested version | Purpose |
| --- | --- | --- |
| Python | 3.13.14 | Compiler, fixtures and flow scripts |
| NumPy | 2.2.5 | Numerical expectations and data packing |
| GCC/G++ | 13.3 | Verilator and ordered FP32 reference |
| GNU Make | 4.3 | Simulator compilation |
| Verilator | 5.022 | RTL simulation with assertions and hierarchical builds |
| Java | 20.0.2 | Tested JVM; Chipyard upstream also documents its JVM requirements |
| sbt | 1.8.2 | Scala build |
| Scala | 2.13.16 | RTL sources |
| Chisel / compiler plugin | 6.7.0 | RTL elaboration |
| CIRCT firtool | 1.75.0 | FIRRTL lowering |
| Yosys with Slang plugin | 0.69+136 | SV synthesis |
| OpenSTA | 3.1.0 | Linked netlist timing |

Python 3.11 or later can install the pinned NumPy package; the software was tested with the version in the table. Do not substitute a different floating-point reference compiler flag such as `-ffast-math`.

## Python setup

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
source scripts/env.sh
```

`uv venv .venv --python 3.13` followed by `uv pip install -r requirements.txt` is an alternative. A conda environment with the same requirements also works. On first use in each process, the numerical reference compiles `examples/arithmetic_native.c` with GCC into a private temporary directory under `.local/reference/`. The loaded object remains valid after its temporary file is removed; independent fixture generators never overwrite a shared object.

## Tools for direct RTL simulation

Install Verilator 5.022, GCC/G++ and Make using your distribution or an isolated conda environment. The supplied complete `generated/DeltaFlowHarness.sv` permits simulation without installing sbt, Chipyard or EasyFloat.

```bash
verilator --version
g++ --version
make --version
```

`scripts/build_sim.sh` handles the conda Verilator executable/include layout inside `.local/verilator_root`. It does not replace the system installation. Use `BUILD_JOBS=32` and `SIM_THREADS=8` as starting values. Reducing build jobs can lower compilation memory. Simulator threads affect host execution time, not hardware cycles.

## Dependencies for Chisel and Chipyard

Obtain [chipyard-fsa](https://github.com/VCA-EPFL/chipyard-fsa), branch `msaga-main`, including its submodules. It supplies the [Chipyard](https://github.com/ucb-bar/chipyard) integration and [EasyFloat](https://github.com/VCA-EPFL/easyfloat), located at `generators/easyfloat/src/main/scala`; it is not copied into this release. The setup command follows FSA's published setup.

```bash
mkdir -p .deps
git clone --branch msaga-main https://github.com/VCA-EPFL/chipyard-fsa.git .deps/chipyard-fsa
cd .deps/chipyard-fsa
./build-setup.sh --skip-ctags --skip-firesim --skip-marshal
cd ../..
source scripts/env.sh
```

Set `CHIPYARD_DIR=/path/to/chipyard-fsa` before sourcing `scripts/env.sh` to use an existing installation. That script sources the dependency's `env.sh` when present. Set `DELTAFLOW_SKIP_CHIPYARD_ENV=1` if your tools are already configured. Existing JVM proxy options are preserved. In a restricted network, configure your Maven/Coursier mirror and proxy before running sbt. The JVM uses a short relative `.local/tmp` directory to avoid sbt 1.8.2's Unix socket path limit on long workspace paths; actual temporary files stay in the current build directory.

## Synthesis and STA tools

Download the Linux build of [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build/releases), which includes the Yosys Slang plugin. Extract it under `.deps/` and pass its Yosys executable using `--yosys`. Do not add this alternate Yosys to the global PATH. Confirm that it supports `read_slang` and `synth -hieropt`:

```bash
.deps/oss-cad-suite/bin/yosys -Q -p 'help read_slang; help synth'
```

For OpenSTA, use a compatible standalone installation or build the [official source](https://github.com/The-OpenROAD-Project/OpenSTA). The upstream build requires CMake, C++, Tcl, SWIG, zlib and Eigen. One local build arrangement is:

```bash
git clone https://github.com/The-OpenROAD-Project/OpenSTA.git .deps/OpenSTA
cmake -S .deps/OpenSTA -B .local/opensta-build -DCMAKE_BUILD_TYPE=Release
cmake --build .local/opensta-build -j 32
```

Pass the resulting `sta` binary explicitly with `--sta`. Follow the upstream README for the dependency packages on your OS. Full-system STA previously used about 185 GiB RAM; plan for at least 210–220 GiB available RAM and several GiB of disk space. A PE probe needs much less memory.
