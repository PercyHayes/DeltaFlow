# TPU trace exports

These retained TPU GDN chunk-forward profiles use B = 1, H = 32, D = 128, C = 64, with T = 256 or 2,048. They contain the original profiler events and are distributed as compressed Chrome Trace JSON.

```bash
mkdir -p .local/profiles
gzip -dc profiles/tpu/gdn_b1_t256.json.gz > .local/profiles/gdn_b1_t256.json
gzip -dc profiles/tpu/gdn_b1_t2048.json.gz > .local/profiles/gdn_b1_t2048.json
```

From the release root, run those commands and open the resulting local JSON using **Open trace file** at [ui.perfetto.dev](https://ui.perfetto.dev/). The T256 uncompressed trace is approximately 428 MB; allow sufficient browser memory. A TPU or installed TPU profiler is not required to view the exports.
