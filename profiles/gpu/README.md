# GPU trace exports

These retained GDN chunk-forward Nsight traces use an NVIDIA A100, H = 32, D = 128 and C = 64. Filename B and T identify batch and sequence length.

| File | Batch | Tokens |
| --- | ---: | ---: |
| gdn_b1_t2048.json | 1 | 2,048 |
| gdn_b64_t2048.json | 64 | 2,048 |
| gdn_b1_t8192.json | 1 | 8,192 |
| gdn_b32_t8192.json | 32 | 8,192 |

Open any JSON using **Open trace file** at [ui.perfetto.dev](https://ui.perfetto.dev/). These are viewable exports; CUDA and Nsight are not required to open them.
