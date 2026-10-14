# GPU profiles

These GDN chunk-forward traces were recorded with Nsight on an NVIDIA A100, using H = 32, D = 128 and chunk size 64. The filenames identify batch size B and sequence length T.

| File | Batch | Tokens |
| --- | ---: | ---: |
| gdn_b1_t2048.json | 1 | 2,048 |
| gdn_b64_t2048.json | 64 | 2,048 |
| gdn_b1_t8192.json | 1 | 8,192 |
| gdn_b32_t8192.json | 32 | 8,192 |

Select **Open trace file** in the [Perfetto web viewer](https://ui.perfetto.dev/) and choose one of these JSON files.
