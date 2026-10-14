# TPU profiles

These GDN chunk-forward traces use B = 1, H = 32, D = 128 and chunk size 64, with T = 256 or 2,048. The files store profiler events in compressed Chrome Trace JSON format.

From the project root, decompress them with:

```bash
mkdir -p .local/profiles
gzip -dc profiles/tpu/gdn_b1_t256.json.gz > .local/profiles/gdn_b1_t256.json
gzip -dc profiles/tpu/gdn_b1_t2048.json.gz > .local/profiles/gdn_b1_t2048.json
```

Select **Open trace file** in the [Perfetto web viewer](https://ui.perfetto.dev/) and choose a JSON file from `.local/profiles/`. The decompressed T256 file is approximately 428 MB.
