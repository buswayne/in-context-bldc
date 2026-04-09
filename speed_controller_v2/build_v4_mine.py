"""
build_v3.py
-----------
Single-script pipeline:

    .pt checkpoint
        ↓  export weights
    model.bin  (kept as artefact)
        ↓  generate C (v3 SAXPY)
    gpt_model_v3.c  (kept as artefact)
        ↓  compile
    gpt_model_v3.dll / .so
        ↓  validate vs PyTorch + benchmark

Usage:
    python build_v3.py --checkpoint checkpoint.pt
    python build_v3.py --checkpoint checkpoint.pt --out_dir deploy/
    python build_v3.py --checkpoint checkpoint.pt --runs 2000 --warmup 500
    python build_v3.py --checkpoint checkpoint.pt --no_native   # cross-compile safe
    python build_v3.py --checkpoint checkpoint.pt --skip_export --skip_generate
"""

import argparse
import ctypes
import importlib.util
import os
import platform
import struct
import subprocess
import sys
import time

import numpy as np
import torch

# ══════════════════════════════════════════════════════════════════════════════
# STAGE 1 — export weights
# ══════════════════════════════════════════════════════════════════════════════

MAGIC   = 0x47505430
VERSION = 1


def _write_tensor(f, tensor):
    arr = tensor.detach().float().cpu().contiguous().numpy()
    f.write(arr.tobytes())


def export_weights(model, out_path):
    cfg      = model.config
    has_bias = int(cfg.bias)

    with open(out_path, "wb") as f:
        f.write(struct.pack("II",      MAGIC, VERSION))
        f.write(struct.pack("iiiiiii", cfg.block_size, cfg.n_layer, cfg.n_head,
                                       cfg.n_embd, cfg.n_u, cfg.n_y, has_bias))

        _write_tensor(f, model.transformer.wte.weight)
        if model.transformer.wte.bias is not None:
            _write_tensor(f, model.transformer.wte.bias)
        _write_tensor(f, model.transformer.wpe.weight)

        for block in model.transformer.h:
            _write_tensor(f, block.ln_1.weight)
            if has_bias and block.ln_1.bias is not None:
                _write_tensor(f, block.ln_1.bias)
            _write_tensor(f, block.attn.c_attn.weight)
            if has_bias: _write_tensor(f, block.attn.c_attn.bias)
            _write_tensor(f, block.attn.c_proj.weight)
            if has_bias: _write_tensor(f, block.attn.c_proj.bias)
            _write_tensor(f, block.ln_2.weight)
            if has_bias and block.ln_2.bias is not None:
                _write_tensor(f, block.ln_2.bias)
            _write_tensor(f, block.mlp.c_fc.weight)
            if has_bias: _write_tensor(f, block.mlp.c_fc.bias)
            _write_tensor(f, block.mlp.c_proj.weight)
            if has_bias: _write_tensor(f, block.mlp.c_proj.bias)

        _write_tensor(f, model.transformer.ln_f.weight)
        if has_bias and model.transformer.ln_f.bias is not None:
            _write_tensor(f, model.transformer.ln_f.bias)
        _write_tensor(f, model.lm_head.weight)
        _write_tensor(f, model.lm_head.bias)

    print(f"  Weights exported → {out_path}")


# ══════════════════════════════════════════════════════════════════════════════
# STAGE 2 — load model.bin
# ══════════════════════════════════════════════════════════════════════════════

def load_model_bin(path):
    with open(path, "rb") as f:
        magic, version = struct.unpack("II", f.read(8))
        assert magic == MAGIC,   f"Bad magic: 0x{magic:X}"
        assert version == VERSION, f"Bad version: {version}"

        block_size, n_layer, n_head, n_embd, n_u, n_y, has_bias = \
            struct.unpack("iiiiiii", f.read(28))

        cfg = dict(block_size=block_size, n_layer=n_layer, n_head=n_head,
                   n_embd=n_embd, n_u=n_u, n_y=n_y, has_bias=bool(has_bias))

        def rd(n):
            return np.frombuffer(f.read(n * 4), dtype=np.float32).copy()

        B  = cfg["has_bias"]
        C  = n_embd;  C3 = 3*C;  C4 = 4*C
        BS = block_size;  L = n_layer

        w = {}
        w["wte_w"] = rd(C * n_u).reshape(C, n_u)
        w["wte_b"] = rd(C)
        w["wpe_w"] = rd(BS * C).reshape(BS, C)

        for l in range(L):
            w[f"ln1_w_{l}"]    = rd(C)
            w[f"ln1_b_{l}"]    = rd(C) if B else None
            w[f"c_attn_w_{l}"] = rd(C3 * C).reshape(C3, C)
            w[f"c_attn_b_{l}"] = rd(C3) if B else None
            w[f"c_proj_w_{l}"] = rd(C * C).reshape(C, C)
            w[f"c_proj_b_{l}"] = rd(C) if B else None
            w[f"ln2_w_{l}"]    = rd(C)
            w[f"ln2_b_{l}"]    = rd(C) if B else None
            w[f"fc_w_{l}"]     = rd(C4 * C).reshape(C4, C)
            w[f"fc_b_{l}"]     = rd(C4) if B else None
            w[f"proj_w_{l}"]   = rd(C * C4).reshape(C, C4)
            w[f"proj_b_{l}"]   = rd(C) if B else None

        w["ln_f_w"]    = rd(C)
        w["ln_f_b"]    = rd(C) if B else None
        w["lm_head_w"] = rd(n_y * C).reshape(n_y, C)
        w["lm_head_b"] = rd(n_y)

    return cfg, w


# ══════════════════════════════════════════════════════════════════════════════
# STAGE 3 — generate C (v3 SAXPY)
# ══════════════════════════════════════════════════════════════════════════════

NL = chr(92) + "n"   # C newline escape inside string literals


def fmt_float(v):
    s = f"{v:.8g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def c_array(name, arr):
    flat  = arr.flatten(order="C")
    n     = len(flat)
    lines = [f"static const float {name}[{n}] = {{"]
    for i in range(0, n, 8):
        chunk = flat[i:i+8]
        vals  = ", ".join(fmt_float(v) for v in chunk)
        comma = "" if (i + 8) >= n else ","
        lines.append(f"    {vals}{comma}")
    lines.append("};")
    return lines


def c_array_transposed(name, arr):
    return c_array(name, arr.T)


def c_array_or_comment(name, arr):
    return c_array(name, arr) if arr is not None else [f"/* {name}: no bias */"]


def c_array_transposed_or_comment(name, arr):
    return c_array_transposed(name, arr) if arr is not None else [f"/* {name}: no bias */"]


def generate_c(cfg, w, T, out_path):
    L  = cfg["n_layer"];  H = cfg["n_head"];  C = cfg["n_embd"]
    nu = cfg["n_u"];      ny = cfg["n_y"];    B = cfg["has_bias"]
    HS = C // H
    C3 = 3 * C;  C4 = 4 * C

    out = []
    a = out.append
    def aa(lines): out.extend(lines); a("")

    a(f"/*")
    a(f" * AUTO-GENERATED by build_v3.py — do not edit by hand.")
    a(f" * Model: L={L}  H={H}  C={C}  T_MAX={T}  N_U={nu}  N_Y={ny}  bias={int(B)}")
    a(f" *")
    a(f" * Key optimization: SAXPY-form matrix multiply with pre-transposed weights.")
    a(f" * Weight matrices stored as W_T[in_dim][out_dim] instead of W[out_dim][in_dim].")
    a(f" * Inner loop: dst[i] += W_T[j*out_dim+i] * src[j]  ->  pure SAXPY,")
    a(f" * which gcc maps to full-width AVX2 instructions automatically.")
    a(f" *")
    a(f" * API:  void gpt_forward(const float input[T_MAX*N_U], float output[T_MAX*N_Y]);")
    a(f" * Compile (host):  gcc -O3 -march=native -DSTANDALONE -o gpt_model_v3 {out_path} -lm")
    a(f" * Compile (SR6):   arm-none-eabi-gcc -O3 -mcpu=cortex-r52 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DSTANDALONE -o gpt_model_v3.elf {out_path} -lm")
    a(f" */")
    a("")
    a(f"#include \"GPT_model.h\"")
    a("#include <math.h>")
    a("#include <string.h>")
    a("")
    a("#ifndef T_MAX")
    a(f"#define T_MAX       {T}")
    a("#endif")
    a(f"#define N_LAYER     {L}")
    a(f"#define N_HEAD      {H}")
    a(f"#define N_EMBD      {C}")
    a("#ifndef N_U")
    a(f"#define N_U         {nu}")
    a("#endif")
    a("#ifndef N_Y")
    a(f"#define N_Y         {ny}")
    a("#endif")
    a(f"#define N_HEAD_SIZE {HS}")
    a(f"#define N_EMBD3     {C3}")
    a(f"#define N_EMBD4     {C4}")
    a("")

    # causal mask
    mask = [1 if tk <= tq else 0 for tq in range(T) for tk in range(T)]
    a(f"/* causal mask: CAUSAL_MASK[tq*T_MAX + tk] = 1 if tk<=tq, else 0 */")
    a(f"static const unsigned char CAUSAL_MASK[{T*T}] = {{")
    for i in range(0, len(mask), 16):
        chunk = mask[i:i+16]
        comma = "" if (i + 16) >= len(mask) else ","
        a("    " + ", ".join(str(v) for v in chunk) + comma)
    a("};")
    a("")

    a("/*")
    a(" * Weight matrices for linear layers are stored TRANSPOSED:")
    a(" *   Original:   W    [out_dim][in_dim]")
    a(" *   Stored as:  W_T  [in_dim][out_dim]")
    a(" * This lets the SAXPY inner loop read W_T sequentially.")
    a(" * Non-linear weights (layernorm scale/bias) are NOT transposed.")
    a(" */")
    a("")

    # weights
    aa(c_array_transposed(        "WTE_W_T",    w["wte_w"]))
    aa(c_array(                   "WTE_B",      w["wte_b"]))
    aa(c_array(                   "WPE_W",      w["wpe_w"]))

    for l in range(L):
        aa(c_array(                    f"LN1_W_{l}",      w[f"ln1_w_{l}"]))
        aa(c_array_or_comment(         f"LN1_B_{l}",      w[f"ln1_b_{l}"]))
        aa(c_array_transposed(         f"C_ATTN_W_T_{l}", w[f"c_attn_w_{l}"]))
        aa(c_array_transposed_or_comment(f"C_ATTN_B_{l}", w[f"c_attn_b_{l}"]))
        aa(c_array_transposed(         f"C_PROJ_W_T_{l}", w[f"c_proj_w_{l}"]))
        aa(c_array_transposed_or_comment(f"C_PROJ_B_{l}", w[f"c_proj_b_{l}"]))
        aa(c_array(                    f"LN2_W_{l}",      w[f"ln2_w_{l}"]))
        aa(c_array_or_comment(         f"LN2_B_{l}",      w[f"ln2_b_{l}"]))
        aa(c_array_transposed(         f"FC_W_T_{l}",     w[f"fc_w_{l}"]))
        aa(c_array_transposed_or_comment(f"FC_B_{l}",     w[f"fc_b_{l}"]))
        aa(c_array_transposed(         f"PROJ_W_T_{l}",   w[f"proj_w_{l}"]))
        aa(c_array_transposed_or_comment(f"PROJ_B_{l}",   w[f"proj_b_{l}"]))

    aa(c_array(                   "LN_F_W",      w["ln_f_w"]))
    aa(c_array_or_comment(        "LN_F_B",      w["ln_f_b"]))
    aa(c_array_transposed(        "LM_HEAD_W_T", w["lm_head_w"]))
    aa(c_array(                   "LM_HEAD_B",   w["lm_head_b"]))

    # compiler guard
    a("#if !defined(__GNUC__) && !defined(__clang__)")
    a("#  define __attribute__(x)")
    a("#endif")
    a("")

    # linear_saxpy
    a("/*")
    a(" * linear_saxpy: W_T is [in_dim][out_dim] (pre-transposed).")
    a(" * dst[t*out_dim + i] += src[t*in_dim + j] * W_T[j*out_dim + i]")
    a(" * Inner loop is a pure SAXPY — both dst and W_T row are contiguous.")
    a(" */")
    a("static void linear_saxpy(")
    a("    float * __restrict__ dst,")
    a("    const float * __restrict__ src,")
    a("    const float * __restrict__ W_T,")
    a("    const float * __restrict__ b,")
    a("    int out_dim, int in_dim)")
    a("{")
    a("    for (int t = 0; t < T_MAX; t++) {")
    a("        float       * __restrict__ d = dst + t * out_dim;")
    a("        const float * __restrict__ s = src + t * in_dim;")
    a("        if (b) { for (int i = 0; i < out_dim; i++) d[i] = b[i]; }")
    a("        else   { for (int i = 0; i < out_dim; i++) d[i] = 0.0f; }")
    a("        for (int j = 0; j < in_dim; j++) {")
    a("            const float * __restrict__ wrow = W_T + j * out_dim;")
    a("            float sv = s[j];")
    a("            for (int i = 0; i < out_dim; i++) d[i] += sv * wrow[i];")
    a("        }")
    a("    }")
    a("}")
    a("")

    # layernorm
    a("static void layernorm(")
    a("    float * __restrict__ dst,")
    a("    const float * __restrict__ src,")
    a("    const float * __restrict__ ww,")
    a("    const float * __restrict__ b,")
    a("    int C)")
    a("{")
    a("    const float eps = 1e-5f;")
    a("    for (int t = 0; t < T_MAX; t++) {")
    a("        const float * __restrict__ x = src + t * C;")
    a("        float       * __restrict__ d = dst + t * C;")
    a("        float mean = 0.0f;")
    a("        for (int i = 0; i < C; i++) mean += x[i];")
    a("        mean /= (float)C;")
    a("        float var = 0.0f;")
    a("        for (int i = 0; i < C; i++) { float v = x[i]-mean; var += v*v; }")
    a("        float inv = 1.0f / sqrtf(var / (float)C + eps);")
    a("        for (int i = 0; i < C; i++)")
    a("            d[i] = ww[i] * (x[i] - mean) * inv + (b ? b[i] : 0.0f);")
    a("    }")
    a("}")
    a("")
    a("static inline float gelu(float x) {")
    a("    return 0.5f * x * (1.0f + erff(x * 0.7071067811865476f));")
    a("}")
    a("")

    # gpt_forward
    scale_str = fmt_float(float(1.0 / HS**0.5))
    a("#if defined(__GNUC__) || defined(__clang__)")
    a("__attribute__((optimize(\"O3\")))")
    a("#endif")
    a("void gpt_forward(const float input[T_MAX * N_U], float output[T_MAX * N_Y])")
    a("{")
    a("    static float x     [T_MAX * N_EMBD];")
    a("    static float normed[T_MAX * N_EMBD];")
    a("    static float tmp   [T_MAX * N_EMBD];")
    a("    static float qkv   [T_MAX * N_EMBD3];")
    a("    static float att   [N_HEAD * T_MAX * T_MAX];")
    a("    static float vacc  [T_MAX * N_EMBD];")
    a("    static float mlph  [T_MAX * N_EMBD4];")
    a("")
    a("    /* embeddings: wte (SAXPY) + wpe (add) */")
    a("    linear_saxpy(x, input, WTE_W_T, WTE_B, N_EMBD, N_U);")
    a("    for (int t = 0; t < T_MAX; t++) {")
    a("        const float * __restrict__ pe = WPE_W + t * N_EMBD;")
    a("        float       * __restrict__ xt = x     + t * N_EMBD;")
    a("        for (int i = 0; i < N_EMBD; i++) xt[i] += pe[i];")
    a("    }")
    a("")

    for l in range(L):
        ln1b = f"LN1_B_{l}"     if B else "NULL"
        ln2b = f"LN2_B_{l}"     if B else "NULL"
        cab  = f"C_ATTN_B_{l}"  if B else "NULL"
        cpb  = f"C_PROJ_B_{l}"  if B else "NULL"
        fcb  = f"FC_B_{l}"      if B else "NULL"
        prb  = f"PROJ_B_{l}"    if B else "NULL"

        a(f"    /* ── block {l} ── */")
        a(f"    layernorm(normed, x, LN1_W_{l}, {ln1b}, N_EMBD);")
        a(f"    linear_saxpy(qkv, normed, C_ATTN_W_T_{l}, {cab}, N_EMBD3, N_EMBD);")
        a("")
        a("    /* scaled dot-product attention */")
        a("    for (int h = 0; h < N_HEAD; h++) {")
        a("        for (int tq = 0; tq < T_MAX; tq++) {")
        a("            float * __restrict__ row = att + (h * T_MAX + tq) * T_MAX;")
        a("            const float * __restrict__ q = qkv + tq*N_EMBD3 + h*N_HEAD_SIZE;")
        a("            for (int tk = 0; tk < T_MAX; tk++) {")
        a("                if (!CAUSAL_MASK[tq * T_MAX + tk]) {")
        a("                    row[tk] = -1e30f; continue;")
        a("                }")
        a("                const float * __restrict__ k = qkv + tk*N_EMBD3 + N_EMBD + h*N_HEAD_SIZE;")
        a("                float dot = 0.0f;")
        a("                for (int d = 0; d < N_HEAD_SIZE; d++) dot += q[d] * k[d];")
        a(f"                row[tk] = dot * {scale_str};")
        a("            }")
        a("        }")
        a("    }")
        a("")
        a("    /* softmax */")
        a("    for (int r = 0; r < N_HEAD * T_MAX; r++) {")
        a("        float * __restrict__ row = att + r * T_MAX;")
        a("        float mx = row[0];")
        a("        for (int i = 1; i < T_MAX; i++) if (row[i] > mx) mx = row[i];")
        a("        float s = 0.0f;")
        a("        for (int i = 0; i < T_MAX; i++) { row[i] = expf(row[i]-mx); s += row[i]; }")
        a("        float inv_s = 1.0f / s;")
        a("        for (int i = 0; i < T_MAX; i++) row[i] *= inv_s;")
        a("    }")
        a("")
        a("    /* weighted value accumulation */")
        a("    memset(vacc, 0, sizeof(float) * T_MAX * N_EMBD);")
        a("    for (int h = 0; h < N_HEAD; h++) {")
        a("        for (int tq = 0; tq < T_MAX; tq++) {")
        a("            float * __restrict__ dst = vacc + tq*N_EMBD + h*N_HEAD_SIZE;")
        a("            const float * __restrict__ row = att + (h*T_MAX + tq)*T_MAX;")
        a("            for (int tk = 0; tk < T_MAX; tk++) {")
        a("                const float * __restrict__ v = qkv + tk*N_EMBD3 + 2*N_EMBD + h*N_HEAD_SIZE;")
        a("                float wt = row[tk];")
        a("                for (int d = 0; d < N_HEAD_SIZE; d++) dst[d] += wt * v[d];")
        a("            }")
        a("        }")
        a("    }")
        a("")
        a(f"    linear_saxpy(tmp, vacc, C_PROJ_W_T_{l}, {cpb}, N_EMBD, N_EMBD);")
        a("    for (int i = 0; i < T_MAX * N_EMBD; i++) x[i] += tmp[i];")
        a("")
        a(f"    /* MLP */")
        a(f"    layernorm(normed, x, LN2_W_{l}, {ln2b}, N_EMBD);")
        a(f"    linear_saxpy(mlph, normed, FC_W_T_{l}, {fcb}, N_EMBD4, N_EMBD);")
        a("    for (int i = 0; i < T_MAX * N_EMBD4; i++) mlph[i] = gelu(mlph[i]);")
        a(f"    linear_saxpy(tmp, mlph, PROJ_W_T_{l}, {prb}, N_EMBD, N_EMBD4);")
        a("    for (int i = 0; i < T_MAX * N_EMBD; i++) x[i] += tmp[i];")
        a("")

    ln_fb = "LN_F_B" if B else "NULL"
    a(f"    layernorm(normed, x, LN_F_W, {ln_fb}, N_EMBD);")
    a("    linear_saxpy(output, normed, LM_HEAD_W_T, LM_HEAD_B, N_Y, N_EMBD);")
    a("}")
    a("")

    # standalone main (for MCU / direct executable use)
    a("#ifdef STANDALONE")
    a("#include <stdio.h>")
    a("#include <stdlib.h>")
    a("")
    a("#if defined(_WIN32)")
    a("#  include <windows.h>")
    a("static double now_sec(void) {")
    a("    LARGE_INTEGER t, f;")
    a("    QueryPerformanceCounter(&t);")
    a("    QueryPerformanceFrequency(&f);")
    a("    return (double)t.QuadPart / (double)f.QuadPart;")
    a("}")
    a("#else")
    a("#  include <time.h>")
    a("static double now_sec(void) {")
    a("    struct timespec ts;")
    a("    clock_gettime(CLOCK_MONOTONIC, &ts);")
    a("    return ts.tv_sec + ts.tv_nsec * 1e-9;")
    a("}")
    a("#endif")
    a("")
    a("int main(int argc, char *argv[])")
    a("{")
    a("    static float input [T_MAX * N_U];")
    a("    static float output[T_MAX * N_Y];")
    a("    if (argc == 3) {")
    a('        FILE *fin = fopen(argv[1], "rb");')
    a("        if (!fin) { perror(argv[1]); return 1; }")
    a("        if (fread(input, sizeof(float), T_MAX*N_U, fin) != (size_t)(T_MAX*N_U)) {")
    a(f'            fprintf(stderr, "Input file too short{NL}"); fclose(fin); return 1;')
    a("        }")
    a("        fclose(fin);")
    a("        gpt_forward(input, output);")
    a('        FILE *fout = fopen(argv[2], "wb");')
    a("        if (!fout) { perror(argv[2]); return 1; }")
    a("        fwrite(output, sizeof(float), T_MAX*N_Y, fout);")
    a("        fclose(fout);")
    a(f'        printf("done T=%d N_U=%d N_Y=%d{NL}", T_MAX, N_U, N_Y);')
    a("    } else {")
    a("        for (int i = 0; i < T_MAX*N_U; i++) input[i] = (float)i * 0.01f;")
    a("        int warmup = 50, runs = 500;")
    a("        for (int i = 0; i < warmup; i++) gpt_forward(input, output);")
    a("        double t0 = now_sec();")
    a("        for (int i = 0; i < runs; i++) gpt_forward(input, output);")
    a("        double elapsed = now_sec() - t0;")
    a(f'        printf("T_MAX=%d  L=%d  H=%d  C=%d{NL}", T_MAX, N_LAYER, N_HEAD, N_EMBD);')
    a(f'        printf("mean_ms=%.6f{NL}",   elapsed/runs*1e3);')
    a(f'        printf("fwd_per_s=%.2f{NL}", runs/elapsed);')
    a("    }")
    a("    return 0;")
    a("}")
    a("#endif /* STANDALONE */")
    a("")

    src = "\n".join(out)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(src)

    n_w = sum(v.size for v in w.values() if v is not None)
    print(f"  C code generated  → {out_path}")
    print(f"  {n_w:,} weight values  ({n_w*4/1024:.1f} KB floats)  "
          f"{len(src.encode())/1024:.0f} KB source")


# ══════════════════════════════════════════════════════════════════════════════
# STAGE 4 — compile shared library
# ══════════════════════════════════════════════════════════════════════════════

def compile_lib(c_path, lib_path, native=True):
    march = ["-march=native"] if native else []
    cmd   = ["gcc", "-O3"] + march + ["-shared", "-fPIC", "-o", lib_path, c_path, "-lm"]
    print(f"  Compiling: {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("  Compilation failed:\n", r.stderr)
        sys.exit(1)
    print(f"  Library compiled  → {lib_path}")


def load_lib(lib_path):
    lib = ctypes.CDLL(lib_path)
    lib.gpt_forward.restype  = None
    lib.gpt_forward.argtypes = [ctypes.POINTER(ctypes.c_float),
                                 ctypes.POINTER(ctypes.c_float)]
    return lib


def unload_lib(lib):
    if platform.system() == "Windows":
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(lib._handle))


def to_ptr(arr):
    assert arr.dtype == np.float32 and arr.flags["C_CONTIGUOUS"]
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


# ══════════════════════════════════════════════════════════════════════════════
# STAGE 5 — validate + benchmark
# ══════════════════════════════════════════════════════════════════════════════

def load_pytorch_model(checkpoint, model_py):
    spec = importlib.util.spec_from_file_location("model_module", model_py)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ckpt       = torch.load(checkpoint, map_location="cpu", weights_only=False)
    model_args = ckpt.get("model_args", ckpt.get("config"))
    state_dict = ckpt.get("model", ckpt)
    config     = mod.GPTConfig(**model_args) if isinstance(model_args, dict) else model_args
    model      = mod.GPT(config)
    model.load_state_dict(state_dict)
    model.eval()
    return model


def pytorch_forward(model, x_np):
    with torch.no_grad():
        x_t = torch.tensor(x_np, dtype=torch.float32).unsqueeze(0)
        return model(x_t).squeeze(0).numpy()


def validate(lib, x_np, y_pt, T, n_u, n_y, tol):
    x_in = np.ascontiguousarray(x_np.flatten(), dtype=np.float32)
    y_c  = np.zeros(T * n_y, dtype=np.float32)
    lib.gpt_forward(to_ptr(x_in), to_ptr(y_c))
    y_c  = y_c.reshape(T, n_y)

    err   = np.abs(y_pt - y_c)
    mx    = err.max()
    worst = np.unravel_index(err.argmax(), err.shape)
    ok    = mx <= tol
    print(f"  Max error : {mx:.4e}  "
          f"(t={worst[0]}  PT={y_pt[worst]:.8f}  C={y_c[worst]:.8f})")
    print(f"  {'✅  PASS' if ok else '❌  FAIL'}  (tol={tol:.0e})")
    return ok


def benchmark_c(lib, x_np, T, n_y, runs, warmup):
    x_in = np.ascontiguousarray(x_np.flatten(), dtype=np.float32)
    y_c  = np.zeros(T * n_y, dtype=np.float32)
    xp, yp = to_ptr(x_in), to_ptr(y_c)
    for _ in range(warmup):
        lib.gpt_forward(xp, yp)
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        lib.gpt_forward(xp, yp)
        times.append(time.perf_counter() - t0)
    return np.array(times)


def benchmark_pytorch(model, x_np, runs, warmup):
    x_t = torch.tensor(x_np, dtype=torch.float32).unsqueeze(0)
    with torch.no_grad():
        for _ in range(warmup):
            model(x_t)
        times = []
        for _ in range(runs):
            t0 = time.perf_counter()
            model(x_t)
            times.append(time.perf_counter() - t0)
    return np.array(times)


def print_timing(label, times, T):
    print(f"  ── {label}")
    print(f"     Mean   {times.mean()*1e3:8.4f} ms  |  "
          f"Median {np.median(times)*1e3:8.4f} ms  |  "
          f"Min {times.min()*1e3:8.4f} ms  |  "
          f"p95 {np.percentile(times,95)*1e3:8.4f} ms  |  "
          f"{1/times.mean():6.0f} fwd/s")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="Export → generate C (v3) → compile → validate → benchmark")
    ap.add_argument("--checkpoint",    required=True,
                    help="PyTorch .pt checkpoint")
    ap.add_argument("--model_py",      default="transformer_zerostep.py",
                    help="Model definition file (default: transformer_zerostep.py)")
    ap.add_argument("--out_dir",       default=".",
                    help="Output directory for all generated files (default: .)")
    ap.add_argument("--skip_export",   action="store_true",
                    help="Skip weight export (reuse existing model.bin)")
    ap.add_argument("--skip_generate", action="store_true",
                    help="Skip C generation (reuse existing gpt_model_v3.c)")
    ap.add_argument("--no_native",     action="store_true",
                    help="Disable -march=native")
    ap.add_argument("--out-name",      default="gpt_model_v3")
    ap.add_argument("--runs",          type=int,   default=1000)
    ap.add_argument("--warmup",        type=int,   default=100)
    ap.add_argument("--seed",          type=int,   default=42)
    ap.add_argument("--tol",           type=float, default=1e-4)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    bin_path = os.path.join(args.out_dir, args.out_name + ".bin")
    c_path   = os.path.join(args.out_dir, args.out_name + ".c")
    ext      = ".dll" if platform.system() == "Windows" else ".so"
    lib_path = os.path.join(args.out_dir, args.out_name + ext)

    W = 62
    sep = "─" * W

    # ── stage 1: load pytorch model ───────────────────────────────────────────
    print(f"\n{sep}")
    print(f"  Loading PyTorch checkpoint: {args.checkpoint}")
    print(sep)
    model = load_pytorch_model(args.checkpoint, args.model_py)
    cfg   = model.config
    print(f"  L={cfg.n_layer}  H={cfg.n_head}  C={cfg.n_embd}  "
          f"T={cfg.block_size}  N_U={cfg.n_u}  N_Y={cfg.n_y}  bias={cfg.bias}")

    # ── stage 2: export weights ───────────────────────────────────────────────
    print(f"\n{sep}")
    print(f"  Stage 1 — Export weights")
    print(sep)
    if not args.skip_export:
        export_weights(model, bin_path)
    else:
        print(f"  Skipped — using existing {bin_path}")

    # ── stage 3: load bin + generate C ───────────────────────────────────────
    print(f"\n{sep}")
    print(f"  Stage 2 — Generate C (v3 SAXPY)")
    print(sep)
    cfg_bin, w = load_model_bin(bin_path)
    T, nu, ny  = cfg_bin["block_size"], cfg_bin["n_u"], cfg_bin["n_y"]
    if not args.skip_generate:
        generate_c(cfg_bin, w, T, c_path)
    else:
        print(f"  Skipped — using existing {c_path}")

    # ── stage 4: compile ──────────────────────────────────────────────────────
    print(f"\n{sep}")
    print(f"  Stage 3 — Compile shared library")
    print(sep)
    compile_lib(c_path, lib_path, native=not args.no_native)

    lib = None
    try:
        lib = load_lib(lib_path)

        # ── random input ──────────────────────────────────────────────────────
        rng  = np.random.default_rng(args.seed)
        x_np = rng.standard_normal((T, nu)).astype(np.float32)

        # ── stage 5a: C benchmark FIRST (cache clean, before PyTorch loads) ──
        print(f"\n{sep}")
        print(f"  Stage 4 — Benchmark  ({args.warmup} warmup + {args.runs} runs,  T={T})")
        print(sep)
        t_c = benchmark_c(lib, x_np, T, ny, args.runs, args.warmup)
        print_timing("v3 C  (ctypes, direct call)", t_c, T)

        # ── stage 5b: pytorch ground truth + validate ─────────────────────────
        print(f"\n{sep}")
        print(f"  Stage 5 — Validate vs PyTorch")
        print(sep)
        y_pt = pytorch_forward(model, x_np)
        validate(lib, x_np, y_pt, T, nu, ny, args.tol)

        # ── stage 5c: pytorch benchmark ───────────────────────────────────────
        t_pt = benchmark_pytorch(model, x_np, args.runs, args.warmup)
        print_timing("PyTorch CPU baseline        ", t_pt, T)

        speedup = t_pt.mean() / t_c.mean()
        print(f"\n{sep}")
        print(f"  v3 C is {speedup:.2f}x {'faster' if speedup >= 1 else 'slower'} "
              f"than PyTorch  (mean latency)")
        print(sep)

        # ── summary of output files ───────────────────────────────────────────
        print(f"\n{sep}")
        print(f"  Output files")
        print(sep)
        for p in [bin_path, c_path, lib_path]:
            size = os.path.getsize(p) / 1024
            print(f"  {p}  ({size:.0f} KB)")
        print()

    finally:
        if lib is not None:
            unload_lib(lib)
        # lib_path is kept as a deliverable — do NOT delete it here


if __name__ == "__main__":
    main()
