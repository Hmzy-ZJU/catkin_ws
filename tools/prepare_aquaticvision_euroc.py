#!/usr/bin/env python3
import argparse
import math
import os
import re
import shutil
from pathlib import Path


IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


def read_text(path):
    return Path(path).read_text(errors="ignore")


def numbers(s):
    return [float(x) for x in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", s)]


def find_dataset_dirs(root):
    root = Path(root)
    seqs = []
    for p in sorted(root.rglob("*")):
        if not p.is_dir():
            continue
        if (p / "Stereo images" / "l1").is_dir() and (p / "Stereo images" / "r1").is_dir():
            sid = sequence_id_from_name(p.name)
            seqs.append((sid, p.name, p))
        elif (p / "l1").is_dir() and (p / "r1").is_dir():
            sid = sequence_id_from_name(p.name)
            seqs.append((sid, p.name, p))
    return seqs


def sequence_id_from_name(name):
    m = re.match(r"^\s*(\d+)", name)
    if m:
        return m.group(1).zfill(2)
    m = re.search(r"(\d+)", name)
    if m:
        return m.group(1).zfill(2)
    return re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_")


def find_sequence(root, key):
    seqs = find_dataset_dirs(root)
    if not seqs:
        raise RuntimeError(f"no AquaticVision sequence folders found under {root}; expected folders containing l1/ and r1/")
    key_norm = key.strip()
    for sid, name, path in seqs:
        if key_norm == sid or key_norm == name or key_norm == path.name:
            return sid, name, path
    lowered = key_norm.lower()
    for sid, name, path in seqs:
        if lowered in name.lower():
            return sid, name, path
    raise RuntimeError(f"sequence not found: {key}. Available: " + ", ".join(f"{sid}:{name}" for sid, name, _ in seqs))


def find_calib_file(root, candidates):
    root = Path(root)
    for rel in candidates:
        p = root / rel
        if p.is_file():
            return p
    for p in root.rglob("*.yaml"):
        low = p.name.lower()
        if any(c.lower().replace("/", os.sep) in str(p).lower() or c.lower() in low for c in candidates):
            return p
    return None


def parse_camera_yaml(path):
    txt = read_text(path)
    vals = {}

    def first_nums(pattern, min_count):
        m = re.search(pattern, txt, re.I | re.S)
        if not m:
            return None
        xs = numbers(m.group(1))
        return xs if len(xs) >= min_count else None

    intr = first_nums(r"intrinsics\s*:\s*\[([^\]]+)\]", 4)
    if intr:
        vals["fx"], vals["fy"], vals["cx"], vals["cy"] = intr[:4]

    dist = first_nums(r"distortion_coeffs\s*:\s*\[([^\]]+)\]", 4)
    if dist:
        vals["k1"], vals["k2"], vals["p1"], vals["p2"] = dist[:4]

    res = first_nums(r"resolution\s*:\s*\[([^\]]+)\]", 2)
    if res:
        vals["width"], vals["height"] = int(res[0]), int(res[1])

    for key in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "width", "height"):
        m = re.search(rf"(?:Camera\.)?{key}\s*:\s*([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)", txt)
        if m:
            vals[key] = float(m.group(1))

    data = first_nums(r"camera_matrix\s*:[\s\S]*?data\s*:\s*\[([^\]]+)\]", 9)
    if data and not all(k in vals for k in ("fx", "fy", "cx", "cy")):
        vals["fx"], vals["fy"], vals["cx"], vals["cy"] = data[0], data[4], data[2], data[5]

    ddata = first_nums(r"distortion_coefficients\s*:[\s\S]*?data\s*:\s*\[([^\]]+)\]", 4)
    if ddata and not all(k in vals for k in ("k1", "k2", "p1", "p2")):
        vals["k1"], vals["k2"], vals["p1"], vals["p2"] = ddata[:4]

    missing = [k for k in ("fx", "fy", "cx", "cy") if k not in vals]
    if missing:
        raise RuntimeError(f"cannot parse {missing} from calibration file {path}")
    vals.setdefault("k1", 0.0)
    vals.setdefault("k2", 0.0)
    vals.setdefault("p1", 0.0)
    vals.setdefault("p2", 0.0)
    return vals


def parse_baseline(root, fallback=None):
    if fallback is not None and fallback > 0:
        return fallback
    for p in Path(root).rglob("*.yaml"):
        txt = read_text(p)
        m = re.search(r"T_cn_cnm1\s*:\s*((?:\s*-\s*\[[^\]]+\]\s*){4})", txt, re.I)
        if m:
            rows = re.findall(r"\[([^\]]+)\]", m.group(1))
            if rows:
                row0 = numbers(rows[0])
                if len(row0) >= 4 and abs(row0[3]) > 1e-6:
                    return abs(row0[3])
        m = re.search(r"baseline\s*:\s*([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)", txt, re.I)
        if m and abs(float(m.group(1))) > 1e-6:
            return abs(float(m.group(1)))
    raise RuntimeError("cannot parse stereo baseline from calibration files; set AQUATIC_BASELINE, e.g. AQUATIC_BASELINE=0.10")


def token_to_ns(token):
    token = token.strip()
    if not token:
        return None
    base = Path(token).stem
    xs = numbers(base)
    if not xs:
        return None
    v = xs[-1]
    if abs(v) > 1e12:
        return str(int(round(v)))
    if abs(v) > 1e6:
        return str(int(round(v * 1000.0)))
    return str(int(round(v * 1e9)))


def image_map(img_dir):
    out = {}
    files = []
    for p in sorted(Path(img_dir).iterdir()):
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS:
            files.append(p)
            out[p.name] = p
            out[p.stem] = p
            ns = token_to_ns(p.stem)
            if ns:
                out[ns] = p
    return out, files


def resolve_image(mapping, fallback_files, token, idx):
    token = token.strip()
    if token in mapping:
        return mapping[token]
    stem = Path(token).stem
    if stem in mapping:
        return mapping[stem]
    ns = token_to_ns(token)
    if ns and ns in mapping:
        return mapping[ns]
    for ext in IMAGE_EXTS:
        if token + ext in mapping:
            return mapping[token + ext]
        if ns and ns + ext in mapping:
            return mapping[ns + ext]
    if idx < len(fallback_files):
        return fallback_files[idx]
    raise RuntimeError(f"cannot resolve image token '{token}' at pair index {idx}")


def load_pairs(seq_dir):
    seq_dir = Path(seq_dir)
    image_root = seq_dir / "Stereo images" if (seq_dir / "Stereo images" / "l1").is_dir() else seq_dir
    lmap, lfiles = image_map(image_root / "l1")
    rmap, rfiles = image_map(image_root / "r1")
    pair_file = None
    for name in ("timestamp_pairs.txt", "timestamp_pair.txt", "timestamps.txt", "times.txt"):
        p = image_root / name
        if p.is_file():
            pair_file = p
            break
    pairs = []
    if pair_file:
        for line in pair_file.read_text(errors="ignore").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            toks = re.split(r"[\s,]+", line)
            if len(toks) == 1:
                lt = rt = toks[0]
            else:
                lt, rt = toks[0], toks[1]
            idx = len(pairs)
            ns = token_to_ns(lt) or token_to_ns(rt) or str(idx)
            pairs.append((ns, resolve_image(lmap, lfiles, lt, idx), resolve_image(rmap, rfiles, rt, idx)))
    else:
        for idx, (lp, rp) in enumerate(zip(lfiles, rfiles)):
            ns = token_to_ns(lp.stem) or token_to_ns(rp.stem) or str(idx)
            pairs.append((ns, lp, rp))
    if not pairs:
        raise RuntimeError(f"no stereo image pairs found in {seq_dir}")
    return pairs


def link_or_copy(src, dst, copy_images=False):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        return
    if copy_images:
        shutil.copy2(src, dst)
    else:
        try:
            os.symlink(src, dst)
        except OSError:
            shutil.copy2(src, dst)


def find_gt(seq_dir):
    seq_dir = Path(seq_dir)
    for name in ("gt.tum", "groundtruth.tum", "groundtruth/gt.tum", "groundtruth/groundtruth.tum"):
        p = seq_dir / name
        if p.is_file():
            return p
    hits = sorted(seq_dir.rglob("*gt*.tum")) + sorted(seq_dir.rglob("*groundtruth*.tum"))
    return hits[0] if hits else None


def write_config(path, cam, baseline, stereo, fps, topk):
    bf = cam["fx"] * baseline if stereo else None
    with open(path, "w", newline="\n") as f:
        f.write("%YAML:1.0\n")
        f.write("\n# Generated for AquaticVision IDVO/off testing\n")
        f.write('Camera.type: "PinHole"\n')
        for k in ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"):
            f.write(f"Camera.{k}: {cam[k]}\n")
        f.write(f"Camera.width: {int(cam.get('width', 346))}\n")
        f.write(f"Camera.height: {int(cam.get('height', 260))}\n")
        f.write(f"Camera.fps: {fps}\n")
        f.write("Camera.RGB: 0\n")
        if stereo:
            f.write(f"Camera.bf: {bf}\n")
            f.write("ThDepth: 40.0\n")
        f.write("\nORBextractor.nFeatures: 900\n")
        f.write("ORBextractor.scaleFactor: 1.2\n")
        f.write("ORBextractor.nLevels: 8\n")
        f.write("ORBextractor.iniThFAST: 12\n")
        f.write("ORBextractor.minThFAST: 5\n")
        f.write("\nViewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1\nViewer.GraphLineWidth: 0.9\n")
        f.write("Viewer.PointSize: 2\nViewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3\n")
        f.write("Viewer.ViewpointX: 0\nViewer.ViewpointY: -0.7\nViewer.ViewpointZ: -1.8\nViewer.ViewpointF: 500\n")
        f.write("\nuwfusion.enable: 0\n")
        f.write("\nInfoSelector.Enable: 1\n")
        f.write(f"InfoSelector.TopK: {topk}\n")
        f.write("InfoSelector.UseUniform: 1\nInfoSelector.w_uniform: 0.12\nInfoSelector.MinPxDist: 5\n")
        f.write("InfoSelector.LambdaInit: 1.0e-3\n")
        f.write("\nInfoKF.Use: 1\nInfoKF.AllowBitsDrop: 1.5\n")
        f.write("InfoKF.Dyn.Alpha: 0.25\nInfoKF.Dyn.Beta: 0.0\nInfoKF.Dyn.TauMin: 0.1\nInfoKF.Dyn.TauMax: 5.0\n")
        f.write("InfoKF.Cum.Decay: 0.95\nInfoKF.Cum.Thr: 0.5\nInfoKF.MaxFramesForce: 120\n")
        f.write("\nEnableAdaptiveIDVO: 0\nAdaptivePolicyType: \"Fixed\"\nEnableAdaptiveLogging: 1\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--sequence", default="")
    ap.add_argument("--out")
    ap.add_argument("--mono-config")
    ap.add_argument("--stereo-config")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--copy-images", action="store_true")
    ap.add_argument("--baseline", type=float, default=0.0)
    ap.add_argument("--topk", type=int, default=95)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        for sid, name, path in find_dataset_dirs(args.root):
            print(f"{sid}|{name}|{path}")
        return

    if not args.sequence or not args.out:
        raise SystemExit("--sequence and --out are required unless --list is used")

    sid, name, seq_dir = find_sequence(args.root, args.sequence)
    out = Path(args.out)
    cam0 = find_calib_file(args.root, ["calibration/cam0_pinhole.yaml", "cam0_pinhole.yaml", "cam0 pinhole.yaml", "cam0"])
    if not cam0:
        raise RuntimeError("cannot find cam0 calibration yaml under dataset root")
    cam = parse_camera_yaml(cam0)
    baseline = parse_baseline(args.root, args.baseline)

    pairs = load_pairs(seq_dir)
    if args.max_frames > 0:
        pairs = pairs[:args.max_frames]
    cam0_dir = out / "mav0" / "cam0" / "data"
    cam1_dir = out / "mav0" / "cam1" / "data"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)
    with open(out / "timestamps.txt", "w", newline="\n") as tf:
        for ns, lp, rp in pairs:
            link_or_copy(lp, cam0_dir / f"{ns}.png", args.copy_images)
            link_or_copy(rp, cam1_dir / f"{ns}.png", args.copy_images)
            tf.write(f"{ns}\n")

    gt = find_gt(seq_dir)
    if gt:
        link_or_copy(gt, out / "gt.tum", copy_images=True)

    if args.mono_config:
        write_config(args.mono_config, cam, baseline, False, args.fps, args.topk)
    if args.stereo_config:
        write_config(args.stereo_config, cam, baseline, True, args.fps, args.topk)

    print(f"sequence_id={sid}")
    print(f"sequence_name={name}")
    print(f"sequence_dir={seq_dir}")
    print(f"out={out}")
    print(f"frames={len(pairs)}")
    print(f"cam0_calib={cam0}")
    print(f"baseline={baseline}")
    print(f"gt_tum={out / 'gt.tum' if gt else ''}")


if __name__ == "__main__":
    main()
