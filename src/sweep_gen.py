import argparse
import itertools
import json
import os
from copy import deepcopy

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True, help="Base params json")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--param", action="append", required=True,
                    help="Format: NAME=val1,val2,val3")
    args = ap.parse_args()

    base = json.load(open(args.base, "r", encoding="utf-8"))

    ranges = []
    for p in args.param:
        name, values = p.split("=", 1)
        vals = [float(x) for x in values.split(",")]
        ranges.append((name, vals))

    os.makedirs(args.out_dir, exist_ok=True)

    names = [r[0] for r in ranges]
    grids = [r[1] for r in ranges]

    idx = 0
    for combo in itertools.product(*grids):
        params = deepcopy(base)
        for n, v in zip(names, combo):
            params[n] = v
        path = os.path.join(args.out_dir, f"params_{idx:03d}.json")
        json.dump(params, open(path, "w", encoding="utf-8"), indent=2)
        idx += 1

    print(f"Generated {idx} param sets into {args.out_dir}")

if __name__ == "__main__":
    main()
