"""MolmoAct2 visual selection -> geometry -> MoveL; --execute enables movement."""

import argparse
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--execute", action="store_true")
    flags, rest = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    if flags.execute and "--dry-run" in rest:
        parser.error("choose --execute or --dry-run")
    if not flags.execute and "--dry-run" not in rest:
        rest.append("--dry-run")
    if not any(arg == "--selector-url" or arg.startswith("--selector-url=") for arg in rest):
        rest.extend(["--selector-url", "http://127.0.0.1:8211"])
    from .run import main as run
    return run(rest)


if __name__ == "__main__":
    raise SystemExit(main())
