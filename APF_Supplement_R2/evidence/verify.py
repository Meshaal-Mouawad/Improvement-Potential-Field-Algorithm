import pathlib, subprocess, sys
# Read-only wrapper. Generation and analysis are separate explicit commands.
raise SystemExit(subprocess.call(["node", str(pathlib.Path(__file__).with_name("verify.cjs")), *sys.argv[1:]]))
