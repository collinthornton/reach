#!/usr/bin/env python3

from libreach_core import YAMLNode, Path, runReachStudy
import argparse

parser = argparse.ArgumentParser(prog="RunReachStudy", description="Wraps the reach study library in Python")
parser.add_argument("results_dir", type=str, help="Where to store the results")
parser.add_argument("config_file", type=str, help="YAML config file")
parser.add_argument("config_name", type=str, help="Name of this config")
parser.add_argument('-s', '--sleep', action='store_true', help="Wait 5 seconds before starting")

args = parser.parse_args()

# Delay start up so we can attach a debugger
if args.sleep == True:
    print("Sleep 5 seconds before starting the reach study")
    import time
    time.sleep(5)

node = YAMLNode.LoadFile(args.config_file)
runReachStudy(node, args.config_name, Path(args.results_dir), False)