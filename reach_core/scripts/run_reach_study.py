#!/usr/bin/env python3

import reach_core.reach_core_python as rp
import argparse
import os
import shutil

os.environ["REACH_PLUGINS"] = "libreach_core_plugins"

parser = argparse.ArgumentParser(prog="RunReachStudy", description="Wraps the reach study library in Python")
parser.add_argument("results_dir", type=str, help="Where to store the results")
parser.add_argument("config_file", type=str, help="YAML config file")
parser.add_argument("config_name", type=str, help="Name of this config")
parser.add_argument('-s', '--sleep', action='store_true', help="Wait 5 seconds before starting")
parser.add_argument('-d', '--delete', action='store_true', help="Delete an existing study at results_dir if it exists")
args = parser.parse_args()

# Delete the old study if requested
if args.delete:
    if os.path.exists(args.results_dir) and os.path.isdir(args.results_dir):
        print("Deleting the existing study at " + args.results_dir)
        shutil.rmtree(args.results_dir)

# Delay start up so we can attach a debugger
if args.sleep == True:
    print("Sleep 5 seconds before starting the reach study")
    import time
    time.sleep(5)

class NewIKSolver(rp.IKSolver):
    def getJointNames(self):
        return ""

    def solveIK(self, isometry, map):
        return []

class NewEvaluator(rp.Evaluator):
    def calculateScore(self):
        return 0.0

class NewDisplay(rp.Display):
    def showEnvironment(self):
        return

    def updateRobotPose(self):
        return

    def showReachNeighborhood(self):
        return

    def showResults(self):
        return

class NewLogger(rp.Logger):
    def setMaxProgress(self, max_progress):
        return

    def printProgress(self, progress):
        return

    def printResults(self, results):
        return

    def print(self, message):
        return

study = rp.ReachStudy(NewIKSolver(), NewEvaluator(), rp.TargetPoseGenerator(), NewLogger(), rp.Parameters(), "test")

node = rp.YAMLNode.LoadFile(args.config_file)
rp.runReachStudy(node, args.config_name, rp.Path(args.results_dir), False)