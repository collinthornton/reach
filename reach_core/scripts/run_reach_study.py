#!/usr/bin/env python3

import reach_core.reach_core_python as rp
import numpy as np
import argparse
import os
import shutil
import threading

os.environ["REACH_PLUGINS"] = "libreach_core_plugins"

parser = argparse.ArgumentParser(prog="RunReachStudy", description="Wraps the reach study library in Python")
parser.add_argument("results_dir", type=str, help="Where to store the results")
#parser.add_argument("config_file", type=str, help="YAML config file")
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
        return [""]

    def solveIK(self, isometry, map):
        return [[0.0]]


class NewEvaluator(rp.Evaluator):
    def calculateScore(self, map):
        return 0.0


class NewTargetPoseGenerator(rp.TargetPoseGenerator):
    def generate(self):
        l = [ ]
        l.append(tuple(map(tuple, np.zeros((4,4), np.double))))
        l.append(tuple(map(tuple, np.ones((4,4), np.double))))
        l.append(tuple(map(tuple, 2*np.ones((4,4), np.double))))
        l.append(tuple(map(tuple, 3*np.ones((4,4), np.double))))

        return l


class NewDisplay(rp.Display):
    def showEnvironment(self):
        return

    def updateRobotPose(self, pose):
        return

    def showReachNeighborhood(self, neighborhood):
        return

    def showResults(self, results):
        return


class NewLogger(rp.Logger):
    def __init__(self):
        super().__init__()
        self.max_progress_ = 0
        self.lock = threading.Lock()

    def setMaxProgress(self, max_progress):
        with self.lock:
            self.max_progress_ = max_progress
        return

    def printProgress(self, progress):
        with self.lock:
            print(str(100*progress/self.max_progress_) + "% complete")
        return

    def printResults(self, results):
        with self.lock:
            print(results.print())
        return

    def print(self, message):
        with self.lock:
            print(message)
        return


ik_solver = NewIKSolver()
evaluator = NewEvaluator()
target_pose_generator = NewTargetPoseGenerator()
display = NewDisplay()
logger = NewLogger()

params = rp.Parameters()
params.radius = 0.4
params.max_steps = 10
params.step_improvement_threshold = 0.01

# Initialize the reach study
study = rp.ReachStudy(ik_solver, evaluator, target_pose_generator, display, logger, params, args.config_name)

path = args.results_dir + "/" + args.config_name
db_file = path + "/study.db"
opt_db_file = path + "/study_optimized.db"

# Create the reach directory if needed
if not os.path.exists(args.results_dir):
    logger.print("Creating the results directory at " + args.results_dir)
    os.makedirs(args.results_dir)

# Create the config directory if needed
if not os.path.exists(path):
    logger.print("Creating the results config subdirectory at " + path)
    os.makedirs(path)

if os.path.isfile(opt_db_file):
    study.load(opt_db_file)
    logger.print("Loaded optimized database")
elif os.path.isfile(db_file):
    study.load(db_file)
    logger.print("Loaded un-optimized database")

    study.optimize()
    study.save(opt_db_file)
else:
    # Run the reach study
    study.run()

    # Save the preliminary results
    study.save(db_file)

    # Optimize the reach study
    study.optimize()

    # Save the optimized results
    study.save(opt_db_file)


# Show the results
results = study.getDatabase()
logger.printResults(results.calculateResults())
display.showEnvironment()
display.showResults(results)

#node = rp.YAMLNode.LoadFile(args.config_file)
#rp.runReachStudy(node, args.config_name, rp.Path(args.results_dir), False)