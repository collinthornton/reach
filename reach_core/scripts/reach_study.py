#!/usr/bin/env python3

import reach_core.reach_core_python as rp
import numpy as np
import os

class NewIKSolver(rp.IKSolver):
    def getJointNames(self):
        names = [ ]
        names.append("joint")
        return names

    def solveIK(self, isometry, map):
        solution = [ ]
        solution.append(list())
        solution[0].append(0.0)
        return solution


class NewEvaluator(rp.Evaluator):
    def calculateScore(self, map):
        score = 4.0
        return score


class NewTargetPoseGenerator(rp.TargetPoseGenerator):
    def generate(self):
        l = [ ]
        l.append(np.zeros((4,4), np.double))
        l.append(np.ones((4,4), np.double))
        l.append(2*np.ones((4,4), np.double))
        l.append(3*np.ones((4,4), np.double))

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

    def setMaxProgress(self, max_progress):
        self.max_progress_ = max_progress
        return

    def printProgress(self, progress):
        print(str(100*progress/self.max_progress_) + "% complete")
        return

    def printResults(self, results):
        print(results.print())
        return

    def print(self, message):
        print(message)
        return


def runReachStudyPy(ik_solver: rp.IKSolver, evaluator: rp.Evaluator, target_pose_generator: rp.TargetPoseGenerator,
                    display: rp.Display, logger: rp.Logger, params: rp.Parameters, config_name: str, results_dir: str):
    # Initialize the reach study
    study = rp.ReachStudy(ik_solver, evaluator, target_pose_generator, display, logger, params, config_name)

    path = results_dir + "/" + config_name
    db_file = path + "/study.db"
    opt_db_file = path + "/study_optimized.db"

    # Create the reach directory if needed
    if not os.path.exists(results_dir):
        logger.print("Creating the results directory at " + results_dir)
        os.makedirs(results_dir)

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

    return
