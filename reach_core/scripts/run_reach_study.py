#!/usr/bin/env python3

import argparse
import os
import shutil
import yaml

from reach_study import *

def main(results_dir, config_name, config_file, sleep, delete_existing):
    # Delete the old study if requested
    if delete_existing:
        if os.path.exists(results_dir) and os.path.isdir(results_dir):
            print("Deleting the existing study at " + args.results_dir)
            shutil.rmtree(args.results_dir)

    # Delay start up so we can attach a debugger
    if sleep:
        input('Press enter to continue')

    # Create a new reach study if no YAML file specified
    if config_file == "":
        ik_solver = NewIKSolver()
        evaluator = NewEvaluator()
        target_pose_generator = NewTargetPoseGenerator()
        display = NewDisplay()
        logger = NewLogger()

        params = rp.Parameters()
        params.radius = 0.4
        params.max_steps = 10
        params.step_improvement_threshold = 0.01

        runReachStudyPy(ik_solver, evaluator, target_pose_generator, display, logger, params, config_name,
                        results_dir)

    # Otherwise load the YAML file and run the reach study
    else:
        with open(args.config_file, "r") as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as e:
                print(e)
                exit()

        opt_config = config["optimization"]
        ik_config = config["ik_solver"]
        pose_gen_config = config["target_pose_generator"]
        eval_config = config["evaluator"]
        display_config = config["display"]
        logger_config = config["logger"]

        loader = rp.PluginLoader()
        loader.search_libraries_env = "REACH_PLUGINS"

        ik_solver_factory = loader.createIKSolverFactoryInstance(ik_config["name"])
        ik_solver = ik_solver_factory.create(ik_config)

        target_pose_factory = loader.createTargetPoseGeneratorFactoryInstance(pose_gen_config["name"])
        target_pose_generator = target_pose_factory.create(pose_gen_config)

        evaluator_factory = loader.createEvaluatorFactoryInstance(eval_config["name"])
        evaluator = evaluator_factory.create(eval_config)

        display_factory = loader.createDisplayFactoryInstance(display_config["name"])
        display = display_factory.create(display_config)

        logger_factory = loader.createLoggerFactoryInstance(logger_config["name"])
        logger = logger_factory.create(logger_config)

        params = rp.Parameters()
        params.radius = opt_config["radius"]
        params.max_steps = opt_config["max_steps"]
        params.step_improvement_threshold = opt_config["step_improvement_threshold"]

        runReachStudyPy(ik_solver, evaluator, target_pose_generator, display, logger, params, config_name, results_dir)


if __name__ == "__main__":
    os.environ["REACH_PLUGINS"] = "libreach_core_plugins"

    parser = argparse.ArgumentParser(prog="RunReachStudy", description="Wraps the reach study library in Python")
    parser.add_argument("results_dir", type=str, help="Where to store the results")
    parser.add_argument("config_name", type=str, help="Name of this config")
    parser.add_argument("--config-file", type=str, help="YAML config file", default="")
    parser.add_argument('-s', '--sleep', action='store_true', help="Wait 5 seconds before starting")
    parser.add_argument('-d', '--delete-existing', action='store_true', help="Delete an existing study at results_dir if it exists")
    args = parser.parse_args()

    main(args.results_dir, args.config_name, args.config_file, args.sleep, args.delete_existing)