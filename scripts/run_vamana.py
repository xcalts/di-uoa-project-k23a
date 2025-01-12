#!/usr/bin/env python3

import yaml
import subprocess


def main():
    # Define parameter values
    a_values = [1.0]
    L_values = list(range(130, 190, 10))  # 110 120 .. 200
    R_values = [10, 20, 30, 40, 50, 60, 70, 80, 90]

    # Path to the YAML configuration
    config_path = "./conf.yaml"

    for a in a_values:
        for L in L_values:
            for R in R_values:
                # Load the YAML file
                with open(config_path, "r") as f:
                    config_data = yaml.safe_load(f)

                # Update configuration parameters
                config_data["a"] = a
                config_data["L_small"] = L
                config_data["R_small"] = R

                # Write the updated configuration back to file
                with open(config_path, "w") as f:
                    yaml.safe_dump(config_data, f, sort_keys=False)

                # Run the commands
                print(f"Running with a={a}, L={L}, R={R}")
                # 1) Initialize
                subprocess.run(
                    [
                        "./bin/app",
                        "initialize",
                        "--conf",
                        config_path,
                        "--algo",
                        "stiched-vamana",
                    ],
                    check=True,
                )

                # 2) Evaluate
                subprocess.run(
                    [
                        "./bin/app",
                        "evaluate",
                        "--conf",
                        config_path,
                        "--algo",
                        "stiched-vamana",
                    ],
                    check=True,
                )
                print("-" * 40)


if __name__ == "__main__":
    main()
