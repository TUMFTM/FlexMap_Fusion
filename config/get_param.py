#!/usr/bin/env python3

import os
import sys
import yaml

def get_current_value(file_path, param_name):
    # Get the directory of the script
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Construct the full path to the config file
    config_file_path = os.path.join(script_dir, file_path)

    # Load the YAML file
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # Check if the 'ros__parameters' key exists
    if 'ros__parameters' not in config['/**']:
        print("Error: 'ros__parameters' key does not exist in the config file.")
        sys.exit(1)

    # Extract the 'ros__parameters' section
    ros_parameters = config['/**']['ros__parameters']

    # Check if the parameter exists in the loaded config
    if param_name not in ros_parameters:
        print(f"Error: Parameter '{param_name}' does not exist in the config file.")
        sys.exit(1)

    # Print the current value of the specified parameter
    current_value = ros_parameters[param_name]
    print(f"Current value of parameter '{param_name}': {current_value}")

if __name__ == "__main__":
    # Check command-line arguments
    if len(sys.argv) != 3:
        print("Usage: get_param.py <config_file_name> <param_name>")
        sys.exit(1)

    # Get command-line arguments
    config_file_name = sys.argv[1]
    param_name = sys.argv[2]

    # Check if the config file name is valid
    valid_config_names = ['lanelet2_osm.param.yaml', 'kiss_icp_georef.param.yaml']
    if config_file_name not in valid_config_names:
        print(f"Error: Invalid config file name. Supported names are {', '.join(valid_config_names)}")
        sys.exit(1)

    # Get and print the current value of the specified parameter
    get_current_value(config_file_name, param_name)
