#!/usr/bin/env python3
# Copyright 2023 Maximilian Leitenstern
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ========================================== //
# Author: Maximilian Leitenstern (TUM)
# Date: 11.11.2023
# ========================================== //
#

import os
import sys
import yaml

def update_config(file_path, param_name, param_value, param_type):
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

    # Change the specified parameter value and type
    if param_type == 'str':
        config['/**']['ros__parameters'][param_name] = param_value
    elif param_type == 'int':
        config['/**']['ros__parameters'][param_name] = int(param_value)
    elif param_type == 'float':
        config['/**']['ros__parameters'][param_name] = float(param_value)
    elif param_type == 'bool':
        if param_value == 'True':
            config['/**']['ros__parameters'][param_name] = True
        else:
            config['/**']['ros__parameters'][param_name] = False
    else:
        print(f"Error: Parameter type '{param_type}' not supported. Supported options are 'str', 'int', 'float', 'bool'.")
        sys.exit(1)

    # Save the modified config back to the YAML file
    with open(config_file_path, 'w') as file:
        yaml.dump(config, file, default_flow_style=False)

if __name__ == "__main__":
    # Check command-line arguments
    if len(sys.argv) != 5:
        print("Usage: config_param.py <config_file_name> <param_name> <param_value> <param_type>")
        sys.exit(1)

    # Get command-line arguments
    config_file_name = sys.argv[1]
    param_name = sys.argv[2]
    param_value = sys.argv[3]
    param_type = sys.argv[4]

    # Check if the config file name is valid
    valid_config_names = ['lanelet2_osm.param.yaml', 'kiss_icp_georef.param.yaml']
    if config_file_name not in valid_config_names:
        print(f"Error: Invalid config file name. Supported names are {', '.join(valid_config_names)}")
        sys.exit(1)

    # Update the specified parameter in the config file
    update_config(config_file_name, param_name, param_value, param_type)

    print(f"Parameter '{param_name}' in '{config_file_name}' set to '{param_value}' with type '{param_type}'")
