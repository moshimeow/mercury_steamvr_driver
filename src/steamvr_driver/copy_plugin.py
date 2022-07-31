#!/usr/bin/env python3

# Copyright 2020, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

import os, sys, shutil

# get absolute input and output paths
input_path = os.path.join(
    sys.argv[1])

output_path = os.path.join(
    sys.argv[2])

# make sure destination directory exists
os.makedirs(os.path.dirname(output_path), exist_ok=True)

shutil.copyfile(input_path, output_path)

print("Copying plugin " + str(input_path) + " to " + str(output_path))
