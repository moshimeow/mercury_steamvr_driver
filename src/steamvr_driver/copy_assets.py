#!/usr/bin/env python3

# Copyright 2020, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

import os
import sys

import shutil

print(sys.argv[1], sys.argv[2], sys.argv[3])

is_file = sys.argv[1] == "FILE"
is_dir = sys.argv[1] == "DIRECTORY"

# get absolute input and output paths
input_path = sys.argv[2]

output_path = sys.argv[3]

# make sure destination directory exists
os.makedirs(os.path.dirname(output_path), exist_ok=True)

if is_file:
    try:
        os.remove(output_path)
    except:
        pass
    shutil.copyfile(input_path, output_path)
elif is_dir:
    shutil.rmtree(output_path, ignore_errors=True)
    shutil.copytree(input_path, output_path, dirs_exist_ok=True)
else:
    print(sys.argv[1], "must be FILE or DIRECTORY")
    sys.exit(1)

print("Copying asset " + str(input_path) + " to " + str(output_path))
