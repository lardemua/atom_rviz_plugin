#!/usr/bin/env python3
import argparse
import os
import re
import rospkg
import subprocess

import ruamel.yaml.comments
from colorama import Fore
from urllib.parse import urlparse

import yaml
from urdf_parser_py.urdf import URDF
import sys
from ruamel.yaml import YAML, comments


def loadYMLConfig(filename):
    """Load configuration from a yml file"""
    try:
        with open(filename, 'r') as f:
            obj = yaml.load(f, Loader=yaml.SafeLoader)
    except OSError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return None

    return obj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-yc", "--yaml_content", help="Yaml with the content to be copied to the yaml format.", type=str,
                    required=True)
    ap.add_argument("-yf", "--yaml_format", help="Yaml with the format (comments included) which should be kept.",
                    type=str, required=True)
    ap.add_argument("-yo", "--yaml_out", help="Yaml file path to be written.", type=str, required=True)
    args = vars(ap.parse_args())

    yaml = YAML() # setup yaml reader object

    # Read the content and format files
    file_content = open(args['yaml_content'], 'r')
    content = yaml.load(file_content)

    file_format = open(args['yaml_format'], 'r')
    format = yaml.load(file_format)

    # Recursive copy of values
    def copy_values(din, dout):
        for item in din.keys():
            t = type(din[item])
            print('iterating ' + item + ' of type ' + str(t))
            if isinstance(din[item], ruamel.yaml.comments.CommentedMap):  # nested
                print('This is nested, going under ... ')
                copy_values(din[item], dout[item])
            else:  # non nested, direct copy
                dout[item] = din[item]

    # Call copy values function
    copy_values(content, format)

    # Dump format (now with content values changed) to file.
    file_out = open(args['yaml_out'], 'w')
    yaml.dump(format, file_out)

if __name__ == "__main__":
    main()
