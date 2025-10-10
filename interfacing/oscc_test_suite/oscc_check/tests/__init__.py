#!/usr/bin/python3

"""
Test CLI Tooling Module
"""

from docopt import docopt
import itertools
import os
import subprocess
import importlib.util

spec = importlib.util.spec_from_file_location(
    "*", "oscc-check.py")

oscc_check = importlib.util.module_from_spec(spec)

spec.loader.exec_module(oscc_check)


class CLI(object):
    """
    CLI Tooling Class.
    """

    @staticmethod
    def valid_flags():
        return [
            ['-V'],
            ['-c'],
            ['-b'],
            ['-h'],
            ['-d'],
            ['-e'],
            ['-l'],
            ['-v'],
            [''],
        ]

    @staticmethod
    def valid_combinatons():
        return [
            ['-V', 'kia_soul_ev'],
            ['-V', 'kia_soul_petrol'],
            ['-V', 'kia_niro'],
            ['-c', 'can0'],
            ['-c', 'can1'],
            ['-c', '0'],
            ['-c', '1'],
            ['-c', 'PCAN_USBBUS0'],
            ['-c', 'PCAN_USBBUS1'],
            ['-b', 'kvaser'],
            ['-b', 'socketcan'],
            ['-b', 'pcan'],
            ['-h'],
            ['-d'],
            ['-e'],
            ['-l'],
            ['-v'],
        ]

    @staticmethod
    def required_flags():
        return [
            ['-V', 'kia_soul_ev'],
            ['-V', 'kia_soul_petrol'],
            ['-V', 'kia_niro'],
        ]

    @staticmethod
    def valid_options():
        return [
            ['-c', 'can0'],
            ['-c', 'can1'],
            ['-c', '0'],
            ['-c', '1'],
            ['-c', 'PCAN_USBBUS0'],
            ['-c', 'PCAN_USBBUS1'],
            ['-b', 'kvaser'],
            ['-b', 'socketcan'],
            ['-b', 'pcan'],
            ['-d'],
            ['-e'],
            # Don't allow tests to loop indefinitely
            # ['-l'],
        ]

    @staticmethod
    def run(args=['']):
        if not isinstance(args, list):
            return False

        flat_args = [item for sublist in args for item in sublist]

        ret = False
        try:
            ret = docopt(oscc_check.__doc__, argv=flat_args)
        except SystemExit as e:
            # Bad args should cause exit with usage statement (first line of doc string).
            if str(e) == oscc_check.__doc__.split('\n', 1)[0]:
                ret = True
            # Successful `-h` flag raises SystemExit with no error.
            elif str(e) == '':
                ret = True
            else:
                ret = False

        return ret
