#!/usr/bin/python3

"""
Test oscc-check 'main' function.
"""

import itertools
import oscccan
import unittest

from docopt import docopt

from hypothesis import given, HealthCheck, reject, settings
from hypothesis.strategies import (
    integers, lists, randoms, text, sampled_from, characters, composite)

from tests import CLI, oscc_check, mocks


class Main(unittest.TestCase):
    """
    Mocked oscc-check.py main().
    """

    def run(self, args=['']):
        if not isinstance(args, list):
            return False

        flat_args = [item for sublist in args for item in sublist]

        ret = True
        try:
            oscc_check.main(docopt(oscc_check.__doc__, argv=flat_args))
        except:
            ret = False

        return ret


@composite
def runtime_args(draw):
    vehicle = draw(sampled_from(CLI.required_flags()))
    options = draw(sampled_from(CLI.valid_options()))
    return [vehicle, options]


@settings(max_examples=25)
@given(runtime_args())
def test_valid_flags_valid_args(args):
    oscc_check.colorama.init = mocks.CallableNoOp
    oscc_check.CanBus = mocks.CanBus
    main = Main()
    assert main.run(args=args)
