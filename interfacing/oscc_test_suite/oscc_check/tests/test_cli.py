#!/usr/bin/python3

"""
Test CLI Tooling Module.
"""

from hypothesis import given, HealthCheck, reject, settings
from hypothesis.strategies import (
    integers, lists, randoms, text, sampled_from, characters, composite)

from tests import CLI, oscc_check

random_args = text(characters(max_codepoint=1000, blacklist_categories=(
    'Cc', 'Cs')), min_size=1).map(lambda s: s.strip()).filter(lambda s: len(s) > 0)


@composite
def flagged_args(draw):
    flag = draw(sampled_from(CLI.valid_flags()))
    arg = draw(random_args)
    return [flag, arg]


@given(lists(random_args))
def test_random_args(args):
    assert CLI.run(args=args)


@given(flagged_args())
def test_valid_flags_random_args(args):
    assert CLI.run(args=args)


@given(lists(sampled_from(CLI.valid_combinatons())))
def test_valid_flags_valid_args(args):
    assert CLI.run(args=args)
