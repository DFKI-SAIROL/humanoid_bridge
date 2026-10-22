"""dex3_probe's argument checks. No DDS: parse_args only."""

from __future__ import annotations

import pytest

from g1_stack.nodes import dex3_probe

# Valid on its own: every refusal below comes from the one flag under test.
COMMAND = ["command", "--side", "left", "--motors", "0", "--q", "0.2"]


@pytest.mark.parametrize("value", ["nan", "NaN", "-nan", "inf", "-inf", "Infinity"])
@pytest.mark.parametrize("flag", ["--tau", "--q", "--hz", "--duration", "--wait", "--ramp"])
def test_non_finite_values_are_refused(flag, value):
    with pytest.raises(SystemExit) as raised:
        dex3_probe.parse_args([*COMMAND, f"{flag}={value}", "--yes"])  # = : "-inf" is no option
    assert raised.value.code == 2


def test_nan_torque_no_longer_slips_under_the_ceiling():
    # abs(nan) > TAU_CEILING is False: the ceiling alone let it through.
    with pytest.raises(SystemExit):
        dex3_probe.parse_args([*COMMAND, "--tau", "nan", "--yes"])


def test_torque_over_the_ceiling_is_refused():
    with pytest.raises(SystemExit):
        dex3_probe.parse_args([*COMMAND, "--tau", str(dex3_probe.TAU_CEILING + 0.1)])


def test_finite_values_parse():
    args = dex3_probe.parse_args([*COMMAND, "--tau", "-0.3", "--hz", "50", "--ramp", "1.5"])
    assert (args.tau, args.hz, args.ramp) == (-0.3, 50.0, 1.5)
    assert dex3_probe.parse_args(COMMAND).q == 0.2


def test_finite_float_rejects_text():
    with pytest.raises(ValueError):
        dex3_probe.finite_float("abc")
