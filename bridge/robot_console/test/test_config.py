"""The console config: what parses, what is refused, and how paths expand."""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
from robot_console.config import ConfigError, load, parse, read_env_file

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "console.yaml"

BASE = {
    "console": {"bridge_config": "/params/G1_config.yaml"},
    "processes": {"node": {"argv": ["python3", "-c", "print(1)"]}},
}


def doc(**process):
    d = copy.deepcopy(BASE)
    d["processes"]["node"].update(process)
    return d


def test_the_shipped_example_loads():
    config = load(EXAMPLE)
    assert config.console.robot == "G1"
    assert config.console.bridge_config.is_file()
    assert config.console.hands is True
    assert [p.name for p in config.processes] == ["lowcmd_rate"]


def test_defaults():
    config = parse(BASE)
    node = config.process("node")
    assert node.group == "run"
    assert node.stop.signals == ("INT", "TERM", "KILL")
    assert node.stop.grace_s == (10.0, 5.0)
    assert node.label == "node"
    assert config.console.lowstate_type == "unitree_hg/msg/LowState"
    assert config.console.camera is None


def test_console_paths_expand_and_resolve_against_the_file(tmp_path):
    config = parse(
        {"console": {"bridge_config": "${CLONE}/p.yaml", "urdf": "rel/g1.urdf"}},
        environ={"CLONE": "/opt/clone"},
        base=tmp_path,
    )
    assert config.console.bridge_config == Path("/opt/clone/p.yaml")
    assert config.console.urdf == tmp_path / "rel/g1.urdf"


def test_params_choices_and_whole_element_text():
    config = parse(
        doc(
            argv=["run", "--ckpt={ckpt}", "--instruction", "{text}"],
            params={
                "ckpt": {"choices": ["a", "b"], "default": "a"},
                "text": {"text": True, "pattern": "^[a-z ]*$"},
            },
        )
    )
    params = config.process("node").params
    assert params["ckpt"].choices == ("a", "b")
    assert params["text"].text and params["text"].pattern == "^[a-z ]*$"


def test_env_braces_are_not_params():
    # ${HOME} inside a bash -c script is the shell's, not a param placeholder.
    parse(doc(argv=["bash", "-c", "echo ${HOME} {not_a_param}"]))


@pytest.mark.parametrize(
    "bad,match",
    [
        ({"argv": []}, "empty"),
        ({"group": "sim"}, "group"),
        ({"bogus": 1}, "unknown key"),
        ({"params": {"p": {"choices": ["x"]}}}, "never appear"),
        ({"argv": ["bash", "-c", "echo {t}"], "params": {"t": {"text": True}}}, "whole element"),
        ({"argv": ["x", "{p}"], "params": {"p": {"choices": ["a"], "default": "b"}}}, "default"),
        ({"argv": ["x", "{p}"], "params": {"p": {"choices": ["a"], "text": True}}}, "exactly one"),
        (
            {"argv": ["x", "{p}"], "params": {"p": {"choices": ["a"], "pattern": "a"}}},
            "only for text",
        ),
        ({"stop": {"signals": ["CTRL-C", "KILL"], "grace_s": [1]}}, "tty"),
        ({"stop": {"signals": ["INT", "KILL"], "grace_s": [1, 2]}}, "grace_s"),
        ({"stop": {"signals": ["STOP"]}}, "allowed"),
        ({"prompts": [{"match": "go", "answers": ["go"]}]}, "tty"),
        ({"requires": ["nobody.ready"]}, "no process"),
        ({"requires": ["node.ready"]}, "itself"),
        ({"requires": ["armed!"]}, "expected armed"),
        ({"ready": {"http": "x", "log": "y"}}, "ready"),
        ({"ready": {"log": "("}}, "bad regex"),
    ],
)
def test_refused(bad, match):
    with pytest.raises(ConfigError, match=match):
        parse(doc(**bad))


def test_bridge_config_is_required():
    with pytest.raises(ConfigError, match="bridge_config"):
        parse({"console": {}})


def test_unknown_top_level_key():
    with pytest.raises(ConfigError, match="unknown key"):
        parse({**BASE, "extra": 1})


def test_tty_allows_ctrl_c_and_prompts():
    spec = parse(
        doc(
            tty=True,
            stop={"signals": ["CTRL-C", "TERM", "KILL"], "grace_s": [5, 5]},
            prompts=[{"match": "ready\\? \\[go/abort\\]", "answers": ["go", "abort"]}],
        )
    ).process("node")
    assert spec.stop.signals[0] == "CTRL-C"
    assert spec.prompts[0].answers == ("go", "abort")


def test_env_file(tmp_path):
    path = tmp_path / ".env"
    path.write_text("# comment\n\nA=1\nexport B='two words'\nC=\"3\"\nnot a line\n1BAD=x\n")
    assert read_env_file(path) == {"A": "1", "B": "two words", "C": "3"}
