"""One thing drives the wheels, and everything else has to ask it first.

These checks read every launch file in the workspace as a syntax tree and
prove the motion path the whole project rests on:

    Nav2 / teleop / voice  ->  /cmd_vel_requested  ->  robot_safety  ->  /cmd_vel

That path has so far only been confirmed by a human watching two live runs.
CI has no Gazebo and never will, so without these tests the claim is a
comment. Here it is a build failure.

Two things make this file different from the launch checks that already
exist. First, the launch files are found by searching the workspace rather
than named one by one, so a launch file added tomorrow - in any package, in
a package that does not exist yet - is covered the moment it is written. The
existing tests name navigation.launch.py and simulation.launch.py
individually and would simply not look at a new one.

Second, where a wiring question is asked, it is answered by following the
code: a remapping written as a variable is chased back to the pair of topic
names it really holds. The existing checks compare text, so renaming a
variable or assigning it twice would slip past them while changing what the
robot does. Each place that happens is called out in a comment below.

When the wiring cannot be followed - a list assembled by a function call, a
name reassigned somewhere else - these tests fail rather than pass. Wiring a
reviewer cannot read is wiring nobody is checking.
"""

import ast
from pathlib import Path
import unittest

import yaml

SRC = Path(__file__).resolve().parents[2]

# Every launch_ros action that starts something which could turn out to be a
# motion source. LifecycleNode counts as much as Node: a managed node drives
# just as hard as an unmanaged one, and a helper blind to it would report an
# empty list for a launch file full of them - every assertion below would
# then pass by finding nothing to complain about.
NODE_ACTIONS = frozenset({"Node", "LifecycleNode", "ComposableNode"})

# The one node allowed to put a velocity on the wheels.
GATE_PACKAGE = "robot_safety"
GATE_EXECUTABLE = "safety_node"

# The names a safety gate goes by, in any of the three places a launch file
# can refer to it.
GATE_NAMES = frozenset({"robot_safety", "safety_node", "safety_controller"})

# The topic the wheels listen to, and the topic everything else must ask on.
WHEELS = "cmd_vel"
REQUEST = "cmd_vel_requested"

# Programs that publish a velocity as their normal job. Anything here is
# assumed to be driving the robot unless its launch file proves otherwise by
# remapping cmd_vel somewhere else. The Nav2 entries are the servers that
# steer during a goal, during a recovery, and while docking; the teleop
# entries are the human equivalents.
#
# This list has to be kept honest by hand, so
# `test_navigation_declares_no_velocity_source_this_file_has_not_heard_of`
# ties it to the list navigation.launch.py keeps for itself: adding a source
# there and not here fails that test with a message saying so.
VELOCITY_SOURCES = frozenset({
    "controller_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "docking_server",
    "teleop_twist_keyboard",
    "teleop_twist_joy",
})

# Calls that would let a package put something on a topic, ask a service for
# something, or send a goal - in short, act rather than watch.
PUBLISHING_CALLS = frozenset({
    "create_publisher",
    "create_client",
    "create_service",
    "ActionClient",
    "send_goal_async",
    "call_async",
})


class Unreadable(Exception):
    """The wiring could not be followed to the topic names it stands for."""


def topic(name: str) -> str:
    """A topic name with the leading slash taken off.

    `cmd_vel` and `/cmd_vel` are the same topic to ROS, and launch files in
    this workspace use both spellings, so comparing the text as written would
    miss half the cases.
    """
    return name.lstrip("/")


# Nodes that pick their velocity topics with a PARAMETER rather than with a
# remapping, and the parameter that names where their output goes.
#
# Almost everything here says where it publishes by remapping cmd_vel, which
# is written in the launch file and can be read straight out of it. Nav2's
# Collision Monitor does not: it takes the topic names as parameters, so its
# launch file never mentions cmd_vel at all and the checks below would either
# see nothing - and pass on a node that could be publishing to the wheels -
# or see a missing remap and fail on a node that is wired correctly.
#
# Either way the launch file alone cannot answer the question, so for these
# the parameter file is read too. A node added here with no parameter file to
# back it up is Unreadable, which fails.
PARAMETER_CONFIGURED_OUTPUT = {"collision_monitor": "cmd_vel_out_topic"}

# The other half of the same problem: what those nodes LISTEN to. A node that
# forwards a velocity is a link in a chain, and a link can only be followed if
# both of its ends can be read.
PARAMETER_CONFIGURED_INPUT = {"collision_monitor": "cmd_vel_in_topic"}


def parameter_configured_topic(node_name: str, wanted: str) -> str:
    """A velocity topic a node names with a parameter rather than a remapping.

    The parameter file is found by looking for the one that configures a node
    with this name, rather than by a path written down here, so moving or
    renaming the file makes this check fail rather than quietly stop running.
    """
    found = []
    for path in sorted(SRC.rglob("config/*.yaml")):
        try:
            document = yaml.safe_load(path.read_text())
        except yaml.YAMLError as error:
            raise Unreadable(f"{path.name}: {error}") from error
        if not isinstance(document, dict):
            continue
        block = document.get(node_name)
        if not isinstance(block, dict):
            continue
        parameters = block.get("ros__parameters")
        if isinstance(parameters, dict) and wanted in parameters:
            found.append((path.name, parameters[wanted]))

    if len(found) != 1:
        raise Unreadable(
            f"{node_name} decides where it publishes with the {wanted} "
            f"parameter, and {len(found)} parameter files in the workspace "
            "set it. Exactly one must, or nothing can say where this node's "
            "velocity ends up."
        )
    name = found[0][1]
    if not isinstance(name, str):
        raise Unreadable(f"{node_name}: {wanted} is not a topic name")
    return topic(name)


def topics_that_reach_the_gate() -> frozenset[str]:
    """Every topic from which a velocity still ends up at the safety gate.

    Being "behind the gate" used to mean one hop: publish to cmd_vel_requested
    and the gate decides on it. The Collision Monitor made the path two hops -
    Nav2 publishes cmd_vel_raw, the monitor forwards to cmd_vel_requested -
    and a check that only knows the one-hop shape would fail a correctly wired
    robot, which is the kind of failure that gets a safety test loosened.

    So the chain is followed instead of assumed. Start at the topic the gate
    itself listens to, and repeatedly add the input of any forwarding node
    whose output is already known to reach the gate. Anything left outside
    this set does not reach the gate, however it is spelled.

    Following it rather than listing it means another constraint layer can be
    inserted later without touching this test, while a node that forwards to
    somewhere else drops out of the set and fails the checks below.
    """
    reaching = {REQUEST}
    changed = True
    while changed:
        changed = False
        for node_name, output_parameter in PARAMETER_CONFIGURED_OUTPUT.items():
            if node_name not in PARAMETER_CONFIGURED_INPUT:
                raise Unreadable(
                    f"{node_name} has a parameter naming where its velocity "
                    "goes but none naming where it comes from, so the path "
                    "through it cannot be followed"
                )
            output = parameter_configured_topic(node_name, output_parameter)
            if output not in reaching:
                continue
            source = parameter_configured_topic(
                node_name, PARAMETER_CONFIGURED_INPUT[node_name]
            )
            if source not in reaching:
                reaching.add(source)
                changed = True

    # A forwarder wired into a loop would leave the wheels unreachable while
    # every membership check below still passed.
    if WHEELS in reaching:
        raise Unreadable(
            "the chain of forwarding nodes leads back to the wheel topic, so "
            "a velocity could reach the wheels without passing the gate"
        )
    return frozenset(reaching)


def docstring_ids(tree: ast.AST) -> set[int]:
    """The docstrings in a file, so they can be left out of a string search.

    The docstrings in this project spell out the motion path in full, so a
    plain search for a topic name finds the explanation rather than the
    wiring, and would pass on a file that says the right thing and does the
    wrong one.
    """
    found = set()
    for node in ast.walk(tree):
        body = getattr(node, "body", None)
        if not isinstance(body, list) or not body:
            continue
        first = body[0]
        if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant):
            found.add(id(first.value))
    return found


def code_strings(source: str) -> set[str]:
    """Every string in a file except the docstrings."""
    tree = ast.parse(source)
    skip = docstring_ids(tree)
    return {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and id(node) not in skip
    }


def called_names(source: str) -> set[str]:
    """Every name that is called in a file, read from the syntax tree.

    From the tree rather than from the text, so the word "publisher" in a
    comment is not mistaken for a publisher.
    """
    names = set()
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Attribute):
            names.add(node.func.attr)
        elif isinstance(node.func, ast.Name):
            names.add(node.func.id)
    return names


def package_sources(package: str) -> list[Path]:
    """Every Python file a package ships, its launch files included.

    The tests directory is left out: a test is allowed to name a topic it is
    asserting nobody publishes.
    """
    return [
        path
        for path in sorted((SRC / package).rglob("*.py"))
        if "tests" not in path.parts
    ]


class LaunchFile:
    """One launch file, with the bits of it that can be followed by reading.

    Nothing here runs the file. Launch files need ROS to execute and CI has
    none, so every question is answered from the syntax tree.
    """

    def __init__(self, path: Path) -> None:
        self.path = path
        self.name = path.name
        self.package = path.parents[1].name
        self.source = path.read_text()
        self.tree = ast.parse(self.source, str(path))
        self.assigned: dict[str, list[ast.expr]] = {}
        self.mutated: set[str] = set()
        self._collect_assignments()

    def __str__(self) -> str:
        return f"{self.package}/{self.name}"

    def _collect_assignments(self) -> None:
        """Note what every name in the file is set to, and how often.

        A name set twice is recorded twice on purpose. Later on that is the
        difference between "this list is definitely that pair of topics" and
        "this list is whatever ran last", and the second is not something to
        certify a robot on.
        """
        for node in ast.walk(self.tree):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name):
                        self.assigned.setdefault(target.id, []).append(node.value)
            elif isinstance(node, ast.AnnAssign):
                if isinstance(node.target, ast.Name) and node.value is not None:
                    self.assigned.setdefault(node.target.id, []).append(node.value)
            elif isinstance(node, ast.AugAssign):
                # `sources += [...]` changes the list without reassigning it.
                if isinstance(node.target, ast.Name):
                    self.mutated.add(node.target.id)
            elif isinstance(node, ast.Call):
                # `sources.append(...)` likewise.
                function = node.func
                if isinstance(function, ast.Attribute) and isinstance(
                    function.value, ast.Name
                ):
                    if function.attr in {
                        "append", "extend", "insert", "remove", "pop", "clear"
                    }:
                        self.mutated.add(function.value.id)

    def settled(self, value: ast.expr) -> ast.expr:
        """What an expression really is, following names that are set once.

        A name that is set more than once, changed in place, or that comes
        from somewhere this file cannot see is refused rather than guessed
        at. That refusal is what makes the remapping checks below stronger
        than a text search: renaming the variable is fine, reassigning it is
        not, and a reviewer reading the file gets the same answer.
        """
        seen: set[str] = set()
        while isinstance(value, ast.Name):
            if value.id in seen:
                raise Unreadable(f"{self}: {value.id} is defined in terms of itself")
            seen.add(value.id)
            if value.id in self.mutated:
                raise Unreadable(
                    f"{self}: {value.id} is changed in place, so what it holds "
                    "cannot be read off this file"
                )
            settings = self.assigned.get(value.id, [])
            if len(settings) != 1:
                raise Unreadable(
                    f"{self}: {value.id} is set {len(settings)} times, so what "
                    "it holds cannot be read off this file"
                )
            value = settings[0]
        return value

    def reachable_strings(self, value: ast.expr) -> set[str]:
        """Every string this expression could end up containing.

        Deliberately generous: it follows names into lists built from other
        lists, so a list comprehension over another list still gives up the
        names inside it. Over-counting is the safe direction for a check
        that something must NOT be in a list.
        """
        found: set[str] = set()
        pending = [value]
        seen: set[str] = set()
        while pending:
            current = pending.pop()
            for node in ast.walk(current):
                if isinstance(node, ast.Constant) and isinstance(node.value, str):
                    found.add(node.value)
                elif isinstance(node, ast.Name) and node.id not in seen:
                    seen.add(node.id)
                    pending.extend(self.assigned.get(node.id, []))
        return found

    def named_list(self, name: str) -> set[str]:
        """The strings held by a list or tuple the file defines at the top."""
        settings = self.assigned.get(name, [])
        if len(settings) != 1:
            raise Unreadable(f"{self}: {name} is set {len(settings)} times")
        return self.reachable_strings(settings[0])

    def node_actions(self) -> list[ast.Call]:
        """Every launch_ros action in this file that starts a process."""
        return [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and getattr(node.func, "id", None) in NODE_ACTIONS
        ]

    def included_launch_files(self) -> set[str]:
        """The launch files this one pulls in with IncludeLaunchDescription.

        Read from the include ACTION rather than from the text of the file,
        because the path is usually held in a variable assigned further up,
        and because the file name and the word IncludeLaunchDescription both
        go on appearing in an import line and a leftover variable after the
        include itself has been deleted. A text search passes on that; this
        does not.
        """
        found: set[str] = set()
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue
            if getattr(node.func, "id", None) != "IncludeLaunchDescription":
                continue
            for text in self.reachable_strings(node):
                if text.endswith(".launch.py"):
                    found.add(Path(text).name)
        return found

    def started_node_names(self) -> set[str]:
        """The `name=` of every node this file starts."""
        return {
            name
            for call in self.node_actions()
            if (name := self.text_argument(call, "name")) is not None
        }

    def keyword(self, call: ast.Call, argument: str) -> ast.expr | None:
        for item in call.keywords:
            if item.arg == argument:
                return item.value
        return None

    def text_argument(self, call: ast.Call, argument: str) -> str | None:
        """A plain-text keyword such as `package=` or `executable=`."""
        value = self.keyword(call, argument)
        if value is None:
            return None
        settled = self.settled(value)
        if isinstance(settled, ast.Constant) and isinstance(settled.value, str):
            return settled.value
        return None

    def describe(self, call: ast.Call) -> str:
        """How to refer to one node action in a failure message."""
        package = self.text_argument(call, "package") or "?"
        executable = self.text_argument(call, "executable") or "?"
        return f"{self} line {call.lineno}: {package}/{executable}"

    def remappings(self, call: ast.Call) -> list[tuple[str, str]]:
        """The topic renames a node action is started with, as real names.

        This is the part the existing tests do by searching the text. Here
        the list is followed back to the pairs of strings it holds, so it
        does not matter what the variable is called, and it is not enough
        for the right-looking line to appear somewhere in the file.
        """
        value = self.keyword(call, "remappings")
        if value is None:
            return []
        settled = self.settled(value)
        if not isinstance(settled, ast.List | ast.Tuple):
            raise Unreadable(
                f"{self.describe(call)}: its remappings are not a plain list, "
                "so what this node is wired to cannot be read off the file"
            )
        pairs = []
        for element in settled.elts:
            element = self.settled(element)
            if not isinstance(element, ast.List | ast.Tuple) or len(element.elts) != 2:
                raise Unreadable(
                    f"{self.describe(call)}: a remapping is not a pair of "
                    "topic names"
                )
            names = [self.settled(part) for part in element.elts]
            if not all(
                isinstance(part, ast.Constant) and isinstance(part.value, str)
                for part in names
            ):
                raise Unreadable(
                    f"{self.describe(call)}: a remapping is built out of "
                    "something other than plain topic names"
                )
            pairs.append((topic(names[0].value), topic(names[1].value)))
        return pairs

    def is_the_gate(self, call: ast.Call) -> bool:
        return (
            self.text_argument(call, "package") == GATE_PACKAGE
            and self.text_argument(call, "executable") == GATE_EXECUTABLE
        )

    def is_a_velocity_source(self, call: ast.Call) -> bool:
        """Whether this node action starts a program that steers the robot."""
        return bool(
            {
                self.text_argument(call, "executable"),
                self.text_argument(call, "name"),
            }
            & VELOCITY_SOURCES
        )

    def parameter_configured_output(self, call: ast.Call) -> str | None:
        """Where this node publishes, when a parameter file decides that.

        None for the ordinary case, where a remapping decides it and the
        launch file can be read directly.
        """
        name = self.text_argument(call, "name") or self.text_argument(
            call, "executable"
        )
        if name not in PARAMETER_CONFIGURED_OUTPUT:
            return None
        return parameter_configured_topic(name, PARAMETER_CONFIGURED_OUTPUT[name])

    def is_a_lifecycle_manager(self, call: ast.Call) -> bool:
        package = self.text_argument(call, "package") or ""
        executable = self.text_argument(call, "executable") or ""
        return package == "nav2_lifecycle_manager" or "lifecycle_manager" in executable

    def managed_node_names(self, call: ast.Call) -> set[str]:
        """The nodes a lifecycle manager is told to bring up and take down."""
        parameters = self.keyword(call, "parameters")
        if parameters is None:
            raise Unreadable(
                f"{self.describe(call)}: a lifecycle manager with no parameters, "
                "so the list of nodes it controls cannot be checked"
            )
        names: set[str] = set()
        found = False
        for node in ast.walk(self.settled(parameters)):
            if not isinstance(node, ast.Dict):
                continue
            for key, value in zip(node.keys, node.values, strict=True):
                if isinstance(key, ast.Constant) and key.value == "node_names":
                    found = True
                    names |= self.reachable_strings(value)
        if not found or not names:
            raise Unreadable(
                f"{self.describe(call)}: the list of nodes this lifecycle "
                "manager controls is not written here, so nothing can confirm "
                "the safety gate is absent from it"
            )
        return names


def launch_files() -> list[LaunchFile]:
    """Every launch file in the workspace, found rather than listed.

    This is the whole point of the file. A launch file added in a new package
    next month is covered by everything below without anyone remembering to
    add it, which is not true of the checks that name their files.
    """
    return [LaunchFile(path) for path in sorted(SRC.rglob("*.launch.py"))]


class LaunchFileDiscoveryTests(unittest.TestCase):
    """The search has to actually find things, or every check below is empty.

    A test that scans a list and finds no problems passes just as loudly when
    the list is empty. These are the floor that stops that happening quietly.
    """

    def setUp(self) -> None:
        self.launches = launch_files()

    def test_launch_files_are_found_in_every_package_that_has_them(self) -> None:
        packages = {launch.package for launch in self.launches}

        # Motion sources are not confined to robot_bringup: the console and
        # the recorder start nodes too, and both sit next to a running robot.
        self.assertLessEqual(
            {
                "robot_bringup",
                "robot_console",
                "robot_description",
                "robot_telemetry",
            },
            packages,
        )

    def test_enough_launch_files_and_nodes_are_found_to_be_worth_checking(
        self,
    ) -> None:
        self.assertGreaterEqual(len(self.launches), 9)

        started = sum(len(launch.node_actions()) for launch in self.launches)
        self.assertGreaterEqual(
            started,
            20,
            "far fewer nodes were found than this workspace starts: the search "
            "is broken and the safety checks below are passing on an empty list",
        )

    def test_every_launch_file_can_be_read(self) -> None:
        # Unreadable here means nothing below looked at it.
        for launch in self.launches:
            with self.subTest(launch=str(launch)):
                compile(launch.source, str(launch.path), "exec")


class SinglePublisherOnTheWheelsTests(unittest.TestCase):
    """Only the safety gate may put a velocity on /cmd_vel. Anywhere.

    /cmd_vel is what the motor driver and the Gazebo DiffDrive plugin listen
    to. A second publisher on it is not a second opinion; it is a way for
    something to keep commanding motion while the gate is trying to stop.
    A live run once found exactly that, with two gates on the topic at once.

    SingleSafetyGateTests in test_bringup.py counts the gates in the two
    launch files it names. This asks the wider question - does anything at
    all, in any package, reach that topic - and answers it for files that
    test did not know existed.
    """

    def setUp(self) -> None:
        self.launches = launch_files()

    def offenders(self) -> list[str]:
        found = []
        for launch in self.launches:
            for call in launch.node_actions():
                pairs = launch.remappings(call)
                renamed_from = {source for source, _ in pairs}
                renamed_onto = {target for _, target in pairs}

                # Three ways to end up on the wheels: be a program that
                # publishes there and not be moved off it, be moved onto it
                # from somewhere else, or be told to publish there by a
                # parameter.
                configured = launch.parameter_configured_output(call)
                if configured is not None:
                    drives = configured == WHEELS
                else:
                    drives = WHEELS in renamed_onto or (
                        launch.is_a_velocity_source(call)
                        and WHEELS not in renamed_from
                    )
                if drives and not launch.is_the_gate(call):
                    found.append(launch.describe(call))
        return found

    def test_nothing_but_the_safety_gate_publishes_to_the_wheels(self) -> None:
        self.assertEqual(
            self.offenders(),
            [],
            "these nodes reach /cmd_vel without going through robot_safety",
        )

    def test_the_safety_gate_is_started_somewhere(self) -> None:
        # Without this the test above would pass on a workspace where nothing
        # publishes to the wheels because nothing gates anything.
        gates = [
            launch.describe(call)
            for launch in self.launches
            for call in launch.node_actions()
            if launch.is_the_gate(call)
        ]

        self.assertTrue(gates, "no launch file starts the safety gate at all")

    def test_the_safety_gate_is_never_remapped_off_the_wheels(self) -> None:
        # A remap on the gate itself would leave the request topic gated and
        # the wheels ungated: everything would look correct and nothing would
        # be driving through the gate.
        for launch in self.launches:
            for call in launch.node_actions():
                if not launch.is_the_gate(call):
                    continue
                with self.subTest(gate=launch.describe(call)):
                    renamed_from = {
                        source for source, _ in launch.remappings(call)
                    }

                    self.assertNotIn(WHEELS, renamed_from)

    def test_no_launch_file_reaches_the_wheels_through_a_command_line(self) -> None:
        # `--ros-args -r cmd_vel:=...` on the command line is a remap that
        # never appears in `remappings=`, so the checks above cannot see it.
        # The same goes for a `ros2 topic pub` run as a process.
        for launch in self.launches:
            for node in ast.walk(launch.tree):
                if not isinstance(node, ast.Call):
                    continue
                for item in node.keywords:
                    if item.arg not in {"arguments", "cmd"}:
                        continue
                    written = [
                        part.value
                        for part in ast.walk(item.value)
                        if isinstance(part, ast.Constant)
                        and isinstance(part.value, str)
                    ]
                    with self.subTest(launch=str(launch), line=node.lineno):
                        for text in written:
                            self.assertNotIn(WHEELS, text)
                        self.assertNotIn("pub", written)


class MotionSourcesAskRatherThanCommandTests(unittest.TestCase):
    """Nav2 is a motion source, not a motion authority.

    Every program that steers has to have cmd_vel renamed to
    cmd_vel_requested, which turns its output into a request the gate decides
    on. behavior_server is the one that matters most: a live run found three
    separate velocity publishers under it, one per recovery behaviour, and
    recoveries run precisely when something has already gone wrong.

    This deliberately overlaps NavigationTopologyTests in test_bringup.py,
    which checks the same thing two weaker ways. Its
    `test_every_velocity_emitting_node_is_remapped_to_the_request_topic`
    searches each node's text for "remappings=gated", so renaming the
    variable breaks it while the robot still behaves; and its
    `test_the_remap_points_away_from_the_wheels` looks for the exact source
    line `gated = [("/cmd_vel", "/cmd_vel_raw")]` anywhere in the file, so a
    second assignment further down, or the list being built up in pieces,
    changes where the robot drives and still passes. The checks below follow
    the list to the topic names it actually holds, and refuse to pass when it
    is assigned twice or changed in place.

    Since the Collision Monitor was wired in, "behind the gate" is two hops
    rather than one: Nav2 publishes cmd_vel_raw, the monitor forwards to
    cmd_vel_requested, and the gate rules on that. These checks follow the
    chain with topics_that_reach_the_gate() instead of naming the one topic
    that used to be the whole answer.
    """

    def setUp(self) -> None:
        self.launches = launch_files()
        self.navigation = next(
            launch for launch in self.launches if launch.name == "navigation.launch.py"
        )

    def velocity_sources(self) -> list[tuple[LaunchFile, ast.Call]]:
        return [
            (launch, call)
            for launch in self.launches
            for call in launch.node_actions()
            if launch.is_a_velocity_source(call) and not launch.is_the_gate(call)
        ]

    def test_every_program_that_steers_is_wired_to_the_request_topic(self) -> None:
        sources = self.velocity_sources()
        reaching = topics_that_reach_the_gate()

        self.assertTrue(sources, "no steering node was found in any launch file")
        for launch, call in sources:
            with self.subTest(node=launch.describe(call)):
                configured = launch.parameter_configured_output(call)
                if configured is not None:
                    # This one names its output topic in a parameter file
                    # instead of remapping, so that is where the answer is.
                    self.assertIn(
                        configured,
                        reaching,
                        "this node steers the robot and is not behind the gate",
                    )
                    continue

                targets = {
                    destination
                    for source, destination in launch.remappings(call)
                    if source == WHEELS
                }
                self.assertTrue(
                    targets,
                    "this node steers the robot and its cmd_vel is not "
                    "remapped anywhere, so it publishes straight to the wheels",
                )
                for destination in targets:
                    self.assertIn(
                        destination,
                        reaching,
                        "this node steers the robot and is not behind the gate",
                    )

    def test_the_recovery_behaviours_are_behind_the_gate_too(self) -> None:
        # Named on its own because it is the easiest one to forget: it is not
        # what drives during a normal goal, so a mistake here shows up only
        # during a recovery, which is the worst possible time to find it.
        gated = [
            launch.remappings(call)
            for launch, call in self.velocity_sources()
            if launch.text_argument(call, "executable") == "behavior_server"
        ]
        reaching = topics_that_reach_the_gate()

        self.assertTrue(gated, "behavior_server is not started by any launch file")
        for pairs in gated:
            targets = {
                destination for source, destination in pairs if source == WHEELS
            }
            self.assertTrue(targets, "behavior_server's cmd_vel is not remapped")
            for destination in targets:
                self.assertIn(destination, reaching)

    def test_a_steering_node_that_needs_a_forwarder_is_launched_beside_one(
        self,
    ) -> None:
        # topics_that_reach_the_gate() proves a forwarding node is CONFIGURED
        # to carry a velocity on to the gate. It cannot prove anybody starts
        # it. Remap Nav2 onto the monitor's input and then not launch the
        # monitor, and every check above still passes while the robot sits
        # still: nothing arrives on cmd_vel_requested, the gate's
        # command_timeout fires, and the cause is invisible from the tests.
        #
        # Fails safe rather than dangerous, which is why it is a separate test
        # and not a louder version of the ones above - but a robot that cannot
        # move and cannot say why is its own kind of problem.
        forwarders = {
            parameter_configured_topic(node_name, PARAMETER_CONFIGURED_INPUT[node_name]): node_name
            for node_name in PARAMETER_CONFIGURED_OUTPUT
        }

        for launch, call in self.velocity_sources():
            if launch.parameter_configured_output(call) is not None:
                continue
            for source, destination in launch.remappings(call):
                if source != WHEELS or destination not in forwarders:
                    continue
                needed = forwarders[destination]
                with self.subTest(node=launch.describe(call), forwarder=needed):
                    reachable = self.launch_files_reachable_from(launch)
                    started = any(
                        other.text_argument(other_call, "executable") == needed
                        or other.text_argument(other_call, "name") == needed
                        for other in reachable
                        for other_call in other.node_actions()
                    )
                    self.assertTrue(
                        started,
                        f"this node publishes to {destination}, which only "
                        f"reaches the gate by way of {needed}, and no launch "
                        f"file reachable from {launch.name} starts {needed}. "
                        "Nothing would forward its velocity, so the robot "
                        "would sit still with no indication why.",
                    )

    def launch_files_reachable_from(self, start: LaunchFile) -> list[LaunchFile]:
        """`start` and everything it includes, directly or through another."""
        by_name = {launch.name: launch for launch in self.launches}
        seen = {start.name}
        pending = [start]
        reachable = [start]
        while pending:
            current = pending.pop()
            for name in current.included_launch_files():
                if name in seen or name not in by_name:
                    continue
                seen.add(name)
                reachable.append(by_name[name])
                pending.append(by_name[name])
        return reachable

    def test_navigation_declares_no_velocity_source_this_file_has_not_heard_of(
        self,
    ) -> None:
        declared = self.navigation.named_list("VELOCITY_SOURCES")

        # navigation.launch.py keeps its own list of the Nav2 nodes that can
        # drive. If a new one is added there, it has to be added to
        # VELOCITY_SOURCES at the top of this file too, or the checks above
        # would quietly stop looking at it.
        self.assertTrue(declared)
        self.assertLessEqual(
            declared,
            set(VELOCITY_SOURCES),
            "navigation.launch.py names a steering node this test file does "
            "not know about: add it to VELOCITY_SOURCES here",
        )

    def test_every_velocity_source_navigation_declares_is_really_started(
        self,
    ) -> None:
        declared = self.navigation.named_list("VELOCITY_SOURCES")
        started = {
            self.navigation.text_argument(call, "name")
            for call in self.navigation.node_actions()
        }

        # A node renamed in one place and not the other leaves the list
        # describing a node that no longer exists, and the real one unchecked.
        self.assertLessEqual(declared, started)


class TheGateOutlivesEverythingTests(unittest.TestCase):
    """The safety gate is never handed to a lifecycle manager.

    A lifecycle manager deactivates the nodes it controls when one of them
    fails or when it shuts down. The gate is the thing that has to still be
    running at that moment: a deactivated gate is a robot with nothing
    between a stale velocity and its wheels.

    test_bringup.py checks this for the two lists navigation.launch.py
    happens to declare today, by name. This asks it of every lifecycle
    manager in the workspace, and follows lists built out of other lists, so
    a manager added in another package is covered as well.
    """

    def setUp(self) -> None:
        self.launches = launch_files()

    def managers(self) -> list[tuple[LaunchFile, ast.Call]]:
        return [
            (launch, call)
            for launch in self.launches
            for call in launch.node_actions()
            if launch.is_a_lifecycle_manager(call)
        ]

    def test_no_lifecycle_manager_anywhere_controls_the_safety_gate(self) -> None:
        managers = self.managers()

        self.assertTrue(managers, "no lifecycle manager was found to check")
        for launch, call in managers:
            with self.subTest(manager=launch.describe(call)):
                managed = launch.managed_node_names(call)

                self.assertEqual(
                    managed & set(GATE_NAMES),
                    set(),
                    "this lifecycle manager can deactivate the safety gate, "
                    "and it deactivates its nodes when something fails",
                )

    def test_the_lists_of_managed_nodes_can_be_read_at_all(self) -> None:
        # The check above can only find the gate in a list it can read, so an
        # unreadable list must not count as a pass. The proof that a list was
        # really read is that it names nodes this same launch file starts: a
        # manager controls the nodes beside it, so an answer with no overlap
        # means the wiring was not followed, not that the gate is safe.
        for launch, call in self.managers():
            with self.subTest(manager=launch.describe(call)):
                managed = launch.managed_node_names(call)

                self.assertTrue(
                    managed & launch.started_node_names(),
                    "none of the nodes this lifecycle manager controls are "
                    "started in the same file, so the list could not be read "
                    "and the check above proved nothing",
                )

    def test_the_gate_is_not_a_lifecycle_node_itself(self) -> None:
        # Started as a LifecycleNode it would come up in `unconfigured` and
        # gate nothing, which looks identical to a working robot until
        # something needs stopping.
        for launch in self.launches:
            for call in launch.node_actions():
                if not launch.is_the_gate(call):
                    continue
                with self.subTest(gate=launch.describe(call)):
                    self.assertEqual(getattr(call.func, "id", None), "Node")


class WitnessesAndVoicesCannotUndoAStopTests(unittest.TestCase):
    """The recorder and the voice can never release the emergency stop.

    The software stop is latched: once it is engaged, only an operator
    sending `emergency_stop_reset` releases it. A recorder that could send
    that topic would no longer be a witness, and a voice that could send it
    could talk its own stop away.

    Both packages are partly covered already, and this is deliberately wider
    than either. test_record_launch.py looks at `robot_telemetry/*.py` only -
    not the launch directory, not setup.py, not any sub-directory added
    later. test_voice_launch.py checks `voice_node.py` alone, so a second
    module in robot_voice - say.py already publishes a topic - could release
    the latch and that test would still pass. These walk both packages
    whole.
    """

    def test_the_recorder_contains_no_way_to_act_on_the_robot(self) -> None:
        for path in package_sources("robot_telemetry"):
            with self.subTest(module=str(path.relative_to(SRC))):
                acting = called_names(path.read_text()) & PUBLISHING_CALLS

                self.assertEqual(
                    acting,
                    set(),
                    f"{path.name} can act on the robot: {sorted(acting)}. "
                    "robot_telemetry records and does nothing else.",
                )

    def test_the_voice_never_names_the_topic_that_releases_the_latch(self) -> None:
        # The topic name has to appear somewhere in the package for a node in
        # it to publish the topic, so its absence from every string in every
        # module is proof no module can. Docstrings are excluded because this
        # package explains the rule at length in prose.
        for path in package_sources("robot_voice"):
            with self.subTest(module=str(path.relative_to(SRC))):
                for text in code_strings(path.read_text()):
                    self.assertNotIn("emergency_stop_reset", text)

    def test_the_voice_never_names_the_wheels_either(self) -> None:
        # An approved destination becomes a Nav2 goal. Nav2's own output is
        # already behind the gate, so the voice has no reason to name a
        # velocity topic at all - not even the request one.
        for path in package_sources("robot_voice"):
            with self.subTest(module=str(path.relative_to(SRC))):
                for text in code_strings(path.read_text()):
                    self.assertNotIn(WHEELS, text)


def source_assignments(tree: ast.AST) -> dict[str, ast.AST]:
    """Every name assigned in a module, so a topic held in one can be read."""
    return {
        target.id: node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }


def constant_strings(node: ast.AST, known: dict[str, ast.AST]) -> set[str]:
    """Every string an expression could evaluate to.

    Over-approximates on purpose. The caller asks "is this topic absent?", so
    a superset can only cause a false failure, never a false pass. A `+` is
    exact; a comprehension widens to everything it draws from, because a
    filtered subset cannot contain what its source did not. Anything else is
    Unreadable, which fails - "I could not read it" is not evidence of
    absence.
    """
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return {node.value}
    if isinstance(node, ast.Name):
        if node.id not in known:
            raise Unreadable(f"name {node.id!r} is not assigned in this file")
        return constant_strings(known[node.id], known)
    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
        return set().union(
            set(), *(constant_strings(item, known) for item in node.elts)
        )
    if isinstance(node, ast.Dict):
        return set().union(
            set(), *(constant_strings(item, known) for item in node.values)
        )
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return constant_strings(node.left, known) | constant_strings(
            node.right, known
        )
    if isinstance(node, (ast.ListComp, ast.SetComp, ast.GeneratorExp)):
        return set().union(
            set(),
            *(
                constant_strings(generator.iter, known)
                for generator in node.generators
            ),
        )
    raise Unreadable(f"cannot read a {type(node).__name__} as strings")


def published_topics(source: str) -> tuple[set[str], list[str]]:
    """Topics passed to create_publisher, and the ones that could not be read.

    The second half matters as much as the first: a topic name assembled at
    runtime is a publisher this reader cannot vouch for, and the caller is
    expected to fail rather than ignore it.
    """
    tree = ast.parse(source)
    known = source_assignments(tree)
    topics: set[str] = set()
    unreadable: list[str] = []
    for call in ast.walk(tree):
        if (
            isinstance(call, ast.Call)
            and getattr(call.func, "attr", None) == "create_publisher"
            and len(call.args) >= 2
        ):
            try:
                topics.update(constant_strings(call.args[1], known))
            except Unreadable as reason:
                unreadable.append(f"line {call.lineno}: {reason}")
    return topics, unreadable


class OnlyTheGateCanReachTheWheelsTests(unittest.TestCase):
    """No package but robot_safety may even open a /cmd_vel publisher.

    Everything above this asks what the launch files *start*. This asks what
    is capable of reaching the wheels at all. A node that opens a cmd_vel
    publisher is a second authority whether or not a launch file starts it
    today - and a launch file that starts it can be written afterwards, by
    someone who never reads this directory.

    Kept from the parallel implementation on the `track-b` branch, which
    caught exactly this and which the launch-file checks above do not: a
    create_publisher added to an ordinary module is invisible to every remap
    and every parameter file.
    """

    def test_no_package_but_the_gate_creates_a_wheel_publisher(self) -> None:
        offenders = []
        scanned = 0
        for package_root in sorted(SRC.iterdir()):
            module_root = package_root / package_root.name
            if not module_root.is_dir():
                continue
            for path in sorted(module_root.rglob("*.py")):
                scanned += 1
                topics, unreadable = published_topics(path.read_text())
                relative = path.relative_to(SRC)

                self.assertEqual(
                    unreadable,
                    [],
                    f"{relative}: a publisher's topic cannot be read, so "
                    "nothing here can prove it is not the wheel topic",
                )
                if package_root.name == GATE_PACKAGE:
                    continue
                if {topic(name) for name in topics} & {WHEELS}:
                    offenders.append(str(relative))

        self.assertEqual(
            offenders,
            [],
            "only robot_safety may publish the topic that reaches the wheels",
        )
        self.assertGreater(scanned, 20, f"only scanned {scanned} modules")

    def test_the_gate_still_publishes_the_wheel_topic(self) -> None:
        # Without this the test above would pass on a workspace where the
        # gate had been moved, renamed, or deleted: it would simply find no
        # offenders because it was looking in the wrong place.
        gate = SRC / GATE_PACKAGE / GATE_PACKAGE / "safety_node.py"
        topics, _ = published_topics(gate.read_text())

        self.assertIn(
            WHEELS,
            {topic(name) for name in topics},
            "robot_safety/safety_node.py no longer publishes the wheel topic: "
            "either the gate moved or this test is looking in the wrong place",
        )


if __name__ == "__main__":
    unittest.main()
