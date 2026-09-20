"""The only part of the analyser that knows rosbag2 exists.

Deliberately thin. Everything it does is open a recording, deserialise each
message once, and hand `records.convert` the result; every judgement about the
run happens afterwards, in functions that have never heard of a bag. Keeping
the seam here is what lets the safety checks be tested without ROS installed,
and it is the reason a change to the storage format cannot quietly change what
counts as a violation.
"""

from collections.abc import Sequence
import json
from pathlib import Path

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from .records import Record, canonical_topic, convert

# Written into the bag directory by graph_probe while the run is recorded. A
# bag stores messages, not publishers, so without this file the question
# "did anything but the gate drive the wheels" has no answer.
PUBLISHERS_FILE = "publishers.json"

NANOSECONDS = 1e-9


def read_records(bag_dir: str | Path) -> list[Record]:
    """Every message in a recording, as plain records in time order."""
    path = Path(bag_dir)
    if not path.exists():
        raise FileNotFoundError(f"no recording at {path}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path)),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }

    records: list[Record] = []
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        type_name = types.get(topic)
        if type_name is None:
            continue
        message = deserialize_message(payload, get_message(type_name))
        records.append(
            Record(timestamp * NANOSECONDS, canonical_topic(topic), convert(message))
        )
    records.sort(key=lambda record: record.timestamp)
    return records


def read_publishers(bag_dir: str | Path) -> dict[str, Sequence[str]] | None:
    """Which nodes were publishing each topic, if the recorder wrote it down.

    Returns None rather than an empty mapping when the file is missing or
    unreadable, so the report can distinguish "nobody was publishing" from
    "nobody recorded who was publishing". They mean very different things.
    """
    path = Path(bag_dir) / PUBLISHERS_FILE
    try:
        document = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    topics = document.get("topics")
    if not isinstance(topics, dict):
        return None
    return {
        canonical_topic(str(topic)): [str(name) for name in names]
        for topic, names in topics.items()
        if isinstance(names, list)
    }
