"""Register DeltaFlow source directories in Chipyard's generator project."""

from pathlib import Path
import os

MARKER = "  // DeltaFlow integration: tracked sources, no fork of FSA."
SOURCE_SETTINGS = "\n" + MARKER + """
  .settings(Compile / unmanagedSourceDirectories ++=
    sys.env.get("DELTAFLOW_REPO_ROOT").toSeq.flatMap(root => Seq(
      file(root) / "rtl/src/main/scala",
      file(root) / "rtl/chipyard/src/main/scala")))
"""


def register_sources(chipyard_dir):
    path = Path(chipyard_dir) / "build.sbt"
    text = path.read_text()
    start = text.index("lazy val chipyard = ")
    end = text.find("\nlazy val ", start + 1)
    if end == -1:
        end = len(text)
    project = text[start:end]
    if SOURCE_SETTINGS in project:
        # Preserve any later settings belonging to other integrations.
        return path
    if MARKER in project:
        raise ValueError("existing DeltaFlow source settings differ; inspect Chipyard build.sbt")
    path.write_text(text[:start] + project.rstrip() + SOURCE_SETTINGS + text[end:])
    return path


if __name__ == "__main__":
    print(
        "DeltaFlow source directories configured in", register_sources(os.environ["CHIPYARD_DIR"])
    )
