"""Keep hardware and assertions; shorten diagnostic source locations."""

from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text()
marker = '// ----- 8< ----- FILE "firrtl_black_box_resource_files.f" ----- 8< -----'
text = text.split(marker, 1)[0]
text = re.sub(r"connected at [^)]*DirectAXI4.scala:\d+:\d+", "connected at DirectAXI4.scala", text)
text = re.sub(r"at DirectAXI4.scala:\d+", "at DirectAXI4.scala", text)
path.write_text(text)
