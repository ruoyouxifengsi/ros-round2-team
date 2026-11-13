from pathlib import Path
import shlex
import xml.etree.ElementTree as ET
here=Path(__file__).resolve().parent
tree=ET.parse(here.parents[1]/'launch/control.launch')
tree.find('.//node[@type="control.py"]').set('launch-prefix','python3 '+shlex.quote(str(here/'instrument.py')))
tree.write(here/'instrumented.launch')
