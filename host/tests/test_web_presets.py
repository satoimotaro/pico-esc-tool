# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""The web configurator's built-in presets (web/js/presets.js) must be the firmware's
(src/apps/presets.h): the same name, family and every (offset, value) byte. Two copies of one
table drift; this is what keeps `preset apply` and the web page writing the same thing."""
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[2]
FAMILY = {"BLUEJAY": "bluejay", "BLUEGILL": "bluegill", "AM32": "am32"}


def firmware_presets():
    s = (ROOT / "src/apps/presets.h").read_text()
    arrays = {m.group(1): [(int(a, 0), int(b, 0)) for a, b in
                           re.findall(r"\{(0x[0-9A-Fa-f]+|\d+),\s*(0x[0-9A-Fa-f]+|\d+)\}", m.group(2))]
              for m in re.finditer(r"static const Byte (\w+)\[\] = \{(.*?)\};", s, re.S)}
    return {name: (FAMILY[fam], arrays[arr])
            for name, fam, arr in re.findall(r'\{\s*"([^"]+)",\s*(\w+),\s*"[^"]*",\s*(\w+),', s)}


def web_presets():
    s = (ROOT / "web/js/presets.js").read_text()
    out = {}
    for name, fam, data in re.findall(r"name: '([^']+)', family: '(\w+)',[^}]*?bytes: '([^']+)'", s, re.S):
        out[name] = (fam, [tuple(int(x, 16) for x in p.split(":")) for p in data.split()])
    return out


def test_same_presets():
    fw, web = firmware_presets(), web_presets()
    assert fw, "no presets parsed from presets.h"
    assert web == fw
