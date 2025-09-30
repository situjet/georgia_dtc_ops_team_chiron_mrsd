"""Minimal PyTAK publisher sending a single CASEVAC / Casualty Collection Point CoT event in downtown Atlanta.

Reverted to a one-shot message for simpler WinTAK validation. The event now uses
the standard MIL-STD2525 / CoT friendly ground unit medical casualty subtype
`a-f-G-U-M-C` so WinTAK renders the proper CASEVAC medical symbol (instead of a
generic BLUFOR square). Stale horizon = 5 minutes.

Requirements:
  pip install pytak==7.0.2

Environment Variables:
  COT_URL       TAK/CoT endpoint URL (e.g. udp://239.2.3.1:6969 or tcp://takserver:8087)
  COT_CALLSIGN  (optional) Callsign for this marker (default: ATL_CASEVAC)

Example:
  set COT_URL=udp://239.2.3.1:6969   # Windows CMD
  export COT_URL=udp://239.2.3.1:6969 # bash
  python operator/tak/publish_atlanta_point.py

The script publishes 5 GeoPoint CoT events covering a small area of central Atlanta.
"""
from __future__ import annotations

import asyncio
import os
import uuid
import datetime as dt
from xml.etree import ElementTree as ET
import configparser
import pytak  # type: ignore

ATL_LAT = 33.760347  # Centennial Olympic Park vicinity
ATL_LON = -84.394977
ATL_CE = 35.0
ATL_LE = 15.0


def _fmt_time(dt_obj: dt.datetime) -> str:
  # Millisecond precision, Zulu.
  return dt_obj.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def build_cot_event(
  lat: float,
  lon: float,
  ce: float = 35.0,
  le: float = 15.0,
  callsign: str = "ATL_CASEVAC",
  uid: str | None = None,
  # Use the battlefield marker for a Casualty Collection Point.
  # This is a common way to get the correct icon in WinTAK.
  event_type: str = "b-r-.-h-c",
  remarks: str = "CASEVAC / Casualty Collection Point",
) -> bytes:
  """Create a Cursor-on-Target (CoT) XML string for a point using xml.etree.

  Ensures stable attribute ordering & millisecond timestamps. Adds <detail>
  with <contact> & <remarks>. Optional extra tags kept minimal for reliability.
  """
  uid = uid or f"ATL-{uuid.uuid4()}"
  now = dt.datetime.now(dt.UTC)
  start_time = _fmt_time(now)
  stale_time = _fmt_time(now + dt.timedelta(minutes=5))

  # Ordered attribute insertion by constructing then reordering (ElementTree preserves insertion order in Python 3.8+)
  event_el = ET.Element("event")
  # Common WinTAK ordering pattern: version, uid, type, time, start, stale, how
  event_el.set("version", "2.0")
  event_el.set("uid", uid)
  # Allow runtime override via COT_EVENT_TYPE env var
  effective_type = os.environ.get("COT_EVENT_TYPE", event_type)
  event_el.set("type", effective_type)
  event_el.set("time", start_time)
  event_el.set("start", start_time)
  event_el.set("stale", stale_time)
  event_el.set("how", "m-g")

  ET.SubElement(
    event_el,
    "point",
    {
      "lat": f"{lat:.6f}",
      "lon": f"{lon:.6f}",
      "hae": "300",  # integer-form safe variant
      "ce": f"{ce}",
      "le": f"{le}",
    },
  )

  detail = ET.SubElement(event_el, "detail")
  ET.SubElement(detail, "contact", {"callsign": callsign})

  # Manually specify the icon path to force the CASEVAC symbol.
  # This overrides the default symbol for the event type.
  # The path points to a common location for the CASEVAC icon in WinTAK's icon sets.
  ET.SubElement(detail, "usericon", {"iconsetpath": "COT_MAPPING_2525C/med/casevac.png"})

  # Always include a minimal takv for client identity
  ET.SubElement(detail, "takv", {"device": "PyTAK", "platform": "python", "version": "1.0", "os": os.name})
  if remarks:
    r = ET.SubElement(detail, "remarks")
    r.text = remarks

  # Serialize without xml declaration (PyTAK handles raw CoT event elements)
  xml_bytes = ET.tostring(event_el, encoding="utf-8")
  # Optionally prepend XML declaration if TAK tool expects it (set COT_XML_DECL=1)
  if os.environ.get("COT_XML_DECL") == "1":
    xml_bytes = b"<?xml version='1.0' encoding='UTF-8'?>" + xml_bytes
  # Ensure newline terminator for TCP stream framing
  # Framing: newline or CRLF if requested
  if os.environ.get("COT_USE_CRLF") == "1":
    if not xml_bytes.endswith(b"\r\n"):
      # Strip lone \n if already there
      if xml_bytes.endswith(b"\n"):
        xml_bytes = xml_bytes[:-1]
      xml_bytes += b"\r\n"
  else:
    if not xml_bytes.endswith(b"\n"):
      xml_bytes += b"\n"
  return xml_bytes


async def enqueue_single_casevac(queue: asyncio.Queue):
  """Enqueue a single CASEVAC CoT event (medical casualty collection point)."""
  callsign = os.environ.get("COT_CALLSIGN", "ATL_CASEVAC")
  uid = f"CASEVAC-{uuid.uuid4()}"
  cot_bytes = build_cot_event(
    ATL_LAT,
    ATL_LON,
    ATL_CE,
    ATL_LE,
    callsign=callsign,
    uid=uid,
    # Allow manual override, default to recognized casualty point.
    event_type=os.environ.get("COT_EVENT_TYPE", "b-r-.-h-c"),
    remarks="Single CASEVAC / CCP point",
  )
  if os.environ.get("DEBUG_COT") == "1":
    print(f"[DEBUG_COT] Enqueue SINGLE:\n{cot_bytes.decode('utf-8').rstrip()}\n")
  await queue.put(cot_bytes)


async def main():
  cot_url = os.environ.get("COT_URL")
  if not cot_url:
    raise SystemExit("COT_URL environment variable is required, e.g. udp://239.2.3.1:6969")

  # PyTAK configuration dict. Additional keys can control TLS, auth, etc.
  # Build config object pytak expects (SectionProxy-like) for helpers.
  parser = configparser.ConfigParser()
  parser['pytak'] = {"COT_URL": cot_url}
  config_section = parser['pytak']

  # Create a transmit queue and a single TXWorker without full CLITool setup
  tx_queue: asyncio.Queue = asyncio.Queue()

  # Enqueue single CASEVAC point
  await enqueue_single_casevac(tx_queue)

  # Create network writer using protocol_factory (returns reader, writer for UDP/TCP etc.)
  # Some protocols return only writer; handle tuple accordingly.
  # For UDP Multicast, protocol_factory will create a Datagram transport wrapper.
  proto = await pytak.protocol_factory(config_section)  # type: ignore[attr-defined]
  if isinstance(proto, tuple):
    reader, writer = proto
  else:
    reader, writer = None, proto  # type: ignore

  tx_worker = pytak.TXWorker(tx_queue, config_section, writer)  # type: ignore[attr-defined]
  task = asyncio.create_task(tx_worker.run())
  # Allow brief time for single packet to flush
  await asyncio.sleep(0.5)
  task.cancel()
  try:
    await task
  except asyncio.CancelledError:
    pass


if __name__ == "__main__":
    asyncio.run(main())
