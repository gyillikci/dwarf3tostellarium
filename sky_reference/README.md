# Sky reference images

Reference imagery for visually confirming the Dwarf 3 is actually pointed at
the Sun or Moon during IMU calibration (`wit_imu.py --calibrate-sun` /
`--calibrate-moon`), captured for the session starting 2026-07-23 ~21:00 UTC.

- `moon_2026-07-23_2100Z.jpg` — exact Moon phase/libration for this hour,
  rendered from real LRO topography/imagery by NASA SVS's
  ["Dial-A-Moon"](https://svs.gsfc.nasa.gov/5587/) (2026 hourly series,
  frame 4894 = hours since 2026-01-01T00:00Z + 1).
- `sun_hmi_flattened_2026-07-23_2100Z.jpg` (4096px) / `_1k` (1024px) —
  near-real-time visible-light photosphere from
  [NASA SDO](https://sdo.gsfc.nasa.gov/data/), HMI continuum with limb
  darkening flattened out (`latest_SIZE_HMIIF.jpg`) so sunspots stay visible
  all the way to the limb. This is the product that actually matches what an
  optical/white-light telescope sees — not an AIA EUV channel (those show the
  corona, not sunspots).
- `sun_hmi_magnetogram_2026-07-23_2045Z.jpg` — HMI magnetogram
  (`latest_SIZE_HMIB.jpg`) for the same moment, kept as corroboration: the
  bipolar magnetic structure lines up with the visible spots, confirming
  they're real active regions and not an imaging artifact.

## Validity window

Good for about 6 hours from the timestamp: the Moon's phase/libration barely
shifts in that time (~1% of a 29.5-day cycle), and the Sun's ~13°/day rotation
only moves sunspot groups ~3° in 6h — same spots, same rough position. Re-fetch
for a materially later session (new Dial-A-Moon frame number; re-hit SDO's
`latest_*` endpoints, which refresh every ~15 min).

## Regenerating

```
# Sun (always "latest"):
https://sdo.gsfc.nasa.gov/assets/img/latest/latest_4096_HMIIF.jpg
https://sdo.gsfc.nasa.gov/assets/img/latest/latest_1024_HMIB.jpg

# Moon (hour-specific frame number = hours since 2026-01-01T00:00Z + 1):
https://svs.gsfc.nasa.gov/vis/a000000/a005500/a005587/frames/730x730_1x1_30p/moon.<NNNN>.jpg
```

NASA imagery is public domain.
