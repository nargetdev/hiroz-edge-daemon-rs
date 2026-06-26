# Summary: Capturing a RAW .cr2 from the camera

## Goal
Use `gphoto2` to capture a RAW `.cr2` file (~14MB) from the camera.

## Camera
- **Canon EOS 6D** detected on `usb:001,005` via `gphoto2 --auto-detect`.

## Image format options
The 6D `imageformat` config exposes these RAW-only choices (among many JPEG/combo modes):
- `32` = RAW (full)
- `33` = mRAW
- `34` = sRAW

## Captures taken
| File | Format (`imageformat`) | Size |
|------|------------------------|------|
| `capture_20260626_124738.cr2` | full RAW (`32`) | 27M |
| `capture_mraw_20260626_124756.cr2` | mRAW (`33`) | 21M |
| `capture_sraw_20260626_124805.cr2` | sRAW (`34`) | **14M** ✅ |

## Key finding
On the Canon EOS 6D, full "RAW" is ~27MB — not 14MB. The **~14MB target matches sRAW** (small RAW).

## Command used (sRAW, ~14MB)
```bash
gphoto2 --set-config imageformat=34 \
        --capture-image-and-download \
        --filename=capture_sraw_$(date +%Y%m%d_%H%M%S).cr2
```

## Notes
- A harmless `setlocale: LC_ALL: cannot change locale (en_US.UTF-8)` warning appears on each call.
- Each capture creates `/capt0000.cr2` on the camera, downloads it, then deletes it from the camera.

## Config enums (Canon EOS 6D)

Pass the index or the label, e.g. `gphoto2 --set-config iso=400`.

### shutterspeed (seconds; `1/x` are fractions)
```
0:30   1:25   2:20   3:15   4:13   5:10.3  6:8   7:6.3  8:5   9:4
10:3.2 11:2.5 12:2   13:1.6 14:1.3 15:1    16:0.8 17:0.6 18:0.5 19:0.4
20:0.3 21:1/4 22:1/5 23:1/6 24:1/8 25:1/10 26:1/13 27:1/15 28:1/20 29:1/25
30:1/30 31:1/40 32:1/50 33:1/60 34:1/80 35:1/100 36:1/125 37:1/160 38:1/200 39:1/250
40:1/320 41:1/400 42:1/500 43:1/640 44:1/800 45:1/1000 46:1/1250 47:1/1600 48:1/2000 49:1/2500
50:1/3200 51:1/4000
```

### iso
```
0:Auto 1:100 2:125 3:160 4:200 5:250 6:320 7:400 8:500 9:640
10:800 11:1000 12:1250 13:1600 14:2000 15:2500 16:3200 17:4000 18:5000 19:6400
```

### aperture (f-number)
```
0:2.8 1:3.2 2:3.5 3:4 4:4.5 5:5 6:5.6 7:6.3 8:7.1 9:8
10:9 11:10 12:11 13:13 14:14 15:16 16:18 17:20 18:22 19:25 20:29 21:32
```

### imageformat
```
0:Large Fine JPEG   1:Large Normal JPEG  2:Medium Fine JPEG  3:Medium Normal JPEG
4:Small Fine JPEG   5:Small Normal JPEG  6:Smaller JPEG      7:Tiny JPEG
8-15: RAW + JPEG combos    16-23: mRAW + JPEG combos    24-31: sRAW + JPEG combos
32:RAW   33:mRAW   34:sRAW
```
