# validation_at_pi4__gphoto2-rs

Small Raspberry Pi validation harness for [gphoto2-rs](https://github.com/maxicarlos08/gphoto2-rs).

This repo is meant to run on `id2-rpi4` with the Canon EOS 6D attached over
local USB.

## Validate

```sh
cargo run -- doctor
```

Expected camera gate:

```text
Model                          Port
----------------------------------------------------------
Canon EOS 6D                   usb:001,004
CAMERA_GREEN model="Canon EOS 6D"
```

Capture one photo:

```sh
cargo run -- capture
```

Or choose an explicit output path:

```sh
cargo run -- capture captures/test.jpg
```

The program prints `CAPTURE_GREEN path=...` after the downloaded image is
present and non-empty.

## Devcontainer

This repo includes a devcontainer intended to run on the Linux Raspberry Pi host
that has the DSLR attached over USB.

The container installs Rust plus the native `gphoto2` runtime and development
packages needed by `gphoto2-rs`. It also passes through `/dev/bus/usb` in
privileged mode so `gphoto2 --auto-detect` can see the camera from inside the
container.

Open the repo with VS Code Dev Containers or compatible tooling, then verify the
camera path from inside the container:

```sh
gphoto2 --auto-detect
```
