# validation_at_pi4__gphoto2-rs

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
