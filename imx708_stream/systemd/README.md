# Boot services for the camera stream nodes

One process owns one camera (see `SPEC_imx708_svc.md`), so each camera gets
its own unit. Both publish JPEG at 1/8 of the active sensor mode
(`--jpeg-scale 3`) to keep bandwidth minimal.

Install on the target host (paths assume the repo at
`/home/blackfinfive/pgwaam_ws/hiroz-edge-daemon-rs`):

```sh
cargo build --release -p imx708_stream
sudo cp imx708_stream/systemd/pgwaam-camera-*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pgwaam-camera-front_wide pgwaam-camera-ov5647
```

Check:

```sh
systemctl status pgwaam-camera-front_wide pgwaam-camera-ov5647
journalctl -u pgwaam-camera-front_wide -f
```
