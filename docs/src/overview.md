# Overview

This repository holds the edge services that run on the pgwaam cell's
Raspberry Pis and publish into the shared ROS 2 graph over Zenoh
(router: `tcp/172.31.1.252:7447`, rmw_zenoh-compatible).

Current services:

- **imx708_stream** — continuous camera streaming node (libcamera-rs).
  One process per camera. See [imx708_stream node](imx708_stream.md).
- **DSLR capture service** — one-shot Canon EOS capture via gphoto2
  (see `SPEC_dslr_svc.md` in the repo root).

Specs live in the repo root (`SPEC_imx708_svc.md`, `SPEC_dslr_svc.md`);
this book documents day-to-day usage.
