# Project Rules

- This repository targets ROS 2 Lyrical Luth only.
- Do not use Jazzy, Kilted, Rolling, or Humble APIs unless explicitly gated behind compatibility shims.
- Generated packages, launch files, service definitions, action definitions, and CI must assume ROS_DISTRO=lyrical.
- All examples should use Lyrical-compatible commands and package names.
