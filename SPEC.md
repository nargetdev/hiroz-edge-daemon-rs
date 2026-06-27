# host context:
we are running locally on the Raspberry Pi host itself:

```validate host
id2-rpi4@id2-rpi4:~ $ hostname
id2-rpi4
```

🤔{✅/❌} this working context is already on `id2-rpi4@id2-rpi4`

# rust libs context
there's a gphoto2 lib for rust at
https://github.com/maxicarlos08/gphoto2-rs

🤔{✅/❌} gphoto2-rs is vendor'd here in cargo.toml

🤔{✅/❌} grok gphoto2-rs source code on disk here (can we see the actual source code now, not just a reference?)


# DSLR camera context
🤔{✅/❌} this Pi has a Canon EOS 6D on the local USB bus

```validate DSLR
id2-rpi4@id2-rpi4:~ $ gphoto2 --auto-detect
Model                          Port
----------------------------------------------------------
Canon EOS 6D                   usb:001,004
```


---
Given all context gates are passing (✅) accomplish goal #1:


((GOAL #1))
snap a single photo from the camera .. mmmkay

display the image we got
🤔{✅/❌} [[show the image]]


# Hiroz integration #

using the following zenoh config

```sh
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/172.31.1.252:7447"]'
```

using hiroz [ZettaScaleLabs/hiroz](https://github.com/ZettaScaleLabs/hiroz)

🤔{✅/❌} using hiroz we are able to receive a ROS2 zenoh CDR message for ROS topic `/chatter`

like ...

```text
data: 'Hello World: 251980'
```

🤔{✅/❌} using hiroz we are able also to publish our own custom message to `/chatter`

like:

```text
CHIRP CHIRP! From gphoto2-rs :: `datetime`
```

# DSLR capture service #

Detailed service and image publishing requirements live in [SPEC_dslr_svc.md](SPEC_dslr_svc.md).

🤔{✅/❌} a ROS2 service definition requests a capture with basic params exposed as enum selections and quickly ACKs the selected params; after gphoto2 capture the CR2 bytes and JPEG `sensor_msgs/CompressedImage` are published via Hiroz, while a status topic keeps the DSLR namespace visible to monitors
