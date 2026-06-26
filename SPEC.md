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
