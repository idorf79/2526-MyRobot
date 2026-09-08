# How to Shrink a SD card

## Using GParted

Use 'sudo gparted' to visually shrink the writable partition of the SD.
Resize in Gparted is only possible if the partition is not mounted.

After shrinking, use PiShrink to add the process to grow the partition again on the first boot.

## Using PiShrink

source: <https://github.com/Drewsif/PiShrink>

### Install needed packages

```bash
sudo apt update && sudo apt install -y wget parted gzip pigz xz-utils udev e2fsprogs
```

### Create a workspace

```bash
mkdir ~/pishrink
cd ~/pishrink
```

### Get PiShrink

Download it:

```bash
wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
```

Make it executable

```bash
chmod +x pishrink.sh
```

### Prepare an image to Shrink

Create an image from the SD card.

Check the device name of the SDCard

```bash
lsblk
```

Check SD cards can show up as 'mmblkX' or 'sdX'.

Make sure the disk(s) are NOT mounted anymore.

```bash
sudo dd bs=4M status=progress conv=fsync if=/dev/mmcblk0 of=~/pishrink/pi-image.img 
```

This will create the pi-image.img, which we can use with PiShrink.

```bash
cd ~/pishrink
sudo ./pishrink.sh ~/pishrink/pi-image.img ~/pishrink/pi-image_shrunk.img

### Write image to SD

Caution: 'if' and 'of' are changed here:

```bash
sudo dd bs=4M status=progress conv=fsync of=/dev/mmcblk0 if=~/pishrink/pi-image.img
```
