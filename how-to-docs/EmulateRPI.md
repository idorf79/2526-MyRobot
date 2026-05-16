# Ubuntu Server on Raspberry PI

Sources:

<https://canonical-ubuntu-hardware-support.readthedocs-hosted.com/boards/how-to/ubuntu_supported/raspberry-pi/>

<https://azeria-labs.com/emulate-raspberry-pi-with-qemu/>

```bash
sudo apt-get install qemu-system
```

```bash
Download: <https://cdimage.ubuntu.com/releases/noble/release/ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz>

Unpack the file:

```bash
unzx <filename>
```

## Gather information

```bash
fdisk -l ubuntu-24.04.4-preinstalled-server-arm64+raspi.img
```

Example: if start sector is 2048, offset = 2048 * 512 = 1048576

```bash
sudo mount -o loop,offset=1048576 ubuntu-24.04.4-preinstalled-server-arm64+raspi.img /mnt
```

```bash
cd ~/qemu_vms/
mkdir boot
```

```bash
sudo touch mnt/ssh
```

```bash
openssl passwd -6

echo 'pi:$6$Ri3p0XhobuAJ7FcB$zIHxkGO5GqVObzk.dNtwBuJnezRZINQsPt3CSdGHwQ4k.lVOGG/9oToKOSpdHgKA2OoZwUjJwhMEoG5T6WsT/1' | sudo tee boot/userconf.txt
```

### Copy out the kernel and initrd

We might need to do this also after an upgrade of the kernel.

```bash
cp /mnt/vmlinuz ./boot/vmlinuz
cp /mnt/initrd.img ./boot/initrd.img

sudo umount /mnt
```

```bash
qemu-system-aarch64 -machine virt -cpu cortex-a72 -smp 6 -m 4G \
    -kernel boot/vmlinuz \
    -initrd boot/initrd.img \
    -append "root=/dev/vda2 rootfstype=ext4 rw panic=0 console=ttyAMA0" \
    -drive format=raw,file=/home/fste/qemu_vms/ubuntu-24.04.4-preinstalled-server-arm64+raspi.img,if=none,id=hd0,cache=writeback \
    -device virtio-blk,drive=hd0,bootindex=0 \
    -netdev user,id=mynet,hostfwd=tcp::2222-:22 \
    -device virtio-net-pci,netdev=mynet \
    -monitor telnet:127.0.0.1:5555,server,nowait
```

## Let's optimze?

```bash
qemu-system-aarch64 -machine virt -cpu cortex-a72 -smp 6 -m 4G \
    -kernel boot/vmlinuz \
    -initrd boot/initrd.img \
    -append "root=/dev/vda2 rootfstype=ext4 rw panic=0 console=ttyAMA0" \
    -drive format=raw,file=/home/fste/qemu_vms/ubuntu-24.04.4-preinstalled-server-arm64+raspi.img,if=none,id=hd0,cache=writeback \
    -device virtio-blk,drive=hd0,bootindex=0 \
    -netdev user,id=mynet,hostfwd=tcp::2222-:22 \
    -device virtio-net-pci,netdev=mynet \
    -monitor telnet:127.0.0.1:5555,server,nowait
```

add '-nographic \', if you want everything in a terminal (usefull for WSL??)

## Increase the image size

Shutdown the emulator

```bash
qemu-img resize -f raw ubuntu-24.04.4-preinstalled-server-arm64+raspi.img +20G
```

Start the emulator and run:

```bash
sudo parted /dev/vda
(parted) print free    # show free space
(parted) resizepart 2 100%
(parted) quit

# Resize the filesystem to fill the partition
sudo resize2fs /dev/vda2
```
