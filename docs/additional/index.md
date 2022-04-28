# Navodila za namestitev Ubuntu 18.04 Server + ROS Melodic na RaspberryPi 4 

- Headless uporaba RPI 
- Povezava preko SSH 
- SSH poveza z Visual Studio Code 

 

## Namestitev Ubuntu 18.04 Server 

Uporabi Disc Management in formatiraj 16 Gb SD kartico 

Snemi ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz (https://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz)

Extrahiraj sliko (7zip) 

Sledi navodilom za namestitev: https://help.ubuntu.com/community/Installation/FromImgFiles 

Na RaspberryPi priklopi ekran, miško in tipkovnico, se prijavi v ubuntu sistem

```python
username: ubuntu
password: ubuntu
```
in spremeni geslo.

Sledi navodilom za postavitev mreže in SSH: https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet 


Spremeni geslo: 

```python linenums="1"
sudo passwd ubuntu 

```

## Namestitev ROS 

Poveži se preko SSH 

Sledi navodilom, namestiš ROS-Base (ker je Ubuntu server, nima smisla nameščati grafična orodja): http://wiki.ros.org/melodic/Installation/Ubuntu 

Desni miškin klik je “paste” v cmd. 

Ko namestis, naredi `catkin_ws` 

```
cd
mkdir catkin_ws 
cd catkin_ws 
mkdir src 
cd .. 
catkin_make 
```

Dodaj še source za ROS spremenljivke 

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc 
```

## Nastavitev statičnega IP 

Vir: https://linuxize.com/post/how-to-configure-static-ip-address-on-ubuntu-18-04/ 

S spodnjim ukazom dobi seznam vseh mrežnih naprav:
```
ip link
```

```python linenums="1"
# This file is generated from information provided by the datasource.  Changes 
# to it will not persist across an instance reboot.  To disable cloud-init's 
# network configuration capabilities, write a file 
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following: 
# network: {config: disabled} 

network: 
   ethernets: 
      eth0: 
         dhcp4: false 
         addresses: 
            - 192.168.65.60/24 
         gateway4: 192.168.65.254 
         nameservers: 
            addresses: [192.168.65.14, 193.2.1.66] 
         version: 2 
```

Povezava s SSH 

V Win CMD se povežeš s:  

ssh RPI_uporabnisko_ime@RPI_IP 

 

Povezava z VS Code 

Sledi navodilom: https://www.raspberrypi.org/blog/coding-on-raspberry-pi-remotely-with-visual-studio-code/ 

