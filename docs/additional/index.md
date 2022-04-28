# Navodila za namestitev Ubuntu 18.04 Server + ROS Melodic na RaspberryPi 4 

- Headless uporaba RPI 
- Povezava preko SSH 
- SSH poveza z Visual Studio Code 

 

## Namestitev Ubuntu 18.04 Server 

Uporabi Disc Management in formatiraj 16 Gb SD kartico 

Snemi ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz (https://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz)

Extrahiraj sliko (7zip) 

Sledi navodilom za namestitev: https://help.ubuntu.com/community/Installation/FromImgFiles 

Na RaspberryPi priklopi ekran, miško in tipkovnico, se prijavi v ubuntu sistem (ui: ubuntu, pwd: ubuntu) in spremeni geslo.

Sledi navodilom za postavitev mreže in SSH 

https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet 


Spremeni geslo: 
```
sudo passwd ubuntu 
```
