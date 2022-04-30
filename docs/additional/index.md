# Navodila za namestitev Ubuntu 18.04 Server + ROS Melodic na RaspberryPi 4 

- Headless uporaba RPI 
- Povezava preko SSH 
- SSH poveza z Visual Studio Code 

 

## Namestitev Ubuntu 18.04 Server 

Uporabi Disc Management in formatiraj 16 Gb SD kartico 

Snemi [ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz](https://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz)

Extrahiraj sliko (7zip) 

Sledi [navodilom za namestitev](https://help.ubuntu.com/community/Installation/FromImgFiles).

Na RaspberryPi priklopi ekran, miško in tipkovnico, se prijavi v ubuntu sistem

```python
username: ubuntu
password: ubuntu
```
in spremeni geslo.

Sledi [navodilom za postavitev mreže in SSH](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet).


Spremeni geslo: 

```python
sudo passwd ubuntu 

```

## Namestitev ROS 

Poveži se preko SSH 

Sledi [navodilom](http://wiki.ros.org/melodic/Installation/Ubuntu), namestiš ROS-Base (ker je Ubuntu server, nima smisla nameščati grafična orodja)

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

[Vir](https://linuxize.com/post/how-to-configure-static-ip-address-on-ubuntu-18-04/)

S spodnjim ukazom dobi seznam vseh mrežnih naprav:
```
ifconfig
```
Poišči ustrezno mrežno kartico (po navadi je `eth0`).

Postavi se v `/etc/netplan`
```
cd /etc/netplan
```
in odpri `01-netcfg.yaml` (če je ni, preveri z `ll`)

```
sudo nano /etc/netplan/01-netcfg.yaml
```

Dodaj spodnjo kodo, kjer nastaviš ustrezne mrežne nastavitve. Pozorni bodite na ustrezne zamike (uporabite presledke ne tabulator).

```python linenums="1" hl_lines="5 6 7 8 9"
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

Na koncu ponovno zaženi netplan
```
sudo netplan apply
```
ter preveri, če so nastavitve pravilne z
```
ifconfig
```

## Povezava s SSH 

V Win CMD se povežeš s:  
```
ssh RPI_uporabnisko_ime@RPI_IP 
```
 

## Povezava z VS Code 

Sledi [navodilom](https://www.raspberrypi.org/blog/coding-on-raspberry-pi-remotely-with-visual-studio-code/)

Pozor! Potrebuješ delujočo mrežno povezavo!

## GPIO na RaspberryPi

[rpi.gpio](https://sourceforge.net/projects/raspberry-gpio-python/)


Namestitev:

```
sudo apt-get update
$ sudo apt-get install python-rpi.gpio
sudo adduser $USER dialout
```

Uporaba:


