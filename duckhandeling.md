# Duckiebot Handeling Guide 
This guide aims to explain the handeling of the duckiebots. 

## Running code on the bots

In terminal:

1. Change dir to DuckieTown workspace
```bash
cd DuckieTownROS 
```

2. Builds the code on the duckiebot named ![duck_name]
  ```bash
dts devel build -f -H ![duck_name]
  ```

3. Runs the launch script named ![launchfile] on ![duck_name]
  ```bash 
dts devel run -H ![duck_name] -L ![launchfile]
  ```

Find the launch files under launchers in the DuckieTownROS dir. 

## Closing docker container on the bots
If docker container is on:

In new terminal:

1.
 ```bash
ssh duckie@![duck_name].local
 ```
Password: quackquack

2.
```bash
docker ps
```
Find the running container 

3.
 ```bash
docker stop ![running_container]
```

4.
  ```bash
  logout
  ```
You do not have to be on a Linux computer to do this. Just be on the same network as the bots and run the commands in e.g Windows PowerShell

## Fleet discover
You need to be on the same network for this to work and if you are on a hotspot have it running before the bots are started. 

In terminal:

1.
  ```bash
dts fleet discover
```

If it worked you should see the bots name and their status. It should be saying ready with a green box.

## Change WIFI on bots

Either connect a keyboard and a screen to it or ssh in to it for this. 

1.
  ```bash
cd /etc
```

2.
  ```bash
sudo nano wpa_supplicant.conf
```

3. Change ssid to WIFI name

4. Change psk to WIFI password

NOTE: Do not have space or any other weird symbols in WIFI name

5. To get out of the page press ctrl+s and then ctrl+x

6. Reboot system by  executing:
```bash
systemctl reboot
```


## LED color guide
We don't know if this guide is accurate but this is what we found that the lights meant. 

* **Blue**: Charging
*  **White**: Booting
* **Red and white**: Running

There is more color and when you think you know what it means add it to the list.  
 

 ## Common Errors 
Here is some common errors we encountered during our time with the duckiebots
 ### Build Error
 If it has major red errors in the terminal when you build try to restart the bot with 
 ```bash
 dts duckiebot reboot ![duck_name]
 ```
 ### Connectivity Error 
 If the bots won't connect to wifi, check the wifi name on the bots to see if the name and password is the same as the connected WIFI. 

 ### One of the bot is not consistent when testing 
 Rebuild and try again


 ## Current configuration 

 **duck1**: Running bot

 **duck2**: Watchtower
 
 **duck3**: Running bot