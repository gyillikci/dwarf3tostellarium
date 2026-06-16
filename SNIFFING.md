# Capturing the Dwarf3 WebSocket protocol

The Dwarf3 app ↔ telescope link is **plaintext `ws://<ip>:9900`** carrying
protobuf. That means you can read every command in the clear once the traffic
is routed through a box you control — no jailbreak, no Frida, no TLS to break.

This is how `dwarflab_controller.py` was built, and how you can find commands it
does not yet implement (e.g. the ROI / subject-tracking command).

## Hardware

A Linux SBC with **one AP-capable Wi-Fi radio** (a Banana Pi works). Confirm the
radio can be an access point:

```bash
iw list | grep -A12 "Supported interface modes"   # must list:  * AP
```

The Dwarf3 must be set to **station mode** (join an existing Wi-Fi) in the DWARF
Lab app — not its own hotspot. With one radio this is required; an AP-only
Dwarf3 would need a second (USB) Wi-Fi radio.

## 1. Turn the Pi into a Wi-Fi AP

```bash
sudo apt install hostapd dnsmasq iptables tcpdump python3
```

`/etc/hostapd/hostapd.conf`:

```
interface=wlan0
ssid=dwarf-lab
hw_mode=g
channel=6
wpa=2
wpa_passphrase=changeme123
wpa_key_mgmt=WPA-PSK
```

Give `wlan0` a static IP, hand out DHCP, and share the Ethernet uplink so the
app can still reach the internet for login:

```bash
sudo ip addr add 192.168.50.1/24 dev wlan0
# dnsmasq: dhcp-range=192.168.50.50,192.168.50.150,12h  (interface=wlan0)
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo systemctl unmask hostapd && sudo systemctl restart hostapd dnsmasq
```

Join **both** the phone and the Dwarf3 to SSID `dwarf-lab`. Note their assigned
IPs (`<PHONE_IP>`, `<DWARF_IP>`).

## 2a. Quick capture (passive)

Often the AP sees client traffic directly:

```bash
sudo tcpdump -i wlan0 -n -w dwarf.pcap 'tcp port 9900'
```

Open `dwarf.pcap` in Wireshark (it has a WebSocket dissector) or export payloads
to `protoc --decode_raw`. If you see the goto/status frames but **not** the
phone↔Dwarf traffic, your Wi-Fi driver forwards client-to-client frames
internally — use the proxy below instead.

## 2b. Guaranteed capture + live decode (active MITM)

Redirect only the phone's connection to the proxy (source-matched so the proxy's
own upstream connection is not looped):

```bash
sudo iptables -t nat -A PREROUTING -s <PHONE_IP> -d <DWARF_IP> \
     -p tcp --dport 9900 -j DNAT --to-destination 192.168.50.1:9900

python3 ws_sniff.py --upstream <DWARF_IP> --listen 0.0.0.0:9900
```

Now use the app normally. Every command prints decoded in real time:

```
▶ APP→DWARF  cmd=11013 ASTRO_START_ONE_CLICK_GOTO_DSO  module=3  type=0
             | f1=f64:10.68 f2=f64:41.27 f3=str:'M31'
◀ DWARF→APP  cmd=15211 NOTIFY_STATE_ASTRO_GOTO  module=9  type=1  | f1=varint:1
```

## 3. Find a new command (e.g. ROI tracking)

1. Start `ws_sniff.py`, then perform the action in the app (draw the tracking
   box and start tracking).
2. Note the new `cmd=` number and its payload fields (`f1=…`, `f2=…`).
3. Add a payload builder + `DwarfLab` method in `dwarflab_controller.py`, then a
   route in `server.py` — modelled on `p_goto_dso` / `goto_dso` / `/api/goto`.

Only the *facts* you learn (command number + field layout) go into this MIT
repo — never decompiled vendor source.
