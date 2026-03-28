# GPS Bringup
GPS module is powered by 12V 2A DC brick. It is then connected to the computer through an ethernet switch.

## Hardware
- **Receiver:** Novatel PwrPak7 (OEM7)
- **IMU:** Epson G320N (inside the PwrPak7 enclosure, not the antenna)
- **Datasheets:**
  - [OEM-EG320N Product Sheet](https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/OEM-EG320N-Product-Sheet/OEM-EG320%20Product%20Sheet.pdf)
  - [OEM-EG320N Sensor Specifications](https://docs.novatel.com/OEM7/Content/Technical_Specs_IMU/OEM_G320_Sensor_Specifications.htm)

## Networking
The Novatel needs to be setup to accept an IP address from an external DHCP server. To do so, I am following

1. connect to the receiver via wifi https://docs.novatel.com/Tools/Content/Manage_Web/OpenWebUI.htm?tocpath=Manage%20Web%7C_____1
1. Enable DHCP
