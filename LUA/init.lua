function startup()
    if file.open("init.lua") == nil then
        print("init.lua deleted or renamed")
    else        
        dofile('main.lua')
        print("Running")
        file.close("init.lua")
    end
end

wifi.setmode(wifi.STATION)
station_cfg={}
station_cfg.ssid="XXX"
station_cfg.pwd="XXX"
wifi.sta.config(station_cfg)
--wifi.sta.config{"",""}
wifi.sta.connect()
wifi.sta.setip({
ip = "192.168.0.51",
netmask = "255.255.255.0",
gateway = "192.168.0.1"
})

BAUD = 115200
DATABITS = 8
PARITY = uart.PARITY_NONE
STOPBITS = uart.STOPBITS_1
uart.setup(0, BAUD, DATABITS, PARITY, STOPBITS, 0)

tmr.alarm(1, 1000, 1, function()
    if wifi.sta.getip() == nil then
        print("Waiting for IP address...")
    else
        tmr.stop(1)
        print("WiFi connection established, IP address: " .. wifi.sta.getip())
        print("You have 3 seconds to abort")
        print("Waiting...")
        tmr.alarm(0, 3000, 0, startup)
    end
end)
