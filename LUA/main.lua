srv = net.createServer(net.TCP, 10)
srv:listen(80,function(conn)
    conn:on("receive", function(conn, payload)
        function esp_update()  
            fregMeas={string.find(payload,"freqMeas=")}
            normal ={string.find(payload,"normal=")} 
            immediate ={string.find(payload,"immediate=")} 
            message = ""
            fregMeasValue=""
            immediateValue=""
            normalValue=""
			
            if fregMeas[2]~=nil then
                fregMeasValue=string.sub(payload,fregMeas[2]+1, fregMeas[2]+1)
                if fregMeasValue == '&' then
                    message = message .. "freq:0"
                elseif fregMeasValue == '' then
                    message = message .. "freq:0"
                else
                    message = message .. "freq:" .. fregMeasValue
                end
            end
            if immediate[2]~=nil then
                immediateValue=string.sub(payload,immediate[2]+1, immediate[2]+1)
                message = message .. "immed:" .. immediateValue
            else
                message = message .. "immed:0"
            end
            if normal[2]~= nil then
                normalValue=string.sub(payload,normal[2]+1, normal[2]+1)
                message = message .. "normal:" .. normalValue
            else
                message = message .. "normal:0"
            end
            if normalValue == "1" or immediateValue == "1" then
                uart.write(0,message)
            end
        end
        
        esp_update()
        -- HTML Header 
        conn:send('HTTP/1.1 200 OK\n\n')
        conn:send('<!DOCTYPE HTML>\n')
        conn:send('<html>\n')
        conn:send('<head><meta  content="text/html; charset=utf-8">\n')
        conn:send('<title>ISP projectt</title></head>\n')
        conn:send('<body>')
        
        -- Przyciski
        conn:send('<form action="" method="POST">\n')
        conn:send('<fieldset style="width:400px"><legend> ISP project</legend>\n')
        conn:send('<div style="width:180px; float: left">Immediate measurement : </div>\n')
        conn:send('<button type="submit" name="immediate" value="1">Immediate</button><br><br>\n')
        conn:send('<div style="width:180px; float: left">Frequency Measurement: </div>\n')
        conn:send('<input list="freqMeas" name="freqMeas"><br>\n')
        conn:send('<button type="submit" name="normal" value="1">SEND</button><br></fieldset>\n')
        conn:send('<datalist id="freqMeas"><option value="1 per minute"><option value="2 per minute">\n')        
        conn:send('<option value="3 per minute"><option value="4 per minute"></datalist>\n')
        conn:send('</form></body></html>\n')
        conn:on("sent", function(conn) conn:close() end)
    end)
end)

function sendRequest(value, sensor_id)
    user = '"user_id":"wojtoka2@gmail.com",'
    sensor_id = '"sensor_id":"' .. sensor_id ..'",'
    desc = '"desc" : "temperatureSensor",'
    meas_val = value
    meas = '"measure_value" : "' .. meas_val .. '"'
    
    http.post('http://alfa.smartstorm.io/api/v1/measure',
      'Content-Type: application/json\r\n',
      '{'.. user .. sensor_id .. desc .. meas .. '}',
      "",
      function(code, data)
        if (code < 0) then
          print("HTTP request failed")
        end
      end)
end

uart.on("data", 30,
  function(data)
    --Temperature
    indexTempValue ={string.find(data,"Temp:")} 
    tempValue=string.sub(data,indexTempValue[2]+1, indexTempValue[2]+5)
    sendRequest(tempValue, "5a4fc06e31cdb871354c1a99")
    --Humidity
    indexHumValue ={string.find(data,"Hum:")} 
    humValue=string.sub(data,indexHumValue[2]+1, indexHumValue[2]+2)
    sendRequest(humValue, "5a5b56c379441c0fdfafe921")
    --Light
    indexLightValue ={string.find(data,"Light:")} 
    lightValue=string.sub(data,indexLightValue[2]+1, indexLightValue[2]+8)
    sendRequest(lightValue, "5a5b56d379441c0fdfafe922")    

end, 0)
