#!/usr/bin/lua

-- require("luarocks.require")


require("socket")

function trim(s)
  return (s:gsub("^%s*(.-)%s*$", "%1"))
end

ip_address_coming = false
line_count = 0
ip_line = "I don't have an IP address!"

f = io.popen("ifconfig")
line = "init"
while line do
  line = f:read()
  if line ~= nil then
    -- print(line)
    if (string.find(line, "br-")) or (string.find(line, "en1:")) then
      ip_address_coming = true
    end

    if ip_address_coming == true and line ~= nil and line_count < 4 then
      line_count = line_count + 1
      line = trim(line)
      if string.find(line, "inet") then
        ip_line = "My IP address: " .. line
        break
      end
    end
  end
end

print(ip_line)

the_sock = require("socket")
seg_ip = the_sock.dns.toip("api.smartenergygroups.com")

if seg_ip == nil then
  print("Can't work out the DNS for the SEG!")
else 
  print("DNS gives SEG API ip: " .. seg_ip)
end 

