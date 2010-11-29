#!/usr/bin/lua
-- ------------------------------------------------------------------------- --
-- aiko_gateway.lua
-- ~~~~~~~~~~~~~~~~
-- Please do not remove the following notices.
-- Copyright (c) 2009 by Geekscape Pty. Ltd.
-- Documentation:  http://groups.google.com/group/aiko-platform
-- License: GPLv3. http://geekscape.org/static/arduino_license.html
-- Version: 0.3
-- ------------------------------------------------------------------------- --
-- See Google Docs: "Project: Aiko: Stream protocol specification"
-- Currently requires an Aiko Gateway (indirect mode only).
-- ------------------------------------------------------------------------- --
--
-- Custom configuration: See "aiko_configuration.lua".
--
-- Some Information
-- ~~~~~~~~~~~~~~~~
--
-- Sending data to SEG:
-- some information as to how the gateway will send data up to SEG with this s-expression:
-- (site site_token (node node_name time_stamp (stream_name value)))
-- to the restful web service at:
-- http://api.smartenergygroups.com/api_sites/stream
-- :method => :put
--
-- where refrence keys, tokens can be obtained from Smart Energy Groups (SEG) here

-- https://smartenergygroups.com/my_things/show_keys
-- site_token = SEG site token
-- node_name = SEG device node name
-- time_stamp = a time stamp values:
--    ? to tell SEG to use the SEG server time in UTC
--    The time stamp in ISO 8601 format stamped by the SEGbox (not yet implemented)
-- stream_name = SEG stream name (belonging to a device)
--
-- Executing commands from SEG:
-- smartenergygroups.com can send commands to commandable SEGmeters or nodes
-- the command comes on the response of the sending data :put command in the form
-- (node_name (relay relay_state shot_seconds command_id));
--
-- where:
-- node_name = the name of your SEG device (refer above)
-- relay = a fixed string from SEG telling the device this is a relay command
-- relay_state = on or off - self explainatory ;)
-- shot_seconds = the following values:
--     ? or 0 remain on indefinately or
--     some value in seconds to remain in the state of on
-- command_id = the SEG command id, for reciepting the completion of the command
--
-- The SEGmeter device will take this command and execute it, and return a message
-- for sending back to SEG for completion of the command like so:
--
-- (node node_name time_stamp (command command_id command_status))
--
-- where:
-- command = a keyword to identify the information being sent to SEG is a command reciept
-- command_status = the status of the command, usually complete, but may be error.
--
-- Some notes about SEG and commands:
-- SEG will retry command sending a number of times if it doesn't get a completed message before giving up.
-- SEG will idempotent commands, e.g. if a switch is on/off/on/off/on/off/on/off a number of times in the polling
-- period, the last command will be sent.
-- SEG will process commands already in a state of executing before sending new commands to the device.
--
--
-- ToDo: Aiko Gateway
-- ~~~~~~~~~~~~~~~~~~
-- - Put protocol version into boot message to Aiko-Node and web service.
-- - Verify protocol version in the Aiko-Node boot message.
--   - Send tweet to owner, if newer software versions are available.

-- - Re-open serial network port 2000, if it closes.

-- - Does "http" variable need to be local, or can it be global ?

-- - Create aiko_gateway.sh, setting environment variables and background run.
-- - Put all configuration parameters into a table.
-- - Command line options: Host/Port, Help and Version.
-- - Handle multiple connected AikoNodes, e.g. Ethernet and ZigBee.
-- - Lua / Lua-Socket co-routines for non-blocking I/O.
-- - Parse S-Expression messages, match open-close brackets (as per Aiko in C).
--   - Use LPeg (Parsing Expression Grammars For Lua) ?
--     See http://www.inf.puc-rio.br/~roberto/lpeg
-- - Maintain last message timestamp for idempotent message check.
-- - LuCI web server integration, e.g. monitor, control, configure, statistics.
-- - SSL connection to https://api.smartenergygroups.com.
-- - Handle some messages from Aiko, e.g. errors, provide date/time.
-- - Handle "debug messages" from Aiko-Node, e.g. "; Lisp comment :)"

-- ToDo: Aiko Node
-- ~~~~~~~~~~~~~~~
-- - Configuration in EEPROM, e.g nodeName, devices, networking, site token and other useful parameters
-- - Profile negotiation, i.e like Telnet negotiation.
-- - Message optimatization, including "transducer identifier" (integer).
-- - Messages ...
--   - (temperature xx.x C)
--   - (light xxx lux)
--   - (button1 on ?)  (button2 on ?)  (button3 on ?)
--   - (relay1 on  ?)  (relay2  on ?)
--   - (sound NOTE ?)
--   - (alert MESSAGE TIME-TO-LIVE) --> LCD screen.
--   - (clock= YYYY-MM-DDThh:mm:ss) --> EEPROM.
--   - (node_name= NODE-NAME) --> LCD screen.
--   - (profile= PROFILE-NAME)
--   - (schedule= SCHEDULE) --> EEPROM.
--   - (serial_number= SERIAL_NUMBER) --> EEPROM.

-- ToDo: Miscellaneous
-- ~~~~~~~~~~~~~~~~~~~
-- - Desktop / laptop version: Provide WxLua GUI.

-- ------------------------------------------------------------------------- --
-- TableSerialization
-- http://lua-users.org/wiki/TableSerialization

-- Network support for the Lua language
-- http://www.tecgraf.puc-rio.br/~diego/professional/luasocket
-- http://www.tecgraf.puc-rio.br/~diego/professional/luasocket/http.html
-- HTTP/1.1 standard, RFC 2616
--   http://tools.ietf.org/html/rfc2616
-- HTTP Basic Authentication Scheme, RFC 2617
--   http://tools.ietf.org/html/rfc2617
-- URLs must conform to RFC 1738
--   http://tools.ietf.org/html/rfc1738
--   [http://][<user>[:<password>]@]<host>[:<port>][/<path>]

-- Using the socket library to read a web page (GOOD)
-- http://www.wellho.net/resources/ex.php4?item=u116/webclient

-- LUA SocketLib and the Coroutines
-- http://www.ozone3d.net/tutorials/lua_socket_lib.php

-- LuaSec – TLS/SSL Support for Lua
-- http://www.inf.puc-rio.br/~brunoos/luasec

-- HowTo: Using the JSON-RPC API
-- http://luci.freifunk-halle.net/Documentation/JsonRpcHowTo

-- LPeg (Parsing Expression Grammars For Lua)
-- http://www.inf.puc-rio.br/~roberto/lpeg
-- ------------------------------------------------------------------------- --

-- TODO: Move functions into a library file -- dofile(FILENAME) or require() ?

function current_directory()
  return(os.getenv("PWD"))
end

-- ------------------------------------------------------------------------- --

function is_production()
-- TODO: Use an environment variable to specify deployment type.

  return(os.getenv("USER") == "root") -- Assume logged in as "root" on OpenWRT
end

-- ------------------------------------------------------------------------- --

function table_to_string(table)
  local result = ''

  if (type(table) == 'table') then
    result = '{ '

    for index = 1, #table do
      result = result .. table_to_string(table[index])
      if (index ~= #table) then
        result = result .. ', '
      end
    end

    result = result .. ' }'
  else
    result = tostring(table)
  end

  return(result)
end

-- ------------------------------------------------------------------------- --

function url_encode(value)
  if (value) then
    value = value:gsub("\n", "\r\n")
    value = value:gsub("([^%w ])",
      function (c) return string.format ("%%%02X", string.byte(c)) end)
    value = value:gsub(" ", "+")
  end

  return(value)
end

-- ------------------------------------------------------------------------- --

function use_production_server()
  local web_host_name = "api.smartenergygroups.com"
-- Some other servers useful for testing things.
-- local web_host_name = "localhost:3000"
-- local web_host_name = "192.168.110.189:3000"

  url = "http://" .. web_host_name .. "/api_sites/stream"

  file_name = current_directory() .. "/data/aiko_test1.data"

  method       = "PUT"
  content_type = "application/x-www-form-urlencoded"
end

-- ------------------------------------------------------------------------- --

function use_development_server()
  local web_host_name = "192.168.110.189:3000"

  url = "http://" .. web_host_name .. "/api_sites/stream"

  file_name = current_directory() .. "/data/aiko_test2.data"

  method       = "POST"
  content_type = "application/xml"
end

-- ------------------------------------------------------------------------- --

function use_php_debug_server()
  use_production()

  url = "http://localhost/~andyg/php/examine_request.php"
end

-- ------------------------------------------------------------------------- --

function custom_sink()
  return function(chunk, error)
    if (not chunk) then
      return(1)
    else
      return(print(chunk))
    end
  end
end

-- ------------------------------------------------------------------------- --

send_message_disabled = false

function send_message(message)

  local http = require("socket.http")
  local response = {}

  if (send_message_disabled) then
    return
  end

  if (debug) then
    print("-- send_message(): start")
  end

  --local body, code, headers, status = http.request(url, "keyword=value")

  local body, code, headers, status = http.request {
    url = url,
    method = method,
    headers = {
      ["content-length"] = message:len(),
      ["content-type"]   = content_type
    },
    --  source = ltn12.source.file(io.open(file_name, "r")),
    source = ltn12.source.string(message),
    --  sink = ltn12.sink.file(io.stdout)
    sink = ltn12.sink.table(response)
    --  sink = custom_sink()
  }

  if (body == nil) then
    print("Error: ", code)
  else
    if (debug) then
      -- TODO: Check status code is either success versus fail
      print("Body:     ", body)  -- Will equal "1", should a generic method be used
      print("Code:     ", code)
      print("Headers:  ", table_to_string(headers))
      print("Status:   ", status)
      print("Response: ", table_to_string(response))
    end
  end

  if (response == nil) then
    print("Error: No HTTP response body")
  else

    response = table.concat(response)
    -- print("-- The fresh SEG response: ", response)

    if (response:sub(1, 6) == "(node ") then
      if (debug) then
        print("-- SEG node command received: ", response)
      end
      -- print("-- The command received from SEG: ", response)

      -- example command (node (segmeter (relay on 0 7734)))

      local start = 7
      local finish = nil

      finish = response:find("\n", 1, PLAIN)

      -- print("Found the finish at: ", finish)

      if (finish ~= nil) then
        -- only want to send so trim it lke so (segmeter (relay on 0 7734))

        local command = nil
        command = response:sub(start, finish - 1)
        -- print("mesage response sub is: ", message )

        if (command) then
          -- print ("The command about to be sent: ", command)
          -- command = "(segmeter (relay on 0 7734))"
          -- command = "(segmeter (relay on 0 7734))"
          print ("SEG command sending now: ", command)
          if (debug) then
            print("-- send message(): command: ", command)
          end

          serial_client:send(command .. ";\n")
          command = ""
        end
      else
        -- print("Fail on finding the command start and finish")
      end
    end

    -- Check response wrapped by site command, e..g (site= new_site_token)
    if (response:sub(1, 7) == "(site= ") then
      local start, finish = response:find("\n", 1, PLAIN)
      local message = response:sub(1, start - 1)
      response = response:sub(finish + 1)

      -- Parse and save new site token
      save_site_token(message:sub(8, -2))
      if (debug) then
        print("-- parse_message(): new site token: " .. site_token)
      end
    end

    -- Misc outputing

    -- if (response == "(status okay)") then
    --   if (debug) then
    --     print("-- send_message(): status: okay")
    --   end
    -- else

    -- This is to back off on the no site token situation
    if (response:find("no_site_token", 1, PLAIN)) then
      if (debug) then
        print("-- Initiating back off on the discovery timeout")
      end
      site_discovery_timeout()
    end
  end

  if (debug) then print("-- send_message(): end") end
end

-- ------------------------------------------------------------------------- --

function send_event_boot(node_name)
  if (debug) then print("-- send_event_boot(): " .. node_name) end

  message = "(status boot 0.3)"
  send_message(wrap_message(message, node_name))
end

-- ------------------------------------------------------------------------- --

function send_event_heartbeat(node_name)
  if (debug) then print("-- send_event_heartbeat(): " .. node_name) end

--message = "(cpu_usage 0 %) (node_count 1 number)"
  message = "(status heartbeat)"
  send_message(wrap_message(message, node_name))
end

-- ------------------------------------------------------------------------- --

function save_site_token(new_site_token)
  if (new_site_token ~= site_token) then
    site_token = new_site_token

    local output = assert(io.open("aiko_configuration.lua", "a"))
    output:write("  site_token = \"" .. site_token .. "\"\n")
    assert(output:close())

    if (debug) then print("-- save_site_token(): saved " .. site_token) end
  end
end

-- ------------------------------------------------------------------------- --

site_discovery_timer = 0

function site_discovery_timeout()
  if (debug) then print("-- site_discovery_timeout():") end

  if (site_discovery_timer == 0) then
    site_discovery_timer = os.time() + site_discovery_gracetime
  else
    if (os.time() > site_discovery_timer) then
      send_message_disabled = true
      if (debug) then print("-- site_discovery_timeout(): expired") end
    end
  end
end

-- ------------------------------------------------------------------------- --

function heartbeat_handler()
  local throttle_counter = 1 -- Always start with a heartbeat

  while (true) do
    throttle_counter = throttle_counter - 1

    if (throttle_counter <= 0) then
      throttle_counter = heartbeat_rate

      send_event_heartbeat(aiko_gateway_name)
    end

    coroutine.yield()
  end
end

-- ------------------------------------------------------------------------- --

function send_file(node_name, file_name)
  if (debug) then print("-- send_file(" .. file_name .. "):") end

  file = io.input(file_name)
  message = io.read("*all")
  send_message(wrap_message(message, node_name))
end

-- ------------------------------------------------------------------------- --

function wrap_message(message, node_name)
  local timestamp = "?"

  return(
    "data_post=" ..
    "(site " .. site_token ..
    "  (node " .. node_name .. " " .. timestamp ..
    "    " .. message .. "))"
  )
end

-- ------------------------------------------------------------------------- --

function serial_handler()

  serial_client = socket.connect(aiko_gateway_address, 2000)
  serial_client:settimeout(serial_timeout_period)  -- 0 --> non-blocking read

  --serial_client:send("")

  local stream, status, partial

  local this_message = nil
  local message = nil
  local joined_message = nil
  local attempt = 0

  while (status ~= "closed") do
    stream, status, partial = serial_client:receive(16768)  -- (1024)

    if (debug) then
      if (status == "timeout") then
        print("Aiko status: bytes received: ", partial:len())
        print("Aiko serial recieved is: ", partial)
      else
        print("Aiko status: ", status)
      end
    end

    -- print ("Aiko stream:  ", stream)  -- TODO: if not "nil" then catenate
    -- print ("Aiko partial: ", partial) -- TODO: if not "nil" then got everything

    if (partial ~= nil and partial:len() > 0) then
      if (debug) then
        print("The AIKO Partial before parse_message is: ", partial)
      end

      for message in partial:gmatch("[^\r\n]+") do

        this_message = nil

        if ( check_message_bracketing(message) ~= true) then
          -- If the above is fail, it's likely a subsequent message may have the goods.

          joined_message = join_partial_messages(message, joined_message, attempt)

          attempt = attempt + 1

          if (joined_message ~= nil) then
            if (check_message_bracketing(joined_message) == true) then
              if (special_debug) then
                print("#WIN on fixing broken message: ", joined_message)
              end
              partial = joined_message
              joined_message = nil
              attempt = 0
            else
              if (debug) then
                print("Still some joining work to make win: ", joined_message)
              end
            end
          else
            -- Joining terminated.
            attempt = 0
          end
        else
          this_message = message
        end

        if (this_message ~= nil) then
          parse_message(this_message)
        end
      end -- the gmatch check
    end

    if (status == "timeout") then
      coroutine.yield()
    end
  end

  serial_client:close()
end

function check_message_bracketing(message)
  local goodness = true
  if (message:sub(1, 1) ~= "("  or  message:sub(-1) ~= ")") then
    if (special_debug) then
      print("SEGmeter message not correctly bracketed: ", message)
    end
    goodness = false
  end
  return goodness
end

function join_partial_messages(message, joined_message, attempt)

  if (special_debug) then
    print("Message joining attempt: ", attempt)
  end

  if (attempt == 0) then
    joined_message = message
    if (debug) then
      print("Initiating message correction: joined_message = ", joined_message)
    end
  else
    joined_message = joined_message .. message
    if (debug) then
      print("Joining message up: joined_message = ", joined_message)
    end
  end

  if (attempt > 3) then
    if (debug) then
      print("Giving up fixing message on attempt: ", attempt)
    end
    joined_message = nil
  end

  return joined_message
end

-- ------------------------------------------------------------------------- --

-- TODO: Move parser into a library file.

-- TODO: Need to assume that we won't get a complete, well-formed message !
-- TODO: Check for partial messages and catenat them, if necessary
-- TODO: Any left-over data (no trailing CR) should be kept for next time !
-- TODO: Implement incomplete message timeout

function parse_message(buffer)
  if (debug) then print("-- parse_message(): start") end

  local last_message = nil

  -- Parse individual Aiko-Node messages, delimited by "carriage return"

  for message in buffer:gmatch("[^\r\n]+") do

    if ( check_message_bracketing(message) == true) then
      -- Check message wrapped by node name, e..g (node name ...)

      if (message:sub(1, 6) ~= "(node ") then
        if (debug) then
          print("SEGmeter message pecularity - doesn't start with 'node'", message)
          print("If the above looks like a SEG command this is ok")
        end
      else

        -- Parse node name
        local node_name = nil
        local start, finish = message:find('?', 7, PLAIN)

        if (start ~= nil) then
          node_name = message:sub(7, start - 2)
          if (debug) then print("-- parse_message(): node: " .. node_name) end
        end

        if (node_name == nil) then
          print("-- parse_message(): ERROR: Couldn't parse node name")
        else

          local token = message:sub(finish + 1, finish + 2)
          if (token == " )") then
              -- Node heart-beat message, ignore for the moment
          else
            if (token == " (") then
              -- Node message containing state update
              message = message:sub(finish + 2, -2)
              if (debug) then print("-- parse_message(): event: ", message) end

              if (message ~= last_message) then
                send_message(wrap_message(message, node_name))
              else
                if (debug) then print("-- message duplicate ignored: ", message) end
              end  -- end the duplicate message check

              last_message = message
            else
              print("-- parse_message(): ERROR: Problem after the node name")
            end
          end
        end
     end
   end

  end -- end the message loop

  if (debug) then print("-- parse_message(): end") end
end

-- ------------------------------------------------------------------------- --

function initialize()
  PLAIN = 1  -- string.find() pattern matching off

  special_debug = true

  use_production_server()   -- Smart Energy Groups web service
--use_development_server()  -- Some development server
--use_php_debug_server()    -- Reflects HTTP request details in the response
end

-- ------------------------------------------------------------------------- --

print("[Aiko-Gateway V0.4 2010-11-28]")

if (not is_production()) then require("luarocks.require") end
require("socket")
require("io")
require("ltn12")
--require("ssl")

require("aiko_configuration")  -- Aiko-Gateway configuration file

initialize()

-- send_file("pebble_1", file_name)  -- Primarily for testing

-- TODO: Keep retrying boot message until success (OpenWRT boot sequence issue)
  send_event_boot(aiko_gateway_name)

coroutine_heartbeat = coroutine.create(heartbeat_handler)

-- TODO: Handle incorrect serial host_name, e.g. not localhost -> fail !
coroutine_serial = coroutine.create(serial_handler)

if (twitter_flag) then coroutine_twitter = coroutine.create(twitter_query) end

while (coroutine.status(coroutine_serial)) ~= "dead" do
  if (debug) then print("-- coroutine.resume(coroutine_heartbeat):") end
  coroutine.resume(coroutine_heartbeat)

  if (debug) then print("-- coroutine.resume(coroutine_serial):") end
  coroutine.resume(coroutine_serial)

  if (twitter_flag) then
    if (debug) then print("-- coroutine.resume(coroutine_twitter):") end
    coroutine.resume(coroutine_twitter)
  end
end
