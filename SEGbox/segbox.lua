#!  /usr/bin/lua
-- ------------------------------------------------------------------------- --
-- segbox.lua
-- ~~~~~~~~~~~~~~~~
--
-- This Lua program listens to the serial output from SEGmeters and other things
-- on the smart energy network, it wil try and send to SEG, if not it will write to a file.
-- once a connection to SEG is established again, it will send on what's in the file.
--
-- Please do not remove the following notices.
-- Copyright (c) 2011 Smart Energy Groups Pty. Ltd.
-- Version: --- see version key below
--   2.4    11.12.2011 has 3g rebootable, data cache, better organisation of parameters
--   2.5    12.12.2011 fixes some initialisation issues
--   2.6    12.01.2012 data rate control
--   2.7    03.02.2012 simplified the catchup
--   2.8    13.02.2012 data storage to disk, fix little bugs and cruft cleanup
--   2.8.1  28.05.2012 splitting storage to chunks and modifying file locations
--   2.8.4  13.06.2012 general maintenance release and bug fixes
--

-- Some Information
-- ~~~~~~~~~~~~~~~~
--
-- Sending data to SEG:
-- some information as to how the gateway will send data up to SEG with this s-expression:
-- (site site_token (node node_name time_stamp (stream_name value)))
-- to the restful web service at:
-- http://api.smartenergygroups.com/sites/<your site_token>
-- :method => :put  (where the body of the put has all the gear to send)
--
-- where keys, tokens etc can be obtained from Smart Energy Groups (SEG) here

-- https://smartenergygroups.com/my_things/show_keys
-- site_token = SEG site token
-- node_name = SEG device node name
-- time_stamp = a time stamp values:
--    ? to tell SEG to use the SEG server time in UTC (optional)
--    The time stamp in ISO 8601 format stamped by the SEGbox
-- stream_name = SEG stream name (belonging to a device)
--
-- Executing commands from SEG:
-- smartenergygroups.com can send commands to commandable SEGmeters or nodes
-- the command comes on the response of the sending data :put command in the form
-- (node_name (relay relay_state shot_seconds command_id));
-- or for other relays
-- (node_name (relay_1 relay_state shot_seconds command_id));
-- (node_name (relay_2 relay_state shot_seconds command_id));
-- (node_name (relay_3 relay_state shot_seconds command_id));
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
-- command = a keyword to identify the information being sent to SEG is a command receipt
-- command_status = the status of the command, usually complete, but may be error.
--
-- Some notes about SEG and commands:
-- SEG will retry command sending a number of times if it doesn't get a completed message before giving up.
-- SEG will idempotent commands, e.g. if a switch is on/off/on/off/on/off/on/off a number of times in the polling
-- period, the last command will be sent.
-- SEG will process commands already in a state of executing before sending new commands to the device.
--

-- ------------------------------------------------------------------------- --
-- Uncomment this is luarocks is required.
-- require("luarocks.require")

require("socket")
require("io")
require("ltn12")
--require("ssl")
require("config_segbox") -- SEGbox configuration
require("logic_segbox")  -- SEGbox logic and special functions



function initialise()

  set_web_host()

  version                   = "2.8.5"
  debug                     = false
  api_debug                 = false
  special_debug             = false
  command_debug             = false
  serial_timeout_period     = 10.0 -- seconds
  heartbeat_rate            = 300 / serial_timeout_period
  site_discovery_gracetime  = 60.0 -- seconds
  PLAIN                     = 1 -- string.find() pattern matching off

  talk_to_seg               = true
  get_segmeter_data         = false

  connection_throttle       = 150
  rapid_heartbeat_timeout   = 200

  time_ok                   = false
  content_type              = "application/x-www-form-urlencoded"  -- to be deprecated
  -- content_type           = "text/plain"                      -- newer type

  data_sub_directory        = "data"
  archived_sub_directory    = "archive"
  data_file_line_index      = 0
  data_file_line_max        = 1000
  data_file_name            = "seg_data"
  node_list_file            = "node_list"

  -- Working buffers
  node_list                 = {}
  node_atoms                = {}
  node_messages             = {}

  time_is_ok                = true            -- tells us if the clock is ntp synced
  time_check_frequency      = 1000

  tube_status               = "disconnected"  -- The connection to the intertubings states, disconnected, connected
  seg_status                = "disconnected"  -- The connection with SEG states, disconnected, connected
  has_cached                = false           -- tells us if there is data in the cache
  sendable_data                 = true        -- toggles whether or not there is sendable datas
  status                    = true            -- used for pcall statii
  display_3g                = false           -- to turn the 3G display on
  toggle_3g                 = false           -- to turn the 3G on and off in the serial handlers

  gateway_address           = "127.0.0.1" -- ser2net host IP address and port --
  listening_port            = 2000

  make_sub_directories()
end

-- ------------------------------------------------------------------------- --




initialise()

print("-----------------------------------")
print("SEGbox " .. version)
print("is starting now!")

local cycle_counter = 0
local loop_counter = 0
local sendable_sexp = ""
run_program = true

coroutine_serial_talker = coroutine.create(serial_talker)
coroutine_services = coroutine.create(services_manager)

if has_3g == true then
  print("I have a 3G service!  Please check me out...")
else
  print("I am a normal SEGbox, no 3G for me")
end

if initialise_intertubes() ~= true then
  print("Fail in initialise_intertubes, and rebooting the SEGbox now")
  run_program = false
end

if timestamp_data == true then
  confirm_time_is_ok()
end

send_event_boot()

coroutine_heartbeat = coroutine.create(heartbeat_handler)

rapid_heartbeat_discover()

print("Statii before main loop SERsALs: " .. coroutine.status(coroutine_serial_talker) .. " TUBES: " .. tube_status .. " SEG: " .. seg_status)

if time_is_ok then
  while run_program do
    if sendable_data == true then
      print(time_now() .. " C-L: " .. cycle_counter .. "-" .. loop_counter .. " SERsALs: " .. coroutine.status(coroutine_serial_talker) .. " TUBES: " .. tube_status .. " SEG: " .. seg_status)
    end

    sendable_data = false

    if (loop_counter > time_check_frequency) then
      loop_counter = 0
      cycle_counter = cycle_counter + 1
      if timestamp_data == true and seg_status == "connected" then
        confirm_time_is_ok()
      end
    end

    if (seg_status == "connected") then
      coroutine.resume(coroutine_heartbeat)
      send_all_stored_data()
    else
      coroutine.resume(coroutine_services)
    end

    if (coroutine.status(coroutine_serial_talker)) ~= "dead" then
      coroutine.resume(coroutine_serial_talker)
    else
      -- restart the serials
      print("A little sleep before re-starting the serials")
      sleepy_sleep(15)
      coroutine_serial_talker = nil
      coroutine_serial_talker = coroutine.create(serial_talker)
    end

    status, message = pcall(do_sendable)

    loop_counter = loop_counter + 1
  end
end

print("The SEGbox has stopped running for:")
print("C-L: " .. cycle_counter .. "-" .. loop_counter .. " SERsALs: " .. coroutine.status(coroutine_serial_talker) .. " TUBES: " .. tube_status .. " SEG: " .. seg_status)

-- restart now managed by crontab deamon
-- reboot_segbox()
