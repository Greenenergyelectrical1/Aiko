-- ------------------------------------------------------------------------- --
-- logic_segbox.lua
-- ~~~~~~~~~~~~~~~~
--
-- Has the main logics for the SEGbox for componentisation and ease of developemt
--
-- Please do not remove the following notices.
-- Copyright (c) 2011 Smart Energy Groups Pty. Ltd.
-- Version: 2.0
-- ------------------------------------------------------------------------- --

-- ------------------------------------------------------------------------- --

function do_sendable()

    local seg_win = false
    local sendable_sexp = ""

    sendable_sexp = output_seg_message_for_nodes()

    --  print("-----------------------------------------------------------------------------")
    --  print("sendable_sexp: " .. sendable_sexp)

    if sendable_sexp ~= "" then
        sendable_sexp = wrap_site_token(sendable_sexp)
        seg_win = seg_talker(sendable_sexp)
        if (seg_win == true) then
            clear_buffers()
        else
            save_messages(output_seg_message_for_nodes()) -- save the unwrapped :D
        end
    end
end

-- ------------------------------------------------------------------------- --

function clear_buffers()
    node_atoms = {}
    node_messages = {}
end

-- ------------------------------------------------------------------------- --

function send_all_stored_data()
    -- Sends all the data stored in the caches.

    local files_table = scandir(data_file_directory())

    if files_table ~= nil then
        for key, file_name in ipairs(files_table) do
            if string.find(file_name, 'seg_data') then
                send_stored_data_file(file_name)
                sleepy_sleep(2)
            end
        end
    end
end

-- ------------------------------------------------------------------------- --

function send_stored_data_file(this_file)

    local seg_win = false
    local sendable_sexp = ""

    print("Processing to send data_file: " .. this_file)

    if file_exists(data_file_path_for_file(this_file)) then

        sendable_sexp = get_sendable_data(this_file)

        if sendable_sexp ~= "" then
            sendable_sexp = wrap_site_token(sendable_sexp)
            seg_win = seg_talker(sendable_sexp)

            if archive_data == true then
                move_to_archive(this_file)
            else
                remove_data_file(this_file)
            end
        end
    end
end

-- ------------------------------------------------------------------------- --

function get_sendable_data(this_file)
    local cached_sexp = ""
    local status = false

    if save_data == true then
        -- check to see if there are any goodies!
        status, cached_sexp = pcall(get_file_data, this_file)

        if status ~= true then
            --  no need to log this, as there is no data file!
            print("Get file data fail, message: " .. cached_sexp)
            cached_sexp = ""
        end
    end
    --  print("cached_sexp: " .. cached_sexp)
    return cached_sexp
end

-- ------------------------------------------------------------------------- --

function get_file_data(this_file)
    -- checks to see if there is something in the file, and returns a big string!
    local big_string
    local data_file = assert(io.open(data_file_path_for_file(this_file)))
    if (data_file ~= nil) then
        big_string = data_file:read("*all")
        if (big_string:len() <= 0) then
            big_string = nil
        else
            print("Has cached datas: ", big_string:len())
            big_string = string.gsub(big_string, "\n", "") -- the newline is removed for SEG
        end
    end
    assert(data_file:close())
    if big_string == nil then
        big_string = ""
    end
    return big_string
end

-- ------------------------------------------------------------------------- --

function node_list_file_path()
    local path = ""
    path = data_path .. node_list_file
    --  print("node_list_file_path: " .. path)
    return path
end

-- ------------------------------------------------------------------------- --
function make_sub_directories()
    local command_string

    command_string = "mkdir " .. data_file_directory()
    assert(os.execute(command_string))
    command_string = "mkdir " .. archive_directory()
    assert(os.execute(command_string))
end

-- ------------------------------------------------------------------------- --

function data_file_path()
    local path = ""
    path = data_path .. data_sub_directory .. "/" .. data_file_name
    -- print("data_file_path: " .. path)
    return path
end

-- ------------------------------------------------------------------------- --

function data_file_directory()
    local path = ""
    path = data_path .. data_sub_directory
    return path
end
-- ------------------------------------------------------------------------- --

function archive_directory()
    local path = ""
    path = data_path .. archived_sub_directory
    return path
end

-- ------------------------------------------------------------------------- --

function data_file_path_for_file(this_file)
    local path = ""
    path = data_path .. data_sub_directory .. "/" .. this_file
    --  print("data_file_path: " .. path)
    return path
end

-- ------------------------------------------------------------------------- --

function archived_data_file_path()
    local path = ""
    path = data_path .. archived_sub_directory .. "/archived_" .. data_file_name
    --  print("data_file_path: " .. path)
    return path
end

-- ------------------------------------------------------------------------- --

function archived_path_for_file(this_file)
    local path = ""
    path = data_path .. archived_sub_directory .. "/archived_" .. this_file
    --  print("data_file_path: " .. path)
    return path
end

-- ------------------------------------------------------------------------- --

function save_messages(sendable_sexp)
    -- Saves the messages to the disk and clears working buffer, or does nothing if not saving
    if save_data == true and sendable_sexp ~= "" then
        print("Writing s-expression " .. sendable_sexp)
        if data_file_line_index > data_file_line_max then
            increment_cached_file()
            data_file_line_index = 0
        end

        local data_file = assert(io.open(data_file_path(), "a")) -- append mode
        data_file:write(sendable_sexp .. " " .. "\n")
        assert(data_file:close())

        data_file_line_index = data_file_line_index + 1

        clear_buffers() -- We clear the working buffer here as it's now in the file!
    end
end

-- ------------------------------------------------------------------------- --

function archive_cached()
    print("Archiving the cached data file")
    local command_string = "cp " .. data_file_path() .. " " .. archived_data_file_path() .. "_" .. time_now()
    -- print("The command to archive the data file: " .. command_string)
    assert(os.execute(command_string))
end

-- ------------------------------------------------------------------------- --

function move_to_archive(this_file)
    print("Moving the data file to archive")
    local command_string = "mv " .. data_file_path_for_file(this_file) .. " " .. archived_path_for_file(this_file) .. "_" .. time_now()
    assert(os.execute(command_string))
end

-- ------------------------------------------------------------------------- --

function increment_cached_file()
    print("Incrementing the cached data file")
    local command_string = "mv " .. data_file_path() .. " " .. data_file_path() .. "_" .. time_now()
    assert(os.execute(command_string))
end

-- ------------------------------------------------------------------------- --

function clean_cached()
    print("cleaning out the cached data file")
    local output = assert(io.open(data_file_path(), "w"))
    assert(output:close())
end

-- ------------------------------------------------------------------------- --

function delete_cached()
    print("deleting the cached data file")
    assert(os.remove(data_file_path()))
end

-- ------------------------------------------------------------------------- --

function remove_data_file(this_file)
    assert(os.remove(data_file_path_for_file(this_file)))
end

-- ------------------------------------------------------------------------- --

function file_exists(path)
    -- Return true if file exists and is readable.
    local file = io.open(path, "rb")
    if file then file:close() end
    return file ~= nil
end

-- ------------------------------------------------------------------------- --


function scandir(dirname)
    callit = os.tmpname()
    os.execute("ls -a1 " .. dirname .. " >" .. callit)
    f = io.open(callit, "r")
    rv = f:read("*all")
    f:close()
    os.remove(callit)

    tabby = {}
    local from = 1
    local delim_from, delim_to = string.find(rv, "\n", from)
    while delim_from do
        table.insert(tabby, string.sub(rv, from, delim_from - 1))
        from = delim_to + 1
        delim_from, delim_to = string.find(rv, "\n", from)
    end
    -- table.insert( tabby, string.sub( rv, from  ) )
    -- Comment out eliminates blank line on end!
    return tabby
end

-- ------------------------------------------------------------------------- --

function wrap_site_token(sexp)
    local output = "(site " .. site_token .. " " .. sexp .. ")"
    return output
end

-- ------------------------------------------------------------------------- --

function process_seg_response(response)

    local win = true
    local command = nil

    if (response ~= nil and response ~= "") then

        -- Check response wrapped by site command, e..g (site= new_site_token)
        if (response:sub(1, 7) == "(site= ") then
            local start, finish = response:find("\n", 1, PLAIN)
            local message = response:sub(1, start - 1)
            response = response:sub(finish + 1)

            -- Parse and save new site token
            save_site_token(message:sub(8, -2))
        end

        if (response:sub(1, 6) == "(node ") then
            -- an example command: (node (segmeter (relay on 0 7734)))
            local start = 7
            local finish = nil
            finish = response:find("\n", 1, PLAIN)
            if (finish ~= nil) then
                -- only want to send so trim it lke so (segmeter (relay on 0 7734))
                command = response:sub(start, finish - 1)
                if (command ~= nil) then
                    serial_client:send(command .. ";\n")
                    command = nil
                end
            end
        end

        if (string.find(response, 'ok') or string.find(response, 'nothing_processed') or string.find(response, 'site=')) then
            if (api_debug) then
                print("we have some SEG wins on response: " .. response)
            end
            -- This is to back off on the no site token situation
        elseif (response:find("discovery_not_initiatied", 1, PLAIN)) then
            print("Initiating back off on the discovery timeout")
            site_discovery_timeout()
        else
            win = false
        end
    else
        -- a nil response
        win = fase
    end

    return win
end

-- ------------------------------------------------------------------------- --

function seg_talker(sexp)

    local talker_win = true
    -- print("...in seg_talker")
    local this_resource = "/sites/" .. site_token
    if (debug) then
        print("Sending an s-expression of length: " .. string.len(sexp))
    end
    local this_response = call_seg_api(this_resource, "PUT", sexp)

    if (this_response ~= nil) then
        talker_win = process_seg_response(this_response)
    else
        print("SEG had some fail :( ")
        talker_win = false
    end

    return talker_win
end

-- ------------------------------------------------------------------------- --

function seg_payload(message)
    if string.find(content_type, 'application') then
        return ("data_post=" .. message)
    else
        return message
    end
end

-- ------------------------------------------------------------------------- --

function call_seg_api(resource, method, message, this_timeout)

    if (talk_to_seg ~= true) then
        return nil
    end

    local uri = "http://" .. web_host_name .. resource

    --  print("SEG resource: " .. uri)

    local http = require("socket.http")
    local response = {}
    local message_length = 0

    if this_timeout ~= nil then
        http.TIMEOUT = this_timeout
    end

    if (message ~= nil) then
        message = seg_payload(message)
        message_length = message:len()
    end

    local body, code, headers, status = http.request{
        url = uri,
        method = method,
        headers = {
            ["content-length"] = message_length,
            ["content-type"] = content_type
        },
        source = ltn12.source.string(message),
        sink = ltn12.sink.table(response)
    }

    if (api_debug) then
        print("Body:     ", body)
        print("Code:     ", code)
        print("Headers:  ", table_to_string(headers))
        print("Status:   ", status)
        print("Response: ", table_to_string(response))
    end

    if (body == nil) then
        if code ~= nil then
            print("Tube problems: ", code)
            if string.find(content_type, 'timeout') then
                tube_status = "disconnected"
            end
        end
        seg_status = "disconnected"
        response = nil
    else
        numerical = tonumber(code)

        if (numerical ~= nil and numerical < 400) then
            if (response == nil) then
                print("Error: No HTTP response body. Code: " .. code)
                response = code
                tube_status = "connected"
            else
                response = table.concat(response)
                print("SEG api response: ", response)
                tube_status = "connected"
                seg_status = "connected"
            end
        else
            print("SEG API fail, code: " .. code)
            tube_status = "connected"
            seg_status = "disconnected"
            response = nil
        end
    end

    return response
end

-- ------------------------------------------------------------------------- --

function read_message(message)
    local my_payload = ""
    local node_name = ""
    -- Check message wrapped by node name, e..g (node name ...)

    if (message:sub(1, 6) ~= "(node ") then
        print("SEGmeter message pecularity - doesn't start with 'node'", message)
    else
        node_name = determine_node_name(message)
        if (node_name ~= nil) then
            my_payload = get_valid_message(message)
            if my_payload ~= "" then
                add_node_atom(node_name, my_payload)
            end
        end
    end
end

-- -------------------------------------------------------------------------  --

function serial_talker()
    -- listens on the serial port and tries to send or store the data
    print("Starting the serial_talker!")
    serial_client = socket.connect(gateway_address, listening_port)
    serial_client:settimeout(serial_timeout_period) -- 0 --> non-blocking read

    local stream, status, partial
    local message = nil
    local command = ""

    while (status ~= "closed") do

        get_segmeter_data = get_data_timer()

        if (get_segmeter_data == true) then
            --  Broadcast message to get datas
            command = "(all_nodes (start_data))"
            if (debug) then
                print("Request data command: " .. command)
            end
            serial_client:send(command .. ";\n")
            get_segmeter_data = false
            command = nil
        end

        stream, status, partial = serial_client:receive(16768) -- (1024)

        if (debug) then
            if (status == "timeout") then
                print("Serial bytes received: ", partial:len())
                if partial ~= nil then
                    print("Unprocessed partial: ")
                    print(partial)
                end
            else
                print("serial_talker status is: ", status)
            end
        end

        if (partial ~= nil and partial:len() > 0) then
            -- Process individual messages, delimited by "carriage return"
            for message in partial:gmatch("[^\r\n]+") do
                -- print(message)
                if (message ~= nil) then
                    if (validate_brackets(message)) then
                        read_message(message)
                    end
                end
            end -- the gmatch check
        end

        if switch_3g == true then
            -- Broadcasts a message to all nodes to restart the 3G service.
            command = "(all_nodes (switch_3g))"
            print("Sending command: " .. command)
            serial_client:send(command .. ";\n")
            switch_3g = false
            command = nil
        end

        if (status == "timeout") then
            atoms_to_timestamped_messages()
            coroutine.yield()
        end
    end

    print("serial_talker is Now closing the serial client...")
    serial_client:close()
end

-- ------------------------------------------------------------------------- --

function my_ip()
    -- works out if the system has an IP address
    local ip_address = nil
    local ip_address_coming = false
    local line_count = 0

    local file = io.popen("ifconfig")
    local line = "init_line"

    while line do
        line = file:read()
        if line ~= nil then
            if (string.find(line, "br-")) or (string.find(line, "en1:")) then
                ip_address_coming = true
            end

            if ip_address_coming == true and line ~= nil and line_count < 4 then
                line_count = line_count + 1
                line = trim_string(line)
                if string.find(line, "inet") then
                    ip_address = line
                    break
                end
            end
        end
    end

    if ip_address ~= nil then
        print("IP address: " .. ip_address)
    else
        print("IP address is not set!")
    end

    return ip_address
end

-- ------------------------------------------------------------------------- --

function has_dns(server_name)
    my_socket = require("socket")
    local seg_ip = my_socket.dns.toip(server_name)

    if seg_ip ~= nil then
        print("DNS found SEG IP: " .. seg_ip)
    else
        print("DNS can't find SEG!")
    end
    return seg_ip
end

-- ------------------------------------------------------------------------- --

function initialise_intertubes()
    -- inits all the intertubes for the device

    local win = false
    local dns_ok = false
    local initalise_count = 1
    local switching_count = 1.0
    print("Checking the good intertubings...")

    while (win == false and initalise_count < connection_throttle) do
        print("Initialise shot " .. initalise_count)

        if production then
            if has_dns(web_host_name) ~= nil then
                print("Haz DNS - win 1 of 2")
                tube_status = "connected"
                dns_ok = true
            else
                print("Fail on DNS - 1 of 2")
            end
        else
            dns_ok = true
        end

        if dns_ok == true and seg_is_up() == true then
            print("Haz seg is_ip - win 2 of 2")
            seg_status = "connected"
            win = true
            break
        else
            print("Fail calling SEG - 2 of 2")
        end

        if win ~= true then
            print("No innertube win yet :(")
        end

        if (has_3g == true and win ~= true) then
            if ((switching_count / 6.0) == 1.0) then
                print("About to hit up boot 3G...")
                press_3g_switch()
                switching_count = 1.0
            end
        end
        os.execute("wifi")
        print("...and a 30 seconds wait to give tubes some moar time to re-init")
        print("")
        sleepy_sleep(30)
        initalise_count = initalise_count + 1
    end

    return win
end

-- ------------------------------------------------------------------------- --

function services_manager()
    local manage_counter = 0

    if has_3g == true then
        while true do
            print("In services_manager, manage_counter: " .. manage_counter)
            if manage_counter >= connection_throttle then
                manage_counter = 0
                print("In services_manager - hitting the 3g switch")
                if has_3g == true then
                    press_3g_switch()
                end
            end
            manage_counter = manage_counter + 1
            coroutine.yield()
        end
    end
end

-- ------------------------------------------------------------------------- --

function show_3g_display()
    -- Broadcasts a message to display the 3G screen.
    print("Displaying up the 3G service..")
    display_3g = true
    coroutine.resume(coroutine_serial)
end

-- ------------------------------------------------------------------------- --

function press_3g_switch()
    print("Hitting up the 3G switch and waiting for serial to send command")
    switch_3g = true
    coroutine.resume(coroutine_serial)
    sleepy_sleep(20)
end

-- ------------------------------------------------------------------------- --

function reboot_segbox()
    print("Initiating reboot!")
    os.execute("reboot")
end

-- ------------------------------------------------------------------------- --

function sleepy_sleep(seconds)
    -- uses the socket for a timeout
    -- socket.sleep(seconds)
    -- uses the OS...
    os.execute("sleep " .. tonumber(seconds))
end

-- ------------------------------------------------------------------------- --

function serial_timeout()
    -- uses the serial handler as a timout

    serial_client = socket.connect(gateway_address, listening_port)
    serial_client:settimeout(serial_timeout_period) -- 0 --> non-blocking read

    local stream, status, partial

    while (status ~= "closed") do
        stream, status, partial = serial_client:receive(16768) -- (1024)

        -- output_message = "...in timeout with status: " .. status
        -- print(output_message)

        if (status == "timeout") then
            break
        end
    end

    serial_client:close()
end

-- ------------------------------------------------------------------------- --

function merge_tables(first_table, second_table)
    if first_table == nil then
        if (type(second_table) == 'table') then
            first_table = second_table
        else
            first_table = {}
        end
    else
        if second_table ~= nil then
            if (type(second_table) == 'table') then
                for key, value in pairs(second_table) do
                    first_table[key] = value
                end
            end
        end
    end

    return first_table
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

    return (result)
end

-- ------------------------------------------------------------------------- --

function url_encode(str)
    if (str) then
        str = string.gsub(str, "\n", "\r\n")
        str = string.gsub(str, "([^%w ])",
            function(c) return string.format("%%%02X", string.byte(c))
            end)
        str = string.gsub(str, " ", "+")
    end
    return str
end

-- -------------------------------------------------------------------------  --

function atoms_to_timestamped_messages()
    local timestamped_atoms
    for node_name, atoms in pairs(node_atoms) do
        timestamped_atoms = timestamp_node_atoms(node_name, atoms)
        add_node_message(node_name, timestamped_atoms)
    end
    node_atoms = {}
end

-- -------------------------------------------------------------------------  --

function timestamp_node_atoms(node_name, atoms)
    if timestamp_data == true then
        return " (node " .. node_name .. " " .. time_now() .. " " .. atoms .. ")"
    else
        return " (node " .. node_name .. " ? " .. atoms .. ")"
    end
end

-- -------------------------------------------------------------------------  --

function determine_node_name(message)
    local this_node = nil
    local start, finish = message:find('?', 7, PLAIN)
    if (start ~= nil) then
        this_node = message:sub(7, start - 2)
    end
    if (this_node ~= nil) then
        add_node(this_node)
    end
    return this_node
end

-- ------------------------------------------------------------------------- --

function get_node_message(node_name)
    local output = nil
    for key, value in pairs(node_messages) do
        if (key == node_name) then
            output = value
            break
        end
    end
    return output
end

-- ------------------------------------------------------------------------- --

function get_node_atoms(node_name)
    local output = nil
    for key, value in pairs(node_atoms) do
        if (key == node_name) then
            output = value
            break
        end
    end
    return output
end

-- -------------------------------------------------------------------------  --

function output_seg_message_for_nodes()
    local output = ""
    for key, value in pairs(node_list) do
        output = output .. output_seg_message_for_node(value)
    end
    return output
end

-- -------------------------------------------------------------------------  --

function output_seg_message_for_node(node_name)
    output = ""

    for key, value in pairs(node_messages) do
        if (key == node_name) then
            output = output .. value
            break
        end
    end
    return output
end

-- -------------------------------------------------------------------------  --

function add_node_atom(node_name, addable_atom)
    local current_atoms = nil
    current_atoms = get_node_atoms(node_name)
    if current_atoms == nil then
        current_atoms = ""
    end
    node_atoms[node_name] = current_atoms .. addable_atom
    -- print("node_atoms[node_name] is: " .. table_to_string(node_atoms[node_name]))
end

-- -------------------------------------------------------------------------  --

function add_node_message(node_name, addable_message)
    local current_message = nil
    current_message = get_node_message(node_name)
    if current_message == nil then
        current_message = ""
    end
    node_messages[node_name] = current_message .. addable_message
end

-- -------------------------------------------------------------------------  --

function get_valid_message(message)
    -- takes a valid message, and removes the payload for a node
    local valid_message = ""
    processable = string.sub(message, 2, -1) -- At 2, to not include the first part of the node detail from the message

    for result in string.gmatch(processable, "%b()") do
        -- print(result)
        valid_message = valid_message .. result
    end

    -- print("The Validified message is: " .. valid_message)
    return valid_message
end

-- -------------------------------------------------------------------------  --

function validate_brackets(message)

    -- checks that a message has balanced brackets
    local valid = false
    for result in string.gmatch(message, "%b()") do
        if (message == result) then
            valid = true
            -- print("Found valid message: " .. message)
        end
        break
    end

    if valid ~= true then
        -- print("Not valid message: " .. message)
    end

    return valid
end

-- ------------------------------------------------------------------------- --

function confirm_time_is_ok()

    print("Attempting to confirm_time_is_ok...")
    local counter = 0
    time_is_ok = false

    while time_is_ok == false do
        counter = counter + 1
        print("Time check iteration: " .. counter)

        time_is_ok = check_time()

        if seg_status == "disconnected" then
            break
        end
        if (time_is_ok == true) then
            print("The time on the SEGbox is ok")
            seg_status = "connected"
            tube_status = "connected"
            break
        else
            print("The time from SEG is fail, Going to try and set now based on the SEG time")
            set_time()
            sleepy_sleep(10)
        end
    end
end

-- -------------------------------------------------------------------------- --

function time_now()
    return os.date("%Y-%m-%dT%H:%M:%S")
end

-- ------------------------------------------------------------------------- --

function check_time()
    local this_time = time_now()
    local win = false
    local search_result = false
    local this_resource = "/api/check_time?remote_time=" .. url_encode(this_time)

    output = call_seg_api(this_resource, "GET", "", 5)

    if (output ~= nil) then
        -- print(table_to_string(output))
        search_result = string.find(output, 'ok')
        if (search_result ~= nil) then
            win = true
        end
    end
    return win
end

-- ------------------------------------------------------------------------- --

function set_time()
    --  sets the OpenWRT linux time - has fail on osx...
    local this_time = time_now()
    local win = false
    local search_result = false
    local this_resource = "/api/seg_time"

    output = call_seg_api(this_resource, "GET", "", 5)

    if (output ~= nil) then
        local seg_response = table_to_string(output)

        --    print("Time response from SEG: " .. seg_response)

        local seg_time = ""
        for character in string.gmatch(table_to_string(output), "[^%a^(^)^'^_]") do
            seg_time = seg_time .. character
        end

        --    remove the space at the start
        seg_time = string.sub(seg_time, 2)

        print("SEG time: " .. seg_time)

        local executable = "date -s " .. seg_time
        --    print("Command for the execution: " .. executable)
        os.execute(executable)
    end
    return win
end

-- ------------------------------------------------------------------------- --

function seg_is_up()
    local win = false
    local this_resource = "/api/is_up"

    output = call_seg_api(this_resource, "GET", "", 25)

    if (output ~= nil) then
        tubes_connected = true
        seg_connected = true
        win = true
    else
        tubes_connected = false
        tubes_connected = false
    end

    return win
end

-- ------------------------------------------------------------------------- --

function new_node_list()
    local data_file = nil
    local path = node_list_file_path()
    data_file = assert(io.open(path, "w"))
    return data_file
end

-- ------------------------------------------------------------------------- --

function open_node_list_read()
    local data_file = nil
    local path = node_list_file_path()
    data_file = assert(io.open(path))
    return data_file
end

-- ------------------------------------------------------------------------- --

function get_node_list()
    -- gets the node list
    local data_file = nil

    status, data_file = pcall(open_node_list_read)

    if status ~= true then
        --    print("Can't open node list file, message: " .. data_file)
        print("Attempting to make a new node list!")

        status, data_file = pcall(new_node_list)
        if status ~= true then
            print("EPIC failure opening the node list file, message: " .. data_file)
            return nil
        end
    end


    local count = 0
    local temp_node_list = {}

    --  print("About to read the node_list data_file...")

    if (data_file ~= nil) then
        while true do
            local line = data_file:read()
            if line == nil then break
            end
            table.insert(temp_node_list, line)
            count = count + 1
        end
    end
    --  print("The nodes on file are: " .. table_to_string(temp_node_list))
    assert(data_file:close())
    return temp_node_list
end

-- ------------------------------------------------------------------------- --

function update_node_list()
    local temp_node_list = nil
    --  print("Getting the node list on file...")
    temp_node_list = get_node_list()

    if temp_node_list ~= nil then
        --    print("Merging the node lists...")
        node_list = merge_tables(node_list, temp_node_list)

        --    print("The Merged: " .. table_to_string(node_list))

        local data_file = assert(io.open(node_list_file_path(), "w"))
        if (data_file ~= nil) then
            for key, value in pairs(node_list) do
                data_file:write(value .. "\n")
            end
        end
        assert(data_file:close())
    end
    return node_list
end

-- ------------------------------------------------------------------------- --

function add_node(node)
    --  print("Looking for the node on file: ", node)
    if has_node(node) ~= true then
        table.insert(node_list, node)
        if (save_data == true) then
            print("About to update_node_list with: " .. table_to_string(node_list))
            status, message = pcall(update_node_list)
            if status ~= true then
                print("Some failure updating the node_list, message: ", message)
            end
        end
    end
end

-- ------------------------------------------------------------------------- --

function has_node(node_name)
    local output = false
    for key, value in pairs(node_list) do
        if (value == node_name) then
            output = true
            break
        end
    end
    return output
end

-- ------------------------------------------------------------------------- --

function rapid_heartbeat_discover()
    -- beats quickly on site_unknown at startup to kick off discovery
    
    if site_token == "site_unknown" then
        print("Now initating rapid heartbeat discover for site_token: site_unknown")
        local heartbeats = 0

        while heartbeats < rapid_heartbeat_timeout do
            if site_token == "site_unknown" then
                send_event_heartbeat()
                print("Having a sleep after a discovery heartbeat " .. heartbeats)
                sleepy_sleep(4)
            else
                print("Finishing up the discovery heartbeats on beat: " .. heartbeats)
                break
            end
            heartbeats = heartbeats + 1
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
            send_event_heartbeat()
        end

        coroutine.yield()
    end
end

-- ------------------------------------------------------------------------- --

function send_event_heartbeat()
    message = "(node " .. segbox_name .. " ? (status heartbeat))"
    seg_talker(wrap_site_token(message))
end

-- ------------------------------------------------------------------------- --

function send_event_boot()
    message = "(node " .. segbox_name .. " ? (status boot " .. version .. "))"
    seg_talker(wrap_site_token(message))
end

-- ------------------------------------------------------------------------- --

function save_site_token(new_site_token)
    if (new_site_token ~= site_token) then
        site_token = new_site_token

        local output = assert(io.open("config_segbox.lua", "a"))
        output:write("  site_token = \"" .. site_token .. "\"\n")
        assert(output:close())

        print("New site_token saved: " .. site_token)
    end
end

-- ------------------------------------------------------------------------- --

next_data_time = 0

function get_data_timer()

    local output = false
    if (debug) then print("In the get_data_timer")
    end

    if (next_data_time == 0) then
        next_data_time = os.time() + data_rate
        output = true
    else
        if (os.time() > next_data_time) then
            next_data_time = os.time() + data_rate
            output = true
            -- print("Time is right to send data")
        end
    end

    return output
end


-- ------------------------------------------------------------------------- --

site_discovery_timer = 0

function site_discovery_timeout()
    if (debug) then
        print("site_discovery_timeout running:")
    end

    if (site_discovery_timer == 0) then
        site_discovery_timer = os.time() + site_discovery_gracetime
    else
        if (os.time() > site_discovery_timer) then
            talk_to_seg = false
            print("Just stopped talking to seg with Discovery timeout")
        end
    end
end

-- ------------------------------------------------------------------------- --

function set_web_host()
    if production == true then
        web_host_name = "api.smartenergygroups.com"
    else
        -- Some other servers useful for testing things
        web_host_name = "192.168.110.189:3000"
        web_host_name = "localhost:3000"
        web_host_name = "rodent_api.smartenergygroups.com"
    end

    print("Using the service: " .. web_host_name)
end

-- ------------------------------------------------------------------------- --

function trim_string(s)
    -- trims whitespaces from the end and start of a string.
    return (s:gsub("^%s*(.-)%s*$", "%1"))
end
