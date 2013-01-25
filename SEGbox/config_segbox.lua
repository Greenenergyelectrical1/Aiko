-- ------------------------------------------------------------------------- --
-- segbox.lua
-- ~~~~~~~~~~~~~~~~
-- Please do not remove the following notices.
-- Copyright (c) 2012 Smart Energy Groups Pty. Ltd.
-- Version: 2.0
-- ------------------------------------------------------------------------- --
-- General controls
-- ------------------------------------------------------------------------- --

  timestamp_data  = true              -- if false, data is not timestamped, and time sycnc on startup is not active.
  has_3g          = false             -- switches a 3g service on and off
  production      = true              -- picks the server
  data_rate       = 45                -- seconds default, to be updated from the cloud
  save_data       = true             -- to persistently save data, otherwise RAM cached
  -- data_path       = "/mnt/memories/"  -- location of data files NAND memories is best :D
  -- data_path       = "dataz/"       -- location of data files dev locale
  data_path       = "/tmp/"           -- location of data files for RAM based locale
  -- data_path = "/Users/samotage/dev/seg/branches/seg_ruby_187/SEGbox/segbox/"
  archive_data    = false             -- copies and archives stored data once sent to SEG, for big NAND drives!


-- ------------------------------------------------------------------------- --
-- Smart Energy Groups site_token = Your unique identifier for the web service API
-- Note site_unknown is a special token for discovery of segboxes.
-- SEGbox will update this file on discovery
-- ------------------------------------------------------------------------- --
  segbox_name = "segbox_vtwo"
  site_token = "site_unknown"
