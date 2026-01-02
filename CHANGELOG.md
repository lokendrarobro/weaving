# Changelog

## v4.1.0

- ### Data logging and management
- `Change` - `GSM value supported from greater then 0 to 2000`
- `Change` - `The fabric width range from 10-1000`
- `New` - `Store the Launch file params in database AppConfig and Camconfig`
- `Fix` - `Round off floating point number upto 2 decimal places`


## v4.0.0

- ### System Alerts and Logs
- `New` - `Notify popup when continue an existing roll.` 

- ### Real-Time Machine Control
- `New` -  `Database modification, primary-key as BIGINT, composity key for tables (job, body, defect).`
- `New` -  `Cropped defect images with margin.`
- `New` -   `save_all_defect_image initialized by params.json.`
- `New` -   `Roll continuity functionality`
- `New` -   `Defected Frame annotation json stored.`

- ### Data logging and management
- `change` - `Database modification, primary-key as BIGINT, composity key for tables (job, body, defect).`
- `Fix` - `Stopping Command issued -ve value.`
- `Fix` - `False system log adding on fail of pulse count value.`
- `Change` - `Error Code into Logging Numbers.`
- `New`    -  `User activities log added.`

- ### User Interface and Experience
- `Change` - `Remove meter label from UI on P, S, T cards.`
-  `New`   -  `Software Build Version add in UI.`


## v3.4.9
 
- ### System Alerts and Logs
- `New` - `Buzzer and side notification when batch count is completed.`
- `New` - `Buzzer and popup when work order is completed.` 

- ### Real-Time Machine Control
- `Change` - `Configurable buzzer on defected body at "start" or "end".`
- `Change` - `Buzzer beep pattern changed 4 to 2.`
- `New` -    `Top/Bottom functionality introduced in Auto-Cut.`
- `Change` - `Configurable force to cut fix cut length.`
- `Change` -  `Configurable stopping duration on defected body.`
 
- ### Data logging and management
- `New` - `Store work order configuration in Database.`
- `New` - `defect_status column added in defect log.` 
- `New` -  `secondary and tertiary cut length in roll job log.` 
 
- ### Roll Management & Control
- `New` - `Work order concept, create new and complete existing work order forcefully.`


- ### User Interface and Experience
- `Change` -  `Show primary, secondary, tertiary, and defective body cards with count, length, and meters (excluding defective) based on cutting mode.`
-  `New`   -   `Table displays timestamps for different cutting cycle states.`

## v3.4.8

- ### Autocut Speed Improvement.
   - `New` - Cycle start electrical mode support added.

## v3.4.7
 
- ### System Alerts and Logs 
  - `Fix` - Random `cam2Stopper` value issue.  
  - `Fix` - Random camera pulse counter value at roll start.  
  - `Change` - More lenient logic for popups and machine stoppage on sensor and encoder errors.  
  - `New` - Popup on app start for license expiry warning based on remaining days.  
  - `New` - Popup alert for database connection loss.  
  - `New` - Notification for cutting deviations from the planned length, with a popup if 10 consecutive cuts deviate.

-  ### Real-Time Machine Control
   - `Fix` - Punch blocking not happening in Auto-Cut, secondary bodies case.
   - `Change` - Buzzer sound only at the end of defected body, removed in case tertiary and secondary body.

-  ###  Data Logging & Management
   - `Fix` - Good-body count increases only case of actual cutting length is within ±10% of the planned cut length, else the body 
             count as defected. 

- ### Inspection Settings & Parameters
   - `New` - Sensitivity for X & Y independently for each defect category.
   - `New` - Defect Category wise network threshold.

- ### Roll Management & Control
   - `New` - Configurable Barcode Scanning Option to enter Roll details.


## v3.4.5

- `New` - Python Data Compute node added which compute data for the dashboard.

## v3.4.4

- `New` - Displaying current and required sensitivity value on stopping window for defects present in the current body.
- `New` - Cut the smaller length if the defective cutlength is 80% of primary cutlength.

### v3.4.3

- `Change` - OK button added in PLC Connection Broken Popup

### v3.4.2

- `Fix` - Popup bugs are fixed (back encoder error , front encoder error are not coming when the dancer sensor works fine)
- `Fix` - Cutter sensor not working error is logged just one
- `Fix` - Even in system_in_front_of_dancer mode, values are calculated and shown in front
- `Fix` - Back Encoder not working (nib roller is not down) error is not coming in system installed in front of dancer
- `FIx` - Camera Counter of Master is reset when camera is connected to avoid garbage value


### v3.4.1

- `Fix` - Offset value is not converted into mm before add it to cam to stopper for validating cutlength is not bigger then cam to stopper in autocut and recut bug.

### v3.4.0
- `Fix` - Data issue with semi auto is fixed (body id for defect is wrong also stopping command issue is wrong so the kg saving calculation is totally wrong in smaller cutlength).
- `Fix` - Ticket NO: code crashing when trying to log frame miss match.
- `Change`- Camera Not Connected restart pop time is increased from 10s to 2 min. popup will appear after 2min of launch if cameras not connect. meanwhile we will see the loading with how many cameras are not connected.
- `New` - Loading will show the information about how many cameras not connected.
- `New` - Loom ID and Roll Length input fields added.
- `Change` - Dancer Sensor Not ON pop limit increased to 40m. will not show the pop until 40m reached.
- `New` - Recipe Modification log added. we can only see recipe changed what is changed is not stored.
- `New` - Logging component State data to kvp_component state table. can get duration of state of realworld map (ON_IDEAL, ON_ENGAGED) 
- `New` - Operator/Admin button added will ask password to change from operator to admin with admin access exposure, threshold can be modified and recipe editor and autocut config page can be accessed.
- `New` - Show last popup button added which shows the last stopping window in all modes.
- `New` - Floating license added which acquire a license from local server. local server configuration is must.
- `Change` - Tables name modified as per kvp (fibc_<table_name>)
- `New` - State Logs added (ON_IDLE, ON_ENGAGED). INFO008 (ON_IDLE), INFO011 (ON_ENGAGED)
- `New` - Ignore Inspection Turn On Logged.
- `New` - PLC Connection error popups. Code `COM002` (For both KWIS PLC and Cutting Machine PLC). OK option exists, it keeps coming if connection stays broken.
- `New` - Popup & Buzzer for Cutting Machine on But System Not Engaged. Code `INFO010`. Comes if machine running for 30M without system on. We read values every 5 seconds. We use front encoder for reading. Once it comes, it comes again and again every 10mtr machine runs.
- `New` - Launch file params for `is_just_cut_connection_done` based on which cutting mode option will be shown in front or not
- `New` - Auto detect of cutting machine PLC Connection. If not connected, then option not seen. Might need to refresh in order for the variable to populate correctly.
- `New` - Panel Item & Separate Page for Testing Ai-CutMaster Added with Cut Lenght, Cycle Start, Connection details, Connection Status & Machine Ready. 

### v3.3.1

- `New` - Slave address setting option added to kwis and cutting machine plc.
- `New` - Tcp mode added for cutting machine plc.


### V3.3.0-rc3

- `New` - Sensitivity based recipe added with Changes to JSON. Need "groups" in config JSON now.
- `New` - Dancer Sensor and Cutter Sensor classes added
- `Refactor` - PLC unused stuff removed
- `Fix` - Camera Indicators to work independently.
- `Fix` - Move forward mm changed to int from uint to handle reverse.
- `Change` - `Systemlogger` renamed to `MySQLClient` 
- `Change` - Slash and other special characters not allowed in Roll ID
- `New` - Image Storage of Cropped Defect images now happens in KVP Format
- `Fix` - Spout blocking through Robro PLC in auto modes also
- `New` - `dancer_control_on_just_cut_mode` & `spout_and_cutter_control_on_just_cut_mode` params for justcut mode added.
- `New` - Stopping Card's different functionalities / visibilities in different modes.
- `Change` - Buzzer Class changed. Timing based buzzers added.
- `New` - System Logs added with msg and msg_code. Need KVP SystemLog with JSON Schema for the same.
- `Change` - Recipe Tables editing from front.
- `Change` - All Sqlite3 things including .py files removed. Everything through MySQL.
- `New` - Launch file option of is_cycle_start_register or not. (Because JP programmer made cycle start a register. So, we're writing 1 there.)
- `Fix` - System won't run when cameras are not ready
- `New` - Launch file option of is_machine_ready_register or not.
- `New` - Launch file option of is_spout_status_register or not.
- `New` - Launch file option of spout_control_via_cutting_machine_com (Because JP programmer made in auto cut mode spout on/off needs to controlled via kwis-fibc software so setting punch status when roll start).
- `New` - Bottom panel button added in semiauto mode (Machine will not stop or punch block only give buzzer at end of the body).
- `New` - Fully ignore disabled defect button added (System will not send the defect information to main node if the defect is disabled).
- `Fix` - Machine ready in autocut/recut is changed to Raising edge.
- `New` - Plc program version reading added (You can see the plc program version in the Terminal).
- `New` - Added a current_body_processed_auto_reset_mode (if it is true plc will not reset the current body processed).
- `New` - Two More classes added in Recipe (manual_marking, stickers).
            


### V3.3.0-rc2

- `New` - System Logger transfer from `sqlite3` to `MySQL`.
- `New` - System Logger GUI connected with Node and Angular based Application using Sidebar of GUI.
- `Refactor` - `SystemLogger` class added for logging system events.
- `New` - `ValueBasedIndicator` class added for controlling the value-based indicators.
- `New` - `TimeBasedIndicator` class added for controlling the time-based indicators.
- `Refactor` - Refactor the unused notification with system indicators and system health.
- `New` - System Diagnostic with numbers GUI added for the system health with Indicators.
- `New` - `JustCut` mode added for the cutting process.
- `Refactor` - Next Stopping position card on GUI to update UX with body count and defect count in `JustCut`, `AutoCut` and `ReCut`.
- `New` - `Tertiary cut length` added for the cutting process.
- `New` - Multiple camera images added in GUI for proper visualization.
- `Fix` - Reporting for the defect body type fixed.
- `New` - `Cutter` in normal closed mode added for the cutting process.
- `Fix` - Stopping window with different size of the defect fixed.
- `Know Bug` - During changing of mode sometime `isMachineAvailable` is always `True` which is keep updating primary body count.

### V3.3.0-rc1

- `Fix` - Remove the `PLCComm` class `processPLCRequest` function.
- `Refactor` - Rearrange the GUI layout.
- `New` - System Health GUI added.
- `New` - `CuttingManager` class added for planning the cutting process.
- `New` - `CuttingMachinePLCComm` class added for managing the cutting machine PLC communication.
- `New` - `AutoCut`, `ReCut` and `SemiAuto` mode added for the cutting process.
- `New` - `BuzzerControl` class added for controlling the buzzer.
- `New` - `InductiveSensor` class added for controlling the inductive sensor.
- `Known Bug` - Hard coded plc address in `CuttingManager` class.

### V3.2.1

- `Fix` - Panel Position overflow issue fixed.

### V3.2.1-rc3

- `Fix` - Immediate stopping with ignore inspection is fixed.
- `Fix` - The problem with the PLC value conversion from `uint16` to `uint64` is fixed.
- `New` - Warning popup added when the `dancer` sensor is not working.
- `New` - Warning popup added when `cut_length` is greater than `cam2stopper_distance`.
- `New` - Add PLC communication class and if disconnected then popup added.
- `New` - Popup for fabric moved under the cutter > 80 meters added.
- `New` - Update the cut length limit to 7 meters.
- `New` - `Release Candidate` added in the version number for pre-release.
- `Fix` - Add 100 millisecond sleep when cutter turn on to make sure the current body process should be zero.

### V3.2.1-rc1/rc2

- `Fix` - The problem with the `Request For Stopping` with larger cutting length is fixed.
- `Fix` - The problem with the `Request For Stopping` with cut length greater than cam to stopper distance is fixed.
- `Fix` - Dependency on the first panel_position for the stopping position is removed.
- `Fix` - Refresh on GUI with hard cache clear added.
- `Fix` - Inconsistency of line drawing in stopping modal image borderline.
- `Fix` - `Next Stopping` on GUI corrected with immediate publishing of the next stopping position.
- `Fix` - Panel Pos appearing before the camera line is fixed.
- `Remove` - Alert for queue size mismatch removed and added command line logs.
- `Fix` - The problem with `fabric_moved_under_camera` pulse when the service call is not successful, is fixed.
- `Fix` - The problem with the `threshold` on GUI with `lower_threshold` for saving image is fixed.
- `New` - `dancer_end2stopper_distance_mm` plot added in `getWorldMap()` function.
- `New` - Alert popup added when `current_body_processed` is greater than twice of `cut_length` as `cutter` sensor is not working.

### V3.2.0

- `Refactor` - Weaving Inspection Node is now divided into `WeavingInspectionNode` and `RealWorldMapNode` for serial execution of operations.
- `Refactor` - `PLCComm` class moved to `RealWorldMapNode` for serial execution of operations.
- `Refactor` - `reportingNode` for serial execution of operations using a queue.
- `Fix` - `threshold` and `exposure` problems fixed in GUI.
- `Fix` - The problem with the `Stopping Modal` being empty is fixed.
- `Fix` - Double Layer `Cam Names` on UI fixed.
- `Fix` - False Positive Icon on UI fixed.
- `New` - Dynamically loading of Camera information from the backend added.
- `New` - `system_installed_in_front_of_dancer` added for Double Layer, if `true` then no need to calculate `cam2stopper_distance`.
- `New` - `distance_from_bottom_to_top_camera_mm` added for Double Layer, the distance between the bottom camera and top camera used to stop the defect accurately.
- `New` - `min_defect_area_px` added in `config/<project_id>.json`, if the defect area is less than this value, then it will be ignored.
- `New` - Alert for queue size mismatch added.
- `Fix` - Bugs with the reporting table fixed which contains similar entries on date change, pagination not proper, and select row not working.
- `New` - `dancer_end2stopper_distance_mm` added in the launch file for handling an accurate position of the defect before the dancer and after the dancer.
- `Fix` - camera info sync and main info sync issue fixed.
- `Fix` - The Plotting of a defect in a panel on the stopping window is displayed with offset removed.
- `Known Bug` - Sometimes Defects are getting stuck on the top.
- `Known Bug` - Stopping Positions inaccuracy exists and dependency on the Dancer Sensor Continues.
- `Known Bug` - Defects move in the World Map when the body is processed continuously.
- `Known Bug` - Sometimes during lunching the image node is dying.

### V3.1.1

- `Fix` - Launching of `datadatabaseOperationsNode.py` Node fixed.
- `Fix` - Timeout removed from `datadatabaseOperationsNode.py` Node.
- `Fix` - Add `gcp` folder in `kwis` installation.
- `Fix` - Add `GCP` service account key in `gcp` folder.
- `New` - Add `datadatabaseOperationsNode.py` Node with new table record as `detailed_roll_log`.
- `Fix` - Fix the `databaseOperationsNode.py` file with `chmod +x` command.
- `Remove` - Remove the `cloudDataNode.py` file.
- `Known Bug` - Sometimes Defects are getting stuck on the top.
- `Known Bug` - Stopping Positions inaccuracy exists and dependency on the Dancer Sensor Continues.
- `Known Bug` - Defects move in World Map when the body is processed continuously.

### V3.1.0

- `New` - License Expiry Date on GUI added.
- `Fix` - Threshold value and exposure value sync issue fixed.
- `Fix` - Weaving node job set acknowledgment added.
- `Fix` - Random turn on of green light and punch block fixed.
- `New` - GUI cross button added in the input field to clear the input.
- `Fix` - Max check for offset (can't be > 2m) and cut length (can't be >6m) added.
- `Fix` - click on defect-identified images, if there is not any image there, a big white background comes.
- `Fix` - Sometimes the stopping window does not show the defect at the bottom.
- `Known Bug` - Sometimes Defects are getting stuck on the top.
- `Known Bug` - Stopping Positions inaccuracy exists and dependency on the Dancer Sensor Continues.
- `Known Bug` - Defects move in the World Map when the body is processed continuously.

### V3.0.0

- `New` - Changed the structure of recipes inside `config/<project_id>.json`. NOT BACK COMPATIBLE.
- `New` - Added Recipe JSON Editor and Refresh Button on GUI.
- `New` - Added YOLO-V8 based inference. .engine and .names file needed. `Darknet` and `TKDNN` Still compatible. Need to check `cuda`, `tensorrt` and other compatibilities.
- `New` - Added a new WorldMapDefect class. This is not good since another defect class exists. Need to merge.
- `New` - Added reading of Cutter Sensor through `Read Input Bits` of `Modbus` Class.
- `New` - Changed the structure of `RealWorldMap` and made it with a single vector.
- `New` - Added cloud based image collection node added Which will collect images from Zip and send to cloud.
- `New` - Added cloud based reporting node added which filter DB and send to cloud.
- `Fix` - Database issue fixed.
- `Fix` - Visualization for Stopping Modal. Still needs further improvement.
- `Fix` - Threshold update in front
- `Fix` - System not restarting / powering off.
- `Fix` - Recompilation issue resolved for `CMakeLists.txt`.
- `Fix` - GUI based Job validation added, Ignore Inspection moved to settings.
- `Fix` - Roll Start and Roll End buttons converted to toggle buttons.
- `Fix` - Entry to DB now only possible if `Roll Start` is pressed as well as inspection is ignored if `Roll Start` isn't pressed.
- `Fix` - Code Dieing issue fixed.
- `Fix` - Camera Counter is reset on `Roll Start`.
- `Fix` - Stopping Modal is not showing any defect.
- `Fix` - Light Turned Off/On is now in sync with GUI toggle button.
- `New` - GUI based camera names added for better fabric visualization added.
- `Fix` - Defect Mismatching issue fixed.
- `Fix` - On System Reset dieing issue fixed.
- `New` - Proper Acknowledgement on Job set added.
- `New` - Automatic Wait till the camera is ready added.
- `Fix` - Continuos Warning popup now replaced with single popup.
- `Fix` - on stopping defect removed.
- `Fix` - Stopping window closing controlled by Cutter Action as well as 500 mm distance after stopping.
- `New` - Loader for automatic recipe loading added.
- `New` - System Logs added.
- `Fix` - Proper License message added when license is expired.
- `Fix` - Problem with Recipe loading in `RealWorldMap` fixed.
- `New` - `Start`/`End` Job button disabled when the system is not ready.
- `Known Bug` - Sometimes Defects are getting stuck on the top.
- `Known Bug` - Stopping Positions inaccuracy exists and dependency on Dancer Sensor Continues.
- `Known Bug` - Sometimes the stopping window is not showing the defect in bottom.
- `Known Bug` - click on defect identified images, if there is not any image there , a big white background comes.
- `Known Bug` - Defects move in World Map, when the body is processed continuously.

### V2.1.0

- `New` - `CMakeLists.txt` add dependencies for generated custom messages.
- `New` - Mode for stopping at every first defect added.
- `New` - Toggle switch for `Stop at last defect` added.
- `New` - Mode Toggle switch with persistency added.
- `New` - Visualization for both `Stop at first defect` and `Stop at last defect` added.
- `Remove` - Defect image map removed from `WeavingInspection` class.

### V2.0.0

- `Improvement` - recipes file will be read from `/config`. It should be `/config/<project_id>.config`
- `Remove` - Option of additional inference when tiling enabled removed.
- `Refactor` - KWIS code refactored.
- `Remove` - Old KWIS database removed. Now database will be stored inside `/db/<project_id>.sqlite` folder.
- `Refactor` - Singleton class notification refactored.
- `New` - `Defect Map` replaced with `RealWorld Map`.
- `Remove` - Removed the `Queue` defect logic with `Array` defect logic.
- `Refactor` - Stopping logic replaced with `Array` defect logic.
- `New` - `Doxygen` documentation added.
- `New` - Defect cutting logic for last panel added.
- `New` - API for KWIS logs added.
- `New` - GUI change such as `Roll Change` now become `Roll Start` and `Roll End` added with `Ignore Inspection` option.
- `New` - `KWIS` database added with `Roll`, `Job`, `Defect`, and `Body` tables.
- `New` - `KWIS` roll, job, defect, and body type for database management added.
- `New` - `WorldMap` added to map the defects in real world as cutter origin.
- `New` - GUI for multiple database visualization with single page added.
- `New` - JSON from GUI to Backend added.
- `Remove` - The cdn link for `KWIS` GUI replaced with local link.
- `New` - `KWIS` `CMakelists.txt` installation script added.
- `New` - `KWIS` GUI Date and Time added.
- `Remove` - `KWIS` on stopping defect zooming removed.

### V1.2.1

- `Fix` - KWIS brightness calculation fixed.
- `Fix` - KWIS version number on the main screen fixed.
- `Fix` - KWIS Database Job entry related bug fixed.

### V1.0.0

- `New` - KWIS product changelog added.
- `New` - Stopping distance calculation done by PLC.F.ID + Pulses and Cam.F.ID + Pulses.
- `New` - Cam Counter and PLC reset every time dancer sensor turns on.
- `Fix` - Logic of Distance between defects fixed.

#### Guide

- < `Fix` / `Improvement` / `New` / `Refactoring` > - < `Description` >.
