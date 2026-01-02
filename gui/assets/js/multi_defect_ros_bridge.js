// ===============================
// AUTHOR      : Himanshu Sharma , Aman Shrivastava
// CREATE DATE : July 23, 2021
// PURPOSE     : Act as a bridge between Javascript web app and other ROS nodes
// VERSION     : V1.0.1
// ===============================

// ******
// Initiate the ROS Connection
// ******
$(document).ready(function () {
  let system_log = "";
  let previous_defect_frame_id = "";
  let previous_defect_frame_id_1 = "";
  let previous_defect_frame_id_2 = "";
  let previous_defect_frame_id_3 = "";
  let rollStateButton;
  let adminState = "false";
  let barcode_config = "";
  let work_orders = "";
  // localStorage.setItem('adminState', adminState);
  var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
  });

  let defectMapImageURL = null; // track for cleanup
  let defectImageURL = null; // track for cleanup
  let defectStopImageURL = null; // track for cleanup
  let defectImageURLs = [];

  function b64toBlob(b64Data, contentType = "image/jpeg", sliceSize = 512) {
    const byteCharacters = atob(b64Data);
    const byteArrays = [];

    for (let offset = 0; offset < byteCharacters.length; offset += sliceSize) {
      const slice = byteCharacters.slice(offset, offset + sliceSize);
      const byteNumbers = new Array(slice.length);

      for (let i = 0; i < slice.length; i++) {
        byteNumbers[i] = slice.charCodeAt(i);
      }

      byteArrays.push(new Uint8Array(byteNumbers));
    }

    return new Blob(byteArrays, { type: contentType });
  }

  ros.on("connection", function () {
    class RosBridgeForJS {
      constructor(obj) {
        this.obj = obj;
        for (const [topicName, value] of Object.entries(this.obj)) {
          const { topicType, type } = value;
          this.obj[topicName] = {
            ...this.obj[topicName],
            pointerToRosObj: new ROSLIB.Topic({
              ros: ros,
              name: topicName,
              messageType: topicType,
            }),
          };
          if (type === "sub") {
            const { callback } = value;
            this.obj[topicName].pointerToRosObj.subscribe(callback);
          }
        }
      }
      publish(topicName, message) {
        const { type } = this.obj[topicName];
        if (type === "pub") {
          message
            ? this.obj[topicName].pointerToRosObj.publish(message)
            : this.obj[topicName].pointerToRosObj.publish();
        }
      }
    }

    class RosJSParam {
      constructor(obj) {
        this.obj = obj;
        for (const [paramName, { callback }] of Object.entries(this.obj)) {
          const param = new ROSLIB.Param({
            ros: ros,
            name: paramName,
          });
          param.get((value) => {
            this.obj[paramName] = {
              ...this.obj[paramName],
              paramValue: value,
            };
            callback(value);
          });
        }
      }
      setParam(paramName, value) {
        new ROSLIB.Param({
          ros: ros,
          name: paramName,
        }).set(value);
        this.obj[paramName].paramValue = value;
      }
      getParam(paramName) {
        return this.obj[paramName].paramValue;
      }
    }

    class GlobalVariables {
      constructor() {
        this.cam2stopper = 0;
      }
    }

    globalVar = new GlobalVariables();

    const rosJSParamInstance = new RosJSParam({
      "/number_of_cameras": {
        callback: (msg) => {
          const numberOfCameras = msg;
          createCameraElements(numberOfCameras);
          createCameraDiagElements(numberOfCameras);
          if (numberOfCameras > 1) {
            $("#image_number").text("Live Images");
          } else {
            $("#image_number").text("Live Image");
          }
        },
      },
    });

    function generateUniqueString() {
      var timestamp = Date.now().toString(36); // Convert current time to base36 string
      var randomString = Math.random().toString(36).substring(2); // Generate a random string

      return timestamp + randomString;
    }

    function clearTableData(tableElement) {
      const tbody = tableElement.getElementsByTagName("tbody")[0];
      while (tbody && tbody.firstChild) {
        tbody.removeChild(tbody.firstChild);
      }
    }

    function getSelectedCuttingMode() {
      var selectedOption = document.querySelector(
        'input[name="cutting_mode_radio"]:checked'
      );
      if (selectedOption) {
        return selectedOption.id;
      }
      return null; // Return null if no option is selected or if there's an error
    }

    // Function to set the selected option based on ID
    function setCuttingModeById(id) {
      var radioButton = document.getElementById(id);
      if (radioButton) {
        radioButton.checked = true;
      }
    }

    function setPunchingStatusYes() {
      var punchingStatusWrapper = document.getElementById("punching_status");
      var radioButtonYes = document.getElementById("punch_yes");

      radioButtonYes.checked = true;
    }

    function disableRadioButtons(selectName) {
      var radioButtons = document.querySelectorAll(
        'input[name="' + selectName + '"]'
      );
      radioButtons.forEach(function (radioButton) {
        radioButton.disabled = true;
      });
    }

    // Function to enable radio buttons by group name
    function enableRadioButtons(selectName) {
      var radioButtons = document.querySelectorAll(
        'input[name="' + selectName + '"]'
      );
      radioButtons.forEach(function (radioButton) {
        radioButton.disabled = false;
      });
    }

    // Function to set punching_status to "no"
    function setPunchingStatusNo() {
      var punchingStatusWrapper = document.getElementById("punching_status");
      var radioButtonNo = document.getElementById("punch_no");

      radioButtonNo.checked = true;
    }

    // Function to get the current punching_status value ("yes" or "no")
    function getPunchingStatus() {
      var radioButtonYes = document.getElementById("punch_yes");

      if (radioButtonYes.checked) {
        return "YES";
      } else {
        return "NO";
      }
    }

    function onAiCutMasterModalClose() {
      // Your code here
      rosBridgeInstanceForJS.publish(
        "/gui/button/ai_cut_master_modal_open",
        new ROSLIB.Message({ data: false })
      );
    }

    const aiCutMasterModalElement =
      document.getElementById("testAiCuttingModal");

    // Add event listener for modal closing
    aiCutMasterModalElement.addEventListener("hidden.bs.modal", function () {
      onAiCutMasterModalClose();
    });
    $("#threshold_value").prop("disabled", true);
    $("#camera_exposure").prop("disabled", true);
    function timeAgo(timestamp) {
      const messageDate = new Date(timestamp);
      const now = new Date();
      const diffInMilliseconds = now - messageDate;

      const hours = Math.floor(diffInMilliseconds / 3600000);
      const minutes = Math.floor((diffInMilliseconds % 3600000) / 60000);
      const seconds = Math.floor(
        ((diffInMilliseconds % 3600000) % 60000) / 1000
      );

      let formattedTime = "";

      if (hours > 0) {
        formattedTime += `${hours}:`;
      }

      if (minutes < 10) {
        formattedTime += `0${minutes}:`;
      } else {
        formattedTime += `${minutes}:`;
      }

      if (seconds < 10) {
        formattedTime += `0${seconds}`;
      } else {
        formattedTime += `${seconds}`;
      }

      return formattedTime;
    }

    $("#editJob, #displayValues").on("click", function () {
      $("#jobSettingsModal").modal("show");
    });

    $("#openCutMasterDialog").on("click", function () {
      if (adminState == "true") {
        $("#cutting_length").css("border-color", "");
        $("#testAiCuttingModal").modal("show");
        rosBridgeInstanceForJS.publish(
          "/gui/button/ai_cut_master_modal_open",
          new ROSLIB.Message({ data: true })
        );
      } else {
        new AWN()["alert"]("Unauthorized User.");
      }
    });
    $("#barcode-done-button").on("click", function () {
      handleParsing();
    });
    $("#barcode_item").on("click", function () {
      resetAllValues();

      // Reset styles for all dropdowns
      const dropdownIds = [
        "roll",
        "loom",
        "width",
        "gsm",
        "roll-weight",
        "roll-length",
      ];

      dropdownIds.forEach((id) => {
        document.getElementById(`${id}-index`).style.border = ""; // Reset border
      });

      setBarcodeConfig(barcode_config); // Assuming this sets the config
      $("#barcode_modal").modal("show");
    });

    $("#barcode_modal").on("shown.bs.modal", function () {
      $("#barcode-input").focus(); // Focus on barcode input field when modal is shown
    });

    $("#cut_length_btn").on("click", function () {
      const cutLength = parseInt($("#cutting_length").val());
      if (cutLength > 10000 || cutLength < 50) {
        $("#cutting_length").css("border-color", "red");
      } else {
        $("#cutting_length").css("border-color", "");
        rosBridgeInstanceForJS.publish(
          "/gui/button/write_cut_length",
          new ROSLIB.Message({ data: cutLength })
        );
      }
    });

    $("#send_cycle_start").on("click", function () {
      Swal.fire({
        title: "Start Machine?",
        text: "Are you sure? This action will start the machine.",
        icon: "warning",
        showCancelButton: true,
        confirmButtonColor: "#3085d6",
        cancelButtonColor: "#d33",
        confirmButtonText: "okay",
      }).then((result) => {
        if (result.isConfirmed) {
          rosBridgeInstanceForJS.publish("/gui/button/cycle_start");
        }
      });
    });

    // $("#viewDiagnostics, #displayHealth").on("click", function () {
    //   $("#SystemHealthModal").modal("show");
    // });

    // Show/hide recut length values based on cutting mode
    if (getSelectedCuttingMode() === "recut") {
      $("#secondarycard").show();
      $("#tertiarycard").show();
      $("#next-stopping-card").hide();
      $("#cutting_plan_row").show();
      $("#primaryCard").show();
      $("#defected_card").show();
      $("#good-body-card").show();

      $("#defect-bodies-info").css("display", "none");
      $("#defect-bodies").hide();
      $(".defect-body-col").css("display", "none");
    } else if (
      getSelectedCuttingMode() === "autocut" ||
      getSelectedCuttingMode() === "justcut"
    ) {
      $("#good-body-card").hide();
      $("#next-stopping-card").hide();
      $("#cutting_plan_row").show();
      $("#primaryCard").show();
      $("#defected_card").show();
      $("#secondarycard").css("display", "none");
      $("#tertiarycard").css("display", "none");
    } else {
      $("#next-stopping-card").show();
      $("#cutting_plan_row").hide();
      $("#primaryCard").show();
      $("#defected_card").show();
      $("#good-body-card").hide();
      $("#defect-bodies-info").css("display", "none");
      $("#defect-bodies").hide();
      $(".defect-body-col").css("display", "none");
      $("#secondarycard").css("display", "none");
      $("#tertiarycard").css("display", "none");
    }

    if (getSelectedCuttingMode() === "justcut") {
      $("#next-stopping-card").show();
      $("#cutting_plan_row").hide();
      $("#primaryCard").show();
      $("#defected_card").show();
      $("#good-body-card").hide();
      $("#dynamic-card").show();
    }

    if (getSelectedCuttingMode() === "semiauto") {
      $("#dynamic-card").hide();
      $("#defect-bodies-info").css("display", "none");
      $("#defect-bodies").hide();
      $("#total-defects-block").removeClass("col-6").addClass("col-12");
      $(".defect-body-col").css("display", "none"); // Hide the separator
    } else {
      $("#defect-bodies").show();
      $("#total-defects-block").removeClass("col-12").addClass("col-6");
      $(".defect-body-col").css("display", "block"); // Show the separator
    }

    //Systems Health Indicator ICON
    function updateSystemHealthIconColor(iconId, color) {
      $("#" + iconId).css("color", color);
    }

    function createCameraElements(numberOfCameras) {
      const container = $("#camera_container_show");
      container.empty(); // Clear previous cameras

      const rowWrapper = $("<div class='row'></div>");

      for (let i = 1; i <= numberOfCameras; i++) {
        const cameraElement = createCameraElement(i);
        rowWrapper.append(cameraElement);

        container.append(rowWrapper);
        // Check if the row is filled with 4 cameras or if it's the last camera
      }
    }

    function createCameraElement(index) {
      const cameraElement = `
          <div class="col mb-2">
              <i class="fas fa-video camera_icon" id="cam${index}_health_icon" style="font-size: 18px; color: #1c4e9b;"></i>
              <br> 
              <span style="color: grey; font-weight: normal; margin:-5px; font-size:small;">
                  Cam${index}:  
              </span> 
              <span id="cam${index}_num_images" style="color: grey; font-weight: normal; margin: 1px; font-size:small">
                  0
              </span> 
          </div>
      `;

      return cameraElement;
    }

    function createCameraDiagElements(numberOfCameras) {
      const container = $("#camera_container");
      const inferenceContainer = $("#camera_infer_container");
      const inferenceWrapper = $("<span>");
      const cameraWrapper = $("<span>");

      for (let i = 1; i <= numberOfCameras; i++) {
        const inferenceElement = createInferenceDiagElement(i);
        const cameraElement = createCameraDiagElement(i);
        cameraWrapper.append(cameraElement);
        inferenceWrapper.append(inferenceElement);
        if (i % 2 === 0) {
          cameraWrapper.append($("<br>"));
          inferenceWrapper.append($("<br>"));
        }
      }

      inferenceContainer.append(inferenceWrapper);
      container.append(cameraWrapper);
    }

    function createCameraDiagElement(index) {
      const cameraElement = $("<span>").html(`
      Cam ${index} :
        <span id="cam${index}_published_images" style="color: #707070">
          0
        </span>
      `);

      return cameraElement[0];
    }

    function createInferenceDiagElement(index) {
      const inferenceElement = $("<span>").html(`
        Cam${index}Infer(ms) :
        <span id="cam${index}_inference_time" style="color: #707070">
          0
        </span>
      `);

      return inferenceElement[0];
    }

    function validateInputs() {
      const errors = [];

      // Define the regular expression for safe characters
      const safeChars = /^[a-zA-Z0-9_\-]+$/;

      // Validate the Roll ID
      const isValidRollId = safeChars.test($("#roll_id").val());

      if (!isValidRollId) {
        errors.push("Invalid Roll ID");
        $("#roll_id").css("border-color", "red");
      } else {
        $("#roll_id").css("border-color", "");
      }

      const isValidLoomNo = safeChars.test($("#loom_num").val());

      if (!isValidLoomNo) {
        errors.push("Invalid Loom Number");
        $("#loom_num").css("border-color", "red");
      } else {
        $("#loom_num").css("border-color", "");
      }

      const isValidBatchCount = safeChars.test($("#batchCount").val());

      if (!isValidBatchCount) {
        errors.push("Invalid Batch Count");
        $("#batchCount").css("border-color", "red");
      } else {
        $("#batchCount").css("border-color", "");
      }

      const isValidRoll_Length = parseFloat($("#roll_meter").val()) <= 10000;
      if (!isValidRoll_Length) {
        errors.push("Invalid Roll Length");
        $("#roll_meter").css("border-color", "red");
      } else {
        $("#roll_meter").css("border-color", "");
      }

      const layer = $("#layer");
      const selectedMaterialType = layer.val();

      const isValidMaterialType =
        selectedMaterialType !== null && selectedMaterialType !== "";

      if (!isValidMaterialType) {
        errors.push("Select Fabric Type");
        layer.css("border-color", "red");
      } else {
        layer.css("border-color", "");
      }

      const recipes = $("#recipes");
      const recipe_name = recipes.val();

      const isValidRecipe = recipe_name !== null && recipe_name !== "";

      if (!isValidRecipe) {
        errors.push("Recipe not selected");
        recipes.css("border-color", "red");
      } else {
        recipes.css("border-color", "");
      }

      const isValidGsm =
        !isNaN(parseFloat($("#gsm").val())) && parseFloat($("#gsm").val()) > 0 && parseFloat($("#gsm").val()) < 1000;
      if (!isValidGsm) {
        errors.push("Invalid GSM the value should be less than 1000");
        $("#gsm").css("border-color", "red");
      } else {
        $("#gsm").css("border-color", "");
      }

      const isValidOffset =
        !isNaN(parseFloat($("#offset").val())) &&
        parseFloat($("#offset").val()) >= 0 &&
        parseFloat($("#offset").val()) <= 10000;
      if (!isValidOffset) {
        errors.push("Invalid Offset (should be less than 200)");
        $("#offset").css("border-color", "red");
      } else {
        $("#offset").css("border-color", "");
      }

      const isValidCutLength =
        !isNaN(parseFloat($("#cut_length").val())) &&
        parseFloat($("#cut_length").val()) > 0 &&
        parseFloat($("#cut_length").val()) <= 700;
      if (!isValidCutLength) {
        errors.push("Invalid Cut Length (should be less than 700)");
        $("#cut_length").css("border-color", "red");
      } else {
        $("#cut_length").css("border-color", "");
      }

      if (
        getSelectedCuttingMode() === "recut" ||
        getSelectedCuttingMode() === "autocut"
      ) {
        if (
          parseFloat($("#cut_length").val() * 10) >
          globalVar.cam2stopper + parseFloat($("#offset").val() * 10)
        ) {
          errors.push(
            "Cut Length Can't be Greater than " +
              (
                globalVar.cam2stopper + parseFloat($("#offset").val() * 10)
              ).toString() +
              "(Cam2Cutter))"
          );
          $("#cut_length").css("border-color", "red");
        }
      }

      if (getSelectedCuttingMode() === "recut") {
        const secondaryCutLength = parseFloat($("#secondary_cut_length").val());
        const tertiaryCutLength = parseFloat($("#tertiary_cut_length").val());
        const cutLength = parseFloat($("#cut_length").val());

        if (
          isNaN(secondaryCutLength) ||
          secondaryCutLength <= 0 ||
          secondaryCutLength >= cutLength
        ) {
          errors.push(
            "Invalid Secondary Cut Length (should be less than or equal to " +
              cutLength +
              ") for Recut Mode"
          );
          $("#secondary_cut_length").css("border-color", "red");
        } else {
          $("#secondary_cut_length").css("border-color", "");
        }

        if (
          isNaN(tertiaryCutLength) ||
          tertiaryCutLength < 0 ||
          tertiaryCutLength >= secondaryCutLength
        ) {
          errors.push(
            "Invalid Tertiary Cut Length (should be less than " +
              secondaryCutLength +
              ") for Recut Mode"
          );
          $("#tertiary_cut_length").css("border-color", "red");
        } else {
          $("#tertiary_cut_length").css("border-color", "");
        }
      }

      const isValidFabricWidth =
        !isNaN(parseFloat($("#fabric_width").val())) &&
        parseFloat($("#fabric_width").val()) > 0 &&  parseFloat($("#fabric_width").val()) < 1000;
      if (!isValidFabricWidth) {
        errors.push("Invalid Fabric Width the value should be less than 10");
        $("#fabric_width").css("border-color", "red");
      } else {
        $("#fabric_width").css("border-color", "");
      }

      const isValidFabricWeight =
        !isNaN(parseFloat($("#roll_weight").val())) &&
        parseFloat($("#roll_weight").val()) > 0;
      if (!isValidFabricWeight) {
        errors.push("Invalid Fabric Weight");
        $("#roll_weight").css("border-color", "red");
      } else {
        $("#roll_weight").css("border-color", "");
      }

      if (errors.length > 0) {
        // Display an error message or handle invalid inputs
        const errorMessage = errors.join("\n and \n");
        new AWN().alert(errorMessage, {
          durations: {
            warning: 4000,
          },
        });
        $("#jobSettingsModal").modal("show");
        return false;
      }

      return true;
    }

    $(".roll_toggle").click(() => {
      if (rollStateButton == "End") {
        ToggleRollState();
        $("#jobSettingsModal").modal("show");
      } else {
        if (validateInputs()) {
          $("#jobSettingsModal").modal("hide");
          ToggleRollState();
        }
      }
    });
    function ToggleRollState() {
      // Your existing code for handling valid inputs
      if (rollStateButton === "End") {
        if ($("#data_collection").prop("checked")) {
          // Checkbox is checked, open the modal
          $("#data_collection_modal").modal("show");
        }

        //when the rollStateButton is "End"
        is_roll_started = false;
        updateRollState();
        $("#editJob").show();
        $("#openCutMasterDialog").prop("disabled", false);
        $("#open_barcode_modal").prop("disabled", false);
        rollStateButton = "Start";
        $(".roll_toggle").text(rollStateButton);
        $(".roll_toggle").css("background-color", "#4d8ceb");
        $(".roll_toggle").css("color", "#ffffff");
      } else {
        is_roll_started = true;
        $("#editJob").hide();
        $("#openCutMasterDialog").prop("disabled", true);
        $("#open_barcode_modal").prop("disabled", true);
        updateRollState();
        rollStateButton = "End";
        $(".roll_toggle").text(rollStateButton);
        $(".roll_toggle").css("background-color", "#f2395e");
        $(".roll_toggle").css("color", "#ffffff");
      }
      $(".roll_toggle").prop("disabled", true);

      var rollIdValue = $("#roll_id").val().toUpperCase();
      // Truncate the roll_id value if it exceeds a certain size
      if (rollIdValue.length > 10) {
        rollIdValue = rollIdValue.slice(0, 10) + "...";
      }
      $("#rollIdValue .value")
        .text(rollIdValue || "NA")
        .css({ color: "black", "font-weight": "bold" });

      $("#gsmValue .value")
        .text($("#gsm").val() || "NA")
        .css("color", "black");
      $("#offsetValue .value")
        .text($("#offset").val() || "NA")
        .css("color", "black");
      $("#fabricWidthValue .value")
        .text($("#fabric_width").val() || "NA")
        .css("color", "black");
      $("#rollWeightValue .value")
        .text($("#roll_weight").val() || "NA")
        .css("color", "black");

      $("#batchCount .value")
        .text($("#batchCount").val() || "NA")
        .css("color", "black");

      $("#cutLengthValue .value")
        .text($("#cut_length").val() || "NA")
        .css("color", "black");
      $("#primarycutLengthValue").text($("#cut_length").val() || "NA");

      var punchstatus = getPunchingStatus();
      $("#punchingStatusValue .value")
        .text(punchstatus)
        .css("color", punchstatus === "YES" ? "green" : "red");

      $("#recipesValue .value")
        .text($("#recipes").val() || "NA")
        .css("color", "black");

      $("#cuttingModeValue .value").text(
        (getSelectedCuttingMode() || "NA").toUpperCase()
      );

      if (
        getSelectedCuttingMode() === "recut" ||
        getSelectedCuttingMode() === "autocut"
      ) {
        $("#cutting_mc_plc_indicator_div").show();
      } else {
        $("#cutting_mc_plc_indicator_div").hide();
      }

      $("#secondaryCutLengthValue .value")
        .text($("#secondary_cut_length").val() || "NA")
        .css("color", "black");
      $("#SecondarycutLengthValue").text(
        $("#secondary_cut_length").val() || "NA"
      );

      $("#tertiaryCutLengthValue .value")
        .text($("#tertiary_cut_length").val() || 0)
        .css("color", "black");

      $("#TertiarycutLengthValue").text(
        $("#tertiary_cut_length").val() || "NA"
      );

      $("#roll_meter_value.value")
        .text($("#roll_meter").val() || 0)
        .css("color", "black");

      $("#loom_num_value.value")
        .text($("#loom_num").val() || 0)
        .css("color", "black");
      // Show or hide secondary cut length based on cutting mode value
    }

    function ToggleRollUIState() {
      if (validateInputs()) {
        // Your existing code for handling valid inputs
        if (rollStateButton === "End") {
          if ($("#data_collection").prop("showchecked")) {
            // Checkbox is checked, open the modal
            $("#data_collection_modal").modal("show");
          }

          //when the rollStateButton is "End"
          is_roll_started = false;
          $("#editJob").show();
          $("#openCutMasterDialog").prop("disabled", false);
          $("#open_barcode_modal").prop("disabled", false);
          rollStateButton = "Start";
          $(".roll_toggle").text(rollStateButton);
          $(".roll_toggle").css("background-color", "#4d8ceb");
          $(".roll_toggle").css("color", "#ffffff");
        } else {
          is_roll_started = true;
          $("#editJob").hide();
          $("#openCutMasterDialog").prop("disabled", true);
          $("#open_barcode_modal").prop("disabled", true);
          rollStateButton = "End";
          $(".roll_toggle").text(rollStateButton);
          $(".roll_toggle").css("background-color", "#f2395e");
          $(".roll_toggle").css("color", "#ffffff");
        }
        $(".roll_toggle").prop("disabled", false);
        $("#stop_at_last_defect_toggle_button").prop("disabled", false);
        $("#roll_id").prop("disabled", false);
        $("#gsm").prop("disabled", false);
        $("#offset").prop("disabled", false);
        $("#cut_length").prop("disabled", false);
        $("#fabric_width").prop("disabled", false);
        $("#roll_weight").prop("disabled", false);
        $("#punch").prop("disabled", false);
        $("#recipes").prop("disabled", false);
        //$("#cutting_mode").prop("disabled", false);
        $("#secondary_cut_length").prop("disabled", false);
        $("#tertiary_cut_length").prop("disabled", false);
        $("#roll_meter").prop("disabled", false);
        $("#loom_num").prop("disabled", false);
        $("#top_bottom_panel_selected").prop("disabled", false);
        $("#fully_ignore_disabled").prop("disabled", false);
        $("#layer").prop("disabled", false);
        $("#batchCount").prop("disabled", false);
        $("#cut-length-job-index").prop("disabled", false);
        $("#punching_status input[type=radio]").prop("disabled", false);
        $("#cutting_mode input[type=radio]").prop("disabled", state);

        var rollIdValue = $("#roll_id").val().toUpperCase();
        // Truncate the roll_id value if it exceeds a certain size
        if (rollIdValue.length > 10) {
          rollIdValue = rollIdValue.slice(0, 10) + "...";
        }
        $("#rollIdValue .value")
          .text(rollIdValue || "NA")
          .css({ color: "black", "font-weight": "bold" });

        $("#gsmValue .value")
          .text($("#gsm").val() || "NA")
          .css("color", "black");
        $("#offsetValue .value")
          .text($("#offset").val() || "NA")
          .css("color", "black");
        $("#fabricWidthValue .value")
          .text($("#fabric_width").val() || "NA")
          .css("color", "black");
        $("#rollWeightValue .value")
          .text($("#roll_weight").val() || "NA")
          .css("color", "black");

        $("#batchCount .value")
          .text($("#batchCount").val() || "NA")
          .css("color", "black");

        $("#cutLengthValue .value")
          .text($("#cut_length").val() || "NA")
          .css("color", "black");
        $("#primarycutLengthValue").text($("#cut_length").val() || "NA");

        var punchstatus = getPunchingStatus();
        $("#punchingStatusValue .value")
          .text(punchstatus)
          .css("color", punchstatus === "YES" ? "green" : "red");

        $("#recipesValue .value")
          .text($("#recipes").val() || "NA")
          .css("color", "black");

        $("#cuttingModeValue .value").text(
          (getSelectedCuttingMode() || "NA").toUpperCase()
        );

        if (
          getSelectedCuttingMode() === "recut" ||
          getSelectedCuttingMode() === "autocut"
        ) {
          $("#cutting_mc_plc_indicator_div").show();
        } else {
          $("#cutting_mc_plc_indicator_div").hide();
        }

        $("#secondaryCutLengthValue .value")
          .text($("#secondary_cut_length").val() || "NA")
          .css("color", "black");
        $("#SecondarycutLengthValue").text(
          $("#secondary_cut_length").val() || "NA"
        );

        $("#tertiaryCutLengthValue .value")
          .text($("#tertiary_cut_length").val() || 0)
          .css("color", "black");

        $("#TertiarycutLengthValue").text(
          $("#tertiary_cut_length").val() || "NA"
        );
        // $("#stop_at_last_defect_toggle_button").prop("disabled", false);
        // $("#roll_id").prop("disabled", false);
        // $("#gsm").prop("disabled", false);
        // $("#offset").prop("disabled", false);
        // $("#cut_len").prop("disabled", false);
        // $("#fabric_width").prop("disabled", false);
        // $("#roll_weight").prop("disabled", false);
        // $("#punch").prop("disabled", false);
        // $("#recipes").prop("disabled", false);
        // $("#cutting_mode").prop("disabled", false);
        // $("#secondary_cut_length").prop("disabled", false);
        // $("#tertiary_cut_length").prop("disabled", false);
        // $("#roll_meter").prop("disabled", false);
        // $("#loom_num").prop("disabled", false);
        // $("#top_bottom_panel_selected").prop("disabled", false);
        // $("#fully_ignore_disabled").prop("disabled", false);
        $("#roll_meter_value.value")
          .text($("#roll_meter").val() || 0)
          .css("color", "black");

        $("#loom_num_value.value")
          .text($("#loom_num").val() || 0)
          .css("color", "black");

        $("#running_meter").html("0.0");

        // $("#primary_length").html("0");
        // document.getElementById("primary_meter").textContent = 0;
        // $("#secondary_length").html("0");
        // document.getElementById("secondary_meter").textContent = 0;
        // $("#tertiary_length").html("0");
        // document.getElementById("tertiary_meter").textContent = 0;
        // $("#defect_length").html("0");
        // $("#no_of_defects").html("0");

        // Show or hide secondary cut length based on cutting mode value
      }
    }

    $("#cutting_mode").on("change", function () {
      if (getSelectedCuttingMode() === "recut") {
        $("#secondary_cut_length_box").show();
        $("#tertiary_cut_length_box").show();
        $(".secondaryCutLengthValue").show();
        $(".tertiaryCutLengthValue").show();
        $("#top_bottom_panel_dev").hide();
        document.getElementById("top_bottom_panel_selected").checked = false;
      } else {
        $("#secondary_cut_length_box").hide();
        $("#tertiary_cut_length_box").hide();
        $(".secondaryCutLengthValue").hide();
        $(".tertiaryCutLengthValue").hide();
        $("#top_bottom_panel_dev").show();
      }
      // if (getSelectedCuttingMode() === "semiauto" || getSelectedCuttingMode() === "justcut" || getSelectedCuttingMode() === "autocut") {
      var mainParent = $(".toggle-btn");
      $(mainParent).removeClass("active");
      // }
    });
    const rosBridgeInstanceForJS = new RosBridgeForJS({
      // ******
      // Subscribers
      // ******
      "/gui/label/system_health_msg": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          $("#health_msg").html(message.data);
        },
      },

      "/gui/label/running_meter": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          // update value
          $("#running_meter").html(parseFloat(message.data).toFixed(1));
        },
      },

      "/gui/label/no_of_defects": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          $("#no_of_defects").html(message.data);
          $("#numberofdefects").html(message.data);
        },
      },

      "/gui/label/next_stopping": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          function isValidJSON(str) {
            try {
              JSON.parse(str);
              return true;
            } catch (error) {
              return false;
            }
          }
          let data;
          let justcutData;
          if (isValidJSON(message.data)) {
            data = JSON.parse(message.data);
          } else {
            justcutData = message.data;
          }

          // check if is punch  blocked in data.
          var isPunchBlocked = data?.is_punch_blocked;

          // console.log(isPunchBlocked);
          // console.log(data?.hasOwnProperty("is_punch_blocked"));

          if (data?.hasOwnProperty("is_punch_blocked")) {
            if (isPunchBlocked) {
              $("#punchblk").text("BLOCKED").css({
                color: "red",
                "background-color": "white",
                "font-size": "1.2rem",
              });
            } else {
              $("#punchblk").text("ALLOWED").css({
                color: "white",
                "background-color": "#61a867",
                "font-size": "0.9rem",
              });
            }
          }
          // next stopping check in data and assigned
          var isNextStoppingDefect = data?.next_stopping_distance || "N/A";
          if (data?.hasOwnProperty("next_stopping_distance")) {
            if (getSelectedCuttingMode() === "semiauto") {
              $("#next_stopping")
                .text(isNextStoppingDefect)
                .css("color", "white");
              $("#defect-bodies-info").css("display", "none");
              $("#defect-bodies").hide();
              $("#total-defects-block").removeClass("col-6").addClass("col-12");
              $(".defect-body-col").css("display", "none"); // Hide the separator
              $("#secondarycard").css("display", "none");
              $("#tertiarycard").css("display", "none");
              $("#next-stopping-card").show();
              $("#cutting_plan_row").hide();
              $("#defected_card").show();
              $("#primaryCard").show();
              $("#good-body-card").hide();
              $("#dynamic-card").hide();

              $("#nextstop_defect").removeClass("col-5").addClass("col-12");
            }
          }
          if (getSelectedCuttingMode() === "justcut") {
            $("#dynamic-card").show();
            $("#next_stopping")
              .text(isNextStoppingDefect)
              .css("color", "white");
            $("#next-stopping-card").show();
            $("#cutting_plan_row").hide();
            $("#primaryCard").show();
            $("#defected_card").show();
            $("#defect-bodies").show();
            $("#total-defects-block").removeClass("col-12").addClass("col-6");
            $(".defect-body-col").css("display", "block");
            $("#good-body-card").hide();
            $("#secondarycard").css("display", "none");
            $("#tertiarycard").css("display", "none");
            $("#nextstop_defect").addClass("col-12").removeClass("col-5");
            if (typeof justcutData === "string") {
              if (justcutData.includes("True")) {
                //   $("#card-label")
                // .text("Mode")
                // .css("color", "white");
                $("#dynamic-value")
                  .text(justcutData)
                  .css({ color: "white", "background-color": "#61a867" });
              } else if (justcutData.includes("False")) {
                // $("#card-label")
                // .text("Mode")
                // .css("color", "white");
                $("#dynamic-value")
                  .text(justcutData)
                  .css({ color: "white", "background-color": "red" });
              } else {
                $("#dynamic-value").text(justcutData).css("color", "white");
              }
            }
          }

          // Update body counters if cutting mode is recut
          if (data?.hasOwnProperty("cutting_info")) {
            const pTarget = data.cutting_info.p_target;
            const pMeter = data.cutting_info.p_meter.toFixed(1);
            const sMeter = data.cutting_info.s_meter.toFixed(1);
            const tMeter = data.cutting_info.t_meter.toFixed(1);
            const dMeter = data.cutting_info.d_meter.toFixed(1);
            var bodyCounters = data.cutting_info.body_counters;

            // Update body counters if cutting mode is recut
            if (getSelectedCuttingMode() === "recut") {
              $("#dynamic-card").hide();
              $("#secondarycard").show();
              $("#tertiarycard").show();
              $("#next-stopping-card").hide();
              $("#cutting_plan_row").show();
              $("#primaryCard").show();
              $("#defected_card").show();
              $("#good-body-card").show();

              $("#nextstop_defect").addClass("col-5").removeClass("col-12");
              if ($("#tertiary_cut_length").val() != 0) {
                // document.getElementById("workorder_target").textContent =
                //   pTarget;
                $("#primary_length").text(bodyCounters[0]);
                // document.getElementById("primary_meter").textContent = pMeter;
                $("#secondary_length").text(bodyCounters[1]);
                // document.getElementById("secondary_meter").textContent = sMeter;
                $("#tertiary_length").text(bodyCounters[2]);
                // document.getElementById("tertiary_meter").textContent = tMeter;
                $("#defect_length").text(bodyCounters[3]);
                document.getElementById("defect_meter").textContent = dMeter;
                $("#defect-bodies-info").css("display", "none");
                $("#defect-bodies").hide();
                $("#total-defects-block")
                  .removeClass("col-6")
                  .addClass("col-12");
                $(".defect-body-col").css("display", "none"); // Hide the separator
              } else {
                // document.getElementById("workorder_target").textContent =
                //   pTarget;
                $("#primary_length").text(bodyCounters[0]);
                // document.getElementById("primary_meter").textContent = pMeter;
                $("#secondary_length").text(bodyCounters[1]);
                // document.getElementById("secondary_meter").textContent = sMeter;
                $("#tertiary_length").text(0);
                // document.getElementById("tertiary_meter").textContent = tMeter;
                $("#defect_length").text(bodyCounters[2]);
                document.getElementById("defect_meter").textContent = dMeter;
              }
              $("#defect-bodies").show();
              $("#total-defects-block").removeClass("col-12").addClass("col-6");
              $(".defect-body-col").css("display", "block"); // Show the separator
            } else if (getSelectedCuttingMode() === "autocut") {
              $("#dynamic-card").hide();
              //document.getElementById("workorder_target").textContent = pTarget;
              //$("#good-body-count").text(bodyCounters[0]);
              $("#primary_length").text(bodyCounters[0]);
              // document.getElementById("primary_meter").textContent = pMeter;
              $("#primary-body-count").text(bodyCounters[0]);
              // document.getElementById("primary_meter").textContent = pMeter;
              $("#defect_length").text(bodyCounters[1]);
              document.getElementById("defect_meter").textContent = dMeter;
              $("#defect-bodies").show();
              $("#total-defects-block").removeClass("col-12").addClass("col-6");
              $(".defect-body-col").css("display", "block");
              $("#good-body-card").show();
              $("#next-stopping-card").hide();
              $("#cutting_plan_row").show();
              $("#primaryCard").show();
              $("#defected_card").show();
              $("#secondarycard").css("display", "none");
              $("#tertiarycard").css("display", "none");
              $("#nextstop_defect").addClass("col-12").removeClass("col-5");
            }

            if (
              getSelectedCuttingMode() === "justcut" ||
              getSelectedCuttingMode() === "semiauto"
            ) {
              //document.getElementById("workorder_target").textContent = pTarget;
              $("#primary_length").text(bodyCounters[0]);
              $("#primary-body-count").text(bodyCounters[0]);
              // document.getElementById("primary_meter").textContent = pMeter;
              $("#defect_length").text(bodyCounters[1]);
              document.getElementById("defect_meter").textContent = dMeter;
            }

            // Update defect color and highlight body counters if needed
            const isDefect = data.cutting_info.is_next_body_contains_defect;
            const nextAvailableCutLength =
              data.cutting_info.next_available_cut_length;
            if (isDefect) {
              $("#defect_length").text(bodyCounters[3]);
              $("#defect_length")
                .css("--glow-color", data.cutting_info.color)
                .addClass("blink-animation");
            } else if (nextAvailableCutLength === $("#cut_length").val()) {
              $("#primary_length")
                .css("--glow-color", data.cutting_info.color)
                .addClass("blink-animation");
            } else if (
              nextAvailableCutLength === $("#secondary_cut_length").val()
            ) {
              $("#secondary_length")
                .css("--glow-color", data.cutting_info.color)
                .addClass("blink-animation");
            } else if (
              nextAvailableCutLength === $("#tertiary_cut_length").val()
            ) {
              $("#tertiary_cut_length")
                .css("--glow-color", data.cutting_info.color)
                .addClass("blink-animation");
            }

            // Remove the blink animation class after 3 seconds
            setTimeout(function () {
              $(".blink-animation").removeClass("blink-animation");
            }, 3000);
          }
        },
      },

      "/gui/chart/defect_data": {
        topicType: "std_msgs/Int32MultiArray",
        type: "sub",
        callback: (message) => {
          let chart = $("#output_chart").get(0).chart;
          chart.data = {
            labels: ["OK", "Type 1", "Type 2", "Type 3"],
            datasets: [
              {
                data: message.data,
                backgroundColor: ["#46AD72", "#EA5061", "#2394F3", "#2394F3"],
              },
            ],
          };
          chart.update();
        },
      },

      "/gui/label/notification": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          if (msg.severity === "popup" && is_popup_on === false) {
            is_popup_on = true;

            if (msg.msg.includes("CSY002")) {
              Swal.fire({
                title: "Stop & Reset",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("CSY008")) {
              Swal.fire({
                title: "Camera Error. Restart system",
                text: msg.msg,
                icon: "error",
                showConfirmButton: false,
                allowOutsideClick: false,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("SHW009")) {
              Swal.fire({
                title: "Dancer Sensor: Please ON every cycle.",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("INFO010")) {
              Swal.fire({
                title: "Warning: Roll Not Started in System!",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("COM002")) {
              Swal.fire({
                title: "PLC Connection Error. Check & Restart",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: false,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("SHW003")) {
              Swal.fire({
                title: "Cutter Sensor: Please ON every cycle.",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("SHW004")) {
              Swal.fire({
                title: "Front Encoder Error",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("SHW011")) {
              Swal.fire({
                title: "Back Encoder Error",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("CUTT011")) {
              Swal.fire({
                title: "KWIS and actual cut length mismatch",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("APP017")) {
              Swal.fire({
                title: "Set Exposure Now",
                text: "Auto Brightness failed: " + msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("License is about to expire in")) {
              Swal.fire({
                html: `
                   <span style="font-size: 16px;">Your license is about to expire  </span>
                   <br>
                    <span style="font-size: 24px; font-weight: bold;">
                      ${msg.msg.split(":")[1].split("days")[0].trim()} days
                    </span>
                    <span style="font-size: 16px;">remaning</span>
                  `,
                icon: "warning",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else if (msg.msg.includes("completed successfully with")) {
              Swal.fire({
                text: msg.msg,
                icon: "success",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            } else {
              Swal.fire({
                title: "Error",
                text: msg.msg,
                icon: "error",
                showConfirmButton: true,
                allowOutsideClick: true,
              }).then((result) => {
                is_popup_on = false;
              });
            }
          } else {
            new AWN()[msg.severity](msg.msg, {
              durations: {
                info: 4000,
                success: 4000,
                alert: 4000,
                warning: 4000,
                error: 4000,
              },
            });
          }
        },
      },

      "/gui/value/brightness_data": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);

          if (msg.hasOwnProperty("brightness_adjusted")) {
            if (msg.brightness_adjusted) {
              $("#brightness_set_status").css("background-color", "green");
            } else {
              $("#brightness_set_status").css("background-color", "red");
            }
          }

          let str = "";
          if (msg.hasOwnProperty("roi_success")) {
            str += "roi: ";
            str += msg.roi_success ? "1" : "0";
          }
          if (msg.hasOwnProperty("current_brightness")) {
            str += " cur: ";
            str += msg.current_brightness.toFixed(2);
          }
          if (msg.hasOwnProperty("ideal_brightness")) {
            str += " ideal: ";
            str += msg.ideal_brightness.toFixed(2);
          }
          if (msg.hasOwnProperty("brightness_tolerance")) {
            str += " tol: ";
            str += msg.brightness_tolerance.toFixed(2);
          }
          // if (msg.hasOwnProperty("max_exposure")) {
          //   str += " max: ";
          //   str += (msg.max_exposure);
          // }
          // if (msg.hasOwnProperty("min_exposure")) {
          //   str += " min: ";
          //   str += (msg.min_exposure);
          // }
          // if (msg.hasOwnProperty("requested_exposure")) {
          //   str += " req: ";
          //   str += (msg.requested_exposure);
          // }

          // brightness_data["roi_success"] = get_roi_success;
          // brightness_data["brightness_adjustment_counter"] = brightness_adjustment_counter;
          // brightness_data["requested_exposure"] = exposure_value_msg.data;
          // brightness_data["current_brightness"] = brightness;
          // brightness_data["ideal_brightness"] = ideal_brightness;
          // brightness_data["brightness_tolerance"] = brightness_tolerance;
          // brightness_data["execution_time"] = execution_timer.getLastDurationInMilliseconds();
          // brightness_data["max_exposure"] = max_exposure_limit;
          // brightness_data["min_exposure"] = min_exposure_limit;
          // let this_str = message.data;
          $("#brightness_values").text(str);
          // $("#brightness_values").html(`<small>${str}</small>`);
        },
      },

      "/gui/value/ai_cut_master_modal_machine_ready_status": {
        topicType: "std_msgs/Bool",
        type: "sub",
        callback: (message) => {
          if (message.data) {
            $("#machine_ready").css("color", "#187318");
          } else {
            $("#machine_ready").css("color", "#ff0000");
          }
        },
      },

      "/gui/value/ai_cut_master_modal_plc_connection_status": {
        topicType: "std_msgs/Bool",
        type: "sub",
        callback: (message) => {
          if (message.data) {
            updateSystemHealthIconColor("connection_status", "#187318");
          } else {
            updateSystemHealthIconColor("connection_status", "#ff0000");
          }
        },
      },

      "/gui/value/real_world_map": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          // console.log(message.data);
          var msg = JSON.parse(message.data);

          let desiredFields = [
            "last_five_dancer_activation_times",
            "last_five_dancer_front_encoder_values",
            "last_five_dancer_back_encoder_values",
            "last_five_loop_close",
          ];
          let displayHeaderValues = ["Time", "Front", "Back", "Loop Close"];

          // Find the minimum length among all arrays in the object
          let minArrayLength = Math.min(
            ...desiredFields.map((key) =>
              msg[key] ? msg[key].length : Infinity
            )
          );

          clearTableData(document.getElementById("dancer_sensor_data_table"));

          desiredFields.forEach((key, index) => {
            if (msg.hasOwnProperty(key) && Array.isArray(msg[key])) {
              let row = `<th>${displayHeaderValues[index]}</th>`;
              let rowColor = "";
              msg[key].slice(0, minArrayLength).forEach((cellData) => {
                // Convert timestamps to time format
                if (key === "last_five_dancer_activation_times") {
                  row += `<td>${timeAgo(cellData)}</td>`;
                } else {
                  row += `<td>${cellData}</td>`;
                  if (
                    key === "last_five_loop_close" &&
                    Math.abs(cellData) > 250
                  ) {
                    rowColor = "yellow";
                  }
                }
              });

              let newRow = document.createElement("tr");
              newRow.innerHTML = row;

              if (msg["dancer_sensor_indicator"] !== "") {
                rowColor = msg["dancer_sensor_indicator"];
              }

              if (
                msg["front_encoder_indicator"] !== "" &&
                key === "last_five_dancer_front_encoder_values"
              ) {
                rowColor = msg["front_encoder_indicator"];
              }

              if (
                msg["back_encoder_indicator"] !== "" &&
                key === "last_five_dancer_back_encoder_values"
              ) {
                rowColor = msg["back_encoder_indicator"];
              }

              if (rowColor && key !== "last_five_dancer_activation_times") {
                newRow.style.backgroundColor = rowColor;
                newRow.style.color = "black";
              }
              $("#dancer_sensor_data_table tbody").append(newRow);
            }
          });

          if (
            msg.hasOwnProperty("current_dancer_back_encoder_mm") &&
            msg.hasOwnProperty("current_dancer_front_encoder_mm")
          ) {
            let table = document.getElementById("dancer_sensor_data_table");

            // Iterate through each row in the table
            for (let i = 0; i < table.rows.length; i++) {
              // Create a new cell for the new column
              let cell = table.rows[i].insertCell(-1); // -1 appends the cell to the end of the row
              // Add content to the new cell
              if (i === 0) {
                // For the header row
                cell.innerHTML = "<span>Now</span>";
              } else if (i === 1) {
                // For other rows
                cell.innerHTML = `${msg["current_dancer_front_encoder_mm"]}`; // Add cell content
                if (msg["dancer_sensor_indicator"] === "") {
                  cell.style.backgroundColor = msg["front_encoder_indicator"]; // Use front encoder color
                } else {
                  cell.style.backgroundColor = msg["dancer_sensor_indicator"]; // Use dancer sensor color
                }
                cell.style.color = "black";
              } else if (i === 2) {
                cell.innerHTML = `${msg["current_dancer_back_encoder_mm"]}`; // Add cell content
                // Check if the dancer sensor indicator is green
                if (msg["dancer_sensor_indicator"] === "") {
                  cell.style.backgroundColor = msg["back_encoder_indicator"]; // Use back encoder color
                } else {
                  cell.style.backgroundColor = msg["dancer_sensor_indicator"]; // Use dancer sensor color
                }
                cell.style.color = "black";
              }
            }
          }

          desiredFields = [
            "last_five_cutter_activation_times",
            "last_five_cutter_encoder_values",
          ];
          displayHeaderValues = ["Time", "Value"];
          minArrayLength = Math.min(
            ...desiredFields.map((key) =>
              msg[key] ? msg[key].length : Infinity
            )
          );
          clearTableData(document.getElementById("cutter_sensor_data_table"));
          desiredFields.forEach((key, index) => {
            if (msg.hasOwnProperty(key) && Array.isArray(msg[key])) {
              msg[key].reverse();
              let row = `<th>${displayHeaderValues[index]}</th>`;
              let rowColor = ""; // Variable to store row color

              msg[key].slice(0, minArrayLength).forEach((cellData) => {
                // Convert timestamps to time format
                if (key === "last_five_cutter_activation_times") {
                  row += `<td>${timeAgo(cellData)}</td>`;
                } else {
                  let this_cellval = parseInt(cellData, 10);

                  if (
                    Math.abs(this_cellval) >
                    parseInt($("#cut_length").val() * 10 * 2)
                  ) {
                    rowColor = "yellow";
                  }
                  row += `<td>${this_cellval}</td>`;
                }
              });

              let newRow = document.createElement("tr");
              newRow.innerHTML = row;

              if (
                rowColor &&
                getSelectedCuttingMode() !== "recut" &&
                getSelectedCuttingMode() !== "autocut" &&
                key !== "last_five_cutter_activation_times"
              ) {
                newRow.style.backgroundColor = rowColor;
                newRow.style.color = "black";
              }

              if (
                msg["cutter_sensor_indicator"] !== "" &&
                key !== "last_five_cutter_activation_times"
              ) {
                newRow.style.backgroundColor = msg["cutter_sensor_indicator"];
                newRow.style.color = "black";
              }

              $("#cutter_sensor_data_table tbody").append(newRow);
            }
          });
          let table = document.getElementById("cutter_sensor_data_table");

          if (msg.hasOwnProperty("current_body_processed_mm")) {
            // Iterate through each row in the table
            for (let i = 0; i < table.rows.length; i++) {
              // Create a new cell for the new column
              let cell = table.rows[i].insertCell(-1); // -1 appends the cell to the end of the row
              // Add content to the new o
              if (i === 0) {
                // For the header row
                cell.innerHTML = "<span>Now</span>";
              } else {
                // For other rows
                cell.innerHTML = `${msg["current_body_processed_mm"]}`; // Add cell content
                cell.style.backgroundColor = `${msg["cutter_sensor_indicator"]}`;
                cell.style.color = "black";
              }
            }
          }

          let cycleFields = [
            "last_five_planning_times",
            "last_five_start_times",
            "last_five_waiting_times",
          ];

          let cycleDisplayHeaders = ["Planning", "Start", "Acknowledge"];

          let minCycleArrayLength = Math.min(
            ...cycleFields.map((key) => (msg[key] ? msg[key].length : Infinity))
          );

          clearTableData(document.getElementById("cycle_time_data_table"));

          cycleFields.forEach((key, index) => {
            if (msg.hasOwnProperty(key) && Array.isArray(msg[key])) {
              let row = `<th>${cycleDisplayHeaders[index]}</th>`;

              msg[key].slice(0, minCycleArrayLength).forEach((cellData) => {
                if (
                  key === "last_five_planning_times" ||
                  key === "last_five_start_times" ||
                  key === "last_five_waiting_times"
                ) {
                  row += `<td>${cellData}</td>`;
                }
              });

              let newRow = document.createElement("tr");
              newRow.innerHTML = row;
              $("#cycle_time_data_table tbody").append(newRow);
            }
          });

          let cycl_table = document.getElementById("cycle_time_data_table");

          // // Iterate through each row in the table
          // for (let i = 0; i < cycl_table.rows.length; i++) {
          //   let cell = cycl_table.rows[i].insertCell(-1); // append at end

          //   if (i === 0) {
          //     // Header row
          //     cell.innerHTML = "<span>Now</span>";
          //   } else {
          //     // Data rows — match to cycleFields[i-1] because row[0] is the header
          //     const key = cycleFields[i - 1];
          //     const arr = msg[key];

          //     // Get last element safely
          //     const lastValue = Array.isArray(arr) && arr.length > 0 ? arr[arr.length - 1] : "";

          //     cell.innerHTML = `${lastValue}`;
          //     cell.style.color = "black";
          //   }
          // }

          let str = "";
          if (msg.hasOwnProperty("average_loop_time_ms")) {
            str += `Loop Time: ${msg.average_loop_time_ms.toFixed(1)} ms`;

            if (msg.average_loop_time_ms > 100) {
              $("#loop_time_status").css("background-color", "red");
              $("#loop_time_detail_text").css("color", "black");
              //background yellow
              $("#loop_time_detail_text").css("background-color", "yellow");
            } else {
              $("#loop_time_status").css("background-color", "green");
              $("#loop_time_detail_text").css("color", "black");
              $("#loop_time_detail_text").css("background-color", "white");
            }
          }

          $("#loop_time_detail_text").html(str);

          if (msg.hasOwnProperty("dancer_sensor_on_off_state")) {
            if (msg.dancer_sensor_on_off_state) {
              $("#dancer_sensor_on_off_status").css(
                "background-color",
                "green"
              );
            } else {
              $("#dancer_sensor_on_off_status").css("background-color", "red");
            }
          }

          if (msg.hasOwnProperty("cutting_plan_status")) {
            $("#cutting_plan_status").css(
              "background-color",
              msg.cutting_plan_status
            );
          }

          if (msg.hasOwnProperty("cutter_sensor_on_off_state")) {
            if (msg.cutter_sensor_on_off_state) {
              $("#cutter_sensor_on_off_status").css(
                "background-color",
                "green"
              );
            } else {
              $("#cutter_sensor_on_off_status").css("background-color", "red");
            }
          }
          if (msg.hasOwnProperty("cutting_cycle_state")) {
            const cuttinginfo = msg.cutting_cycle_state;
            $("#good-body-count").text(cuttinginfo);
          }

          if (msg.hasOwnProperty("dancer_sensor_on_off_state")) {
            // updateSystemHealthIconColor(
            //   "dancer_sensor_on_off_state",
            //   msg.dancer_sensor_on_off_state
            // );

            $("#dancer_sensor_on_off_status").css(
              "background-color",
              msg.dancer_sensor_on_off_state
            );
          }

          if (msg.hasOwnProperty("cutter_sensor_on_off_state")) {
            updateSystemHealthIconColor(
              "cutter_sensor_on_off_state",
              msg.cutter_sensor_on_off_state
            );
          }

          if (msg.hasOwnProperty("cutting_mc_plc_indicator_color")) {
            // Update cutting_mc_plc icon color
            updateSystemHealthIconColor(
              "cutting_mc_plc",
              msg.cutting_mc_plc_indicator_color
            );
          }

          if (msg.hasOwnProperty("kwis_plc_indicator_color")) {
            // Update cutting_mc_plc icon color
            updateSystemHealthIconColor(
              "kwis_plc",
              msg.kwis_plc_indicator_color
            );
          }

          // if (msg.hasOwnProperty("cutter_indicator_color")) {
          //   updateSystemHealthIconColor(
          //     "cutter_sensor",
          //     msg.cutter_indicator_color
          //   );
          // }

          // if (msg.hasOwnProperty("dancer_indicator_color")) {
          //   updateSystemHealthIconColor(
          //     "dancer_sensor",
          //     msg.dancer_indicator_color
          //   );
          // }

          if (msg.hasOwnProperty("dancer_indicator_color")) {
            updateSystemHealthIconColor(
              "loop_close_indicator",
              msg.dancer_sync_indicator_color
            );
          }

          if (msg.hasOwnProperty("cutter_encoder_indicator_color")) {
            updateSystemHealthIconColor(
              "cutter_encoder",
              msg.cutter_encoder_indicator_color
            );
          }

          if (msg.hasOwnProperty("camera_encoder_indicator_color")) {
            updateSystemHealthIconColor(
              "camera_encoder",
              msg.camera_encoder_indicator_color
            );
          }

          if (msg.hasOwnProperty("fabric_sync_indicator_color")) {
            updateSystemHealthIconColor(
              "fabric_sync_encoder",
              msg.fabric_sync_indicator_color
            );
          }

          if (msg.hasOwnProperty("last_cam_mm")) {
            $("#last_cam_mm").html(msg.last_cam_mm);
          }
          if (msg.hasOwnProperty("last_cutter_mm")) {
            $("#last_cutter_mm").html(msg.last_cutter_mm);
          }
          if (msg.hasOwnProperty("last_loop_close_value")) {
            $("#last_loop_close_value").html(msg.last_loop_close_value);
          }
          if (msg.hasOwnProperty("last_loop_close_time")) {
            $("#last_loop_close_time").html(msg.last_loop_close_time);
          }
          if (msg.hasOwnProperty("current_cam_mm")) {
            $("#current_cam_mm").html(msg.current_cam_mm);
          }
          if (msg.hasOwnProperty("current_cutter_mm")) {
            $("#current_cutter_mm").html(msg.current_cutter_mm);
          }
          if (msg.hasOwnProperty("current_body_mm")) {
            $("#current_body_mm").html(msg.current_body_mm);
          }
        },
      },

      "/gui/ack/roll_continue": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = message.data;
          let notifier = new AWN(); // Create a new notification instance

          // Define the confirm and cancel callbacks
          let onOk = () => {
            notifier.warning("Continue Roll");
            rosBridgeInstanceForJS.publish(
              "/gui/button/continueAck",
              new ROSLIB.Message({
                data: 1,
              })
            );
          };

          let onCancel = () => {
            notifier.warning("Stop Roll");
            rosBridgeInstanceForJS.publish(
              "/gui/button/continueAck",
              new ROSLIB.Message({
                data: 0,
              })
            );
            ToggleRollUIState();
          };

          // Show the confirmation dialog
          notifier.confirm(
            "<br> <b>Roll ID already exists. Continuing with existing roll.</b> <br>" +
              msg +
              "<br><div style='text-align:center'>Do you want to continue ?</div>",
            onOk,
            onCancel,
            {
              labels: {
                confirm: "Warning",
              },
            }
          );
        },
      },

      "/gui/value/all_values": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);

          const roll_id = $("#roll_id");
          const gsm = $("#gsm");
          const offset = $("#offset");
          const cut_len = $("#cut_length");
          const fabric_width = $("#fabric_width");
          const roll_weight = $("#roll_weight");
          const batchCount = $("#batchCount");
          const layer = $("#layer");
          const punch = $("#punching_status");
          const recipes = $("#recipes");
          const secondary_cut_length = $("#secondary_cut_length");
          const cut_lengths = JSON.parse(msg.secondary_cut_lengths);
          const secondary_cut_length_value = cut_lengths[0] ?? 0;
          const tertiary_cut_length_value = cut_lengths[1] ?? 0;
          const tertiary_cut_length = $("#tertiary_cut_length");
          const cutting_mode = $("#cutting_mode");
          const loom_num_value = $("#loom_num");
          const roll_meter_value = $("#roll_meter");
          const work_order = $("#cut-length-job-index");

          if (msg.hasOwnProperty("fully_ignore_disabled")) {
            document.getElementById("fully_ignore_disabled").checked =
              msg.fully_ignore_disabled;
          }

          if (msg.hasOwnProperty("top_bottom_panel_selected")) {
            const checkbox = document.getElementById(
              "top_bottom_panel_selected"
            );
            const toggleContainer = checkbox.closest(".toggle-btn");

            const isChecked =
              msg.top_bottom_panel_selected === true ||
              msg.top_bottom_panel_selected === "true";
            checkbox.checked = isChecked;

            // Dispatch event if toggle styling needs it
            checkbox.dispatchEvent(new Event("change"));

            // Optional: Manually control classes if CSS toggle depends on it
            if (toggleContainer) {
              toggleContainer.classList.toggle("active", isChecked);
            }
          }

          if (msg.hasOwnProperty("Barcode")) {
            barcode_config = JSON.parse(msg.Barcode);
          }

          if (msg.hasOwnProperty("WorkOrder")) {
            work_orders = JSON.parse(msg.WorkOrder);
          }
          if (msg.hasOwnProperty("work_order_id")) {
            const workOrderId = msg.work_order_id;

            // Wait until the options are added before setting the value
            // Use a small delay or use an event after options are loaded if possible
            setTimeout(() => {
              const optionExists =
                work_order.find(`option[value="${workOrderId}"]`).length > 0;

              if (optionExists) {
                work_order.val(workOrderId).change();
              } else {
                console.warn(
                  "Option not found for work_order_id:",
                  workOrderId
                );
              }

              console.log("work_order selected value", work_order.val());
            }, work_orders.length);
          }

          if (msg.hasOwnProperty("BatchCount")) {
            batchCount.val(msg.BatchCount);
          }
          if (msg.hasOwnProperty("layer_value")) {
            layer.val(msg.layer_value);
          }

          if (msg.hasOwnProperty("roll_length")) {
            roll_meter_value.val(msg.roll_length);
          }

          if (msg.hasOwnProperty("loom_number")) {
            loom_num_value.val(msg.loom_number);
          }

          const select = document.getElementById("cut-length-job-index");
          select.innerHTML =
            "<option disabled selected>Select Work Order</option>"; // Reset with default option

          if (Array.isArray(work_orders)) {
            work_orders.forEach((order) => {
              const option = document.createElement("option");
              option.value = order.work_order_id;
              option.textContent = `${order.order_name} (${
                order.cut_length / 10
              })`;
              select.appendChild(option);
            });
          } else {
            console.warn(
              "Expected work_orders to be an array, got:",
              typeof work_orders
            );
          }

          const selectworkorder = document.getElementById("selectWorkOrder");
          selectworkorder.innerHTML =
            "<option disabled selected>Select Work Order</option>"; // Reset with default option

          if (Array.isArray(work_orders)) {
            work_orders.forEach((order) => {
              const option = document.createElement("option");
              option.value = order.work_order_id;
              option.textContent = `${order.order_name}`;
              selectworkorder.appendChild(option);
            });
          } else {
            console.warn(
              "Expected work_orders to be an array, got:",
              typeof work_orders
            );
          }

          // Handle dropdown change and fill fields
          selectworkorder.addEventListener("change", function () {
            const selectedId = this.value;
            const selectedOrder = work_orders.find(
              (order) => order.work_order_id == selectedId
            );

            if (selectedOrder) {
              document.getElementById("completeTarget").value =
                selectedOrder.target_pcs;
              document.getElementById("completeLength").value =
                selectedOrder.cut_length / 10.0;

              document.getElementById("currectpcs").value =
                selectedOrder.current_pcs;
            } else {
              document.getElementById("completeTarget").value = "";
              document.getElementById("completeLength").value = "";
              document.getElementById("currectpcs").value = "";
            }
          });

          var rollIdValue = msg.roll_id.toUpperCase();
          // Truncate the roll_id value if it exceeds a certain size
          if (rollIdValue.length > 10) {
            rollIdValue = rollIdValue.slice(0, 10) + "...";
          }
          $("#rollIdValue .value")
            .text(rollIdValue || "NA")
            .css({ color: "black", "font-weight": "bold" });

          $("#gsmValue .value")
            .text(msg.gsm || "NA")
            .css("color", "black");

          $("#offsetValue .value")
            .text(msg.offset || "NA")
            .css("color", "black");

          $("#fabricWidthValue .value")
            .text(msg.fabric_width || "NA")
            .css("color", "black");

          $("#rollWeightValue .value")
            .text(msg.roll_weight || "NA")
            .css("color", "black");

          $("#cutLengthValue .value")
            .text(msg.cut_length || "NA")
            .css("color", "black");

          $("#punchingStatusValue .value")
            .text(msg.punch_on ? "YES" : "NO")
            .css("color", msg.punch_on ? "green" : "red");

          $("#cuttingModeValue .value").text(
            (msg.cutting_mode || "NA").toUpperCase()
          );

          $("#secondaryCutLengthValue .value")
            .text(secondary_cut_length_value || "NA")
            .css("color", "black");

          $("#tertiaryCutLengthValue .value")
            .text(tertiary_cut_length_value || 0)
            .css("color", "black");

          $("#roll_meter_value.value")
            .text(roll_meter_value || 0)
            .css("color", "black");

          $("#loom_num_value.value")
            .text(loom_num_value || 0)
            .css("color", "black");

          if (msg.punch_on) {
            setPunchingStatusYes();
          } else {
            setPunchingStatusNo();
          }
          //$("#punching_status").val(msg.punch_on ? "YES" : "NO");

          // Update the main_processed_images
          $(".main_processed_images").html(msg.main_processed_images);
          is_roll_started = msg.is_roll_started;
          const state = msg.is_roll_started;
          $("#stop_at_last_defect_toggle_button").prop("disabled", state);
          roll_id.val(msg.roll_id);
          gsm.val(msg.gsm);
          offset.val(msg.offset);
          cut_len.val(msg.cut_length);
          secondary_cut_length.val(secondary_cut_length_value);
          tertiary_cut_length.val(tertiary_cut_length_value);
          setCuttingModeById(msg.cutting_mode);
          //cutting_mode.val(msg.cutting_mode);
          fabric_width.val(msg.fabric_width);
          roll_weight.val(msg.roll_weight);
          $("#stop_at_last_defect_toggle_button").prop(
            "checked",
            msg.stop_at_last_defect
          );
          $("#ignore_inspection").prop("checked", msg.ignore_inspection);
          $("#data_collection").prop("checked", msg.data_collection);
          $("#pixel_per_mm").val(msg.pixel_per_mm);
          $("#pulse_per_mm").val(msg.pulse_per_mm);

          $("#running_meter").html(parseFloat(msg.total_meters_run).toFixed(1));
          $("#no_of_defects").html(msg.total_num_defects);
          //$("#numberofdefects").html(msg.total_num_defects);

          roll_id.prop("disabled", state);
          gsm.prop("disabled", state);
          offset.prop("disabled", state);
          cut_len.prop("disabled", state);
          secondary_cut_length.prop("disabled", state);
          tertiary_cut_length.prop("disabled", state);
          loom_num_value.prop("disabled", state);
          roll_meter_value.prop("disabled", state);
          recipes.prop("disabled", state);
          batchCount.prop("disabled", state);
          layer.prop("disabled", state);
          work_order.prop("disabled", state);
          cutting_mode.prop("disabled", state);
          $("#punching_status input[type=radio]").prop("disabled", state);
          $("#cutting_mode input[type=radio]").prop("disabled", state);
          $("#top_bottom_panel_selected").prop("disabled", state);
          if (getSelectedCuttingMode() === "recut") {
            $("#secondary_cut_length_box").show();
            $("#tertiary_cut_length_box").show();
            $(".secondaryCutLengthValue").show();
            $(".tertiaryCutLengthValue").show();
            $("#secondarycard").show();
            $("#tertiarycard").show();
            $("#next-stopping-card").hide();
            $("#cutting_plan_row").show();
            $("#primaryCard").show();
            $("#defected_card").show();
            $("#good-body-card").hide();

            $("#nextstop_defect").addClass("col-5").removeClass("col-12");
            $("#defect-bodies-info").css("display", "none");
            $("#defect-bodies").show();
            $("#total-defects-block").removeClass("col-12").addClass("col-6");
            $(".defect-body-col").css("display", "block");
          } else if (
            getSelectedCuttingMode() === "autocut" ||
            getSelectedCuttingMode() === "justcut"
          ) {
            $("#good-body-card").hide();
            $("#next-stopping-card").hide();
            $("#cutting_plan_row").show();
            $("#primaryCard").show();
            $("#defected_card").show();
            $("#nextstop_defect").addClass("col-12").removeClass("col-5");
            if (getSelectedCuttingMode() === "justcut") {
              $("#good-body-card").hide();
            }
          } else {
            $("#secondary_cut_length_box").hide();
            $("#tertiary_cut_length_box").hide();
            $(".secondaryCutLengthValue").hide();
            $(".tertiaryCutLengthValue").hide();
            $("#secondarycard").css("display", "none");
            $("#tertiarycard").css("display", "none");
            $("#next-stopping-card").show();
            $("#cutting_plan_row").hide();
            $("#primaryCard").show();
            $("#defected_card").show();
            $("#nextstop_defect").removeClass("col-5").addClass("col-12");
            $("#defect-bodies").hide();
            $("#total-defects-block").removeClass("col-6").addClass("col-12");
            $(".defect-body-col").css("display", "none");
          }
          cutting_mode.prop("disabled", state);
          punch.prop("disabled", state);
          recipes.prop("disabled", state);
          fabric_width.prop("disabled", state);
          roll_weight.prop("disabled", state);
          if (is_roll_started) {
            rollStateButton = "End";
            $(".roll_toggle").text(rollStateButton);
            $(".roll_toggle").css("background-color", "#f2395e");
            $(".roll_toggle").css("color", "#ffffff");
            $("#editJob").hide();
          } else {
            $("#editJob").show();
          }

          if (msg.hasOwnProperty("cam2stopper_distance_mm")) {
            globalVar.cam2stopper = msg.cam2stopper_distance_mm;
          }

          if (msg.hasOwnProperty("just_cut_connections_available")) {
            if (!msg.just_cut_connections_available) {
              hideMode("justcut");
            } else {
              // showMode('justcut');
            }
          }

          if (msg.hasOwnProperty("cutting_machine_plc_connected")) {
            if (!msg.cutting_machine_plc_connected) {
              hideMode("autocut");
              hideMode("recut");
            } else {
              // showMode('autocut');
              // showMode('recut');
            }
          }
        },
      },

      "/gui/label/main_node_health_data": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          $(".main_processed_images").html(msg.main_processed_images);
        },
      },

      "/gui/value/recipe_list": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          clearTimeout(reloaderId);
          var msg = JSON.parse(message.data);
          $("#recipes").html("");
          $.each(msg, function (key, value) {
            $("#recipes").append(
              $("<option></option>")
                .attr("value", key)
                .text(key)
                .prop("selected", value)
            );
          });
          $("#recipesValue .value")
            .text($("#recipes").val() || "NA")
            .css("color", $("#recipes").val() ? "black" : "grey");
          if (msg.length == 0) {
            new AWN()["warning"]("No Recipe Exists", {
              durations: {
                warning: 10000,
              },
            });
            return;
          }
        },
      },

      "/gui/value/loading_screen_data_and_status": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          updateModalMessage(msg.message);
          if (msg.close_loader) {
            systemLoaderModal.hide();
          }
        },
      },

      "/gui/cam_node/config": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          var expiry = "Not Active";
          if (
            msg.hasOwnProperty("license_expiry_date") &&
            !isNaN(Date.parse(msg.license_expiry_date))
          ) {
            expiry = new Date(msg.license_expiry_date);
            expiry =
              expiry.getDate() +
              "-" +
              (expiry.getMonth() + 1) +
              "-" +
              expiry.getFullYear();
          }

          var rounded_val = Math.round(msg.threshold * 10) / 10;
          var objSelect = document.getElementById("threshold_value");
          $("#camera_exposure").val(msg.exposure);
          $("#license_expires").html(expiry);
          $("#save_image").prop("checked", msg.save_all_defect_images);
          objSelect.value = rounded_val.toString();
        },
      },

      "/gui/label/cam_node_health_data": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          var camId = msg.cam_id;
          if (msg.hasOwnProperty("camera_indicator_color")) {
            // Update the camera icon color
            updateSystemHealthIconColor(
              "cam" + camId + "_health_icon",
              msg.camera_indicator_color
            );
          }

          let this_text =
            msg.num_imgs_published.toString() +
            " / " +
            msg.inference_time.toString() +
            " ms";

          // $("#cam" + camId + "_published_images").text(this_text);
          $("#cam" + camId + "_num_images").html(this_text);
          // $("#cam" + camId + "_inference_time").html(msg.inference_time);
        },
      },

      "/gui/value/ai_cut_master_modal_data": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);

          if (msg.hasOwnProperty("connection_status")) {
            updateSystemHealthIconColor(
              "connection_status",
              msg.connection_status
            );

            if (msg.hasOwnProperty("connection_details")) {
              $("#connection_details").text("");
              displayConnectionDetails(
                JSON.parse(msg.connection_details),
                "connection_details"
              );
            }
          }
        },
      },

      "/gui/image/compressed": {
        topicType: "sensor_msgs/CompressedImage",
        type: "sub",
        callback: (message) => {
          // const frameIdPositionString = message.header.frame_id;
          // const frameIdPosition = JSON.parse(frameIdPositionString);
          // const frameId = frameIdPosition.cam_id;
          // const position = frameIdPosition.position;
          // const imageData = message.data;

          // // Check if image element exists for the camera, if not, create one
          // let imageElement = $(`#live_image_${frameId}`);
          // if (!imageElement.length) {
          //   // Create image element
          //   const positionElement = $("<h6 class='cam_position'></h6>").text(
          //     position
          //   );
          //   imageElement = $("<img class='img-fluid cam-images'>").attr(
          //     "id",
          //     `live_image_${frameId}`
          //   );
          //   $("#camera_container_img").append(imageElement);

          //   // Create position element
          //   imageElement.after(positionElement);
          // }

          // // Update position display
          // $(`#position_${frameId}`).text(position);

          // // Update image for the camera
          // const imageSrc = "data:image/jpg;base64," + imageData;
          // imageElement.attr("src", imageSrc);

          // // Check if any modal is open and close it
          // if ($("#defected_image_modal").is(":visible")) {
          //   $("#defected_image_modal").modal("toggle");
          // }
          const frameIdPositionString = message.header.frame_id;
          const frameIdPosition = JSON.parse(frameIdPositionString);
          const frameId = frameIdPosition.cam_id;
          const position = frameIdPosition.position;
          const imageData = message.data;

          // Check if image element exists for the camera, if not, create one
          let imageElement = $(`#live_image_${frameId}`);
          if (!imageElement.length) {
            // Create image element
            const positionElement = $("<h6 class='cam_position'></h6>").text(
              position
            );
            imageElement = $("<img class='img-fluid cam-images'>")
              .attr("id", `live_image_${frameId}`)
              .data("oldUrl", null); // keep oldUrl attached to element
            $("#camera_container_img").append(imageElement);

            // Create position element
            imageElement.after(positionElement);
          }

          // Update position display
          $(`#position_${frameId}`).text(position);

          // Convert Base64 → Blob → ObjectURL
          const blob = b64toBlob(imageData, "image/jpeg");
          const newUrl = URL.createObjectURL(blob);

          // Revoke old URL to prevent leaks
          const oldUrl = imageElement.data("oldUrl");
          if (oldUrl) {
            URL.revokeObjectURL(oldUrl);
          }

          // Update image and store newUrl
          imageElement.attr("src", newUrl);
          imageElement.data("oldUrl", newUrl);

          // Check if any modal is open and close it
          if ($("#defected_image_modal").is(":visible")) {
            $("#defected_image_modal").modal("toggle");
          }
        },
      },

      "/gui/defect_map_image/compressed": {
        topicType: "sensor_msgs/CompressedImage",
        type: "sub",
        callback: (message) => {
          // let camNames = JSON.parse(message.header.frame_id);
          // if ($("#camera_names_container").children().length == 0) {
          //   // do something
          //   $("#camera_names_container").html("");
          //   camNames.forEach((elem) => {
          //     if (elem.includes("_")) {
          //       elem = elem.replace(/_/g, "<br/>");
          //     }
          //     $("#camera_names_container").append(
          //       `<div class="camera_name">${elem}</div>`
          //     );
          //   });
          // }

          // $("#defect_map_image").attr(
          //   "src",
          //   "data:image/jpg;base64," + message.data
          // );
          // Parse camera names only once
          let camNames = JSON.parse(message.header.frame_id);
          if ($("#camera_names_container").children().length === 0) {
            $("#camera_names_container").html("");
            camNames.forEach((elem) => {
              if (elem.includes("_")) {
                elem = elem.replace(/_/g, "<br/>");
              }
              $("#camera_names_container").append(
                `<div class="camera_name">${elem}</div>`
              );
            });
          }

          // Cleanup old blob URL if exists
          if (defectMapImageURL) {
            URL.revokeObjectURL(defectMapImageURL);
          }

          // Convert base64 -> Blob -> ObjectURL
          const blob = b64toBlob(message.data, "image/jpeg");
          defectMapImageURL = URL.createObjectURL(blob);

          // Set the new blob URL
          $("#defect_map_image").attr("src", defectMapImageURL);
        },
      },

         "/gui/defected_image/compressed": {
        topicType: "sensor_msgs/CompressedImage",
        type: "sub",
        callback: (message) => {
          previous_defect_frame_id_3 = previous_defect_frame_id_2;
          previous_defect_frame_id_2 = previous_defect_frame_id_1;
          previous_defect_frame_id_1 = message.header.frame_id;

          if (!message.data) {
            console.warn("message.data empty for defect image.");
            $("#defected_image").attr("src", "");
            return;
          }

          // Decode
          const byteChars = atob(message.data);
          const byteArray = new Uint8Array(byteChars.length);
          for (let i = 0; i < byteChars.length; i++)
            byteArray[i] = byteChars.charCodeAt(i);
          const blob = new Blob([byteArray], { type: "image/jpeg" });

          // Store
          const newURL = URL.createObjectURL(blob);
          defectImageURLs.unshift(newURL);
          if (defectImageURLs.length > 4) {
            const oldURL = defectImageURLs.pop();
            URL.revokeObjectURL(oldURL);
          }

          // Show main image
          $("#defected_image").attr("src", defectImageURLs[0]);
          $("#defected_image_modal").modal("toggle");

          // Update previous images (show last 2)
          const numPrevious = $(".previous_image").length;
          console.log("numPrevious:", numPrevious, "URLs:", defectImageURLs);
          for (let i = 0; i < numPrevious; i++) {
            const url = defectImageURLs[i + 1];
            $(".previous_image:eq(" + i + ")").attr(
              "src",
              url || "assets/img/white.jpg"
            );
          }
        },
      },

      "/gui/stopped_defect_image": {
        topicType: "weaving_inspection/StoppingDefectImages",
        type: "sub",
        callback: (message) => {
          // JSON.parse(message.stopping_img.header.frame_id);

          // // Convert main stopped defect image
          // const stoppedBlob = b64toBlob(
          //   message.stopping_img.data,
          //   "image/jpeg"
          // );

          // // 🧹 cleanup old URL if still held
          // if (defectStopImageURL) {
          //   URL.revokeObjectURL(defectStopImageURL);
          //   defectStopImageURL = null;
          // }

          // defectStopImageURL = URL.createObjectURL(stoppedBlob);
          // $("#stopped_defect_image").attr("src", defectStopImageURL);

          // // Clear zoom container
          // $("#stopped_defect_zoom_images").html("");
          // var container = $("#stopped_defect_zoom_images");

          // // Loop through defects
          // for (var i = 0; i < message.defects.length; i++) {
          //   const defectBlob = b64toBlob(message.defects[i].data, "image/jpeg");
          //   const defectUrl = URL.createObjectURL(defectBlob);

          //   var img = $("<img>", {
          //     class: "img-fluid",
          //     src: defectUrl,
          //   });
          //   img.on("load", function () {
          //     URL.revokeObjectURL(defectUrl);
          //   });
          // }
          let camNames = JSON.parse(message.stopping_img.header.frame_id);

          // Convert main stopped defect image
          const stoppedBlob = b64toBlob(
            message.stopping_img.data,
            "image/jpeg"
          );

          // Reuse the top-level tracker and revoke previous
          if (defectStopImageURL) {
            URL.revokeObjectURL(defectStopImageURL);
            defectStopImageURL = null;
          }

          defectStopImageURL = URL.createObjectURL(stoppedBlob);
          
          $("#stopped_defect_image").attr("src", defectStopImageURL);

          // Clear zoom container
          $("#stopped_defect_zoom_images").html("");
          var container = $("#stopped_defect_zoom_images");

          // Loop through defects
          for (var i = 0; i < message.defects.length; i++) {
            const defectBlob = b64toBlob(message.defects[i].data, "image/jpeg");
            const defectUrl = URL.createObjectURL(defectBlob);

            var img = $("<img>", {
              class: "img-fluid",
              src: defectUrl,
            });
            img.on("load", function () {
              URL.revokeObjectURL(defectUrl);
            });

            var defect_info = JSON.parse(message.defects[i].header.frame_id);

            // Create the paragraph element
            var paragraph = $("<p>", {
              text: `${defect_info["id"]} (${defect_info["type"]} ${defect_info["cam_pose"]})`,
            });

            // Create the span element
            var spanClassName = "";
            if (!defect_info["defect_disabled"]) {
              spanClassName = "zoom_defect_border";
            }
            var span = $("<span>", {
              class: `zoom_defect_span ${spanClassName}`,
            });

            span.append(paragraph);
            span.append(img);
            container.append(span);
          }

          // Show modal if not visible
          if (!$("#stopped_defect_image_modal").is(":visible")) {
            $("#stopped_defect_image_modal").modal("show");
          }

          // Populate camera names only once
          if ($("#stop_defect_camera_names_container").children().length == 0) {
            $("#stop_defect_camera_names_container").html("");
            camNames.forEach((elem) => {
              $("#stop_defect_camera_names_container").append(
                `<div class="stop_cam_name">${elem}</div>`
              );
            });
          }
        },
      },

      "/gui/value/version": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          $("#build_version").html(msg.build_version);
          $("#version").html(msg.version);
        },
      },

      "/gui/ack/job": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          $(".roll_toggle").prop("disabled", false);
        },
      },
      "/gui/value/stopping_window_close": {
        topicType: "std_msgs/Empty",
        type: "sub",
        callback: (message) => {
          if ($("#stopped_defect_image_modal").is(":visible")) {
            $("#stopped_defect_image_modal").modal("hide");
            $("#stopped_defect_zoom_images").html("");
          }
        },
      },

      "/gui/button/version": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      // ******
      // Define Publishers
      // ******
      "/gui/button/reset": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/btn_save_past_images": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/show_last_popup": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/btn_check_punch_block": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/btn_check_machine_stop": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/btn_check_buzzer": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/btn_false_positive": {
        topicType: "std_msgs/Int16",
        type: "pub",
      },

      "/gui/button/shutdown": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/value/threshold": {
        topicType: "std_msgs/Float32",
        type: "pub",
      },

      "/gui/value/exposure": {
        topicType: "std_msgs/Int16",
        type: "pub",
      },

      "/gui/checkbox/light_toggle": {
        topicType: "std_msgs/Bool",
        type: "pub",
      },

      "/gui/checkbox/stop_at_last_defect": {
        topicType: "std_msgs/Bool",
        type: "pub",
      },

      "/gui/checkbox/save_image": {
        topicType: "std_msgs/Bool",
        type: "pub",
      },

      "/gui/value/pixel_per_mm": {
        topicType: "std_msgs/Float32",
        type: "pub",
      },

      "/gui/value/pulse_per_mm": {
        topicType: "std_msgs/Float32",
        type: "pub",
      },

      "/gui/sync/configuration": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/onchange/metadata": {
        topicType: "std_msgs/String",
        type: "pub",
      },

      "/gui/checkbox/ignore_inspection": {
        topicType: "std_msgs/Bool",
        type: "pub",
      },

      "/gui/checkbox/data_collection": {
        topicType: "std_msgs/Bool",
        type: "pub",
      },

      "/gui/button/clear_data": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/upload_data": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },
      "/gui/button/write_cut_length": {
        topicType: "std_msgs/Int16",
        type: "pub",
      },
      "/gui/button/ai_cut_master_modal_open": {
        topicType: "std_msgs/Bool",
        type: "pub",
      },
      "/gui/button/cycle_start": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },
      "/gui/set/barcode": {
        topicType: "std_msgs/String",
        type: "pub",
      },
      "/gui/set/workOrder": {
        topicType: "std_msgs/String",
        type: "pub",
      },
      "/gui/button/continueAck": {
        topicType: "std_msgs/Int8",
        type: "pub",
      },
    });

    $("#open_barcode_modal").on("click", function () {
      $("#scan_barcode_input").val("");
      $("#scan_barcode_modal").modal("show");
    });
    $("#open_workorder_modal").on("click", function () {
      workOrderName.value = "";
      workOrderCutLength.value = "";
      workOrderTargetPcs.value = "";
      document.getElementById("completeTarget").value = "";
      document.getElementById("completeLength").value = "";
      document.getElementById("currectpcs").value = "";
      const selectworkorder = document.getElementById("selectWorkOrder");
      selectworkorder.selectedIndex = 0;
      $("#workorder_Modal").modal("show");
    });

    $("#scan_barcode_modal").on("shown.bs.modal", function () {
      $("#scan_barcode_input").focus();
    });

    $("#scan_barcode_input").on("keypress", function (event) {
      if (event.key === "Enter") {
        event.preventDefault();
        $("#barcode_submit").click();
      }
    });

    $("#barcode_submit").on("click", function () {
      barCode = $("#scan_barcode_input").val();
      if (barCode) {
        const parts = parseBarcodeByConfig(barCode);
        if (parts) {
          $("#jobSettingsModal").modal("show");
          $("#roll_id").val(parts[barcode_config.indexes.roll - 1]);
          $("#gsm").val(parts[barcode_config.indexes.gsm - 1]);
          $("#fabric_width").val(parts[barcode_config.indexes.width - 1]);
          $("#roll_weight").val(parts[barcode_config.indexes.rollweight - 1]);
          $("#roll_meter").val(parts[barcode_config.indexes.rollLength - 1]);
          $("#loom_num").val(parts[barcode_config.indexes.loom - 1]);
        }
      } else {
        new AWN().alert("Empty Barcode! Unable to parse.", {
          durations: {
            warning: 4000,
          },
        });
      }
    });

    var is_roll_started = false;
    var is_popup_on = false;
    const systemLoaderModal = new bootstrap.Modal(
      document.getElementById("systemLoaderModal"),
      {
        keyboard: false,
      }
    );
    systemLoaderModal.show();
    const setIgnoreInspectionAccess = (isAdmin) => {
      const toggle = $("#ignore_inspection");
      toggle.prop("disabled", false); // allow click handler to show alert
      toggle.attr("aria-disabled", !isAdmin);
      toggle.css("opacity", isAdmin ? "1" : "0.6");
    };
    adminState = localStorage.getItem("adminState");
    if (adminState == "true") {
      $("#threshold_value").prop("disabled", false);
      $("#camera_exposure").prop("disabled", false);
      $("#admin_enable_text").text("#Admin");
    } else {
      $("#admin_enable_text").text("Operator");
    }
    setIgnoreInspectionAccess(adminState == "true");
    const reloaderId = setTimeout(function () {
      location.reload();
    }, 3000);

    // ******
    // Functions
    // ******
    function updateRollState() {
      const roll_id = $("#roll_id");
      const gsm = $("#gsm");
      const offset = $("#offset");
      const cut_len = $("#cut_length");
      const fabric_width = $("#fabric_width");
      const roll_weight = $("#roll_weight");
      const punch = $("#punching_status");
      const recipes = $("#recipes");
      const layer = $("#layer");
      const cutting_mode = $("#cutting_mode");
      const secondary_cut_length = $("#secondary_cut_length");
      const tertiary_cut_length = $("#tertiary_cut_length");
      const roll_meter = $("#roll_meter");
      const loom_num = $("#loom_num");
      const work_order = document.getElementById("cut-length-job-index");
      const workOrdere = $("#cut-length-job-index");
      const batchCount = $("#batchCount");

      const state = is_roll_started;
      $("#stop_at_last_defect_toggle_button").prop("disabled", state);
      roll_id.prop("disabled", state);
      gsm.prop("disabled", state);
      offset.prop("disabled", state);
      cut_len.prop("disabled", state);
      fabric_width.prop("disabled", state);
      roll_weight.prop("disabled", state);
      punch.prop("disabled", state);
      recipes.prop("disabled", state);
      cutting_mode.prop("disabled", state);
      secondary_cut_length.prop("disabled", state);
      tertiary_cut_length.prop("disabled", state);
      roll_meter.prop("disabled", state);
      loom_num.prop("disabled", state);
      batchCount.prop("disabled", state);
      layer.prop("disabled", state);
      workOrdere.prop("disabled", state);
      $("#top_bottom_panel_selected").prop("disabled", state);
      $("#fully_ignore_disabled").prop("disabled", state);
      if (state) {
        disableRadioButtons("punching_radio");
        disableRadioButtons("cutting_mode_radio");
      } else {
        enableRadioButtons("punching_radio");
        enableRadioButtons("cutting_mode_radio");
      }

      const selectedWorkOrderId =
        work_order.selectedIndex > 0 ? work_order.value : "0";
      const metadata = {
        cut_length: parseInt(cut_len.val()),
        fabric_width: parseInt(fabric_width.val()),
        roll_weight: parseInt(roll_weight.val()),
        gsm: parseInt(gsm.val()),
        roll_id: roll_id.val(),
        offset: parseInt(offset.val()),
        punching_status: getPunchingStatus() === "YES",
        recipe_name: recipes.val(),
        layer_value: layer.val(),
        fully_ignore_disabled: $("#fully_ignore_disabled").prop("checked"),
        top_bottom_panel_selected: $("#top_bottom_panel_selected").prop(
          "checked"
        ),
        ignore_inspection: $("#ignore_inspection").prop("checked"),
        is_roll_started,
        id: generateUniqueString(),
        cutting_mode: getSelectedCuttingMode(),
        roll_length: parseFloat(roll_meter.val()),
        loom_id: loom_num.val(),
        work_order_id: selectedWorkOrderId,
        batch_count: parseInt(batchCount.val()),
      };
      if (metadata.cutting_mode == "recut") {
        var cut_lengths_arr = [parseInt(secondary_cut_length.val())];

        if (parseInt(tertiary_cut_length.val()) != 0) {
          cut_lengths_arr.push(parseInt(tertiary_cut_length.val()));
        }
        metadata.secondary_cut_lengths = cut_lengths_arr;
      }
      rosBridgeInstanceForJS.publish(
        "/gui/onchange/metadata",
        new ROSLIB.Message({
          data: JSON.stringify(metadata),
        })
      );
    }

    for (let i = 1; i <= 3; i++) {
      $(`#previous_defect_image_${i}`).click(function () {
        var src = $(this).attr("src");
        if (src !== "assets/img/white.jpg") {
          $("#previous_defect_image_modal").css("display", "block");
          $("#previous_defect_image").attr("src", src);
          if (i === 1) {
            previous_defect_frame_id = previous_defect_frame_id_1;
          } else if (i === 2) {
            previous_defect_frame_id = previous_defect_frame_id_2;
          } else if (i === 3) {
            previous_defect_frame_id = previous_defect_frame_id_3;
          } else {
            console.log("Error: Clicked wrong previous defect image");
          }
        }
      });
    }

    function hideMode(mode) {
      const radio = document.getElementById(mode);
      const label = document.querySelector(`label[for=${mode}]`);

      if (radio) radio.style.display = "none"; // Hide the radio button
      if (label) label.style.display = "none"; // Hide the corresponding label
    }

    function showMode(mode) {
      const radio = document.getElementById(mode);
      const label = document.querySelector(`label[for=${mode}]`);

      if (radio) radio.style.display = "inline-block"; // Show the radio button
      if (label) label.style.display = "inline-block"; // Show the corresponding label
    }

    $("#close_previous_defect_modal").click(() => {
      $("#previous_defect_image_modal").css("display", "none");
    });
    $("#reset").click(() => {
      let notifier = new AWN();
      let onOk = () => {
        rosBridgeInstanceForJS.publish("/gui/button/reset");
      };

      notifier.confirm(
        "Please only reset when machine is stopped",
        onOk,
        true,
        {
          labels: {
            confirm:
              "Are you Sure? This removes all defects between cutter & camera.",
          },
        }
      );
    });

    $(".cb-value").click(function () {
      var mainParent = $(this).parent(".toggle-btn");
      if ($(mainParent).find("input.cb-value").is(":checked")) {
        $(mainParent).addClass("active");
      } else {
        $(mainParent).removeClass("active");
      }
    });

    $("#btn_save_past_images").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/btn_save_past_images");
    });

    $("#btn_show_last_popup").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/show_last_popup");
    });

    $("#btn_check_punch_block").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/btn_check_punch_block");
    });

    $("#btn_check_machine_stop").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/btn_check_machine_stop");
    });

    $("#btn_check_buzzer").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/btn_check_buzzer");
    });

    $("#data_collection_clear").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/clear_data");
    });

    $("#data_collection_upload").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/upload_data");
    });

    $("#btn_false_positive").click(() => {
      rosBridgeInstanceForJS.publish(
        "/gui/button/btn_false_positive",
        new ROSLIB.Message({ data: parseInt(previous_defect_frame_id) })
      );
      $("#previous_defect_image_modal").css("display", "none");
    });

    $("#shutdown").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/shutdown");
      setTimeout(function () {
        window.location.href = "http://127.0.0.1:4100/all-app"; // Redirect to the new URL
      }, 1000); // 1-second delay
    });

    $("#change_access_btn").click(() => {
      if (adminState == "true") {
        $("#admin_enable_text").text("Operator");
        $("#threshold_value").prop("disabled", true);
        $("#camera_exposure").prop("disabled", true);
        adminState = "false";
        localStorage.setItem("adminState", adminState);
        setIgnoreInspectionAccess(false);
      } else {
        $("#user_access_modal").modal("show");
      }
    });

    $("#threshold_value").change(() => {
      rosBridgeInstanceForJS.publish(
        "/gui/value/threshold",
        new ROSLIB.Message({ data: parseFloat($("#threshold_value").val()) })
      );
    });

    $("#recipe_btn").click(() => {
      if (adminState == "true") {
        if (!is_roll_started) {
          window.location.href = "recipe_table.html";
        } else {
          new AWN()["alert"](
            "The Roll is started you can;t create or modify recipe."
          );
          return;
        }
      } else {
        new AWN()["alert"]("Unauthorized User.");
        return;
      }
    });

    $("#access_level_btn").click(() => {
      var password = "admin123";
      if ($("#admin_password").val() === password) {
        $("#threshold_value").prop("disabled", false);
        $("#camera_exposure").prop("disabled", false);
        $("#admin_enable_text").text("#Admin");
        adminState = "true";
        localStorage.setItem("adminState", adminState);
        setIgnoreInspectionAccess(true);
        new AWN()["success"]("User Authorized Successfully.");
      } else {
        adminState = "false";
        localStorage.setItem("adminState", adminState);
        setIgnoreInspectionAccess(false);
        new AWN()["alert"]("Wrong Password!");
      }
    });

    /*
     Accepting params from operator - region ends
    */
    $("#save_image").click(() => {
      if ($("#save_image").prop("checked") == true) {
        rosBridgeInstanceForJS.publish(
          "/gui/checkbox/save_image",
          new ROSLIB.Message({ data: true })
        );
      } else if ($("#save_image").prop("checked") == false) {
        rosBridgeInstanceForJS.publish(
          "/gui/checkbox/save_image",
          new ROSLIB.Message({ data: false })
        );
      }
    });

    $("#sidebarToggle").click(() => {
      if ($("#accordionSidebar").width() <= "55") {
        document.getElementById("report_color").style.background =
          "rgb(28, 78, 155)";
        document.getElementById("setting_color").style.background =
          "rgb(28, 78, 155)";
        document.getElementById("parameter_color").style.background =
          "rgb(28, 78, 155)";
      }
    });
    const barcodeInput = document.getElementById("barcode-input");
    const parsingTypeDropdown = document.getElementById("parsing-type");
    const setButton = document.getElementById("set-config");

    const sumbitWokrOrderButton = document.getElementById("sumbit-work-order");
    const completeWokrOrderButton = document.getElementById(
      "complete-work-order"
    );

    const workOrderName = document.getElementById("orderName");
    const workOrderCutLength = document.getElementById("cutLength");
    //const workOrderbBatchCount = document.getElementById("batchcount");
    const workOrderTargetPcs = document.getElementById("target");
    workOrderName.setAttribute("autocomplete", "off");
    workOrderCutLength.setAttribute("autocomplete", "off");
    //workOrderbBatchCount.setAttribute("autocomplete", "off");
    workOrderTargetPcs.setAttribute("autocomplete", "off");

    const valueInputs = {
      roll: document.getElementById("roll-value"),
      loom: document.getElementById("loom-value"),
      width: document.getElementById("width-value"),
      gsm: document.getElementById("gsm-value"),
      rollweight: document.getElementById("roll-weight-value"),
      rollLength: document.getElementById("roll-length-value"),
    };

    const indexDropdowns = {
      roll: document.getElementById("roll-index"),
      loom: document.getElementById("loom-index"),
      width: document.getElementById("width-index"),
      gsm: document.getElementById("gsm-index"),
      rollweight: document.getElementById("roll-weight-index"),
      rollLength: document.getElementById("roll-length-index"),
    };

    // Clear all fields and reset dropdowns
    function resetAllValues() {
      barcodeInput.value = "";
      Object.values(valueInputs).forEach((input) => (input.value = ""));
      Object.values(indexDropdowns).forEach((dropdown) => {
        dropdown.innerHTML = '<option value="NA">NA</option>'; // Reset with "NA" option
      });
    }

    function resetValues() {
      Object.values(valueInputs).forEach((input) => (input.value = ""));
      Object.values(indexDropdowns).forEach((dropdown) => {
        dropdown.innerHTML = '<option value="NA">NA</option>'; // Reset with "NA" option
      });
    }

    // Parse barcode into parts based on parsing type
    function parseBarcode(barcode) {
      const parsingType = parsingTypeDropdown.value;
      parsingTypeDropdown.style.border = "";
      if (!barcode.includes(parsingType)) {
        new AWN().alert(
          `Invalid Parsing type "${parsingType}" selection, unable to parse Barcode.`,
          {
            durations: {
              warning: 4000,
            },
          }
        );
        parsingTypeDropdown.style.border = "1px solid red";
        resetValues();
        return null;
      }

      return barcode.split(parsingType); // Split using the selected delimiter
    }

    function parseBarcodeByConfig(barcode) {
      if (barcode_config == null) {
        new AWN().alert(
          `Invalid Barcode Configuration found! Save a Barcode Config`,
          {
            durations: {
              warning: 4000,
            },
          }
        );
        return null;
      }
      const parsingType = barcode_config.parsingType;
      if (!parsingType || parsingType == "Undefined") {
        new AWN().alert(
          `Invalid Barcode Configuration found! Save a Barcode Config`,
          {
            durations: {
              warning: 4000,
            },
          }
        );
        return null;
      }

      if (!barcode.includes(parsingType)) {
        new AWN().alert(
          `Invalid Parsing type "${parsingType}" selection, unable to parse Barcode.`,
          {
            durations: {
              warning: 4000,
            },
          }
        );
        return null;
      }

      return barcode.split(parsingType); // Split using the selected delimiter
    }

    // Generate and display the index-to-value map
    function generateIndexMap(parts) {
      const map = {};
      parts.forEach((part, index) => {
        map[`Index (${index + 1})`] = part; // Start index from 1
      });
      console.log("Parsed Map:", map);
      return map;
    }

    // Function to set barcodeConfig in the UI based on configData
    function setBarcodeConfig(barcodeConfig = {}) {
      resetValues();
      const indexes = barcodeConfig.indexes || {};
      const parsingType = barcodeConfig.parsingType || ":";

      // Set the parsing type dropdown (if provided in barcodeConfig)
      parsingTypeDropdown.value = parsingType || "";
      parsingTypeDropdown.disabled = true;

      // Loop through indexDropdowns and set the values from barcodeConfig.indexes if available
      Object.keys(indexDropdowns).forEach((key) => {
        const dropdown = indexDropdowns[key];
        const indexValue = indexes[key]; // Get the index value from barcodeConfig.indexes

        // Clear any existing options in the dropdown
        dropdown.innerHTML = "";

        // Add "NA" option first
        const naOption = document.createElement("option");
        naOption.value = "NA";
        naOption.textContent = "NA";
        dropdown.appendChild(naOption);

        // Add available options based on barcodeConfig.indexes (sequential index options)
        // If the key is present in barcodeConfig.indexes, it will be considered
        var i = 1;
        Object.keys(indexes).forEach((indexKey, index) => {
          if (indexes[indexKey] != "NA") {
            const option = document.createElement("option");
            option.value = i; // 1-based index for option value
            option.textContent = `Index (${i})`; // Text content with 1-based index
            dropdown.appendChild(option);
            i += 1;
          }
        });

        // Set the dropdown value based on the barcodeConfig.indexes or "NA" fallback
        if (indexValue) {
          dropdown.value = indexValue; // Set to the config value if available
          dropdown.disabled = true; // Disable the dropdown if config is available
        } else {
          dropdown.value = "NA"; // Fallback to "NA" if config value is not present
        }
      });
    }

    // Populate index dropdowns dynamically
    function populateIndexes(parts, barcodeConfig = null) {
      var maxIndex = 0;
      if (barcodeConfig && Object.keys(barcodeConfig).length != 0) {
        // Extract the values from the `indexes` object
        const values = Object.values(barcodeConfig.indexes);
        // Filter out non-numeric values and convert to numbers
        const numericValues = values
          .filter((value) => !isNaN(value))
          .map(Number);
        // Find the maximum value
        maxIndex = Math.max(...numericValues);
      }
      Object.keys(indexDropdowns).forEach((key, sequentialIndex) => {
        // console.log(key, sequentialIndex);

        const dropdown = indexDropdowns[key];
        dropdown.innerHTML = ""; // Clear existing options
        dropdown.disabled = false; // Disable the dropdown if config is available

        // Add "NA" option
        const naOption = document.createElement("option");
        naOption.value = "NA";
        naOption.textContent = "NA";
        dropdown.appendChild(naOption);

        // Add parts as options
        parts.forEach((part, index) => {
          const option = document.createElement("option");
          option.value = index + 1; // Start index from 1
          option.textContent = `Index (${index + 1})`; // Display 1-based index
          dropdown.appendChild(option);
        });

        if (
          barcodeConfig &&
          barcodeConfig.parsingType &&
          barcodeConfig.parsingType == parsingTypeDropdown.value
        ) {
          // Set default value based on barcodeConfig or sequential index
          if (barcodeConfig.indexes && barcodeConfig.indexes[key] != "NA") {
            dropdown.value =
              barcodeConfig.indexes[key] <= parts.length
                ? barcodeConfig.indexes[key]
                : "NA"; // Use config if available
          } else {
            dropdown.value = maxIndex < parts.length ? maxIndex + 1 : "NA"; // Default sequential behavior

            if (maxIndex < parts.length) {
              maxIndex = maxIndex + 1;
            }
          }
        } else {
          dropdown.value =
            sequentialIndex < parts.length ? sequentialIndex + 1 : "NA"; // Default sequential behavior
        }
      });
    }

    // Update values in the fields based on selected indexes
    function updateValues(parts, barcodeConfig = null) {
      Object.keys(indexDropdowns).forEach((key) => {
        const selectedIndex = indexDropdowns[key].value; // Get selected index from the dropdown

        valueInputs[key].value =
          selectedIndex !== "NA" && selectedIndex <= parts.length
            ? parts[selectedIndex - 1] || "" // Adjust for 1-based index
            : ""; // Fill with an empty string if "NA" or out of range
      });
    }

    // Handle barcode parsing and update all fields
    function handleParsing() {
      const barcode = barcodeInput.value;

      if (barcode) {
        parsingTypeDropdown.disabled = false;
        const parts = parseBarcode(barcode);
        if (parts) {
          generateIndexMap(parts);
          populateIndexes(parts, barcode_config);
          updateValues(parts, barcode_config);
        }
      }
    }

    // Event listener for the Enter key
    barcodeInput.addEventListener("keypress", (event) => {
      if (event.key === "Enter") {
        handleParsing();
      }
    });

    // Event listener for parsing type changes
    parsingTypeDropdown.addEventListener("change", () => {
      $("#barcode-input").focus();
      const barcode = barcodeInput.value;
      if (barcode) {
        const parts = parseBarcode(barcode);
        if (!parts) {
          resetValues();
        } else {
          populateIndexes(parts, barcode_config);
          updateValues(parts, barcode_config);
        }
      } else {
        resetAllValues();
      }
    });

    barcodeInput.addEventListener("input", () => {
      var barcodeValue = barcodeInput.value;
      if (barcodeValue == "") {
        setBarcodeConfig(barcode_config);
      }
    });

    // Event listeners for dropdown changes
    Object.keys(indexDropdowns).forEach((key) => {
      indexDropdowns[key].addEventListener("change", () => {
        const barcode = barcodeInput.value;
        if (barcode) {
          const parts = parseBarcode(barcode);
          if (parts) {
            updateValues(parts, barcode_config);
          }
        }
      });
    });

    // Function to validate and generate a notification if any value is empty
    function validateBarcodeConfig(configData) {
      const { parsingType, indexes } = configData;

      // Reset styles for all dropdowns
      const dropdownIds = [
        "roll",
        "loom",
        "width",
        "gsm",
        "rollweight",
        "roll-length",
      ];
      dropdownIds.forEach((id) => {
        const elementId =
          id === "rollLength"
            ? "roll-length-index"
            : id === "rollweight"
            ? "roll-weight-index"
            : `${id}-index`;
        document.getElementById(elementId).style.border = ""; // Reset border
      });

      // Check if Roll ID is not set
      if (!parsingType || indexes.roll === "NA") {
        document.getElementById("roll-index").style.border = "1px solid red"; // Highlight Roll ID dropdown
        new AWN().alert(
          `[Saving Config Failed!] Roll ID is compulsory. Please ensure it is selected.`,
          {
            durations: {
              warning: 4000,
            },
          }
        );
        return false;
      }

      // Collect selected indexes in an array
      const selectedIndexes = Object.entries(indexes);

      // Find duplicates
      const duplicates = selectedIndexes.filter(
        ([key, value], index, arr) =>
          value !== "NA" && arr.filter(([_, v]) => v === value).length > 1
      );

      // Highlight duplicates with a red border
      duplicates.forEach(([key]) => {
        const elementId =
          key === "rollLength"
            ? "roll-length-index"
            : key === "rollweight"
            ? "roll-weight-index"
            : `${key}-index`;
        document.getElementById(elementId).style.border = "1px solid red";
      });

      if (duplicates.length > 0) {
        new AWN().alert(
          `[Saving Config Failed!] Duplicate indexes found. Please ensure all fields have unique selections.`,
          {
            durations: {
              warning: 4000,
            },
          }
        );
        return false;
      }

      return true; // All validations passed
    }

    sumbitWokrOrderButton.addEventListener("click", (event) => {
      event.preventDefault();

      let hasError = false;

      // Clear previous error messages and borders
      document.querySelectorAll(".error-msg").forEach((el) => el.remove());
      workOrderName.style.border = "";
      workOrderCutLength.style.border = "";
      //workOrderbBatchCount.style.border = "";
      workOrderTargetPcs.style.border = "";

      // Validate each field
      if (!workOrderName.value.trim()) {
        workOrderName.style.border = "2px solid red";
        showInlineError(workOrderName, "Field can't be empty");
        hasError = true;
      }

      // if (!workOrderbBatchCount.value.trim()) {
      //   workOrderbBatchCount.style.border = "2px solid red";
      //   showInlineError(workOrderbBatchCount, "Field can't be empty");
      //   hasError = true;
      // }

      // if (
      //   Number(workOrderbBatchCount.value) > Number(workOrderTargetPcs.value)
      // ) {
      //   workOrderbBatchCount.style.border = "2px solid red";
      //   showInlineError(
      //     workOrderbBatchCount,
      //     "Batch count catn't be greater that target pcs."
      //   );
      //   hasError = true;
      // }

      if (!workOrderCutLength.value.trim()) {
        workOrderCutLength.style.border = "2px solid red";
        showInlineError(workOrderCutLength, "Field can't be empty");
        hasError = true;
      }

      if (!workOrderTargetPcs.value.trim()) {
        workOrderTargetPcs.style.border = "2px solid red";
        showInlineError(workOrderTargetPcs, "Field can't be empty");
        hasError = true;
      }

      if (hasError) return;

      const WorkOrderData = {
        action: "create",
        data: {
          name: workOrderName.value,
          cut_length: parseInt(workOrderCutLength.value),
          //batch_count: parseInt(workOrderbBatchCount.value),
          target_pcs: parseInt(workOrderTargetPcs.value),
        },
      };

      rosBridgeInstanceForJS.publish(
        "/gui/set/workOrder",
        new ROSLIB.Message({
          data: JSON.stringify(WorkOrderData),
        })
      );

      $("#workorder_Modal").modal("hide");
    });

    // Helper function to show inline error
    function showInlineError(inputElement, message) {
      const errorEl = document.createElement("div");
      errorEl.className = "error-msg";
      errorEl.style.color = "red";
      errorEl.style.fontSize = "0.8rem";
      errorEl.textContent = message;
      inputElement.parentElement.appendChild(errorEl);
    }

    completeWokrOrderButton.addEventListener("click", (event) => {
      event.preventDefault(); // Prevent form submission and page refresh
      const selectworkorder = document.getElementById("selectWorkOrder");
      const selectedWorkOrderId = selectworkorder.value;
      const WorkOrderData = {
        action: "complete",
        data: {
          work_order_id: selectedWorkOrderId,
        },
      };
      rosBridgeInstanceForJS.publish(
        "/gui/set/workOrder",
        new ROSLIB.Message({
          data: JSON.stringify(WorkOrderData),
        })
      );

      $("#workorder_Modal").modal("hide");
    });

    // Set configuration button
    setButton.addEventListener("click", () => {
      const barcode = barcodeInput.value;

      if (barcode) {
        const parts = parseBarcode(barcode);
        if (parts) {
          const configData = {
            parsingType: parsingTypeDropdown.value,
            indexes: {
              loom: indexDropdowns.loom.value,
              gsm: indexDropdowns.gsm.value,
              roll: indexDropdowns.roll.value,
              rollweight: indexDropdowns.rollweight.value,
              width: indexDropdowns.width.value,
              rollLength: indexDropdowns.rollLength.value,
            },
          };

          if (validateBarcodeConfig(configData)) {
            rosBridgeInstanceForJS.publish(
              "/gui/set/barcode",
              new ROSLIB.Message({
                data: JSON.stringify(configData),
              })
            );
            $("#barcode_modal").modal("hide");
          }
        } else {
          new AWN().alert(
            `[Saving Config Failed!] Invalid Parsing type "${parsingTypeDropdown.value}" selection, unable to parse Barcode.`,
            {
              durations: {
                warning: 4000,
              },
            }
          );
        }
      }
    });
    function updateModalMessage(newMessage) {
      document.getElementById("dynamicMessage").textContent = newMessage;
    }

    function sidebarToggle() {
      if ($("#accordionSidebar").width() <= "55") {
        document.getElementById("sidebarToggle").click();
      }
    }

    function sidebarColorToggle(color, navbar, item) {
      var background = $("#" + color).css("background-color");
      if (background === "rgb(28, 78, 155)") {
        $("#" + color).css({
          background: "rgba(255, 255, 255, 0.21)",
          borderBottomWidth: "0px",
        });
        $("#" + navbar).css("border-color", "transparent");
        $("#" + item).css({
          borderBottomStyle: "solid",
          borderColor: "rgb(255, 255, 255)",
        });
      } else {
        $("#" + color).css("background", "rgb(28, 78, 155)");
        $("#" + navbar).css("border-color", "rgb(255, 255, 255)");
        $("#" + item).css("border-bottom-style", "none");
      }
    }

    function displayConnectionDetails(details, id) {
      const container = document.getElementById(id);
      for (const [key, value] of Object.entries(details)) {
        if (typeof value === "object") {
          for (const [key1, value1] of Object.entries(value)) {
            const label = document.createElement("label");
            label.textContent = `${key1}: ${value1}`;
            container.appendChild(label);
            container.appendChild(document.createElement("br"));
          }
        } else {
          const label = document.createElement("label");
          label.innerHTML = `<b>${key}</b>: ${value}`;
          container.appendChild(label);
          container.appendChild(document.createElement("br"));
        }
      }
    }

    $("#report_color").click(() => {
      sidebarToggle();
      sidebarColorToggle("report_color", "report_navbar", "report_item");
    });

    $("#setting_color").click(() => {
      sidebarToggle();
      sidebarColorToggle("setting_color", "setting_navbar", "setting_item");
    });

    $("#parameter_color").click(() => {
      sidebarToggle();
      sidebarColorToggle(
        "parameter_color",
        "parameter_navbar",
        "parameter_item"
      );
    });

    $("#ignore_inspection").click(() => {
      const isChecked = $("#ignore_inspection").prop("checked");
      if (adminState !== "true") {
        $("#ignore_inspection").prop("checked", !isChecked);
        new AWN()["alert"]("Unauthorized User.");
        return;
      }
      rosBridgeInstanceForJS.publish(
        "/gui/checkbox/ignore_inspection",
        new ROSLIB.Message({ data: isChecked })
      );
    });

    $("#refreshButton").on("click", function () {
      $("#refreshButton").addClass("rotating_sync");
      setTimeout(function () {
        $("#refreshButton").removeClass("rotating_sync");
        location.reload(true);
      }, 1500);
    });

    // setInterval(function () {
    //   $("#refreshButton").addClass("rotating_sync");
    //   setTimeout(function () {
    //     $("#refreshButton").removeClass("rotating_sync");
    //     location.reload(true);
    //   }, 1500);
    // }, 1800000); // 30 minutes

    $("#data_collection").click(() => {
      rosBridgeInstanceForJS.publish(
        "/gui/checkbox/data_collection",
        new ROSLIB.Message({ data: $("#data_collection").prop("checked") })
      );
    });

    $("#camera_exposure").change(() => {
      const exposure = parseInt($("#camera_exposure").val());
      rosBridgeInstanceForJS.publish(
        "/gui/value/exposure",
        new ROSLIB.Message({ data: exposure })
      );
    });

    $("#pixel_per_mm").change(() => {
      const my_val = parseFloat($("#pixel_per_mm").val());
      rosBridgeInstanceForJS.publish(
        "/gui/value/pixel_per_mm",
        new ROSLIB.Message({ data: my_val })
      );
    });

    $("#pulse_per_mm").change(() => {
      const my_val = parseFloat($("#pulse_per_mm").val());
      rosBridgeInstanceForJS.publish(
        "/gui/value/pulse_per_mm",
        new ROSLIB.Message({ data: my_val })
      );
    });
    function clearInput(inputId) {
      var inputField = $("#" + inputId);

      // Check if the input field is not disabled before clearing its value
      if (!inputField.prop("disabled")) {
        inputField.val("");
        inputField.focus();
      }
    }

    $(".close-icon").on("click", function () {
      // Get the input field id from the data attribute
      var inputId = $(this).siblings("input").attr("id");

      // Add glow effect to the icon
      var icon = $(this).find("i");
      icon.addClass("glow-icon");

      // Remove the glow class after a delay (adjust the timeout value as needed)
      setTimeout(function () {
        icon.removeClass("glow-icon");
      }, 500);

      clearInput(inputId);
    });

    // $("#light_toggle_button").change(() => {
    //   rosBridgeInstanceForJS.publish(
    //     "/gui/checkbox/light_toggle",
    //     new ROSLIB.Message({ data: $("#light_toggle_button").prop("checked") })
    //   );
    // });

    $("#stop_at_last_defect_toggle_button").change(() => {
      rosBridgeInstanceForJS.publish(
        "/gui/checkbox/stop_at_last_defect",
        new ROSLIB.Message({
          data: $("#stop_at_last_defect_toggle_button").prop("checked"),
        })
      );
    });

    $(".accordion-item").click(() => {
      if ($("#navigation_bar").hasClass("toggled")) {
        $("#sidebarToggle").click();
      }
    });

    setTimeout(function () {
      rosBridgeInstanceForJS.publish("/gui/sync/configuration");
      rosBridgeInstanceForJS.publish("/gui/button/version");
    }, 500);
  });
});
