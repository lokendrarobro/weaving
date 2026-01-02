$(document).ready(function () {
  var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
  });

  ros.on("connection", function () {
    console.log("Connected");
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

    const rosBridgeInstanceForJS = new RosBridgeForJS({
      "/gui/value/config": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          try {
            jsonData = JSON.parse(message.data);

            populateDataTable(jsonData);
          } catch (error) {
            console.error("Error parsing JSON data:", error);
          }
        },
      },

      "/gui/label/notification": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          if (msg.severity === "popup") {
            let notifier = new AWN();
            let onOk = () => {
              notifier.warn("Warning");
            };
            notifier.confirm(msg.msg, onOk, false, {
              labels: {
                confirm: "Warning",
              },
            });
            return;
          }
          new AWN()[msg.severity](msg.msg, {
            durations: {
              info: 2000,
              success: 2000,
              alert: 2000,
              warning: 2000,
            },
          });
        },
      },
      "/gui/sync/all_nn_files": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var nnFiles = JSON.parse(message.data);
          // Get the select elements
          const namesFilesSelect = document.getElementById("namesFiles");
          const engineFilesSelect = document.getElementById("engineFiles");

          // Function to populate a select tag with options
          function populateSelect(selectElement, options) {
            selectElement.innerHTML = ""; // Add this line to clear previous options
            options.forEach((optionValue) => {
              const option = document.createElement("option");
              option.value = optionValue;
              option.textContent = optionValue;
              selectElement.appendChild(option);
            });
            // Select the first option
            if (selectElement.options.length > 0) {
              selectElement.selectedIndex = 0;
            }
          }

          // Populate the select tags with the data
          defectGroupNamemap = nnFiles.names_data;
          populateSelect(namesFilesSelect, nnFiles.names);
          populateSelect(engineFilesSelect, nnFiles.engines);
          setDefaultValues();
        },
      },
      "/gui/value/version": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var msg = JSON.parse(message.data);
          $("#build_version").html(msg.build_version)
          $("#version").html(msg.version);
        },
      },
      // ******
      // Define Publishers
      // ******
      "/gui/button/version": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },
      "/gui/button/save_json": {
        topicType: "std_msgs/String",
        type: "pub",
      },

      "/gui/button/shutdown": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/button/json_editor": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },

      "/gui/sync/recipes": {
        topicType: "std_msgs/String",
        type: "pub",
      },

      "/gui/save/recipes": {
        topicType: "std_msgs/String",
        type: "pub",
      },

      "/gui/list/all_nn_files": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },
    });

    setTimeout(function () {
      rosBridgeInstanceForJS.publish("/gui/list/all_nn_files");
      rosBridgeInstanceForJS.publish("/gui/button/version");
      rosBridgeInstanceForJS.publish("/gui/button/json_editor");
    }, 200);

    $("#refreshButton").on("click", function () {
      $("#refreshButton").addClass("rotating_sync");
      setTimeout(function () {
        $("#refreshButton").removeClass("rotating_sync");
        location.reload();
      }, 1500);
    });

    const namesFilesSelect = document.getElementById("namesFiles");
    namesFilesSelect.addEventListener("change", function () {
      const selectedName = namesFilesSelect.value;

      var selectedGroup = defectGroupNamemap[selectedName];
      document.getElementById("NumClasses").value = selectedGroup.length;

      defect_group_name = {};
      selectedGroup.forEach((groupName, index) => {
        defect_group_name[index] = groupName;
      });
      defectGroupName = defect_group_name;
      showGroupNames(defect_group_name);
    });

    var defectGroupNamemap = {};

    var selectedRecipe;

    var defectGroupName = {
      0: "hole",
      1: "shuttle_fault",
      2: "warp_break_hole",
      3: "tiny_hole",
      4: "contamination",
      5: "weft_missing",
      6: "small_shuttle_group",
      7: "small_shuttle_fault",
      8: "loose_thread",
      9: "warp_damage",
      10: "weft_damage",
      11: "edge_damage",
      12: "gapping",
      13: "warp_missing",
      14: "weft_patch",
      15: "manual_marking",
      16: "sticker",
    };
    console.log("the defect groups is:", defectGroupName);
    function validateInputs(isModify = false) {
      const errors = [];
      const prefix = isModify ? "modify" : "";

      const recipeNameSelector = `#${prefix}RecipeName`;
      const nmsThresholdSelector = `#${prefix}NmsThreshold`;
      const exposureSelector = `#${prefix}Exposure`;
      const numClassesSelector = `#${prefix}NumClasses`;
      // const thresholdSelector = `#${prefix}Threshold`;
      //const sensitivityXSelector = `#${prefix}Sensitivity_x`;

      const recipeName = $(recipeNameSelector).val();

      const safeChars = /^(?=.*[a-zA-Z])[a-zA-Z0-9_\-]+$/;

      // Validate the Recipe Name
      const isValidRecipeName = safeChars.test(recipeName);

      if (!isValidRecipeName) {
        errors.push("Invalid Recipe Name");
        $(recipeNameSelector).css("border-color", "red");
      } else {
        if (!isModify) {
          // Ensure no other recipe is present by that name in jsonData
          const existingRecipeNames = Object.keys(jsonData);
          if (existingRecipeNames.includes(recipeName)) {
            errors.push("Recipe Name already exists");
            $(recipeNameSelector).css("border-color", "red");
          } else {
            $(recipeNameSelector).css("border-color", "");
          }
        } else {
          $(recipeNameSelector).css("border-color", "");
        }
      }

      const nmsThreshold = parseFloat($(nmsThresholdSelector).val());
      const isValidNmsThreshold =
        !isNaN(nmsThreshold) && nmsThreshold > 0 && nmsThreshold <= 1;
      if (!isValidNmsThreshold) {
        errors.push("Invalid NmsThreshold (should be between 0 and 1)");
        $(nmsThresholdSelector).css("border-color", "red");
      } else {
        $(nmsThresholdSelector).css("border-color", "");
      }

      const exposure = parseFloat($(exposureSelector).val());
      const isValidExposure =
        !isNaN(exposure) && exposure >= 4 && exposure <= 400;
      if (!isValidExposure) {
        errors.push("Invalid Exposure (should be between 4 and 400)");
        $(exposureSelector).css("border-color", "red");
      } else {
        $(exposureSelector).css("border-color", "");
      }

      const numClasses = parseFloat($(numClassesSelector).val());
      const isValidNumClasses = !isNaN(numClasses) && numClasses > 0;
      if (!isValidNumClasses) {
        errors.push("Invalid NumClasses");
        $(numClassesSelector).css("border-color", "red");
      } else {
        $(numClassesSelector).css("border-color", "");
      }

      let hasInvalidInput = false; // Boolean to track if any row has invalid input

      var tableSelector = isModify
        ? "#modifyGroupInfoTable"
        : "#groupInfoTable";

      $(tableSelector + " tbody tr").each(function () {
        var groupName = $(this).find("td:eq(1)").text();
        var groupId = Object.keys(defectGroupName).find(
          (key) => defectGroupName[key] === groupName
        );
        if (groupId !== undefined) {
          var enabled = $(this).find(".enabled-input").prop("checked");
          if (groupName === "hole" && groupName === "shuttle_fault") {
            enabled = true;
          }

          var sensitivityX = parseFloat(
            $(this).find(".sensitivity-x-input").val()
          );
          var sensitivityY = parseFloat(
            $(this).find(".sensitivity-y-input").val()
          );
          var threshold = parseFloat($(this).find(".threshold-input").val());

          const isValidSensitivityX =
            !isNaN(sensitivityX) && sensitivityX > 0 && sensitivityX <= 100;
          const isValidSensitivityY =
            !isNaN(sensitivityY) && sensitivityY > 0 && sensitivityY <= 100;
          const isValidThreshold =
            !isNaN(threshold) && threshold >= 0.1 && threshold <= 0.9;

          // Update border colors for sensitivity X
          if (!isValidSensitivityX) {
            $(this).find(".sensitivity-x-input").css("border-color", "red");
            hasInvalidInput = true; // Mark as invalid
          } else {
            $(this).find(".sensitivity-x-input").css("border-color", "");
          }

          // Update border colors for sensitivity Y
          if (!isValidSensitivityY) {
            $(this).find(".sensitivity-y-input").css("border-color", "red");
            hasInvalidInput = true; // Mark as invalid
          } else {
            $(this).find(".sensitivity-y-input").css("border-color", "");
          }

          // Update border colors for threshold
          if (!isValidThreshold) {
            $(this).find(".threshold-input").css("border-color", "red");
            hasInvalidInput = true; // Mark as invalid
          } else {
            $(this).find(".threshold-input").css("border-color", "");
          }
        } else {
          console.log("Group name not found:", groupName);
        }
      });

      // Log the boolean to indicate if there was any invalid input
      if (hasInvalidInput) errors.push("Invalid group input.");

      /*  const threshold = parseFloat($(thresholdSelector).val());
        const isValidThreshold =
          !isNaN(threshold) && threshold > 0 && threshold <= 1;
        if (!isValidThreshold) {
          errors.push("Invalid Threshold (should be between 0 and 1)");
          $(thresholdSelector).css("border-color", "red");
        } else {
          $(thresholdSelector).css("border-color", "");
        }
          */

      if (errors.length > 0) {
        // Display an error message or handle invalid inputs
        const errorMessage = errors.join("\n and \n");
        new AWN().alert(errorMessage, {
          durations: {
            warning: 4000,
          },
        });

        return false;
      }

      return true;
    }

    // Example usage:
    // validateInputs(); // for original inputs
    // validateInputs(true); // for modified inputs

    let jsonData = {
      DEFAULT: {
        camera: {
          exposure_time: 200,
        },
        nn: {
          disabled_class_ids: [3, 8, 6, 4, 2, 12, 7],
          groups: {
            1: {
              ids: [1],
              sensitivity: 95,
              enabled: true,
            },
            2: {
              ids: [2],
              sensitivity: 95,
              enabled: true,
            },
            3: {
              ids: [3],
              sensitivity: 95,
              enabled: true,
            },
            4: {
              ids: [4],
              sensitivity: 93,
              enabled: false,
            },
            5: {
              ids: [5],
              sensitivity: 95,
              enabled: true,
            },
            6: {
              ids: [6],
              sensitivity: 98,
              enabled: true,
            },
            7: {
              ids: [7],
              sensitivity: 95,
              enabled: true,
            },
            8: {
              ids: [8],
              sensitivity: 97,
              enabled: true,
            },
            9: {
              ids: [9],
              sensitivity: 95,
              enabled: true,
            },
            10: {
              ids: [10],
              sensitivity: 95,
              enabled: true,
            },
            11: {
              ids: [11],
              sensitivity: 95,
              enabled: true,
            },
            12: {
              ids: [12],
              sensitivity: 95,
              enabled: true,
            },
            13: {
              ids: [13],
              sensitivity: 95,
              enabled: true,
            },
            14: {
              ids: [14],
              sensitivity: 95,
              enabled: true,
            },
          },
          nms_threshold: 0.65,
          num_classes: 20,
          threshold: 0.1,
          tiling: true,
          yolov8: {
            engine_file: "fibc_network_25.engine",
            names_file: "fibc_network_25.names",
          },
        },
      },
    };

    function populateDataTable(jsonData) {
      // Iterate through the JSON data and populate the DataTable
      var index = 0; // Initialize an index counter

      $.each(jsonData, function (recipeName, recipeData) {
        if (recipeName !== "DEFAULT") {
          var nnCell =
            recipeData.nn.yolov8.names_file +
            "<br>" +
            recipeData.nn.yolov8.engine_file;
          const namesFilesSelect = document.getElementById("namesFiles");
          namesFilesSelect.value = recipeData.nn.yolov8.names_file;

          const engineFilesSelect = document.getElementById("engineFiles");
          engineFilesSelect.value = recipeData.nn.yolov8.engine_file;

          var group_names = defectGroupNamemap[recipeData.nn.yolov8.names_file];
          if (!group_names || group_names.length === 0)
          {
              rosBridgeInstanceForJS.publish("/gui/list/all_nn_files");
              rosBridgeInstanceForJS.publish("/gui/button/version");
              rosBridgeInstanceForJS.publish("/gui/button/json_editor");
              return;
            }

          // Show defect information for the default groups
          defect_group_name = {};
          group_names.forEach((groupName, index) => {
            defect_group_name[index] = groupName;
          });
          defectGroupName = defect_group_name;

          var groupsHtml = getGroupsInfo(recipeData.nn.groups);
          namesFiles[recipeName] = recipeData.nn.yolov8.names_file;
          engineFiles[recipeName] = recipeData.nn.yolov8.engine_file;
          var row = [
            recipeName,
            recipeData.camera.exposure_time,
            recipeData.nn.nms_threshold,
            // recipeData.nn.threshold,
            groupsHtml, // Insert group info HTML into the row
            nnCell,
            '<div class="recipe-operations"><i id="modify-recipe" class="fas fa-pencil-alt"></i><i id="delete-recipe" class="fas fa-trash-alt"></i></div>',
          ];
          var table = $("#recipeTable").DataTable();
          var newRow = table.row.add(row).draw(false).node();

          // Disable delete-recipe icon for the first key in jsonData
          if (index === 0) {
            selectedRecipe = recipeName;
            $(newRow)
              .find("#delete-recipe")
              .addClass("disabled")
              .css("pointer-events", "none") // Disable click
              .css("color", "grey") // Change color to grey
              .css("opacity", "0.5");
          }

          // Increment the index counter
          index++;

          // Add click event handler to delete button
          $(newRow);
        }
      });
    }

    function deleteRecipe(recipeName) {
      if (jsonData.hasOwnProperty(recipeName)) {
        delete jsonData[recipeName];
        //delete from namesFiles and engineFiles
        delete namesFiles[recipeName];
        delete engineFiles[recipeName];
        console.log("Updated JSON Data:", jsonData);
        rosBridgeInstanceForJS.publish(
          "/gui/button/save_json",
          new ROSLIB.Message({
            data: JSON.stringify(jsonData) + "<D", //<D representing delete operation
          })
        );
        return jsonData; // Return the updated JSON data
      } else {
        console.log("Recipe not found:", recipeName);
        return null;
      }
    }
    let rownumber = 1;

    function createInputGroupHtml(
      groupId,
      groupName,
      imageSrc,
      sensitivity_x,
      sensitivity_y,
      threshold,
      enabled
    ) {
      var X_sensitivityInput = "";
      var Y_sensitivityInput = "";
      var thresholdInput = "";
      var enabledInput = "";

      if (groupName !== "hole" && groupName !== "shuttle_fault") {
        enabledInput =
          '<label class="checkbox-container" style="background-color: ' +
          (enabled ? "#95002d" : "#666666") +
          '">' +
          '<input type="checkbox" class="form-check-input enabled-input" ' +
          (enabled ? "checked" : "") +
          ">" +
          '<span class="checkmark" style="display: ' +
          (enabled ? "inline" : "none") +
          '"><i class="fas fa-check"></i></span>' +
          '<span class="crossmark" style="display: ' +
          (enabled ? "none" : "inline") +
          '"><i class="fas fa-times"></i></span>' +
          "</label>";
      } else {
        enabledInput =
          '<label class="checkbox-container disabled-group" style="background-color: ' +
          (enabled ? "#95002d" : "#666666") +
          '">' +
          '<input type="checkbox" class="form-check-input enabled-input" checked disabled>' + // Set the default value to checked
          '<span class="checkmark" style="display: ' +
          (enabled ? "inline" : "none") +
          '"><i class="fas fa-check"></i></span>' +
          '<span class="crossmark" style="display: ' +
          (enabled ? "none" : "inline") +
          '"><i class="fas fa-times"></i></span>' +
          "</label>";
      }

      // if (groupName !== "shuttle_fault") {
      // sensitivityInput =
      //   '<div class="sensitivity-container">' +
      //   '<div class="range-slider ' +
      //   (!enabled ? "disabled" : "") +
      //   '" style="--min:1; --max:100; --value:' +
      //   sensitivity +
      //   "; --text-value:" +
      //   sensitivity +
      //   '; --suffix:%">' +
      //   '<input type="range" class="custom-range sensitivity-range enable-range" min="1" max="100" step="1" value="' +
      //   sensitivity +
      //   '" data-toggle="tooltip" data-placement="top" title="Sensitivity: ' +
      //   sensitivity +
      //   '" ' +
      //   (!enabled ? "disabled" : "") +
      //   " oninput=\"this.parentNode.style.setProperty('--value', this.value); this.parentNode.style.setProperty('--text-value', this.value); this.nextElementSibling.textContent = this.value;\">" +
      //   "<output class='output-sensitivity'>" +
      //   sensitivity +
      //   "</output>" +
      //   '<div class="range-slider__progress"></div>' +
      //   "</div>" +
      //   "</div>";
      // } else {
      //   sensitivityInput =
      //     '<div class="sensitivity-container disabled">' +
      //     '<div class="range-slider disabled' +
      //     (!enabled ? "disabled" : "") +
      //     '" style="--min:1; --max:100; --value:' +
      //     sensitivity +
      //     "; --text-value:" +
      //     sensitivity +
      //     '; --suffix:%">' +
      //     '<input type="range" class="custom-range sensitivity-range" disabled min="1" max="100" step="1" value="' +
      //     sensitivity +
      //     '" data-toggle="tooltip" data-placement="top" title="Sensitivity: ' +
      //     sensitivity +
      //     '" ' +
      //     (!enabled ? "disabled" : "") +
      //     " oninput=\"this.parentNode.style.setProperty('--value', this.value); this.parentNode.style.setProperty('--text-value', this.value); this.nextElementSibling.textContent = this.value;\">" +
      //     "<output class='output-sensitivity'>" +
      //     sensitivity +
      //     "</output>" +
      //     '<div class="range-slider__progress" disabled></div>' +
      //     "</div>" +
      //     "</div>";
      // }

      // Add a condition to disable the enabledInput if groupName is "hole"

      // Sensitivity X input
      sensitivity_x = sensitivity_x || 100;
      X_sensitivityInput =
        '<div class="sensitivity-container">' +
        '<input type="text" class="form-control sensitivity-x-input enable-range" ' +
        'id="sensitivity-x-' +
        rownumber +
        '" ' +
        'value="' +
        sensitivity_x +
        '" ' +
        'placeholder="Sensitivity X (1-100)" ' +
        (enabled ? "" : "disabled") +
        " required " +
        " oninput=\"if(this.value === '') this.value = ''; else if(this.value < 1) this.value = 1; else if(this.value > 100) this.value = 100; if(!this.value.match(/^\\d*$/)) this.value = this.value.replace(/[^\\d]/g, '')\">" +
        "</div>";

      // Sensitivity Y input
      sensitivity_y = sensitivity_y || 100;
      Y_sensitivityInput =
        '<div class="sensitivity-container">' +
        '<input type="text" class="form-control sensitivity-y-input enable-range" ' +
        'id="sensitivity-y-' +
        rownumber +
        '" ' +
        'value="' +
        sensitivity_y +
        '" ' +
        'placeholder="Sensitivity Y (1-100)" ' +
        (enabled ? "" : "disabled") + // Light gray when disabled
        " oninput=\"if(this.value === '') this.value = ''; else if(this.value < 1) this.value = 1; else if(this.value > 100) this.value = 100; if(!this.value.match(/^\\d*$/)) this.value = this.value.replace(/[^\\d]/g, '')\">" +
        "</div>";

      // Threshold input
      threshold = threshold || 0.2;
      thresholdInput =
        '<div class="sensitivity-container">' +
        '<input type="text" class="form-control threshold-input enable-range" ' +
        'id="threshold-' +
        rownumber +
        '" ' +
        'value="' +
        threshold +
        '" ' +
        'placeholder="Threshold (0.1-0.9)" ' +
        (enabled ? "" : "disabled") + // Light gray when disabled
        " oninput=\"this.value = this.value.replace(/[^0-9.]/g, '');" + // Allow only numbers and one decimal point
        " if (this.value.includes('.')) { this.value = this.value.replace(/(\\..*?)\\..*/g, '$1'); }" + // Prevent multiple dots
        " if (/^\\d*\\.\\d{3,}$/.test(this.value)) { this.value = parseFloat(this.value).toFixed(2).replace(/\\.00$/, ''); }" + // Allow up to 2 decimal places
        " if (parseFloat(this.value) > 0.9) { this.value = '0.9'; }\"" + // Cap input at 0.9
        ' onblur="if (this.value) { this.value = Math.min(Math.max(parseFloat(this.value), 0.1), 0.9); }">' + // Validate range on blur
        "</div>";

      rownumber++;

      // var defectPhoto =
      //   '<img class="defect-group-photo" src="' +
      //   imageSrc +
      //   '" alt="' +
      //   groupName +
      //   '">';

      // Event delegation for handling enabled input change
      $(document).on("change", ".enabled-input", function () {
        var $row = $(this).closest("tr");
        var $sensitivityInput = $row.find(".enable-range");
        $sensitivityInput.prop("disabled", !this.checked);
        $sensitivityInput
          .closest(".range-slider")
          .toggleClass("disabled", !this.checked);
      });
      return [
        // defectPhoto,
        groupName,
        X_sensitivityInput,
        Y_sensitivityInput,
        thresholdInput,
        enabledInput,
      ];
    }

    function showGroupNames(groupnames) {
      // Clear previous data
      $("#modifyGroupInfoTable tbody").empty();
      $("#groupInfoTable tbody").empty();

      // Iterate over groups to display defect information
      $.each(groupnames, function (groupId, groupInfo) {
        var groupName = groupnames[groupId] || "Unknown511";
        var sensitivity_x = groupInfo.sensitivity_x;
        var sensitivity_y = groupInfo.sensitivity_y;
        var threshold = groupInfo.threshold;
        var enabled = true;
        var imageSrc = "/assets/img/defect_groups/" + groupId + ".jpg";

        // Create a row for each defect group with input fields
        var row = createInputGroupHtml(
          groupId,
          groupName,
          imageSrc,
          sensitivity_x,
          sensitivity_y,
          threshold,
          enabled
        );

        // Append the row to the table body
        var defectRow = "<tr><td>" + row.join("</td><td>") + "</td></tr>";
        $("#modifyGroupInfoTable tbody").append(defectRow);
        $("#groupInfoTable tbody").append(defectRow);

        // Toggle the visibility of checkmark/crossmark based on enabled state
        var $checkboxContainer = $(".checkbox-container").last();
        var $checkmark = $checkboxContainer.find(".checkmark");
        var $crossmark = $checkboxContainer.find(".crossmark");

        if (enabled) {
          $checkmark.show();
          $crossmark.hide();
        } else {
          $checkmark.hide();
          $crossmark.show();
        }
      });

      // Handle checkbox change event
      $(".enabled-input").change(function () {
        var $checkbox = $(this);
        var $checkboxContainer = $checkbox.closest(".checkbox-container");
        var isChecked = $checkbox.prop("checked");
        var $checkmark = $checkboxContainer.find(".checkmark");
        var $crossmark = $checkboxContainer.find(".crossmark");

        // Toggle background color and visibility of checkmark/crossmark based on checkbox state
        $checkboxContainer.css(
          "background-color",
          isChecked ? "#95002d" : "#666666"
        );
        console.log(isChecked, "===============");
        $checkmark.toggle(isChecked);
        $crossmark.toggle(!isChecked);
      });
    }

    function showDefectInfo(groups, groupNames) {
      // Clear previous data
      $("#modifyGroupInfoTable tbody").empty();
      $("#groupInfoTable tbody").empty();

      console.log(groups);
      console.log(groupNames);

      // Iterate over groups to display defect information
      $.each(groups, function (groupId, groupInfo) {
        console.log(groupInfo);
        var groupName = groupNames[groupId] || "Unknown511";
        var sensitivity_x = groupInfo.sensitivity_x;
        var sensitivity_y = groupInfo.sensitivity_y;
        var threshold = groupInfo.threshold;
        var enabled = groupInfo.enabled;
        var imageSrc = "/assets/img/defect_groups/" + groupId + ".jpg";

        // Create a row for each defect group with input fields
        var row = createInputGroupHtml(
          groupId,
          groupName,
          imageSrc,
          sensitivity_x,
          sensitivity_y,
          threshold,
          enabled
        );

        // Append the row to the table body
        var defectRow = "<tr><td>" + row.join("</td><td>") + "</td></tr>";
        $("#modifyGroupInfoTable tbody").append(defectRow);
        $("#groupInfoTable tbody").append(defectRow);

        // Toggle the visibility of checkmark/crossmark based on enabled state
        var $checkboxContainer = $(".checkbox-container").last();
        var $checkmark = $checkboxContainer.find(".checkmark");
        var $crossmark = $checkboxContainer.find(".crossmark");

        if (enabled) {
          $checkmark.show();
          $crossmark.hide();
        } else {
          $checkmark.hide();
          $crossmark.show();
        }
      });

      // Handle checkbox change event
      $(".enabled-input").change(function () {
        var $checkbox = $(this);
        var $checkboxContainer = $checkbox.closest(".checkbox-container");
        var isChecked = $checkbox.prop("checked");
        var $checkmark = $checkboxContainer.find(".checkmark");
        var $crossmark = $checkboxContainer.find(".crossmark");

        // Toggle background color and visibility of checkmark/crossmark based on checkbox state
        $checkboxContainer.css(
          "background-color",
          isChecked ? "#95002d" : "#666666"
        );
        console.log(isChecked, "===============");
        $checkmark.toggle(isChecked);
        $crossmark.toggle(!isChecked);
      });
    }

    $("#recipeTable").on("click", "#modify-recipe", function () {
      var row = $(this).closest("tr");

      var recipeName = row.find("td:eq(0)").text();
      console.log("JSON Data:", jsonData);

      var recipeData = jsonData[recipeName];
      $("#modifyRecipeName").val(recipeName);

      $("#modifyRecipeName").prop("disabled", true);

      $("#modifyExposure").val(recipeData.camera.exposure_time);
      $("#modifyNmsThreshold").val(recipeData.nn.nms_threshold);
      $("#modifyNumClasses").val(recipeData.nn.num_classes);
      $("#modifyThreshold").val(recipeData.nn.threshold);
      $("#modifyTiling").val(recipeData.nn.tiling ? "Yes" : "No");
      $("#modifyNamesFiles")
        .val(recipeData.nn.yolov8.names_file)
        .prop("disabled", true);
      $("#modifyEngineFiles")
        .val(recipeData.nn.yolov8.engine_file)
        .prop("disabled", true);

      var group_names = defectGroupNamemap[recipeData.nn.yolov8.names_file];
      console.log("the modiwygd2g", recipeData.nn.yolov8.names_file);
      defect_group_name = {};
      group_names.forEach((groupName, index) => {
        defect_group_name[index] = groupName;
      });
      defectGroupName = defect_group_name;

      // Show defect information for the default groups
      showDefectInfo(recipeData.nn.groups, group_names);
      console.log("pressing the modify recipe 1");
      $("#modifyRecipeModal").modal("show");
      console.log("pressing the modify recipe 2");
      // Save the recipe name for later reference
      $("#modifyRecipeModal").data("recipe-name", recipeName);
    });

    function getGroupsInfo(groups) {
      var groupsHtml = '<div class="group-info-row">'; // Start the row container
      $.each(groups, function (groupId, groupInfo) {
        var groupName = defectGroupName[groupId] || "Unknown302"; // Get the name from the mapping or use "Unknown" if not found
        var enabledClass = groupInfo.enabled ? "enabled-true" : "enabled-false";
        var groupInfoHtml =
          '<div class="group-info-card ' +
          enabledClass +
          '">' +
          "<strong>Group Name: </strong>" +
          groupName +
          "<br>" +
          "<strong>Sensitivity_x: </strong>" +
          '<span class="sensitivity-value">' +
          (groupInfo.sensitivity_x || "N/A") +
          "</span>" +
          "<br>" +
          "<strong>Sensitivity_y:</strong>" +
          '<span class="sensitivity-value">' +
          (groupInfo.sensitivity_y || "N/A") +
          "</span>" +
          "<br>" +
          "<strong>Threshold:</strong>" +
          '<span class="sensitivity-value">' +
          (groupInfo.threshold || "N/A") +
          "</span>" +
          "</div>";

        groupsHtml += groupInfoHtml;
      });
      groupsHtml += "</div>"; // Close the row container
      return groupsHtml;
    }

    $("#recipeTable").on("click", "#delete-recipe", function () {
      var row = $(this).closest("tr");

      // Extract recipe name from the row
      var recipeName = row.find("td:eq(0)").text();

      // Ensure at least one recipe remains
      var table = $("#recipeTable").DataTable();
      if (table.rows().count() <= 1) {
        let notifier = new AWN();
        notifier.alert("At least one recipe must remain!", {
          durations: { alert: 3000 },
        });
        return;
      }

      // Show a confirmation dialog before deleting the recipe
      let notifier = new AWN();
      notifier.confirm(
        "Are you sure you want to delete the '" + recipeName + "recipe'?",
        (ok) => {
          deleteRecipe(recipeName);

          // Remove the row from the DataTable
          var table = $("#recipeTable").DataTable();
          table.row(row).remove().draw();

          delete jsonData[recipeName];
        }
      );
    });

    $("#submit_modified_recipe").click(function () {
      if (validateInputs(true)) {
        let notifier = new AWN();
        notifier.confirm(
          "Are you sure you want to modify this recipe?",
          (ok) => {
            var originalRecipeName =
              $("#modifyRecipeModal").data("recipe-name");

            var recipeName = $("#modifyRecipeName").val();
            var table = $("#recipeTable").DataTable();
            var row = table.row(function (idx, data, node) {
              return data[0] === originalRecipeName;
            });

            $("#modifyRecipeModal").modal("hide");

            // Get the modified recipe data
            var modifiedRecipeData = {
              camera: {
                exposure_time: parseInt($("#modifyExposure").val()),
              },
              nn: {
                nms_threshold: parseFloat($("#modifyNmsThreshold").val()),
                num_classes: parseInt($("#modifyNumClasses").val()),
                threshold: 0.2, // default threshold
                tiling: $("#modifyTiling").val() === "Yes" ? true : false,
                yolov8: {
                  names_file: $("#modifyNamesFiles").val(),
                  engine_file: $("#modifyEngineFiles").val(),
                },
                groups: getGroupsData(true),
              },
            };
            // Update the original JSON data with modified recipe details
            jsonData[recipeName] = modifiedRecipeData;
            console.log(jsonData);

            row
              .data([
                recipeName,
                modifiedRecipeData.camera.exposure_time,
                modifiedRecipeData.nn.nms_threshold,
                //modifiedRecipeData.nn.threshold,
                getGroupsInfo(modifiedRecipeData.nn.groups),
                modifiedRecipeData.nn.yolov8.names_file +
                  "<br>" +
                  modifiedRecipeData.nn.yolov8.engine_file,
                '<div class="recipe-operations"><i id="modify-recipe" class="fas fa-pencil-alt"></i><i id="delete-recipe" class="fas fa-trash-alt"></i></div>',
              ])
              .draw();
            if ($("#modifyRecipeName").val() == selectedRecipe) {
              row
                .nodes()
                .to$()
                .find(".recipe-operations #delete-recipe")
                .prop("disabled", true)
                .addClass("disabled")
                .css("pointer-events", "none") // Disable click
                .css("color", "grey") // Change color to grey
                .css("opacity", "0.5");
            }

            // Log the modified JSON data
            console.log("Modified JSON Data:", jsonData);
            rosBridgeInstanceForJS.publish(
              "/gui/button/save_json",
              new ROSLIB.Message({
                data: JSON.stringify(jsonData) + "<M",
              })
            );
          }
        );
      }
    });

    $("#submit_recipe").click(function () {
      if (validateInputs()) {
        let notifier = new AWN();
        notifier.confirm(
          "Are you sure you want to create this recipe?",
          (ok) => {
            var recipeName = $("#RecipeName").val();

            // Get user-input values
            var exposure = parseInt($("#Exposure").val());
            var nmsThreshold = parseFloat($("#NmsThreshold").val());
            var numClasses = parseInt($("#NumClasses").val());
            var threshold = parseFloat($("#Threshold").val());
            var sensitivity_x = parseInt($("#Senstivity_x").val());

            var newRecipeData = {
              camera: { exposure_time: exposure },
              nn: {
                nms_threshold: nmsThreshold,
                num_classes: numClasses,
                threshold: 0.2, // default threshold
                tiling: true,
                yolov8: {
                  engine_file: $("#engineFiles").val(),
                  names_file: $("#namesFiles").val(),
                },
                groups: getGroupsData(false),
              },
            };
            jsonData[recipeName] = newRecipeData;
            namesFiles[recipeName] = $("#namesFiles").val();
            engineFiles[recipeName] = $("#engineFiles").val();

            rosBridgeInstanceForJS.publish(
              "/gui/button/save_json",
              new ROSLIB.Message({
                data: JSON.stringify(jsonData) + "<C",
              })
            );

            console.log("New Recipe data", newRecipeData);
            console.log("New Recipe data", newRowData);

            console.log("Updated JSON Data:", jsonData);

            $("#recipeName").val("");

            $("#createRecipeModal").modal("hide");

            var table = $("#recipeTable").DataTable();
            var newRowData = [
              recipeName,
              exposure,
              nmsThreshold,
              getGroupsInfo(newRecipeData.nn.groups),
              namesFiles[recipeName] + "<br>" + engineFiles[recipeName],
              '<div class="recipe-operations"><i id="modify-recipe" class="fas fa-pencil-alt"></i><i id="delete-recipe" class="fas fa-trash-alt"></i></div>',
            ];
            table.row.add(newRowData).draw();
            console.log("Modified Groups:", newRecipeData.nn.groups);
          }
        );
      }
    });

    $("#createRecipeBtn").click(function () {
      rosBridgeInstanceForJS.publish("/gui/list/all_nn_files");
      $("#recipeName").val("");

      $("#createRecipeModal").modal("show");
    });

    $("#createRecipeModal").on("hidden.bs.modal", function () {
      $("#recipeName").val(""); // Clear recipe name
    });

    function setDefaultValues() {
      var defaultRecipeData;

      if (jsonData.hasOwnProperty("DEFAULT")) {
        defaultRecipeData = jsonData.DEFAULT;
      } else if (Object.keys(jsonData).length > 0) {
        defaultRecipeData = jsonData[Object.keys(jsonData)[0]];
      } else {
        console.log("No recipes found in JSON data");
      }

      var exposure = defaultRecipeData.camera.exposure_time;
      var nmsThreshold = defaultRecipeData.nn.nms_threshold;
      var numClasses = defaultRecipeData.nn.num_classes;
      var threshold = defaultRecipeData.nn.threshold;

      $("#Exposure").val(exposure);
      $("#NmsThreshold").val(nmsThreshold);
      $("#NumClasses").val(numClasses);
      $("#Threshold").val(threshold);
      $("#RecipeName").val("");

      const namesFilesSelect = document.getElementById("namesFiles");
      namesFilesSelect.value = defaultRecipeData.nn.yolov8.names_file;

      const engineFilesSelect = document.getElementById("engineFiles");
      engineFilesSelect.value = defaultRecipeData.nn.yolov8.engine_file;

      var group_names =
        defectGroupNamemap[defaultRecipeData.nn.yolov8.names_file];

      // Show defect information for the default groups
      defect_group_name = {};
      group_names.forEach((groupName, index) => {
        defect_group_name[index] = groupName;
      });
      defectGroupName = defect_group_name;
      showDefectInfo(defaultRecipeData.nn.groups, group_names);
    }

    $("#createRecipeModal").on("shown.bs.modal", function () {
      // setDefaultValues();
    });

    function getGroupsData(isModified) {
      var groupsData = {};
      var tableSelector = isModified
        ? "#modifyGroupInfoTable"
        : "#groupInfoTable";

      $(tableSelector + " tbody tr").each(function () {
        var groupName = $(this).find("td:eq(0)").text();
        console.log("the groupppp name is :", groupName);
        var groupId = Object.keys(defectGroupName).find(
          (key) => defectGroupName[key] === groupName
        );
        if (groupId !== undefined) {
          var enabled = $(this).find(".enabled-input").prop("checked");
          if (groupName === "hole" && groupName === "shuttle_fault") {
            enabled = true;
          }
          //var sensitivity = $(this).find(".sensitivity-range").val();
          var sensitivity_x =
            $(this).find(".sensitivity-x-input").val() || "undefined";
          var sensitivity_y =
            $(this).find(".sensitivity-y-input").val() || "undefined";
          var threshold = $(this).find(".threshold-input").val() || "undefined";
          /*
            groupsData[groupId] = {
              enabled: enabled,
              ids: [parseInt(groupId)],
              sensitivity: parseInt(sensitivity),
            };
            */
          groupsData[groupId] = {
            enabled: enabled,
            ids: [parseInt(groupId)],
            sensitivity_x: parseFloat(sensitivity_x),
            sensitivity_y: parseFloat(sensitivity_y),
            threshold: parseFloat(threshold),
          };
        } else {
          console.log("Group name not found:", groupName);
        }
      });

      console.log(
        isModified ? "Modified Groups:" : "Created Groups:",
        groupsData
      );
      return groupsData;
    }

    populateDataTable(jsonData);

    $("#shutdown").click(() => {
      window.location.href = "http://localhost:4100/all-app";
      //rosBridgeInstanceForJS.publish("/gui/button/shutdown");
    });
  });
});

// Topics :-

// "/gui/value/config" -> Publish recipe json.
// "/gui/label/notification" -> Label
// "/gui/sync/all_nn_files" -> Used in Camera Node to publish all NN files
// "/gui/value/version" -> Used in WebGUIServer.py
// "/gui/button/version" ->  Used in WebGUIServer.py
// "/gui/button/save_json" -> Used in CameraNode to save the Recipes JSON
// "/gui/button/shutdown" -> Used in WebGUIServer.py
// "/gui/button/json_editor" -> Used in CameraNode to load json file and publish it
// "/gui/sync/recipes"  -> Not used in backend
// "/gui/save/recipes"  -> Not used in backend
// "/gui/list/all_nn_files" -> Used in CameraNode to get the all names from JobRecipeManager
