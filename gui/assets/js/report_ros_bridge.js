// ===============================
// AUTHOR     : Himanshu Sharma, Vijay Singh Purohit (vijay@robrosystems.com)
// CREATE DATE     : December, 27, 2021
// PURPOSE     : Act as a bridge between Javascript web app and other ROS nodes for reporting
// VERSION     : V1.1.0 alpha
// SPECIAL NOTES:
// ===============================
// Change History:
//
//==================================

// ******
// Initiate the ROS Connection
// ******
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
    const rosBridgeInstanceForJS = new RosBridgeForJS({
      "/gui/table/query_response": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          var dataSet = JSON.parse(message.data);
          var draw = dataSet.draw + 1;
          var header_columns = [];
          var headers = dataSet.header;
          var header_key = dataSet.header_key;
          var headers_len = headers.length;
          for (var i = 0; i < headers_len; i++) {
            var header = {};
            header.data = header_key[i];
            header.title = headers[i];
            header_columns.push(header);
          }
          var sanitizedString = dataSet.data.replace(
            /[\u0000-\u001F\u007F-\u009F]/g,
            ""
          );

          sanitizedString = sanitizedString.replace(/""/g, '"NULL"');
          var rowData = JSON.parse(sanitizedString);

          // Check if DataTable is already initialized
          if ($.fn.DataTable.isDataTable("#reporting_table")) {
            var existingHeaders = $("#reporting_table")
              .DataTable()
              .columns()
              .header()
              .toArray()
              .map(function (header) {
                return header.textContent;
              });

            // Check if headers are the same
            if (
              JSON.stringify(existingHeaders) === JSON.stringify(headers) &&
              // Check if there is data to display
              rowData.length > 0
            ) {
              const data = {
                draw,
                data: rowData,
              };
              var table = $("#reporting_table").DataTable();
              table.clear();
              dataCallback(data);
            } else {
              var table = $("#reporting_table").DataTable();
              table.destroy();
              $("#reporting_table").html("");

              var is_contains_image = header_key.indexOf("image_path");
              var options = {};
              if (is_contains_image > -1) {
                options = {
                  columnDefs: [
                    {
                      targets: is_contains_image,
                      render: function (data) {
                        return (
                          '<img src="' + data + '" width="150" height="50">'
                        );
                      },
                    },
                  ],
                };
              }

              $("#reporting_table").DataTable({
                data: rowData,
                columns: header_columns,
                // serverSide: true,
                processing: true,
                paging: true,
                searching: true,
                bInfo: true,
                pageLength: 10,
                bLengthChange: true,
                ajax: dataGetProcess,
                reload: false,
                scrollX: true,
                autoWidth: true,
                ...options,
              });
            }
          } else {
            $("#reporting_table").DataTable({
              data: rowData,
              columns: header_columns,
              // serverSide: true,
              processing: true,
              paging: true,
              searching: true,
              bInfo: true,
              pageLength: 10,
              bLengthChange: true,
              ajax: dataGetProcess,
              reload: false,
              scrollX: true,
              autoWidth: true,
            });
          }
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
      "/gui/button/shutdown": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },
      "/gui/table/query": {
        topicType: "std_msgs/String",
        type: "pub",
      },
      "/gui/button/version": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },
    });
    var minDate, maxDate;
    const handleChange = function () {
      var reporting_request = {
        table_name: $("#select_table option:selected").val(),
        start_date: minDate.val().toISOString().split("T")[0],
        end_date: maxDate.val().toISOString().split("T")[0],
        offset: 0,
        length: 100,
      };
      // Convert object to JSON string
      var jsonData = JSON.stringify(reporting_request);
      rosBridgeInstanceForJS.publish(
        "/gui/table/query",
        new ROSLIB.Message({ data: jsonData })
      );
    };
    // change reporting event on page load
    $("#select_table").click(() => {
      handleChange();
    });
    let dataCallback;
    let settingsUpdate;
    // date filterj
    var dataGetProcess = function (data, callback, settings) {
      if (!data.start) {
        data.start = 0;
      }
      if (!data.length) {
        data.length = 100;
      }
      var reporting_request = {
        table_name: $("#select_table option:selected").val(),
        start_date: minDate.val().toISOString().split("T")[0],
        end_date: maxDate.val().toISOString().split("T")[0],
        offset: data.start,
        length: data.length,
      };
      // Convert object to JSON string
      var jsonData = JSON.stringify(reporting_request);
      rosBridgeInstanceForJS.publish(
        "/gui/table/query",
        new ROSLIB.Message({ data: jsonData })
      );
      dataCallback = callback;
      settingsUpdate = settings;
    };

    minDate = new DateTime($("#min_date"), {
      format: "YYYY-MM-DD",
    });
    maxDate = new DateTime($("#max_date"), {
      format: "YYYY-MM-DD",
    });
    minDate.val(new Date().toISOString().split("T")[0]);
    maxDate.val(new Date().toISOString().split("T")[0]);

    setTimeout(function () {
      rosBridgeInstanceForJS.publish("/gui/button/version");
      handleChange();
    }, 500);

    // Refilter the table
    $("#min_date, #max_date").on("change", function () {
      if (minDate.val() && maxDate.val()) {
        handleChange();
      }
    });
    $("#shutdown").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/shutdown");
      setTimeout(function () {
        window.close();
      }, 1500);
    });

    $("#refreshButton").on("click", function () {
      $("#refreshButton").addClass("rotating_sync");
      setTimeout(function () {
        $("#refreshButton").removeClass("rotating_sync");
        location.reload(true);
      }, 1500);
    });
    $("#open_manual_button").click(() => {
      rosJSBridgePubSubInstance.publish("/gui/button/open_manual");
    });
  });
});
