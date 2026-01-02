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

    let jsonEditorInstance;
    const rosBridgeInstanceForJS = new RosBridgeForJS({
      "/gui/value/config": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          const dataSet = JSON.parse(message.data);
          const container = document.getElementById("json_editor_container");
          jsonEditorInstance = new JSONEditor(container);
          jsonEditorInstance.set(dataSet);
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
      "/gui/value/version": {
        topicType: "std_msgs/String",
        type: "sub",
        callback: (message) => {
          $("#version").html(message.data);
        },
      },
      // ******
      // Define Publishers
      // ******
      "/gui/button/version": {
        topicType: "std_msgs/Empty",
        type: "pub",
      },
      // Json Editor
      "/gui/button/json_editor": {
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
    });

    setTimeout(function () {
      rosBridgeInstanceForJS.publish("/gui/button/json_editor");
      rosBridgeInstanceForJS.publish("/gui/button/version");
      rosBridgeInstanceForJS.publish("/gui/button/configuration");
    }, 100);

    $("#logout_button").click(() => {
      let notifier = new AWN();
      let onOk = () => {
        rosBridgeInstanceForJS.publish("/gui/button/logout");
        setTimeout(() => {
          window.location.href = "login.html";
        }, 100);
      };
      notifier.confirm("Do you Really Want to Logout ?", onOk, {
        labels: {
          confirm: "Warning",
        },
      });
    });

    $("#refreshButton").on("click", function () {
      $("#refreshButton").addClass("rotating_sync");
      setTimeout(function () {
        $("#refreshButton").removeClass("rotating_sync");
        location.reload(true);
      }, 1500);
    });

    $("#shutdown").click(() => {
      rosBridgeInstanceForJS.publish("/gui/button/shutdown");
      setTimeout(function () {
        window.close();
      }, 1500);
    });
    document.getElementById("json_save_btn").addEventListener("click", () => {
      if (jsonEditorInstance) {
        try {
          const confirmed = confirm(
            "Are you sure you want to save the edited JSON?"
          );
          const editedData = jsonEditorInstance.get();
          if (confirmed) {
            rosBridgeInstanceForJS.publish("/gui/button/save_json", {
              data: JSON.stringify(editedData),
            });
          }
        } catch (error) {
          new AWN()["alert"](error.toString(), {
            durations: {
              info: 2000,
              success: 2000,
              alert: 2000,
              warning: 2000,
            },
          });
        }
      }
    });
  });
});
