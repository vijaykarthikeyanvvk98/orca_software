import QtQuick 2.15
import QtQuick.Shapes

import Qt.labs.platform 1.1
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import Qt5Compat.GraphicalEffects
import QtQuick.Effects
import QtQuick.Window 2.15
import QtQuick.Controls.Material 2.15
ApplicationWindow {
    id: root
    visible: true
    color: "#011026"
    /*background: Rectangle
    {
       anchors.fill: parent
       color:"#011026"
    }*/


    Connections{
        target: link;
        function onErrordetected()
        {
            err_data = link.errorcatched()
            err_device = err_data[0];
            error_type = err_data[1];
            error_text = err_device+error_type;

            console.log("Error Data:", err_data)
                    console.log("Device:", err_device)
                    console.log("Error Type:", error_type)
                    console.log("Error Text:", error_text)
        }

    }
    signal map_image_Captured
    property string vid_path: ""
    property var err_data: [];
    property var help_text_Array: []
    property int err_device:0;
    property int error_type:0;
    property string error_text:"";
    property string err_text:"";
    property string err_warning_path:"";

    property real y6:link ? link.pitch_deg: 0.0
    property bool edit_not: false
    property color neonblue: "#00FFFF"
    property color neonGreen: "#39FF14"
    property string ip_address: ""
    property int speed_min: 1000
    property int speed_max: 2000
    property real pitch: 0.0
    property real yaw:link ? link.yaw_deg :0.0// "0.0\u00B0"
    property real roll:link ?link.roll_deg: 0.0
    property real temp: link ? link.temperature_C : 0.0
    property string press: "0 (mbar)"
    property real depth: link ?link.depth_m:0.0
    property string voltage: "0.0 V(0%)"
    property string battery: ''
    property var modeEnabled: [true, true, true, true]
    property int activeMode: -1
    property real channel_1: 0.0
    property real channel_2: 0.0
    property real channel_3: 0.0
    property real channel_4: 0.0
    property real z_axis: 0.0
    property real l: 0.0
    property real r: 0.0
    property int arm: 0
    property bool arm_status: false
    property bool arm_status2: false
    property int mapped_x_axis: 0
    property int mapped_y_axis: 0
    property int mapped_z_axis: 0
    property int intensity: 0
    property int volume: 0
    property bool connect_pop_status: false
    property bool connection_status: false
    property bool connect_status: true
    property real size: 100
    property real _reticleHeight: 1
    property real _reticleSpacing: size * 0.15
    property real _reticleSlot: _reticleSpacing + _reticleHeight
    property real _longDash: size * 0.35
    property real _shortDash: size * 0.25
    property real _fontSize: 0.005 * root.width
    property var data_model: ["qrc:/resources/images/compass-needle.png", "qrc:/resources/images/temperature.png", "qrc:/resources/images/pressure.png", "qrc:/resources/images/depth.png" /*,"qrc:/resources/images/100%.png"*/
    ]
    property var data_model2: ["0.0\u00B0", "0.0\u00B0C", "0 (mbar)", "0(m)" /*,"0.0 V(0%)"*/
    ]
    property real yaw_value: 0
    property real pitch_value :0
    property real roll_value: 0
    //property real temperature:
    //property real speed
    property bool selectall_status3: false
    property bool start_or_stop: false
    property bool pause_play: false
    property string record_tool_text: "Start\nVideo Recording"
    property string record_tool_text2: "Pause Recording"
    property bool start_status: false
    property bool browser_status: false
    property bool stop_status: false
    property bool pause_status: false
    property bool pause_status2: false
    property int a_mode: 0
    property int b_mode: 0
    property int c_mode: 0
    property int d_mode: 0
    property int start_mode: 1
    property int select_mode: 2
    property int l1_mode: 8
    property int l2_mode: 0
    property int r1_mode: 9
    property int r2_mode: 0
    property int left_mode: 0
    property int right_mode: 0
    property int up_mode: 0
    property int down_mode: 0
    property int lX_mode: 0
    property int lY_mode: 0
    property int rX_mode: 0
    property int rY_mode: 0
    property int joystick_mode: 0
    property string direction: "S"
    property string graph_direction: ""
    property bool button_Pressed: false
    property real bright: 0.0
    property real contrast: 0.0
    property bool isConnected:false
    property bool online: false
    property bool online_2: false
    property string logFileDir: ""
    property string logFileDir2: ""
    property string logFileDir3: ""
    property bool depth_set_not: false
    property int depth_set_count: 0
    property bool key_0: false
    property int li_key: 50
    property int sp_key: 50
    menuBar: MenuBar {
        id: menuBar

        Menu {
            title: "Device"


            /*Action {
                id: menu_1
                text: "Test link"
                onTriggered: {
                    //ip_address = "169.254.191.126"
                    ip_address = "127.0.0.1"

                    connection_status = true
                    connect_stats(ip_address)
                }
            }*/
            Action {
                id: menu_2
                text: "ORCA"
                onTriggered: {
                    //console.log("haifedf")
                    if (connection2.source == "qrc:/resources//images/connect.png") {
                        if (ip_address === "") {
                            ip_address = "192.168.56.2"
                        }
                        connection_status = true
                        connect_stats(ip_address)
                    } else if (connection2.source == "qrc:/resources//images/connected.svg") {
                        connection_status = false
                        ip_address = ""
                        disconnect_stats()
                    }
                }
            }

            delegate: MenuItem {
                id: menuItem3

                contentItem: Text {
                    text: menuItem3.text
                    //font.family: font_family
                    font.bold: true
                    font.italic: false
                    font.underline: false
                    font.strikeout: false
                    font.pixelSize: Math.min(root.width/70,root.height/60) // "Palatino Linotype"
                    //font: menuItem3.font
                    //opacity: enabled ? 1.0 : 0.3
                    color: menuItem3.highlighted ? "#ffffff" : "#21be2b"
                    horizontalAlignment: Text.AlignLeft
                    verticalAlignment: Text.AlignVCenter
                }

                background: Rectangle {
                    implicitWidth: 200
                    implicitHeight: 40
                    opacity: enabled ? 1 : 0.3
                    color: menuItem3.highlighted ? "#21be2b" :"#011026"
                }
            }
            background: Rectangle {
                implicitWidth: 200
                implicitHeight: 40
                color: "#011026"
                border.color: "#21be2b"
                radius:2
            }
        }

        Menu {
            title:"Camera Settigs"

            Action {
                                           text: "Web Camera"
                                           onTriggered: {
                                               VideoStreamer.openVideoCamera("0")
                                               opencvImage.visible = true
                                           }
                                       }
                            Action{
                                text:"RTSP Camera"
                                onTriggered: rtspDialog.open()
                            }

            delegate: MenuItem {
                id: menuItem4

                contentItem: Text {
                    text: menuItem4.text
                    //font.family: font_family
                    font.bold: true
                    font.italic: false
                    font.underline: false
                    font.strikeout: false
                    font.pixelSize: Math.min(root.width/70,root.height/60) // "Palatino Linotype"
                    //opacity: enabled ? 1.0 : 0.3
                    color: menuItem4.highlighted ? "#ffffff" : "#21be2b"
                    horizontalAlignment: Text.AlignLeft
                    verticalAlignment: Text.AlignVCenter
                }

                background: Rectangle {
                    implicitWidth: 200
                    implicitHeight: 40
                    opacity: enabled ? 1 : 0.3
                    color: menuItem4.highlighted ? "#21be2b" : "#011026"
                }
            }
            background: Rectangle {
                implicitWidth: 200
                implicitHeight: 40
                // color: current_theme_color2/"#ffffff"
                border.color: "#21be2b"
                radius:  0.005*root.width  // 5
            }
        }
        Menu {
            id: more_menu
            title: "More"
            /*Action {
                text: "Debug Terminal"
                onTriggered: {
                    stack.push(terminal_page)
                }
            }*/


            /*Action {
                text: "Command Prompt"
                onTriggered: {
                    CmdLauncher.openCommandPrompt()
                    //main_page.call_pop()
                    //stack.push(terminal_page)
                }
            }*/
            Action {
                text: "About"

                onTriggered: {
                    about_window.open()
                }
            }
            /*Action {
                text: "User Manual"
                onTriggered: {
                    myLink.openLocalPdf()
                }
            }*/
            Menu {
                title: "Help" // ✅ This replaces Action + nested MenuItem


                /*Action {
                    text: "Contents"
                    onTriggered: helpPopup.open()
                }*/
                Action {
                    text: "Application Information"
                    onTriggered: info_window.open()
                }
                Action {
                    text: "Software Support"
                    onTriggered: support_window.open()
                }
                Action {
                    text: "Report Issue"


                    /*onTriggered: Qt.openUrlExternally(
                                     "https://yourdomain.com/issues")*/
                    onTriggered: {
                        report_window.open()
                    }
                }


                /*Action {
                    text: "UI Tour"
                    onTriggered: waitpop.open()
                }*/
                delegate: MenuItem {
                    id: menuItem6

                    contentItem: Text {
                        text: menuItem6.text
                        font.bold: true
                        font.italic: false
                        font.underline: false
                        font.strikeout: false
                        font.pixelSize: Math.min(root.width/70,root.height/60)  // "Palatino Linotype"                    //opacity: enabled ? 1.0 : 0.3
                        color: menuItem6.highlighted ? "#ffffff" : "#21be2b"
                        horizontalAlignment: Text.AlignLeft
                        verticalAlignment: Text.AlignVCenter
                    }

                    background: Rectangle {
                        implicitWidth: 200
                        implicitHeight: 40
                        opacity: enabled ? 1 : 0.3
                        color: menuItem6.highlighted ? "#21be2b" : "#011026"
                    }
                }
                background: Rectangle {
                    implicitWidth: 200
                    implicitHeight: 40
                    color: "#011026" //"#ffffff"
                    border.color: "#21be2b"
                    radius: 0.005*root.width  // 5
                }
            }

            delegate: MenuItem {
                id: menuItem5

                contentItem: Text {
                    text: menuItem5.text
                    font.bold: true
                    font.italic: false
                    font.underline: false
                    font.strikeout: false
                    font.pixelSize: Math.min(root.width/70,root.height/60)  // "Palatino Linotype"                    //opacity: enabled ? 1.0 : 0.3
                    color: menuItem5.highlighted ? "#ffffff" : "#21be2b"
                    horizontalAlignment: Text.AlignLeft
                    verticalAlignment: Text.AlignVCenter
                }

                background: Rectangle {
                    implicitWidth: 200
                    implicitHeight: 40
                    opacity: enabled ? 1 : 0.3
                    color: menuItem5.highlighted ? "#21be2b" : "#011026"
                }
            }
            background: Rectangle {
                implicitWidth: 200
                implicitHeight: 40
                color: "#011026" //"#ffffff"
                border.color: "#21be2b"
                radius:  0.005*root.width  // 5
            }
        }
        delegate: MenuBarItem {
            id: menuBarItem

            contentItem: Text {
                text: menuBarItem.text
                //font.family: font_family
                font.bold: true
                font.italic: false
                font.underline: false
                font.strikeout: false
                font.pixelSize: Math.min(root.width/70,root.height/60) // "Palatino Linotype"
                //opacity: enabled ? 1.0 : 0.3
                color: menuBarItem.highlighted ? "#ffffff" : "#21be2b"
                horizontalAlignment: Text.AlignLeft
                verticalAlignment: Text.AlignVCenter
                //elide: Text.ElideRight
            }

            background: Rectangle {
                implicitWidth: 40
                implicitHeight: 40
                opacity: enabled ? 1 : 0.3
                color: menuBarItem.highlighted ? "#21be2b" :"#011026" //"transparent"
            }
        }

        background: Rectangle {
            implicitWidth: 40
            implicitHeight: 40
            color: "#011026"


            /*gradient: Gradient {
                GradientStop {
                    position: 0.0
                    color: "#00ff00"
                } // Bright lime green
                GradientStop {
                    position: 0.3
                    color: "#32cd32"
                } // Lime green
                GradientStop {
                    position: 0.6
                    color: "#228b22"
                } // Forest green
                GradientStop {
                    position: 0.8
                    color: "#006400"
                } // Dark green
                GradientStop {
                    position: 1.0
                    color: "#003300"
                } // Very dark green
            }*/
            radius: 0.005*root.width // 5
        }

        Component.onCompleted: {
            //menu_height = menuBar.height
        }
    }
    Dialog {
        id: rtspDialog
        modal: true
        width: 0.4*root.width
        height: 0.008*root.height // medium height
        x:parent.width/3.5
        y:parent.height/3.5
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        function encodeRtsp(url) {
            var matches = url.match(/^(rtsp:\/\/)([^:]+):(.+)@(.+)$/)
            if (!matches) return url
            var protocol = matches[1]
            var username = matches[2]
            var password = encodeURIComponent(matches[3])
            var host = matches[4]
            return protocol + username + ":" + password + "@" + host
        }
        background: Rectangle {
            radius: 16
            color: "#0F172A"
            border.color: "#38BDF8"
            border.width: 0.002*root.width
        }
        Column {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 16
            Text {
                text: "RTSP Camera"
                font.pixelSize: 22
                font.bold: true
                color: "white"
                horizontalAlignment: Text.AlignHCenter
                width: parent.width
            }
            Rectangle {
                height: 1
                width: parent.width
                color: "#38BDF8"
            }
            Text {
                text: "Enter RTSP URL"
                font.pixelSize: 14
                color: "#93C5FD"
            }
            TextField {
                id: rtspField
                width: parent.width
                height: 40
                placeholderText: ""
                text: ""
                color: "white"
                background: Rectangle {
                    radius: 8
                    color: "#111827"
                    border.color: "#38BDF8"
                }

                padding: 10
            }
            Row {
                spacing: 16
                anchors.horizontalCenter: parent.horizontalCenter

                Button {
                    text: "Open Camera"
                    width: 140
                    height: 40

                    background: Rectangle {
                        radius: 10
                        color: "#22C55E" // green
                    }

                    onClicked: {
                        VideoStreamer.changeCamera();

                    }
                }
                Button {
                    text: "Cancel"
                    width: 100
                    height: 40

                    background: Rectangle {
                        radius: 10
                        color: "#334155"
                    }

                    onClicked: rtspDialog.close()
                }
            }
        }
    }
    Component.onCompleted: {
        //VideoStreamer.openVideoCamera( "rtsp://admin:Vikra@123@192.168.56.50:554/video/live?channel=1&subtype=0");
        opencvImage.visible = true
        /*fileModel.append({
                             "fileName": "C:/Users/Vijay/Documents/rough.txt"
                         })*/
        //logFileDir = myLink.create_directory2(0)
        //logFileDir2 = myLink.create_directory2(1)
        //logFileDir3 = myLink.create_directory2(2)
        help_text_Array = ["🛈 Help:\n\n-User Interface not Responding.\n- Data Update Issue.\n- Software Appearance.\n- Check Logs.\n- Use 'Debug Terminal' for developer logs.\n- Tutorial.\n\nFor detailed instructions, refer to the User Manual."]

    }
    Connections {
        target: liveImageProvider

        function onImageChanged() {
            opencvImage.reload()
        }
    }
    Connections {
        target: VideoStreamer

        /*function onWrite_finished() {
            VideoStreamer.increment_counter()
        }*/
    }
    Rectangle {
        id: imageRect
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.margins: 0.005 * root.width
        color: "transparent"
        border.color: "white"
        border.width: 0.0025*root.width
        visible: true
        Image {
            id: opencvImage
            width: 0.99 * parent.width
            height: 0.99*parent.height // 16:9 aspect ratio
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.centerIn: parent
            clip:true
            fillMode: Image.PreserveAspectCrop
            property bool counter: false
            visible: true
            source: "qrc:/resources/images/dummy_template3.jpg"
            asynchronous: false
            cache: false
            function reload() {
                counter = !counter
                source = "image://live/image?id=" + counter
            }
        }
}
/*Connections {
        target: loader4.item

        function onMsl_value_changed() {
            var value = loader4.item.spinbox_value //console.log(loader4.item.spinbox_value)

            VideoStreamer.change_cam(value)
        }
    }*/
    /*Dialog {
        id: message_template
        z: 2
        x: parent.width / 2.25
        y: parent.height / 2.25
    Text {
            id: template_text
            color: "white"
            text: "Confirmation"
            font.bold: true
            font.pointSize: 0.01 * root.width
            style: Text.Sunken
            anchors.top: parent.top
            anchors.topMargin: 0.01 * parent.width
            anchors.left: parent.left
            anchors.leftMargin: 0.05 * parent.width
        }
        contentItem: Text {
            id: template_content
            color: "white"
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 0.00725 * root.width
            style: Text.Sunken
            anchors.top: template_text.bottom
            anchors.topMargin: 0.04 * parent.width
            anchors.left: parent.left
            anchors.leftMargin: 0.05 * parent.width
            anchors.right: parent.right
            anchors.rightMargin: 0.05 * parent.width
        }
        footer: DialogButtonBox {
            alignment: Qt.AlignCenter
            background: Rectangle {
                anchors.fill: parent
                color: "transparent"
            }
            Button {
                DialogButtonBox.buttonRole: DialogButtonBox.AcceptRole
                background: Rectangle {
                    color: "#1a3154"
                }
                contentItem: Text {
                    id: response_button1
                    text: "Save"
                    font.pixelSize: Math.min(root.width / 90, root.height / 70)
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
            Button {
                DialogButtonBox.buttonRole: DialogButtonBox.RejectRole
                background: Rectangle {
                    color: "#1a3154"
                }
                contentItem: Text {
                    id: response_button2
                    text: "Close"
                    font.pixelSize: Math.min(root.width / 90, root.height / 70)
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }

        background: Rectangle {
            color: "black"
            opacity: 0.75
            radius: 0.01 * parent.width
            border.color: "#21be2b"
        }

        onAccepted: {
            if (connect_pop_status == true) {
                if (connection2.source == "qrc:/resources//images/connect.png") {
                    connect_status = true
                    connection2.source = "qrc:/resources//images/connected.png"
                    mission_timer.start()
                    //template_text2.text = "Acknowledgement"
                    template_content2.text = "Alert!!!\nDevice has been connected successfully"
                    message_template2.open()
                } else if (connection2.source == "qrc:/resources//images/connected.png") {
                    if (!connection_status) {

                        myLink.reset_ip()
                        connection2.source = "qrc:/resources//images/connect.png"
                        connect_status = !connect_status
                        timer_reset()
                        //template_text2.text = "Acknowledgement"
                        template_content2.text = "Alert!!!\nDevice is disconnected successfully"
                        message_template2.open()
                    } else if (connection_status) {

                    }
                }
            } else if (coordinate_pop_status == true) {

            } else if (log_save_status1 == true) {
                myLink.writeToLogFile()
                //template_text2.text = "Acknowledgement"
                template_content2.text = "Log Data has been saved successfully"
                message_template2.open()
            } else if (gps_valid == true) {
                drawer.open()
            }

            message_template.close()
        }
        onRejected: {
            message_template.close()
        }
    }*/

    function connect_stats(string) {
        clear_status()
        template_text.text = "Confirmation"
        template_content.text = "Do you want to connect to the selected Device?"
        response_button1.text = "Yes"
        response_button2.text = "No"
        connect_pop_status = true
        message_template.open()
    }

    function disconnect_stats() {
        clear_status()
        template_text.text = "Confirmation"
        template_content.text = "Do you want to disconnect the selected Device?"
        response_button1.text = "Yes"
        response_button2.text = "No"
        connect_pop_status = true
        message_template.open()
    }

    function clear_status() {
        connect_pop_status = false
    }

    Dialog {
        id: message_template2
        z: 2
        x: root.width / 2.25
        y: root.height / 2.25

        Text {
            id: template_text2
            color: "white"
            //text: "Acknowledgement"
            font.bold: true
            font.pointSize: 0.01 * root.width
            style: Text.Sunken
            anchors.top: parent.top
            anchors.topMargin: 0.01 * parent.width
            anchors.left: parent.left
            anchors.leftMargin: 0.05 * parent.width
        }

        contentItem: Text {
            id: template_content2
            color: "white"
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 0.006 * root.width
            style: Text.Sunken
            anchors.top: parent.top
            anchors.topMargin: 0.04 * parent.width
            anchors.left: parent.left
            anchors.leftMargin: 0.05 * parent.width
            anchors.right: parent.right
            anchors.rightMargin: 0.05 * parent.width
        }
        footer: DialogButtonBox {
            alignment: Qt.AlignCenter
            //buttonLayout: DialogButtonBox.WinLayout
            background: Rectangle {
                anchors.fill: parent
                color: "transparent"
            }

            Button {
                //text: qsTr("Save")
                DialogButtonBox.buttonRole: DialogButtonBox.AcceptRole
                background: Rectangle {
                    color: "#1a3154"
                }
                contentItem: Text {
                    id: response_button4
                    text: "Close"
                    font.pixelSize: Math.min(root.width / 90, root.height / 70)
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }

        background: Rectangle {
            color: "black"
            opacity: 0.75
            radius: 0.01 * parent.width
            border.color: "#21be2b"
        }

        onAccepted: {
            message_template2.close()
        }
    }

    /*Dialog {
        id: message_template3
        z: 2
        x: parent.width / 2.25
        y: parent.height / 2.25

        Text {
            id: template_text3
            color: "white"
            text: "Alert"
            font.bold: true
            font.pointSize: 0.01 * root.width
            style: Text.Sunken
            anchors.top: parent.top
            anchors.topMargin: 0.01 * parent.width
            anchors.left: parent.left
            anchors.leftMargin: 0.05 * parent.width
        }

        contentItem:
            Text {
            id: template_content3
            color: "white"
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            font.pointSize: 0.0075 * root.width
            style: Text.Sunken
            anchors.top: template_text3.bottom
            anchors.topMargin: 0.04 * parent.width
            anchors.left: parent.left
            anchors.leftMargin: 0.05 * parent.width
        }

        background: Rectangle {
            color: "black"
            opacity: 0.75
            radius: 0.01 * parent.width
            border.color: "#21be2b"
        }

        onRejected: {
            message_template.close()
        }
    }*/
    Timer {
        id: timer
        repeat: true
        running: true
        interval: 500

        onTriggered: {
        }
    }
    Rectangle {
        id: tray_5
        anchors.bottom: imageRect.bottom
        anchors.bottomMargin: 0.005 * root.width
        anchors.right: box_8.left
        anchors.leftMargin:0.005*parent.width
        anchors.rightMargin: 0.005 * parent.width
        width: box_5.width
        height: box_5.height
        color: box_5.color
        opacity: box_5.opacity
        radius: box_5.radius
        //visible: mapContainer.height>root.height/2
        /*Button {
            id: search3
            width: 0.025 * root.width
            height: 0.025 * root.width
            anchors.horizontalCenter: tray_5.horizontalCenter
            anchors.verticalCenter: tray_5.verticalCenter
            anchors.verticalCenterOffset: -0.4 * search3.width
            property bool reset: false
            signal add
            z: 1

            background: Rectangle {
                color: "grey" //"black"
                radius: 0.05 * root.width
                opacity: 1.0
            }

            contentItem: Image {
                id: area_20
                source: "qrc:/resources/images/settings_2.png"
                anchors.centerIn: parent
                width: 0.03 * parent.width
                height: width
                smooth: true
            }
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    waitpop.open()
                }
            }
        }
        Loader {
            id: loader4
            anchors.horizontalCenter: tray_5.horizontalCenter
            anchors.verticalCenter: tray_5.verticalCenter
            anchors.verticalCenterOffset: 0.65 * search3.height
            width: 0.85 * parent.width
            height: .3 * parent.height
            source: "spinbox.qml"
            z: 2
            visible: true
        }
        MouseArea {
            anchors.fill: parent
            hoverEnabled: true

            onEntered: {
            }

            onExited: {


            }
        }*/
    }

    ColumnLayout {
        anchors.top: tray_5.top
               anchors.bottom: tray_5.bottom
               anchors.left: tray_5.left
               anchors.right: tray_5.right
               spacing: 0.001 * parent.height
        Text {
            id: p12
            Layout.fillWidth: true
            Layout.fillHeight:true
            text:roll.toFixed(1)
            font.bold: true
            style: Text.Sunken
            font.pixelSize: Math.min(box_5.width / 3, box_5.height / 3)
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }
        Text {
            id: p13
            Layout.fillWidth: true
            Layout.fillHeight: true
            text: qsTr("Roll(\u00B0)")
            font.bold:true
            font.pixelSize: Math.min(box_5.width / 7, box_5.height / 6)
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }
      }

    function tooltip_reset() {
        tooltip_template.x = 0
        tooltip_template.y = 0
        tooltip_template.text = ""
        tooltip_template.visible = false
    }
    function tooltip_set(x, y, text) {
        tooltip_template.x = x
        tooltip_template.y = y
        tooltip_template.text = text
        tooltip_template.visible = true
        start_Timer3.start()
    }

    Timer {
        id: start_Timer3
        interval: 1500 // Timer interval in milliseconds
        running: false // Start the timer when the application starts

        repeat: false

        onTriggered: {
            tooltip_reset()
        }
    }

    ToolTip {
        id: tooltip_template
        x: 0
        y: 0
        z: 1
        text: ""
        contentItem: Text {
            color: neonGreen
            text: tooltip_template.text
        }
        background: Rectangle {
            color: "black"
            opacity: 0.5
            border.color: "#21be2b"
        }
    }
    Timer {
        id: start_Timer
        interval: 1000 // Timer interval in milliseconds (1 second in this case)
        running: false // Start the timer when the application starts
        repeat: true
        property int elapsedTime: 0
        onTriggered: {
            updateTimer()
        }
    }
    function updateTimer() {
        var hours = Math.floor(start_Timer.elapsedTime / 3600)
        var minutes = Math.floor((start_Timer.elapsedTime % 3600) / 60)
        var seconds = start_Timer.elapsedTime % 60
        timerText.text = padNumber(hours, 2) + ":" + padNumber(
                    minutes, 2) + ":" + padNumber(seconds, 2)
        start_Timer.elapsedTime++
    }
    function padNumber(num, size) {
        var s = num.toString()
        while (s.length < size) {
            s = "0" + s
        }
        return s
    }
    Rectangle {
        id: statusindicator2
        width: 0.01 * root.width
        height: width
        radius: 100
        //anchors.centerIn: parent
        color: "red"
        anchors.bottom: imageRect.bottom
        anchors.left: parent.left
        anchors.leftMargin: 0.01 * root.width
        anchors.bottomMargin: 0.007 * parent.width
        opacity: 1
        property bool isBlink: false
        onIsBlinkChanged: {
            if (isBlink)
                startBlinkAnimation()
            else
                stopBlinkAnimation()
        }function startBlinkAnimation() {
            blinkAnimation.running = true
        }
        function stopBlinkAnimation() {
            blinkAnimation.running = false
            opacity = 1
        }
        SequentialAnimation {
            id: blinkAnimation
            loops: Animation.Infinite
            PropertyAnimation {
                target: statusindicator2
                property: "opacity"
                from: 1
                to: 0
                duration: 500
            }
            PropertyAnimation {
                target: statusindicator2
                property: "opacity"
                from: 0
                to: 1
                duration: 300
            }
        }
        Timer {
            id: start_Timer2
            interval: 500
            running: false // Start the timer when the application starts
            repeat: true
            property int elapsedTime: 1 // Elapsed time in seconds
            onTriggered: {
                statusindicator2.isBlink = !statusindicator2.isBlink // Toggle blinking
            }
        }
    }
    Text {
                      id: timerText
                      color: "White"
                      visible: statusindicator2.visible
                      anchors.left: statusindicator2.right
                      anchors.leftMargin:0.005*root.width
                      anchors.bottom: statusindicator2.bottom
                      //anchors.bottomMargin: 0.005*parent.width
                      font.pixelSize:Math.min(root.width/47,root.height/37)
                      opacity: statusindicator2.opacity
                  }
    /*Rectangle {
        id: tray_3
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: imageRect.top
        anchors.margins: 0.005 * parent.width
        color: "#5978AC"
        opacity: 0.5
        radius: 5
        z: 0
    }*/
    function updateTimer_2() {
        var hours = Math.floor(mission_timer.elapsedTime / 3600)
        var minutes = Math.floor((mission_timer.elapsedTime % 3600) / 60)
        var seconds = mission_timer.elapsedTime % 60
        mission_time_count_2.text = padNumber(hours, 2) + ":" + padNumber(
                    minutes, 2) + ":" + padNumber(seconds, 2)
        mission_timer.elapsedTime++
    }
    Image {
        id: vikra
        source: "qrc:/resources/images/vikra_2.jpeg"
        anchors.right: parent.right
        anchors.rightMargin: 0.02 * parent.width
        anchors.top: parent.top
        anchors.topMargin: 0.02 * parent.height
        width: 0.055 * parent.width
        height: width
    }
    Rectangle {
        id: modePanel
        anchors.right: imageRect.right
        anchors.rightMargin: 0.005 * root.width
        anchors.top: vikra.bottom
        anchors.topMargin: 0.1 * parent.height
        width: 0.085 * root.width
        height: 0.42 * root.height
        radius: 0.005*root.width
        color: "#05090F"
        border.color: "#FFFFFF"
        border.width: 1
        z: 10
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 0.015 * root.width
            spacing: 0.015 * root.height

            ROVButton {
                text: "HOLD"
                index: 0
                baseColor: "#2ECC71"
            }

            ROVButton {
                text: "MANUAL"
                index: 1
                baseColor: "#3498DB"
            }

            ROVButton {
                text: "DEPTH"
                index: 2
                baseColor: "#F39C12"
            }

            ROVButton {
                text: "STABILIZE"
                index: 3
                baseColor: "#E74C3C"
            }
        }
    }
    component ROVButton: Rectangle {
        id: btn

        property string text
        property int index
        property color baseColor

        Layout.fillWidth: true
        Layout.preferredHeight: parent.height / 4.6

        radius: height * 0.30
        scale: mouseArea.pressed ? 0.95 : 1.0

        Behavior on scale {
            NumberAnimation { duration: 80 }
        }

        border.width: activeMode === index ? 2 : (mouseArea.containsMouse ? 2 : 1)
        border.color: Qt.lighter(baseColor, 1.4)

        color: mouseArea.pressed
               ? Qt.darker(baseColor, 1.4)
               : mouseArea.containsMouse
                 ? Qt.lighter(baseColor, 1.25)
                 : activeMode === index
                   ? Qt.lighter(baseColor, 1.4)
                   : baseColor

        Behavior on color {
            ColorAnimation { duration: 120 }
        }

        Text {
            anchors.centerIn: parent
            text: btn.text
            color: "white"
            font.bold: true
            font.pixelSize: height * 0.65
        }

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor

            onClicked: {
                activeMode = btn.index
                console.log("Mode selected:", btn.text)
                link.sendModeCommand(btn.index)
            }

        }
    }
    function capture_map_image() {
        var saveDirectory
        var rectangles = [imageRect]
        var rect, count_ss = 0
        for (var i = 0; i < rectangles.length; i++) {
            (function (rect) {
                var date = new Date()
                var formattedDate = date.toLocaleString(Qt.locale("en_IN"),
                                                        "dd.MM.yyyy-hh.mm.ss")
                var fileName = "/" + formattedDate + "_" + i
                rect.grabToImage(function (result) {
                    var filePath = logFileDir2 + fileName
                    filePath = filePath.replace("file:///", "")
                    if (result.saveToFile(filePath + ".jpeg")) {
                        count_ss++
                    } else {
                        console.log("Failure")
                    }
                }, Qt.size(opencvImage.sourceSize.width,
                           opencvImage.sourceSize.height))
            })(rectangles[i])
        }
        map_image_Captured()
    }

    ListView {
        id: fileListView
        visible: false
        model: ListModel {
            id: fileModel
        }
        delegate: Item {
            width: fileListView.width
            height: 40

            Rectangle {
                width: parent.width
                height: 40
                color: "lightgray"
                border.color: "gray"
                radius: 5

                Text {
                    anchors.centerIn: parent
                    text: model.fileName
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {}
                }
            }
        }
    }
    onMap_image_Captured: {
        clear_status()
        template_content2.text = "Frame has been\n Captured Successfully!!!"
        message_template2.open()
    }
    function timer_reset() {
        mission_timer.stop()
        mission_timer.elapsedTime = 0
    }
        Rectangle {
        id: box_6
        anchors.bottom: imageRect.bottom
        anchors.bottomMargin: 0.005 * root.width
        anchors.left: box_1.right
        anchors.leftMargin:0.005*parent.width
        anchors.rightMargin: 0.005 * parent.width
        width: box_5.width
        height: box_5.height
        color: box_5.color
        opacity: box_5.opacity
        radius: box_5.radius
        /*gradient:Gradiet{
            GradientStop{
                position: 0.0; color:"#111111"
            }
            GradientStop {
                position:1.0;color:"#333333"}
        }*/
        }


    ColumnLayout {
        anchors.top: box_6.top
               anchors.bottom: box_6.bottom
               anchors.left: box_6.left
               anchors.right: box_6.right
               spacing: 0.001 * parent.height
        Text {
            id: p10
            Layout.fillWidth: true
            Layout.fillHeight:true
            text:temp.toFixed(1)
            font.bold: true
            style: Text.Sunken
            font.pixelSize: Math.min(box_5.width / 3, box_5.height / 3)
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }
        Text {
            id: p11
            Layout.fillWidth: true
            Layout.fillHeight: true
            text: qsTr("Temperature(\u00B0C)")
            font.bold:true
            font.pixelSize: Math.min(box_5.width / 7, box_5.height / 6)
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }
      }

    Rectangle {
        id: box_7
        anchors.bottom: imageRect.bottom
        anchors.bottomMargin: 0.005 * root.width
        anchors.left: box_6.right
        anchors.rightMargin:0.005*parent.width
        anchors.leftMargin: 0.005 * parent.width
        width: box_5.width
        height: box_5.height
        color: box_5.color
        opacity: box_5.opacity
        radius: box_5.radius
}
    ColumnLayout {
        anchors.top: box_7.top
               anchors.bottom: box_7.bottom
               anchors.left: box_7.left
               anchors.right: box_7.right
        spacing:0.005*root.width

        Text {
            id: p14
            Layout.fillWidth: true
            text:yaw.toFixed(1)
            font.bold: true
            style: Text.Sunken
            font.pixelSize: Math.min(box_5.width / 3, box_5.height / 3)
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }
        Text {
            id: batteryText
            Layout.fillWidth: true
            text: "Heading(\u00B0)"
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: Math.min(box_5.width / 7, box_5.height / 6)
            font.bold:true
            color: "#FFFFFF"
        }


        }
    Rectangle {
        id: box_1
        anchors.bottom: imageRect.bottom
        anchors.bottomMargin: 0.005 * root.width
        anchors.left: box_5.right
        anchors.leftMargin: 0.005 * parent.width
        width: box_5.width
        height: box_5.height
        color: box_5.color
        opacity: box_5.opacity
        radius: box_5.radius
    }
    ColumnLayout {
        anchors.top: box_1.top
        anchors.bottom: box_1.bottom
        anchors.left: box_1.left
        anchors.right: box_1.right
        spacing: 0.001 * parent.height
        Text {
            id: y10
            Layout.fillWidth: true
            text:depth.toFixed(1)
            font.bold: true
            style: Text.Sunken
            font.pixelSize: Math.min(box_5.width / 3, box_5.height / 3)
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }
        Text {
            id: y11
            Layout.fillWidth: true
            text: qsTr("Depth(m)")
            font.bold:true
            font.pixelSize: Math.min(box_5.width / 7, box_5.height / 6)
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: "white"
        }
    }
    Rectangle {
            id: box_8
            anchors.bottom: imageRect.bottom
            anchors.bottomMargin: 0.005 * root.width
            anchors.right: box_5.left
            anchors.rightMargin: 0.005 * parent.width
            width: box_5.width
            height: box_5.height
            color: box_5.color
            opacity: box_5.opacity
            radius: box_5.radius
    }
    ColumnLayout {
        anchors.top: box_8.top
        anchors.bottom: box_8.bottom
        anchors.left: box_8.left
        anchors.right: box_8.right
        spacing: 0.001 * parent.height
    Text {
        id: p1
        Layout.fillWidth: true
        text: y6.toFixed(1)
        font.bold: true
        style: Text.Sunken
        font.pixelSize: Math.min(box_5.width / 3, box_5.height / 3)
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        color: "white"
    }
    Text {
        id: p3
        Layout.fillWidth: true
        text: qsTr("Pitch(\u00B0)")
        font.bold:true
        font.pixelSize: Math.min(box_5.width / 7, box_5.height / 6)
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        color: "white"
        }
    }

    Rectangle {
        id: box_5
        anchors.bottom: imageRect.bottom
        anchors.bottomMargin: 0.005 * root.width
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.horizontalCenterOffset: -0.05*root.width
        width: 0.075 * root.width
        height: 0.1125 * root.height
        color: "black"
        opacity: 0.5
        radius: 0.005*root.width
    }
    Rectangle {
        id: gauge
        anchors {
            left: imageRect.left
            leftMargin: 0.02 * root.width
            top: parent.top
            topMargin: 0.02 * parent.height
        }
        color: "black"
        width: 0.075 * root.width
        height: width
        radius: 100
        z: 100
        Image {
            id: comp
            source: "qrc:/resources/images/compass.png"
            width: 0.10*root.width
            height: width
            anchors.fill: parent
        }
    }
    Image {
        id: needle1
        source: "qrc:/resources/images/compass-needle.png"
        anchors.centerIn: gauge
        width: 0.56 * gauge.width
        visible: gauge.visible
        z: gauge.z
        height: gauge.height * .7
        transformOrigin: Item.Center
        rotation: yaw
        Behavior on rotation {
            NumberAnimation {
                duration: 300
                easing.type: Easing.OutQuad
            }
        }
    }

    /*Item {
        id: hud_visual
        width: artificialHorizon2.width
        height: artificialHorizon2.height
        z:5
        anchors.top: artificialHorizon2.top
        anchors.left: artificialHorizon2.left
        anchors.bottom: artificialHorizon2.bottom
        // === API (matches your old gauge) ===
        property real minimumValue: -90
        property real maximumValue: 90
        property real value: 0

        property real startAngle: 90
        property real endAngle: -90
        property real tickStep: 45
        property int minorTickCount: 10

        // === Derived ===
        readonly property real range: maximumValue - minimumValue

        function valueToAngle(v) {
             return 90 + (v - minimumValue) / range * (-180)
         }

        // === Major Tick Marks ===
        Repeater {
            model: hud_visual.range / hud_visual.tickStep + 1
            delegate: Rectangle {
                width: hud_visual.width * 0.01
                height: hud_visual.width * 0.075
                color: "#e34c22"
                antialiasing: true

                anchors.centerIn: parent
                transform: Rotation {
                    angle:hud_visual.valueToAngle(
                              hud_visual.minimumValue + index * hud_visual.tickStep)
                    origin.x: width / 2
                    origin.y: hud_visual.height / 2
                }
            }
        }

        // === Labels ===
        Repeater {
            model: hud_visual.range / hud_visual.tickStep + 1
            delegate: Text {
                text: hud_visual.minimumValue + index * hud_visual.tickStep
                font.bold: true
                font.pointSize: Math.min(hud_visual.width / 23,
                                          hud_visual.height / 15)
                color: "white"
                antialiasing: true

                anchors.centerIn: parent
                transform: Rotation {
                    angle: hud_visual.valueToAngle(hud_visual.minimumValue + index * hud_visual.tickStep)
                    origin.x: hud_visual.width / 2
                    origin.y: hud_visual.height / 2
                }
                y: hud_visual.height * 0.1
            }
        }

        // === Needle ===
        Repeater {
            model: hud_visual.range / hud_visual.tickStep + 1

            delegate: Rectangle {

                width: hud_visual.width * 0.02
                height: hud_visual.width * 0.1
                color: "#e34c22"

                anchors.centerIn: parent

                transform: Rotation {
                    angle: hud_visual.valueToAngle(
                              hud_visual.minimumValue +
                              index * hud_visual.tickStep)
                    origin.x: width / 2
                    origin.y: hud_visual.height / 2
                }
            }
        }
    }*/

    OpacityMask {
             anchors.fill: artificialHorizon2
             source: artificialHorizon2
             maskSource: mask
             z: 1
         }
         Rectangle {
             id: mask
             anchors.fill: artificialHorizon2
             radius: artificialHorizon2.width / 2
             color: "black"
             visible: false
             z: 1
         }
         Image {
             id: line_1
             source: "qrc:/resources/images/crossHair.svg"
             anchors.centerIn: artificialHorizon2
             mipmap: true
             z: 2
             width: gauge.width
             sourceSize.width: width
             fillMode: Image.PreserveAspectFit
             visible:true

         /*Image {
             id: head_2
             width: 0.25 * parent.width
             height: 0.005 * root.height
             anchors.top: artificialHorizon2.top
             anchors.horizontalCenter: parent.horizontalCenter
             z: 3
             visible: true
             source: "qrc:/resources/images/up_button_red.svg"
         }*/


             /*Connections {
                 target: hud_visual
                 function onWidthChanged() {
                     head_2.requestPaint()
                 }
             }
             onPaint: {
                 var ctx = getContext("2d")
                 ctx.reset()
                 ctx.fillStyle = "red"
                 ctx.beginPath()
                 ctx.moveTo(0, height) // Start from the bottom-left corner
                 ctx.lineTo(width / 2, 0) // Draw to the top-middle point
                 ctx.lineTo(width, height) // Draw to the bottom-right corner
                 ctx.closePath()
                 ctx.fill()
             }*/
         }
         Rectangle {
             id: artificialHorizon2
             visible: false
             width: gauge.width
             height: gauge.width
             anchors
             {
                 top:modePanel.top
                 left:gauge.left
                 //margins:0.005*root.width
             }

             color: "transparent"
             radius: 100

             Rectangle {
                 id: artificialHorizon
                 width: root.width * 4
                 height: root.height * 8
                 anchors.centerIn: parent
                 visible: true
                 opacity:  1.0

                 Rectangle {
                     height: size * 0.75
                     width: size * 0.9
                     anchors.horizontalCenter: parent.horizontalCenter
                     anchors.verticalCenter: parent.verticalCenter
                     color: Qt.rgba(0, 0, 0, 0)
                     clip: true
                     z: 2
                     Item {
                         height: parent.height
                         width: parent.width
                         Column {
                             anchors.horizontalCenter: parent.horizontalCenter
                             anchors.verticalCenter: parent.verticalCenter
                             spacing: _reticleSpacing
                             Repeater {
                                 model: 36
                                 Rectangle {
                                     property int _pitch: -(modelData * 5 - 90)
                                     anchors.horizontalCenter: parent.horizontalCenter
                                     width: (_pitch % 10) === 0 ? _longDash : _shortDash
                                     height: _reticleHeight
                                     color: "white"
                                     antialiasing: true
                                     smooth: true

                                     Text {
                                         id: pitch_indicator_1
                                         anchors.horizontalCenter: parent.horizontalCenter
                                         anchors.horizontalCenterOffset: -(_longDash)
                                         anchors.verticalCenter: parent.verticalCenter
                                         smooth: true
                                         //font.family: ScreenTools.demiboldFontFamily
                                         font.pointSize:  0.5 * _fontSize
                                         text: _pitch
                                         font.bold: true
                                         style: Text.Sunken
                                         color: "white"
                                         visible: (_pitch !== 0)
                                                  && ((_pitch % 10) === 0)
                                     }

                                     Text {
                                         id: pitch_indicator_2
                                         anchors.horizontalCenter: parent.horizontalCenter
                                         anchors.horizontalCenterOffset: (_longDash)
                                         anchors.verticalCenter: parent.verticalCenter
                                         smooth: true
                                         //font.family: ScreenTools.demiboldFontFamily
                                         font.pointSize:  0.5 * _fontSize
                                         text: (_pitch)
                                         font.bold: true
                                         style: Text.Sunken
                                         color: "white"
                                         visible: (_pitch !== 0)
                                                  && ((_pitch % 10) === 0)
                                     }
                                 }
                             }
                         }
                         transform: [
                             Translate {
                                 y: (y6 * _reticleSlot / 7) - (_reticleSlot / 2)
                             }
                         ]
                     }
                 }

                 Rectangle {
                     id: sky
                     anchors.fill: parent
                     z: 1
                     smooth: true
                     antialiasing: true
                     gradient: Gradient {
                         GradientStop {
                             position: 0.25
                             color: Qt.hsla(0.6, 1.0, 0.25)
                         }
                         GradientStop {
                             position: 0.5
                             color: Qt.hsla(0.6, 0.5, 0.55)
                         }
                     }
                 }
                 Rectangle {
                     id: ground
                     z: 1
                     height: sky.height / 2
                     anchors {
                         left: sky.left
                         right: sky.right
                         bottom: sky.bottom
                     }
                     smooth: true
                     antialiasing: true
                     gradient: Gradient {
                         GradientStop {
                             position: 0.0
                             color: Qt.hsla(0.25, 0.5, 0.45)
                         }
                         GradientStop {
                             position: 0.25
                             color: Qt.hsla(0.25, 0.75, 0.25)
                         }
                     }
                 }
                 transform: [
                     Translate {
                         y: y6
                     },
                     Rotation {
                         origin.x: artificialHorizon.width / 2
                         origin.y: artificialHorizon.height / 2
                         angle: roll
                     }
                 ]
             }
         }

    function mapValue(value, inputMin, inputMax, outputMin, outputMax) {
        var mappedValue = (value - inputMin) * (outputMax - outputMin)
                / (inputMax - inputMin) + outputMin
        return Math.min(Math.max(mappedValue, outputMin),
                        outputMax)
    }
    ColumnLayout {
        id: column

        spacing: 0.005 * root.width
    }
    RowLayout {
        id: row6
        Layout.fillWidth: pause_recording.visible ? true : false
    }
    Button
        {
            id:start_stop_recording
            width:0.02*root.width
            height:0.02*root.width
            anchors
            {
                top:box_5.top
                topMargin:0.005*root.width
                horizontalCenter:box_5.horizontalCenter
                horizontalCenterOffset:pause_recording.visible==false?0:-0.25*box_5.width
            }

            Behavior on anchors.horizontalCenterOffset {
                PropertyAnimation { duration: 300; easing.type: Easing.InOutQuad }
            }
            visible: true
            background: Rectangle
            {
                anchors.fill:parent
                color:"grey"//"#011026"
                opacity:0.5
                radius:100
            }

            contentItem: Canvas {
                id:sta_sto
                anchors.fill: parent
                contextType: "2d"
                anchors.centerIn: parent

                 onPaint: {
                     var ctx = getContext("2d");
                     ctx.clearRect(0, 0, width, height); // Clear the canvas

                     if (!start_or_stop) {
                         ctx.fillStyle = "#eadeda";
                         ctx.beginPath();
                         ctx.moveTo(0.3 * width, 0.2 * height);
                         ctx.lineTo(0.3 * width, 0.8 * height);
                         ctx.lineTo(0.8 * width, 0.5 * height);
                         ctx.closePath();
                         ctx.fill();
                                 } else { // Draw stop icon when recording
                         ctx.fillStyle = "red";

                                     ctx.fillRect(0.3 * width, 0.25 * height, 0.45 * width, 0.5 * height);
                                 }

                 }
             }
            MouseArea
                {
                    anchors.fill: parent
                    hoverEnabled: true
                    onClicked:
                    {
                        //console.log(start_or_stop)
                        start_or_stop = !start_or_stop
                        start_stop_recording.contentItem.requestPaint();

                        if(start_or_stop)
                        {
                            record_tool_text="Stop and\nSave Recording"
                        statusindicator2.color="darkgreen";
                        VideoStreamer.start_recording();
                            /*template_text2.text="Alert!!!"
                            template_content2.text="Video has been saved successfully!!!"
                            message_template2.open()*/
                        //console.log("Record started")
                        statusindicator2.visible=true
                        start_Timer.start()
                            pause_recording.visible=true
                        }

                        else if(!start_or_stop)
                        {
                            record_tool_text="Start\nVideo Recording"
                            statusindicator2.color="red"
                            VideoStreamer.stop_recording();
                            //console.log("Recording stopped");
                            start_Timer.elapsedTime=0
                            timerText.text="00:00:00"
                            statusindicator2.visible=false
                            pause_recording.visible=false
                            start_Timer.stop()


                        }

                    }
                    onPressed:
                            {
                                start_status = !start_status
                            }

                    onReleased:
                            {
                                start_status = !start_status
                            }

                    onEntered:
                    {

                       tooltip_set(start_stop_recording.x+mouseX,start_stop_recording.y-0.05*root.height,record_tool_text)
                    }

                    onExited:
                    {
                        tooltip_reset()
                    }
                }
        }
    Button {
        id: pause_recording
        width: pause_recording.visible ? 0.02*root.width : 0
        height:0.02*root.width
        visible: false
        anchors {
            top: box_5.top
            topMargin:0.005*root.width
            horizontalCenter: box_5.horizontalCenter
            horizontalCenterOffset: pause_recording.visible == false ? 0 : +0.25 * box_5.width
        }
        background: Rectangle {
            anchors.fill:parent
            color: "grey" //"#011026"
            opacity: 0.5
            radius: 100
        }
        property bool pressed_not: true
        contentItem: Canvas {
            id: pause_rec2
            anchors.fill: parent
            contextType: "2d"
            anchors.centerIn: parent
            onPaint: {
                var ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height) // Clear the canvas
                ctx.fillStyle = "#eadeda"

                if (!pause_play) {
                    ctx.fillStyle = "#eadeda"
                    ctx.fillRect(0.30 * width, 0.2 * height, 0.15 * width,
                                 0.6 * height)
                    ctx.fillRect(0.55 * width, 0.2 * height, 0.15 * width,
                                 0.6 * height)
                } else {
                    ctx.fillRect(0.30 * width, 0.2 * height, 0.15 * width,
                                 0.6 * height)

                    ctx.beginPath()
                    ctx.moveTo(0.5 * width, 0.2 * height)
                    ctx.lineTo(0.5 * width, 0.8 * height)
                    ctx.lineTo(0.8 * width, 0.5 * height)
                    ctx.closePath()
                    ctx.fill()
                }
            }
        }
        MouseArea {
            anchors.fill: parent
            hoverEnabled: true

            onClicked: {
                pause_play = !pause_play
                pause_recording.contentItem.requestPaint()
                if (pause_play) {
                    record_tool_text2 = "Play Recording"
                    start_Timer.stop()
                    VideoStreamer.pauseStreaming()
                    start_Timer2.start()
                    statusindicator2.color = "red"
                } else if (!pause_play) {
                    record_tool_text2 = "Pause Recording"
                    start_Timer.start()
                    VideoStreamer.pauseStreaming()
                    start_Timer2.stop()
                    statusindicator2.stopBlinkAnimation()
                    statusindicator2.color = "green"
                }
            }
            onPressed: {
                pause_recording.pressed_not = true
            }
            onReleased: {
                pause_recording.pressed_not = false
            }
            onEntered: {
                tooltip_set(pause_recording.x + mouseX,
                            pause_recording.y - 0.05 * root.height,
                            record_tool_text2)
            }

            onExited: {
                tooltip_reset()
            }
        }
    }

    Button {
        id: capture_ss2
        width:0.02*root.width
        height:0.02*root.width
        visible: true
        property bool pressed_not: false
        background: Rectangle {
            color: "grey"
            anchors.fill:parent
            opacity: 0.5
            radius: 100
            border.color: capture_ss2.pressed_not ? "white" : "transparent"
            border.width: 0.005 * root.width
        }
        anchors {
            bottom:imageRect.bottom
            bottomMargin:0.0075*root.width
            horizontalCenter: box_5.horizontalCenter
            horizontalCenterOffset: -0.01
                                    * box_5.width
        }

        contentItem: Image {
            source: "qrc:/resources/images/screenshot3.png"
            anchors.fill: parent
        }
        MouseArea {
            id: area_102
            anchors.fill: parent
            hoverEnabled: true

            onClicked: {
                capture_image()

            }
            onPressed: {
                capture_ss2.pressed_not = true
            }

            onReleased: {
                capture_ss2.pressed_not = false
            }
            onEntered: {
                tooltip_set(capture_ss2.x + mouseX,
                            capture_ss2.y - capture_ss2.height,
                            "Capture the\n Frame")
            }

            onExited: {
                tooltip_reset()
            }
        }
    }
    /*Connections {
        target: myLink

        function onDataUpdated() {
            var data = myLink.get_data()
            y10.text = yaw = data[0]
            p10.text = depth = data[1]
            data = []
        }
    }*/
    function capture_image()
                   {
                        var saveDirectory;
                            if (Qt.platform.os === "windows") {
                                saveDirectory = StandardPaths.writableLocation(StandardPaths.PicturesLocation);
                            } else {
                               saveDirectory = StandardPaths.writableLocation(StandardPaths.HomeLocation);
                            }
                            var logFileDir = saveDirectory + "/orcascreenshots";
                            var rectangles = [imageRect]
                            var rect,count_ss= 0;
                            for (var i = 0; i < rectangles.length; i++) {
                                (function (rect) {
                                    var date = new Date();
                                    var formattedDate = date.toLocaleString(Qt.locale("en_IN"), "dd.MM.yyyy-hh.mm.ss");
                                    var fileName = "/screenshot_" + formattedDate + "_" + i;
                                    rect.grabToImage(function (result) {
                                        var filePath = logFileDir + fileName+".jpeg";
                                        filePath = filePath.replace("file:///", "");
                                        if (result.saveToFile(filePath)) {
                                            count_ss++
                                        } else {
                                            console.log("Failure");
                                        }
                                    }, Qt.size(opencvImage.sourceSize.width, opencvImage.sourceSize.height));
                                })(rectangles[i]);
                            }
                               map_image_Captured()

               }
    Popup {
           id: about_window
           width: 0.45 * root.width
           height: 0.6 * root.height
           anchors.centerIn: parent
           //visible: on_console
           closePolicy: Popup.CloseOnPressOutside
           z: 1
           modal: true
           background: Rectangle {
              // color: "transparent"

               radius: 0.005*root.width
               opacity: 1.0
           }
           focus: true

           ScrollView {
               id: scroll2
               anchors {
                   top: parent.top
                   left: parent.left
                   right: parent.right
                   bottom: parent.bottom
                   margins: 0.005 * root.width
               }
               width: about_window.width
               height: 0.95 * about_window.height
               clip: true

               ColumnLayout {
                   width: scroll2.width
                   spacing: 0.0075 * root.width
                   Text {
                       Layout.fillWidth: true
                       id: about_title
                       text: "\Orca "
                       font.bold: true
                       font.italic: false

                       font.underline: false
                       font.strikeout: false
                       font.pixelSize: Math.min(root.width / 40, root.height / 30)
                       color:"black"
                       horizontalAlignment: Text.AlignHCenter
                   }

                   Text {
                       Layout.fillWidth: true
                       id: about_body
                       text: "\n\nVersion: 3.1.0\n\n\nThe ORCA is a centralized interface designed for monitoring,controlling and commanding interface for the Amphibious Crawling Robot developed by VIKRA.This application is designed to provide real-time vehicle status updates,enables real-time acquisition of sensor data from the vehicle, and allow seamless control of its movements and operations.It also supports the operation of the integrated Cone Penetration Tester (CPT) for geotechnical assessments and analysis.With a focus on performance, reliability, and usability, the control station offers a smooth,intuitive and  user-friendly  user interface(UI) to ensure intuitive interaction, reliable performance monitoring, tailored for efficient field operations in diverse terrains i.e., across both land and shallow water environments.\n\n\nThis Software is built on Qt which is a open-source software for developing embedded applications based on Qt 5.12.2 (MINGW,x64).Along with, 3rd party open-source library used for Video processing.No copyright violation taken place"
                       //font.family: font_family
                       //font.bold: true
                       font.italic: false
                       font.underline: false
                       font.strikeout: false
                       font.pixelSize: Math.min(root.width / 70, root.height / 60)
                       color: "black"
                       wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                   }

                   Text {
                       Layout.fillWidth: true
                       id: about_title2
                       text: "About VIKRA "
                       //font.family: font_family
                       font.bold: true
                       font.italic: false
                       font.underline: false
                       font.strikeout: false
                       font.pixelSize: Math.min(root.width / 40, root.height / 30)
                       color: "black"
                       //horizontalAlignment: Text.AlignHCenter
                   }

                   Text {
                       Layout.fillWidth: true
                       id: about_body2
                       text: "Vikra Ocean Tech is a pioneering ocean robotics startup focused on developing autonomous underwater platforms, including ASVs, ROVs, and advanced underwater surveillance systems. Our technology empowers industries to explore, monitor, and maintain marine and inland water environments with precision. We specialize in dam, bridge, and reservoir inspections, aquaculture, environmental research, and deep-sea experiments. At Vikra, we are committed to enhancing underwater exploration and operations through innovation and cutting-edge solutions.

   Experts in autonomous underwater robotics (ASVs, ROVs, surveillance).
   Diverse applications: dam inspections, aquaculture, research, and naval.
   Offering deep-sea experimentation for academic and commercial projects.
   Developers of Koorma, an amphibious land and water robot."
                       //font.family: font_family
                       font.bold: false
                       font.italic: false
                       font.underline: false
                       font.strikeout: false
                       font.pixelSize: Math.min(root.width / 70, root.height / 60)
                       color: "black"
                       wrapMode: Text.WrapAtWordBoundaryOrAnywhere
                   }
               }
           }
       }


       Popup {
               id: helpPopup
               width: 0.3 * root.width
               height: 0.3 * root.height
               anchors.centerIn: parent
               focus: true
               closePolicy: Popup.CloseOnPressOutside
               z: 1
               modal: true

               Rectangle {
                   anchors.fill: parent
                   color: "#001f2f"
                   radius: 0.005*root.width
               }
               ScrollView {
                   id: scroll3
                   anchors {
                       top: parent.top
                       left: parent.left
                       right: parent.right
                       bottom: parent.bottom
                       margins: 0.005 * root.width
                   }
                   padding: 0.005 * root.width
                   width: helpPopup.width
                   height: 0.95 * helpPopup.height
                   clip: true

                   ColumnLayout {
                       width: scroll3.width
                       spacing: 0.0075 * root.width
                       Repeater {
                           model: help_text_Array
                           Text {
                               text: modelData
                               wrapMode: Text.Wrap
                               //font.family: font_family
                               font.bold: true
                               font.italic: false
                               font.underline: false
                               font.strikeout: false
                               //textFormat: Text.RichText
                               color: "white"
                               font.pixelSize: Math.min(root.width / 50, root.height / 40)
                               //onLinkActivated: Qt.openUrlExternally(link)
                           }
                       }
                   }
               }
           }

           Popup {
               id: report_window
               width: 0.25 * root.width
               height: 0.3 * root.height
               anchors.centerIn: parent
               focus: true
               closePolicy: Popup.CloseOnPressOutside
               z: 1
               modal: true
               background: Rectangle {
                   color: "transparent"
               }

               Rectangle {
                   anchors.fill: parent
                   color: "#001f2f"
                   radius: 0.005*root.width
               }
               ColumnLayout {
                   width: 0.85 * parent.width
                   height: parent.height
                   anchors.top: parent.top
                   anchors.margins: 0.005 * root.width
                   anchors.horizontalCenter: parent.horizontalCenter
                   spacing: 0.0075 * root.width
                   Item {
                       width: 0.5 * parent.width
                       height: parent.height
                       Layout.fillWidth: true
                       Layout.fillHeight: true

                       TextArea {
                           id: min7
                           Layout.fillWidth: true
                           width: parent.width
                           height: 0.5 * report_window.height

                           //Layout.fillWidth: true
                           //Layout.alignment: Qt.AlignTop
                           //Layout.topMargin:  0.04*parent.width
                           //Layout.leftMargin:  0.02*parent.width
                           //Layout.rightMargin:  0.02*parent.width
                           placeholderText: "Describe the issue"
                           wrapMode: TextArea.WrapAtWordBoundaryOrAnywhere // Enable text wrapping
                           placeholderTextColor: "grey"
                           font.pixelSize: Math.min(root.width / 70, root.height / 60)
                           font.kerning: false
                           font.family: "Helvetica [Cronyx]"
                           font.capitalization: Font.Capitalize
                           //font.bold : true
                           selectByMouse: true
                           background: Rectangle {
                               radius: 0.005*root.width
                               border.color: min7.focus ? "#21be2b" : "transparent"
                           }

                           onPressed: {
                               min7.focus = true
                           }
                       }
                   }
                   Item {
                       Layout.fillWidth: true
                   }
                   Item {
                       Layout.fillWidth: true
                   }
                   Item {
                       width: 0.5 * parent.width
                       Layout.fillWidth: true
                       Layout.fillHeight: true
                       Layout.alignment: Qt.AlignBottom
                       Button {
                           Layout.topMargin: 0.04 * parent.width
                           //Layout.rightMargin: 0.03*parent.width
                           Layout.fillWidth: true
                           Layout.fillHeight: true
                           //z:1
                           //font.pixelSize: Math.min(parent.width / 20, parent.height / 10)
                           background: Rectangle {
                               color:"#011026"// background_colortheme2
                               radius: 0.005*root.width // 5
                               border.color:  neonGreen
                           }

                           contentItem: Text {
                               //text: export_log.text
                               text: "Submit"
                               //font.family: "Segoe UI Black"
                               font.pixelSize: Math.min(root.width / 60,
                                                        root.height / 50)
                               style: Text.Sunken
                               //font.family: font_family
                               font.bold:true// bold_not
                               font.italic: false
                               font.underline: false
                               font.strikeout: false
                               //font: export_log.font
                               //opacity: enabled ? 1.0 : 0.3
                               color: "White"
                               horizontalAlignment: Text.AlignHCenter
                               verticalAlignment: Text.AlignVCenter
                               //elide: Text.ElideRight
                           }

                           MouseArea {
                               anchors.fill: parent
                               onClicked: {
                                   if (min7.text.length > 0) {
                                       ReportSender.sendReport(min7.text)
                                       min7.text = "" // Clear the form
                                       report_window.close()
                                       msg_heading = "Acknowledgement"
                                       msg_text = "Message Status              "
                                       msg_text2 = "Report Submitted Succesfully"
                                       msg.open() // Optional: close popup
                                   } else {
                                       msg_heading = "Error"
                                       msg_text = "Message Status              "
                                       msg_text2 = "No Description found"
                                       msg.open() // Optional: close popup
                                   }
                               }
                           }
                       }
                   }
               }
           }

           Popup {
               id: support_window
               width: 0.275 * root.width
               height: 0.26 * root.height
               anchors.centerIn: parent
               focus: true
               closePolicy: Popup.CloseOnPressOutside
               z: 1
               modal: true
               background: Rectangle {
                   color: "transparent"
               }

               Rectangle {
                   anchors.fill: parent
                   color: "#001f2f"
                   radius: 0.005*root.width
               }

               ColumnLayout {
                   width: 0.85 * parent.width
                   height: 0.975 * parent.height
                   anchors.top: parent.top
                   anchors.margins: 0.005 * root.width
                   anchors.horizontalCenter: parent.horizontalCenter
                   spacing: 0.0075 * root.width
                   Item {
                       Layout.fillWidth: true
                       Layout.fillHeight: true
                       Text {
                           //text: export_log.text
                           text: "Contact "
                           wrapMode: Text.Wrap
                           //font.family: font_family
                           font.bold: true
                           font.italic: false
                           font.underline: false
                           font.strikeout: false
                           font.pixelSize: Math.min(root.width / 70, root.height / 60)
                           style: Text.Sunken
                           //font: export_log.font
                           //opacity: enabled ? 1.0 : 0.3
                           color: "White"
                           horizontalAlignment: Text.AlignHCenter
                           verticalAlignment: Text.AlignVCenter
                           //elide: Text.ElideRight
                       }
                   }

                   Item {
                       Layout.fillWidth: true
                       Layout.fillHeight: true

                       Text {
                           //text: export_log.text
                           text: "<a href=https://vikraoceantech.com/#contact style='color:lightblue;'>https://vikraoceantech.com/#contact</a>"
                           textFormat: Qt.RichText
                           //wrapMode: Text.Wrap
                           //font.family: font_family
                           font.bold: true
                           font.italic: false
                           font.underline: false
                           font.strikeout: false
                           font.pixelSize: Math.min(root.width / 70, root.height / 60)
                           color: "White"
                           horizontalAlignment: Text.AlignHCenter
                           verticalAlignment: Text.AlignVCenter
                           onLinkActivated: Qt.openUrlExternally(
                                                "https://vikraoceantech.com/#contact")
                       }
                   }
                   Item {
                       Layout.fillWidth: true
                       Layout.fillHeight: true
                       Text {
                           text: "(or)"
                           //font.family: font_family
                           font.bold: true
                           font.italic: false
                           font.underline: false
                           font.strikeout: false
                           font.pixelSize: Math.min(root.width / 70, root.height / 60)
                           style: Text.Sunken

                           //font: export_log.font
                           //opacity: enabled ? 1.0 : 0.3
                           color: "White"
                           horizontalAlignment: Text.AlignHCenter
                           verticalAlignment: Text.AlignVCenter
                           //elide: Text.ElideRight
                       }
                   }
                   Item {
                       width: 0.16 * parent.width
                       Layout.fillWidth: true
                       Layout.fillHeight: true
                       Text {
                           text: "Please visit <a href='https://vikraoceantech.com' style='color:lightblue;'>vikraoceantech.com</a><br>for further information."
                           textFormat: Qt.RichText

                           //font.family: font_family



                           font.bold: true
                           font.italic: false
                           font.underline: false
                           font.strikeout: false
                           font.pixelSize: Math.min(root.width / 70, root.height / 60)
                           color: "White"
                           visible: true
                           z: 1

                           MouseArea {
                               anchors.fill: parent
                               cursorShape: Qt.PointingHandCursor
                               onClicked: {
                                   Qt.openUrlExternally("https://vikraoceantech.com")
                                   support_window.close()
                               }
                           }
                       }
                   }
               }
           }
           Rectangle {
                   id: err_rect
                   //text: "No error"
                   //anchors.top:root.top
                   parent: root.menuBar // Set main menu toolbar as parent for tool button
                   anchors.top:parent.top
                   anchors.horizontalCenter: parent.horizontalCenter
                   //anchors.topMargin: model.height>parent.height/2?0:0.01*parent.height
                   //x:parent.width/2
                   //anchors.horizontalCenter: parent.horizontalCenter
                   //anchors.verticalCenter: parent.verticalCenter
                   width: Math.min(parent.width * 0.35, 420)
                       height: parent.height * 0.75
                       radius: 6
                   //visible: false
                   //anchors.leftMargin:cam.height<root.height/2?0.025*parent.width:0.075*parent.width
                   //font.pixelSize:Math.min(root.width/65,root.height/45)//main_page.model.height>parent.height/2?Math.min(root.width/75,root.height/55):Math.min(root.width/55,root.height/35)
                   color: "transparent"
                   visible: err_text !== ""

                      clip: true
                   anchors.centerIn: parent

                   /*gradient: Gradient {
                                                GradientStop { position: 0.0; color: "lightsteelblue" }
                                                GradientStop { position: 1.0; color: "blue" }
                                            }*/
                   //z:1
                   RowLayout {
                       anchors.fill: parent

                      // width: parent.width
                     //  height: parent.height
                       spacing: 0.005 * parent.width

                       Item {
                           width: parent.width
                           height: parent.height
                           Layout.fillWidth: true
                           Layout.fillHeight: true
                           Image {
                               id: err_symbol
                               source: err_warning_path
                               //Layout.alignment: Qt.AlignVCenter
                              // sourceSize.width: parent.width
                               //sourceSize.height: parent.height
                               fillMode : Image.PreserveAspectFit
                               Layout.preferredWidth:height
                               Layout.preferredHeight:height
                               //yout.fillHeight: true
                           }
                       }

                       Text {
                           id: err_msg_text
                           text: err_text
                           font.bold: true
                           Layout.fillWidth: true
                          // Layout.fillHeight: true
                           verticalAlignment: Text.AlignVCenter
                           horizontalAlignment: Text.AlignHCenter
                           width: parent.width
                           height: parent.height
                           color: "yellow"
                           font.pixelSize: Math.min(root.width / 65, root.height / 45)
                           //font.family: font_family
                           //font.bold: bold_not
                           font.italic: false
                           font.underline: false
                           font.strikeout: false
                       }
                   }

                   property bool isBlink: false

                   // Start blinking when isBlink changes
                   onIsBlinkChanged: {
                       if (isBlink)
                           startBlinkAnimation1()
                       else
                           stopBlinkAnimation1()
                   }

                   function startBlinkAnimation1() {
                       blinkAnimation1.running = true
                   }

                   function stopBlinkAnimation1() {
                       blinkAnimation1.running = false
                       opacity = 1 // Ensure the indicator is visible after stopping
                   }

                   SequentialAnimation {
                       id: blinkAnimation1
                       loops: Animation.Infinite

                       PropertyAnimation {
                           target: err_rect
                           property: "visible"
                           from: true
                           to: false
                           duration: 500
                       }
                       PropertyAnimation {
                           target: err_rect
                           property: "visible"
                           from: false
                           to: true
                           duration: 300
                       }
                   }

                   Timer {
                       id: start_Timer5
                       interval: 750 // Timer interval in milliseconds
                       running: false // Start the timer when the application starts
                       repeat: true

                       property int elapsedTime: 1 // Elapsed time in seconds

                       onTriggered: {
                           err_rect.isBlink = !err_rect.isBlink // Toggle blinking
                       }
                   }
               }
}
