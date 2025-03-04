// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only

import QtQuick 6.2
import QtQuick.Controls
import jeep

Window {
    width: Constants.width
    height: Constants.height




    visible: true
    title: foo()

    function foo() {
        console.log("hello world")
        return "JEEP";
    }

    // TextInput {
    //     id: textElement
    //     x: 50; y: 25
    //     text: "JEEP"
    //     font.family: "Helvetica"; font.pixelSize: 50
    // }

    // Rectangle {
    //     id: ben
    //     x: 50; y: 75; height: 5
    //     width: textElement.width
    //     color: "navy"
    // }

    // Row {
    //     spacing: 3

    //     Repeater {
    //         model: 70

    //         Rectangle {
    //             x: 50; y: 50
    //             width: 5
    //             opacity: 0.45
    //             height: 100
    //             color: "white"
    //         }
    //     }
    // }

    property int rpm: 3000  // Dynamic RPM value (0 to 8000)
        property int maxRPM: 8000
        property int numBars: 100  // Number of gauge segments

        Column {
            anchors.centerIn: parent
            spacing: 20

            Row {
                spacing: 5
                anchors.horizontalCenter: parent

                Repeater {
                    model: numBars
                    Rectangle {
                        width: 8
                        height: 150
                        opacity: rpm >= threshold ? 1 : 0.8

                        // Determine threshold for each bar
                        property int threshold: ((index + 1) * (maxRPM / numBars))

                        // Set color based on RPM value
                        color: rpm >= threshold ? (threshold < 5000 ? "green" :
                                                   threshold < 7000 ? "yellow" :
                                                   "red") : "gray"


                        // Smooth animation when RPM changes
                        Behavior on color {
                            ColorAnimation { duration: 100 }
                        }
                    }
                }
            }

            // Slider to simulate RPM changes
            Slider {
                width: 400
                from: 0
                to: maxRPM
                value: rpm
                onValueChanged: rpm = value
            }
        }


    Image {
        x: 0; y: 0; z: -1
        source: "./images/jeep-backsplash.jpg"
        width: Constants.width
        height: Constants.height
    }

    // Rectangle {
    //     x: 100; y: 50; z: -5
    //     height: 100
    //     width: 250
    //     color: ben.color
    // }

}

