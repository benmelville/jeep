// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only

import QtQuick 6.2
import jeep

Window {
    width: Constants.width
    height: Constants.height

    Text {
        text: "JEEP"
    }

    visible: true
    title: foo()

    function foo() {
        console.log("hello world")
        return "JEEP";
    }

    Rectangle {
        x: 100; y: 200
        height: 100 * width
        width: 2
        color: "green"
    }

    Rectangle {
        x: 100; y: 50; z: -5
        height: 100
        width: 250
        color: "purple"
    }

}

