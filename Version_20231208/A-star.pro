QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
    about.cpp \
    analysis.cpp \
    astar.cpp \
    main.cpp \
    mainwindow.cpp \
    maplabel.cpp \
    setxyDialog.cpp

HEADERS += \
    about.h \
    analysis.h \
    astar.h \
    mainwindow.h \
    maplabel.h \
    setxyDialog.h

FORMS +=

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

TARGET = xxfer

RESOURCES += \
    icon.qrc

RC_ICONS = purchase.ico
