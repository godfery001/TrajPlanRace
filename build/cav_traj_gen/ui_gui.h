/********************************************************************************
** Form generated from reading UI file 'gui.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GUI_H
#define UI_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "battery.h"
#include "display_3d.h"

QT_BEGIN_NAMESPACE

class Ui_GUI
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_4;
    DISPLAY_3D *display_3d;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout_3;
    QCheckBox *checkBox_Ref_Path;
    QCheckBox *checkBox_RefPath_Heading;
    QCheckBox *checkBox_RefPath_Cr;
    QCheckBox *checkBox_BestPath_Heading;
    QCheckBox *checkBox_BestPath_Cr;
    QCheckBox *checkBox_Axis;
    QCheckBox *checkBox_Obstacle;
    QCheckBox *checkBox_BestPath;
    QFrame *line_5;
    QVBoxLayout *verticalLayout_3;
    QGridLayout *gridLayout_2;
    QSlider *horizontalSlider_scale;
    QLabel *label_3;
    QSlider *horizontalSlider_Y;
    QLabel *label;
    QSlider *horizontalSlider_X;
    QLabel *label_2;
    QGridLayout *gridLayout;
    QLabel *label_4;
    QSlider *horizontalSlider_obstacle_sensitivity;
    QLabel *label_5;
    QSlider *horizontalSlider_target_time;
    QFrame *line_6;
    QVBoxLayout *verticalLayout;
    QCheckBox *checkBox_check_obstacles;
    QCheckBox *checkBox_ay_soft_const;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_Change_View;
    QVBoxLayout *verticalLayout_6;
    QLCDNumber *lcdNumber;
    Battery *battery;
    QTextEdit *textEdit_info;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *GUI)
    {
        if (GUI->objectName().isEmpty())
            GUI->setObjectName(QString::fromUtf8("GUI"));
        GUI->setEnabled(true);
        GUI->resize(1346, 839);
        QPalette palette;
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(181, 217, 229, 238));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        QBrush brush2(QColor(255, 255, 255, 238));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush2);
        QBrush brush3(QColor(218, 236, 242, 238));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        QBrush brush4(QColor(90, 108, 114, 238));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush4);
        QBrush brush5(QColor(120, 145, 153, 238));
        brush5.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        QBrush brush6(QColor(255, 255, 255, 255));
        brush6.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush);
        QBrush brush7(QColor(218, 236, 242, 246));
        brush7.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush7);
        QBrush brush8(QColor(255, 255, 220, 255));
        brush8.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        QBrush brush9(QColor(0, 0, 0, 128));
        brush9.setStyle(Qt::SolidPattern);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Active, QPalette::PlaceholderText, brush9);
#endif
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush7);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush9);
#endif
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush9);
#endif
        GUI->setPalette(palette);
        QFont font;
        font.setPointSize(12);
        GUI->setFont(font);
        GUI->setMouseTracking(true);
        GUI->setStyleSheet(QString::fromUtf8("background:rgba(181, 217, 229, 238)"));
        centralWidget = new QWidget(GUI);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayout_5 = new QVBoxLayout(centralWidget);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(18);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(-1, -1, -1, 6);
        display_3d = new DISPLAY_3D(centralWidget);
        display_3d->setObjectName(QString::fromUtf8("display_3d"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(display_3d->sizePolicy().hasHeightForWidth());
        display_3d->setSizePolicy(sizePolicy);
        display_3d->setMinimumSize(QSize(1071, 600));

        verticalLayout_4->addWidget(display_3d);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(12);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(18, -1, 18, -1);
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        checkBox_Ref_Path = new QCheckBox(centralWidget);
        checkBox_Ref_Path->setObjectName(QString::fromUtf8("checkBox_Ref_Path"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(checkBox_Ref_Path->sizePolicy().hasHeightForWidth());
        checkBox_Ref_Path->setSizePolicy(sizePolicy1);
        checkBox_Ref_Path->setStyleSheet(QString::fromUtf8(""));
        checkBox_Ref_Path->setChecked(true);
        checkBox_Ref_Path->setTristate(true);

        gridLayout_3->addWidget(checkBox_Ref_Path, 0, 0, 1, 1);

        checkBox_RefPath_Heading = new QCheckBox(centralWidget);
        checkBox_RefPath_Heading->setObjectName(QString::fromUtf8("checkBox_RefPath_Heading"));
        sizePolicy1.setHeightForWidth(checkBox_RefPath_Heading->sizePolicy().hasHeightForWidth());
        checkBox_RefPath_Heading->setSizePolicy(sizePolicy1);
        checkBox_RefPath_Heading->setStyleSheet(QString::fromUtf8("color: rgb(150, 150, 150);"));

        gridLayout_3->addWidget(checkBox_RefPath_Heading, 1, 0, 1, 1);

        checkBox_RefPath_Cr = new QCheckBox(centralWidget);
        checkBox_RefPath_Cr->setObjectName(QString::fromUtf8("checkBox_RefPath_Cr"));
        sizePolicy1.setHeightForWidth(checkBox_RefPath_Cr->sizePolicy().hasHeightForWidth());
        checkBox_RefPath_Cr->setSizePolicy(sizePolicy1);
        checkBox_RefPath_Cr->setStyleSheet(QString::fromUtf8("color: rgb(150, 150, 150);"));

        gridLayout_3->addWidget(checkBox_RefPath_Cr, 1, 1, 1, 2);

        checkBox_BestPath_Heading = new QCheckBox(centralWidget);
        checkBox_BestPath_Heading->setObjectName(QString::fromUtf8("checkBox_BestPath_Heading"));
        sizePolicy1.setHeightForWidth(checkBox_BestPath_Heading->sizePolicy().hasHeightForWidth());
        checkBox_BestPath_Heading->setSizePolicy(sizePolicy1);
        checkBox_BestPath_Heading->setStyleSheet(QString::fromUtf8("color: rgb(150, 150, 150);"));

        gridLayout_3->addWidget(checkBox_BestPath_Heading, 3, 0, 1, 1);

        checkBox_BestPath_Cr = new QCheckBox(centralWidget);
        checkBox_BestPath_Cr->setObjectName(QString::fromUtf8("checkBox_BestPath_Cr"));
        sizePolicy1.setHeightForWidth(checkBox_BestPath_Cr->sizePolicy().hasHeightForWidth());
        checkBox_BestPath_Cr->setSizePolicy(sizePolicy1);
        checkBox_BestPath_Cr->setStyleSheet(QString::fromUtf8("color: rgb(150, 150, 150);"));

        gridLayout_3->addWidget(checkBox_BestPath_Cr, 3, 1, 1, 2);

        checkBox_Axis = new QCheckBox(centralWidget);
        checkBox_Axis->setObjectName(QString::fromUtf8("checkBox_Axis"));
        sizePolicy1.setHeightForWidth(checkBox_Axis->sizePolicy().hasHeightForWidth());
        checkBox_Axis->setSizePolicy(sizePolicy1);
        checkBox_Axis->setStyleSheet(QString::fromUtf8(""));
        checkBox_Axis->setChecked(true);

        gridLayout_3->addWidget(checkBox_Axis, 4, 0, 1, 1);

        checkBox_Obstacle = new QCheckBox(centralWidget);
        checkBox_Obstacle->setObjectName(QString::fromUtf8("checkBox_Obstacle"));
        sizePolicy1.setHeightForWidth(checkBox_Obstacle->sizePolicy().hasHeightForWidth());
        checkBox_Obstacle->setSizePolicy(sizePolicy1);
        checkBox_Obstacle->setCursor(QCursor(Qt::IBeamCursor));
        checkBox_Obstacle->setStyleSheet(QString::fromUtf8(""));
        checkBox_Obstacle->setChecked(true);

        gridLayout_3->addWidget(checkBox_Obstacle, 5, 0, 1, 1);

        checkBox_BestPath = new QCheckBox(centralWidget);
        checkBox_BestPath->setObjectName(QString::fromUtf8("checkBox_BestPath"));
        sizePolicy1.setHeightForWidth(checkBox_BestPath->sizePolicy().hasHeightForWidth());
        checkBox_BestPath->setSizePolicy(sizePolicy1);
        checkBox_BestPath->setStyleSheet(QString::fromUtf8(""));
        checkBox_BestPath->setChecked(true);

        gridLayout_3->addWidget(checkBox_BestPath, 2, 0, 1, 1);

        gridLayout_3->setColumnStretch(0, 2);
        gridLayout_3->setColumnStretch(1, 3);

        horizontalLayout->addLayout(gridLayout_3);

        line_5 = new QFrame(centralWidget);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setFrameShape(QFrame::VLine);
        line_5->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_5);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setSpacing(6);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalSlider_scale = new QSlider(centralWidget);
        horizontalSlider_scale->setObjectName(QString::fromUtf8("horizontalSlider_scale"));
        sizePolicy1.setHeightForWidth(horizontalSlider_scale->sizePolicy().hasHeightForWidth());
        horizontalSlider_scale->setSizePolicy(sizePolicy1);
        horizontalSlider_scale->setMinimum(-100);
        horizontalSlider_scale->setMaximum(100);
        horizontalSlider_scale->setPageStep(10);
        horizontalSlider_scale->setValue(0);
        horizontalSlider_scale->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_scale, 0, 1, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        QFont font1;
        font1.setPointSize(11);
        label_3->setFont(font1);
        label_3->setStyleSheet(QString::fromUtf8(""));
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_3, 2, 0, 1, 1);

        horizontalSlider_Y = new QSlider(centralWidget);
        horizontalSlider_Y->setObjectName(QString::fromUtf8("horizontalSlider_Y"));
        sizePolicy1.setHeightForWidth(horizontalSlider_Y->sizePolicy().hasHeightForWidth());
        horizontalSlider_Y->setSizePolicy(sizePolicy1);
        horizontalSlider_Y->setMinimum(-100);
        horizontalSlider_Y->setMaximum(100);
        horizontalSlider_Y->setValue(0);
        horizontalSlider_Y->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_Y, 2, 1, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font1);
        label->setStyleSheet(QString::fromUtf8(""));
        label->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label, 0, 0, 1, 1);

        horizontalSlider_X = new QSlider(centralWidget);
        horizontalSlider_X->setObjectName(QString::fromUtf8("horizontalSlider_X"));
        sizePolicy1.setHeightForWidth(horizontalSlider_X->sizePolicy().hasHeightForWidth());
        horizontalSlider_X->setSizePolicy(sizePolicy1);
        horizontalSlider_X->setMinimum(-100);
        horizontalSlider_X->setMaximum(100);
        horizontalSlider_X->setValue(0);
        horizontalSlider_X->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(horizontalSlider_X, 1, 1, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setFont(font1);
        label_2->setStyleSheet(QString::fromUtf8(""));
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_2, 1, 0, 1, 1);

        gridLayout_2->setColumnStretch(0, 1);
        gridLayout_2->setColumnStretch(1, 2);

        verticalLayout_3->addLayout(gridLayout_2);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(-1, 9, -1, -1);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setFont(font1);
        label_4->setStyleSheet(QString::fromUtf8(""));
        label_4->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_4, 0, 0, 1, 1);

        horizontalSlider_obstacle_sensitivity = new QSlider(centralWidget);
        horizontalSlider_obstacle_sensitivity->setObjectName(QString::fromUtf8("horizontalSlider_obstacle_sensitivity"));
        sizePolicy1.setHeightForWidth(horizontalSlider_obstacle_sensitivity->sizePolicy().hasHeightForWidth());
        horizontalSlider_obstacle_sensitivity->setSizePolicy(sizePolicy1);
        horizontalSlider_obstacle_sensitivity->setMinimum(1);
        horizontalSlider_obstacle_sensitivity->setMaximum(5);
        horizontalSlider_obstacle_sensitivity->setPageStep(1);
        horizontalSlider_obstacle_sensitivity->setValue(3);
        horizontalSlider_obstacle_sensitivity->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_obstacle_sensitivity, 0, 1, 1, 1);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        QFont font2;
        font2.setPointSize(11);
        font2.setStyleStrategy(QFont::NoAntialias);
        label_5->setFont(font2);
        label_5->setStyleSheet(QString::fromUtf8(""));
        label_5->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_5, 1, 0, 1, 1);

        horizontalSlider_target_time = new QSlider(centralWidget);
        horizontalSlider_target_time->setObjectName(QString::fromUtf8("horizontalSlider_target_time"));
        sizePolicy1.setHeightForWidth(horizontalSlider_target_time->sizePolicy().hasHeightForWidth());
        horizontalSlider_target_time->setSizePolicy(sizePolicy1);
        horizontalSlider_target_time->setMinimum(2);
        horizontalSlider_target_time->setMaximum(8);
        horizontalSlider_target_time->setPageStep(1);
        horizontalSlider_target_time->setValue(5);
        horizontalSlider_target_time->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_target_time, 1, 1, 1, 1);

        gridLayout->setColumnStretch(0, 1);
        gridLayout->setColumnStretch(1, 2);

        verticalLayout_3->addLayout(gridLayout);

        verticalLayout_3->setStretch(0, 3);
        verticalLayout_3->setStretch(1, 2);

        horizontalLayout->addLayout(verticalLayout_3);

        line_6 = new QFrame(centralWidget);
        line_6->setObjectName(QString::fromUtf8("line_6"));
        line_6->setFrameShape(QFrame::VLine);
        line_6->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line_6);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        checkBox_check_obstacles = new QCheckBox(centralWidget);
        checkBox_check_obstacles->setObjectName(QString::fromUtf8("checkBox_check_obstacles"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(checkBox_check_obstacles->sizePolicy().hasHeightForWidth());
        checkBox_check_obstacles->setSizePolicy(sizePolicy2);
        checkBox_check_obstacles->setStyleSheet(QString::fromUtf8(""));
        checkBox_check_obstacles->setChecked(true);

        verticalLayout->addWidget(checkBox_check_obstacles);

        checkBox_ay_soft_const = new QCheckBox(centralWidget);
        checkBox_ay_soft_const->setObjectName(QString::fromUtf8("checkBox_ay_soft_const"));
        sizePolicy2.setHeightForWidth(checkBox_ay_soft_const->sizePolicy().hasHeightForWidth());
        checkBox_ay_soft_const->setSizePolicy(sizePolicy2);
        checkBox_ay_soft_const->setStyleSheet(QString::fromUtf8(""));
        checkBox_ay_soft_const->setChecked(true);

        verticalLayout->addWidget(checkBox_ay_soft_const);


        horizontalLayout->addLayout(verticalLayout);

        horizontalLayout->setStretch(0, 2);
        horizontalLayout->setStretch(2, 2);
        horizontalLayout->setStretch(4, 1);

        verticalLayout_4->addLayout(horizontalLayout);


        horizontalLayout_2->addLayout(verticalLayout_4);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(12);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetMinimumSize);
        pushButton_Change_View = new QPushButton(centralWidget);
        pushButton_Change_View->setObjectName(QString::fromUtf8("pushButton_Change_View"));
        sizePolicy1.setHeightForWidth(pushButton_Change_View->sizePolicy().hasHeightForWidth());
        pushButton_Change_View->setSizePolicy(sizePolicy1);
        QFont font3;
        font3.setPointSize(14);
        pushButton_Change_View->setFont(font3);
        pushButton_Change_View->setStyleSheet(QString::fromUtf8(""));

        verticalLayout_2->addWidget(pushButton_Change_View);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        lcdNumber = new QLCDNumber(centralWidget);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        sizePolicy1.setHeightForWidth(lcdNumber->sizePolicy().hasHeightForWidth());
        lcdNumber->setSizePolicy(sizePolicy1);
        QFont font4;
        font4.setBold(true);
        font4.setWeight(75);
        lcdNumber->setFont(font4);
        lcdNumber->setStyleSheet(QString::fromUtf8("color: white;"));
        lcdNumber->setSmallDecimalPoint(false);
        lcdNumber->setDigitCount(4);
        lcdNumber->setMode(QLCDNumber::Dec);
        lcdNumber->setProperty("value", QVariant(0.000000000000000));

        verticalLayout_6->addWidget(lcdNumber);

        battery = new Battery(centralWidget);
        battery->setObjectName(QString::fromUtf8("battery"));

        verticalLayout_6->addWidget(battery);


        verticalLayout_2->addLayout(verticalLayout_6);

        textEdit_info = new QTextEdit(centralWidget);
        textEdit_info->setObjectName(QString::fromUtf8("textEdit_info"));
        sizePolicy1.setHeightForWidth(textEdit_info->sizePolicy().hasHeightForWidth());
        textEdit_info->setSizePolicy(sizePolicy1);
        textEdit_info->setMinimumSize(QSize(150, 81));
        textEdit_info->setFocusPolicy(Qt::StrongFocus);
        textEdit_info->setAutoFillBackground(true);
        textEdit_info->setStyleSheet(QString::fromUtf8("background:rgba(181, 217, 229, 238)"));

        verticalLayout_2->addWidget(textEdit_info);

        verticalLayout_2->setStretch(0, 2);
        verticalLayout_2->setStretch(1, 4);
        verticalLayout_2->setStretch(2, 12);

        horizontalLayout_2->addLayout(verticalLayout_2);

        horizontalLayout_2->setStretch(0, 5);
        horizontalLayout_2->setStretch(1, 1);

        verticalLayout_5->addLayout(horizontalLayout_2);

        GUI->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(GUI);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(1);
        sizePolicy3.setVerticalStretch(2);
        sizePolicy3.setHeightForWidth(statusBar->sizePolicy().hasHeightForWidth());
        statusBar->setSizePolicy(sizePolicy3);
        statusBar->setFocusPolicy(Qt::TabFocus);
        GUI->setStatusBar(statusBar);

        retranslateUi(GUI);

        QMetaObject::connectSlotsByName(GUI);
    } // setupUi

    void retranslateUi(QMainWindow *GUI)
    {
        GUI->setWindowTitle(QApplication::translate("GUI", "Trajectory Planning", nullptr));
        checkBox_Ref_Path->setText(QApplication::translate("GUI", "Ref Path", nullptr));
        checkBox_RefPath_Heading->setText(QApplication::translate("GUI", "Heading", nullptr));
        checkBox_RefPath_Cr->setText(QApplication::translate("GUI", "Cr", nullptr));
        checkBox_BestPath_Heading->setText(QApplication::translate("GUI", "Heading", nullptr));
        checkBox_BestPath_Cr->setText(QApplication::translate("GUI", "Cr", nullptr));
        checkBox_Axis->setText(QApplication::translate("GUI", "Axis", nullptr));
        checkBox_Obstacle->setText(QApplication::translate("GUI", "Obstacle", nullptr));
        checkBox_BestPath->setText(QApplication::translate("GUI", "Best Path", nullptr));
        label_3->setText(QApplication::translate("GUI", "Y", nullptr));
        label->setText(QApplication::translate("GUI", "Scale", nullptr));
        label_2->setText(QApplication::translate("GUI", "X", nullptr));
        label_4->setText(QApplication::translate("GUI", "Sensitivity", nullptr));
        label_5->setText(QApplication::translate("GUI", "Target time", nullptr));
        checkBox_check_obstacles->setText(QApplication::translate("GUI", "Check obstacles", nullptr));
        checkBox_ay_soft_const->setText(QApplication::translate("GUI", "Soft constraint", nullptr));
        pushButton_Change_View->setText(QApplication::translate("GUI", "Change View", nullptr));
    } // retranslateUi

};

namespace Ui {
    class GUI: public Ui_GUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GUI_H
