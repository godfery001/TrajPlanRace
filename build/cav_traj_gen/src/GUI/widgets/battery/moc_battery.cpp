/****************************************************************************
** Meta object code from reading C++ file 'battery.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/cav_traj_gen/src/GUI/widgets/battery/battery.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'battery.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Battery_t {
    QByteArrayData data[31];
    char stringdata0[408];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Battery_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Battery_t qt_meta_stringdata_Battery = {
    {
QT_MOC_LITERAL(0, 0, 7), // "Battery"
QT_MOC_LITERAL(1, 8, 8), // "setRange"
QT_MOC_LITERAL(2, 17, 0), // ""
QT_MOC_LITERAL(3, 18, 8), // "minValue"
QT_MOC_LITERAL(4, 27, 8), // "maxValue"
QT_MOC_LITERAL(5, 36, 11), // "setMinValue"
QT_MOC_LITERAL(6, 48, 11), // "setMaxValue"
QT_MOC_LITERAL(7, 60, 8), // "setValue"
QT_MOC_LITERAL(8, 69, 5), // "value"
QT_MOC_LITERAL(9, 75, 13), // "setAlarmValue"
QT_MOC_LITERAL(10, 89, 10), // "alarmValue"
QT_MOC_LITERAL(11, 100, 14), // "setBorderWidth"
QT_MOC_LITERAL(12, 115, 11), // "borderWidth"
QT_MOC_LITERAL(13, 127, 15), // "setBorderRadius"
QT_MOC_LITERAL(14, 143, 12), // "borderRadius"
QT_MOC_LITERAL(15, 156, 11), // "setBgRadius"
QT_MOC_LITERAL(16, 168, 8), // "bgRadius"
QT_MOC_LITERAL(17, 177, 13), // "setHeadRadius"
QT_MOC_LITERAL(18, 191, 10), // "headRadius"
QT_MOC_LITERAL(19, 202, 19), // "setBorderColorStart"
QT_MOC_LITERAL(20, 222, 16), // "borderColorStart"
QT_MOC_LITERAL(21, 239, 17), // "setBorderColorEnd"
QT_MOC_LITERAL(22, 257, 14), // "borderColorEnd"
QT_MOC_LITERAL(23, 272, 18), // "setAlarmColorStart"
QT_MOC_LITERAL(24, 291, 15), // "alarmColorStart"
QT_MOC_LITERAL(25, 307, 16), // "setAlarmColorEnd"
QT_MOC_LITERAL(26, 324, 13), // "alarmColorEnd"
QT_MOC_LITERAL(27, 338, 19), // "setNormalColorStart"
QT_MOC_LITERAL(28, 358, 16), // "normalColorStart"
QT_MOC_LITERAL(29, 375, 17), // "setNormalColorEnd"
QT_MOC_LITERAL(30, 393, 14) // "normalColorEnd"

    },
    "Battery\0setRange\0\0minValue\0maxValue\0"
    "setMinValue\0setMaxValue\0setValue\0value\0"
    "setAlarmValue\0alarmValue\0setBorderWidth\0"
    "borderWidth\0setBorderRadius\0borderRadius\0"
    "setBgRadius\0bgRadius\0setHeadRadius\0"
    "headRadius\0setBorderColorStart\0"
    "borderColorStart\0setBorderColorEnd\0"
    "borderColorEnd\0setAlarmColorStart\0"
    "alarmColorStart\0setAlarmColorEnd\0"
    "alarmColorEnd\0setNormalColorStart\0"
    "normalColorStart\0setNormalColorEnd\0"
    "normalColorEnd"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Battery[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
      14,  162, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,  104,    2, 0x0a /* Public */,
       1,    2,  109,    2, 0x0a /* Public */,
       5,    1,  114,    2, 0x0a /* Public */,
       6,    1,  117,    2, 0x0a /* Public */,
       7,    1,  120,    2, 0x0a /* Public */,
       7,    1,  123,    2, 0x0a /* Public */,
       9,    1,  126,    2, 0x0a /* Public */,
       9,    1,  129,    2, 0x0a /* Public */,
      11,    1,  132,    2, 0x0a /* Public */,
      13,    1,  135,    2, 0x0a /* Public */,
      15,    1,  138,    2, 0x0a /* Public */,
      17,    1,  141,    2, 0x0a /* Public */,
      19,    1,  144,    2, 0x0a /* Public */,
      21,    1,  147,    2, 0x0a /* Public */,
      23,    1,  150,    2, 0x0a /* Public */,
      25,    1,  153,    2, 0x0a /* Public */,
      27,    1,  156,    2, 0x0a /* Public */,
      29,    1,  159,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,    4,
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, QMetaType::Double,    4,
    QMetaType::Void, QMetaType::Double,    8,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Double,   10,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   12,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void, QMetaType::Int,   16,
    QMetaType::Void, QMetaType::Int,   18,
    QMetaType::Void, QMetaType::QColor,   20,
    QMetaType::Void, QMetaType::QColor,   22,
    QMetaType::Void, QMetaType::QColor,   24,
    QMetaType::Void, QMetaType::QColor,   26,
    QMetaType::Void, QMetaType::QColor,   28,
    QMetaType::Void, QMetaType::QColor,   30,

 // properties: name, type, flags
       3, QMetaType::Double, 0x00095103,
       4, QMetaType::Double, 0x00095103,
       8, QMetaType::Double, 0x00095103,
      10, QMetaType::Double, 0x00095103,
      12, QMetaType::Int, 0x00095103,
      14, QMetaType::Int, 0x00095103,
      16, QMetaType::Int, 0x00095103,
      18, QMetaType::Int, 0x00095103,
      20, QMetaType::QColor, 0x00095103,
      22, QMetaType::QColor, 0x00095103,
      24, QMetaType::QColor, 0x00095103,
      26, QMetaType::QColor, 0x00095103,
      28, QMetaType::QColor, 0x00095103,
      30, QMetaType::QColor, 0x00095103,

       0        // eod
};

void Battery::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Battery *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setRange((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->setRange((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->setMinValue((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->setMaxValue((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->setValue((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->setValue((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->setAlarmValue((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->setAlarmValue((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->setBorderWidth((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->setBorderRadius((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->setBgRadius((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->setHeadRadius((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->setBorderColorStart((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 13: _t->setBorderColorEnd((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 14: _t->setAlarmColorStart((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 15: _t->setAlarmColorEnd((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 16: _t->setNormalColorStart((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 17: _t->setNormalColorEnd((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        default: ;
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<Battery *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< double*>(_v) = _t->getMinValue(); break;
        case 1: *reinterpret_cast< double*>(_v) = _t->getMaxValue(); break;
        case 2: *reinterpret_cast< double*>(_v) = _t->getValue(); break;
        case 3: *reinterpret_cast< double*>(_v) = _t->getAlarmValue(); break;
        case 4: *reinterpret_cast< int*>(_v) = _t->getBorderWidth(); break;
        case 5: *reinterpret_cast< int*>(_v) = _t->getBorderRadius(); break;
        case 6: *reinterpret_cast< int*>(_v) = _t->getBgRadius(); break;
        case 7: *reinterpret_cast< int*>(_v) = _t->getHeadRadius(); break;
        case 8: *reinterpret_cast< QColor*>(_v) = _t->getBorderColorStart(); break;
        case 9: *reinterpret_cast< QColor*>(_v) = _t->getBorderColorEnd(); break;
        case 10: *reinterpret_cast< QColor*>(_v) = _t->getAlarmColorStart(); break;
        case 11: *reinterpret_cast< QColor*>(_v) = _t->getAlarmColorEnd(); break;
        case 12: *reinterpret_cast< QColor*>(_v) = _t->getNormalColorStart(); break;
        case 13: *reinterpret_cast< QColor*>(_v) = _t->getNormalColorEnd(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<Battery *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: _t->setMinValue(*reinterpret_cast< double*>(_v)); break;
        case 1: _t->setMaxValue(*reinterpret_cast< double*>(_v)); break;
        case 2: _t->setValue(*reinterpret_cast< double*>(_v)); break;
        case 3: _t->setAlarmValue(*reinterpret_cast< double*>(_v)); break;
        case 4: _t->setBorderWidth(*reinterpret_cast< int*>(_v)); break;
        case 5: _t->setBorderRadius(*reinterpret_cast< int*>(_v)); break;
        case 6: _t->setBgRadius(*reinterpret_cast< int*>(_v)); break;
        case 7: _t->setHeadRadius(*reinterpret_cast< int*>(_v)); break;
        case 8: _t->setBorderColorStart(*reinterpret_cast< QColor*>(_v)); break;
        case 9: _t->setBorderColorEnd(*reinterpret_cast< QColor*>(_v)); break;
        case 10: _t->setAlarmColorStart(*reinterpret_cast< QColor*>(_v)); break;
        case 11: _t->setAlarmColorEnd(*reinterpret_cast< QColor*>(_v)); break;
        case 12: _t->setNormalColorStart(*reinterpret_cast< QColor*>(_v)); break;
        case 13: _t->setNormalColorEnd(*reinterpret_cast< QColor*>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject Battery::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_Battery.data,
    qt_meta_data_Battery,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Battery::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Battery::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Battery.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Battery::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 18)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 18;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 18)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 18;
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 14;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 14;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 14;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 14;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 14;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
