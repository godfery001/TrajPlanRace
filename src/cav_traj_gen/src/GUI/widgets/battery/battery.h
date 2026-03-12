/*
 * battery.h
 * Battery widget class is defined here.
 * **YOU DO NOT NEED TO REVISE ANYTHING HERE**
*/

#ifndef BATTERY_H
#define BATTERY_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>

class Battery : public QWidget
{
    Q_OBJECT

        Q_PROPERTY(double minValue READ getMinValue WRITE setMinValue)
        Q_PROPERTY(double maxValue READ getMaxValue WRITE setMaxValue)
        Q_PROPERTY(double value READ getValue WRITE setValue)
        Q_PROPERTY(double alarmValue READ getAlarmValue WRITE setAlarmValue)

        Q_PROPERTY(int borderWidth READ getBorderWidth WRITE setBorderWidth)
        Q_PROPERTY(int borderRadius READ getBorderRadius WRITE setBorderRadius)
        Q_PROPERTY(int bgRadius READ getBgRadius WRITE setBgRadius)
        Q_PROPERTY(int headRadius READ getHeadRadius WRITE setHeadRadius)

        Q_PROPERTY(QColor borderColorStart READ getBorderColorStart WRITE setBorderColorStart)
        Q_PROPERTY(QColor borderColorEnd READ getBorderColorEnd WRITE setBorderColorEnd)

        Q_PROPERTY(QColor alarmColorStart READ getAlarmColorStart WRITE setAlarmColorStart)
        Q_PROPERTY(QColor alarmColorEnd READ getAlarmColorEnd WRITE setAlarmColorEnd)

        Q_PROPERTY(QColor normalColorStart READ getNormalColorStart WRITE setNormalColorStart)
        Q_PROPERTY(QColor normalColorEnd READ getNormalColorEnd WRITE setNormalColorEnd)

    public:
        explicit Battery(QWidget *parent = 0);
        ~Battery();

    protected:
        void paintEvent(QPaintEvent *);
        void drawBorder(QPainter *painter);
        void drawBg(QPainter *painter);
        void drawHead(QPainter *painter);
        void updateValueLabel();  // 更新标签的方法
        void resizeEvent(QResizeEvent *event);

    private:
        double minValue;                //最小值
        double maxValue;                //最大值
        double value;                   //目标电量
        double alarmValue;              //电池电量警戒值

        int borderWidth;                //边框粗细
        int borderRadius;               //边框圆角角度
        int bgRadius;                   //背景进度圆角角度
        int headRadius;                 //头部圆角角度

        QColor borderColorStart;        //边框渐变开始颜色
        QColor borderColorEnd;          //边框渐变结束颜色

        QColor alarmColorStart;         //电池低电量时的渐变开始颜色
        QColor alarmColorEnd;           //电池低电量时的渐变结束颜色

        QColor normalColorStart;        //电池正常电量时的渐变开始颜色
        QColor normalColorEnd;          //电池正常电量时的渐变结束颜色

        QRectF batteryRect;             //电池主体区域

        QLabel *valueLabel;  // 用于显示当前值的标签
        QVBoxLayout *layout;    // 布局指针


    public:
        double getMinValue()            const;
        double getMaxValue()            const;
        double getValue()               const;
        double getAlarmValue()          const;

        int getBorderWidth()            const;
        int getBorderRadius()           const;
        int getBgRadius()               const;
        int getHeadRadius()             const;

        QColor getBorderColorStart()    const;
        QColor getBorderColorEnd()      const;

        QColor getAlarmColorStart()     const;
        QColor getAlarmColorEnd()       const;

        QColor getNormalColorStart()    const;
        QColor getNormalColorEnd()      const;

        QSize sizeHint()                const;
        QSize minimumSizeHint()         const;

    public Q_SLOTS:
        //设置范围值
        void setRange(double minValue, double maxValue);
        void setRange(int minValue, int maxValue);

        //设置最大最小值
        void setMinValue(double minValue);
        void setMaxValue(double maxValue);

        //设置电池电量值
        void setValue(double value);
        void setValue(int value);

        //设置电池电量警戒值
        void setAlarmValue(double alarmValue);
        void setAlarmValue(int alarmValue);

        //设置边框粗细
        void setBorderWidth(int borderWidth);
        //设置边框圆角角度
        void setBorderRadius(int borderRadius);
        //设置背景圆角角度
        void setBgRadius(int bgRadius);
        //设置头部圆角角度
        void setHeadRadius(int headRadius);

        //设置边框渐变颜色
        void setBorderColorStart(const QColor &borderColorStart);
        void setBorderColorEnd(const QColor &borderColorEnd);

        //设置电池电量报警时的渐变颜色
        void setAlarmColorStart(const QColor &alarmColorStart);
        void setAlarmColorEnd(const QColor &alarmColorEnd);

        //设置电池电量正常时的渐变颜色
        void setNormalColorStart(const QColor &normalColorStart);
        void setNormalColorEnd(const QColor &normalColorEnd);
};

#endif // BATTERY_H
