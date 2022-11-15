#pragma once

#include <qobject.h>

class GUIUpdater : public QObject
{
    Q_OBJECT
    
public:

    void updateLabelCount(std::string count)    {emit SIGNAL_updateLabelCount(count);}
    void updateLabelCountI(std::string count)   {emit SIGNAL_updateLabelCountI(count);}
    void updateLabelCountI2(std::string count)  {emit SIGNAL_updateLabelCountI2(count);}
    void setUiEnabled(bool enabled)             {emit SIGNAL_setUiEnabled(enabled);}
    
    static GUIUpdater* _updaterInstance;
    static void init();
    
signals:
    void SIGNAL_updateLabelCount(std::string count);
    void SIGNAL_updateLabelCountI(std::string count);
    void SIGNAL_updateLabelCountI2(std::string count);
    void SIGNAL_setUiEnabled(bool enabled);
};
