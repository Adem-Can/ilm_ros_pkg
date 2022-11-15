#pragma once

#include <qobject.h>

class GUIUpdater : public QObject
{
    Q_OBJECT
    
public:

    //Update label count for point clouds
    void updateLabelCount(std::string count)    {emit SIGNAL_updateLabelCount(count);}

    //Update label count for images
    void updateLabelCountI(std::string count)   {emit SIGNAL_updateLabelCountI(count);}

    //
    void setUiState(int state)                  {emit SIGNAL_setUiState(state);}
    
    //The gui updater instance
    static GUIUpdater* _updaterInstance;

    //Sets up a global updater instance. Call once at the beginning
    static void init();
    
signals:
    void SIGNAL_updateLabelCount(std::string count);
    void SIGNAL_updateLabelCountI(std::string count);
    void SIGNAL_setUiState(int state);
};
